// Copyright 2026 OSRA
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

mod blockage_monitor;
mod plan_registry;
pub mod ros_node;

pub use ros_node::PlanExecutorRosNode;

use crate::{
    blockage_monitor::{route_intersects_map, BlockageMonitor},
    plan_registry::PlanRegistry,
};
use mapf_post::{
    na::{Isometry2, Vector2},
    spatial_allocation::{CurrentPosition, Grid2D},
    SemanticWaypoint, Trajectory, WaypointFollower,
};
use ros_env::builtin_interfaces;
use ros_env::geometry_msgs::msg::Pose;
use ros_env::nav2_msgs;
use ros_env::nav2_msgs::msg::Costmap;
use ros_env::nav_msgs::msg::OccupancyGrid;
use ros_env::rmf_prototype_msgs;
use ros_env::rmf_prototype_msgs::msg::{
    DestinationConstraints, Plan, PlanError, PlanId, PlanRelease, RobotStatus, SafeZone,
    SafeZoneId, TargetOrientation, TrafficDependency,
};
use ros_env::std_msgs;
use std::{
    collections::{BTreeMap, HashMap},
    sync::Arc,
    time::Instant,
};

pub struct RobotState {
    pub radius: f32,
    pub latest_status: Option<RobotStatus>,
    pub plan: Option<Plan>,
    pub waypoint_follower: Option<WaypointFollower>,
    pub safe_zone_version: u64,
    pub last_incremental_target_wp: Option<usize>,
    blockage_monitor: BlockageMonitor,
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct ExecutionStepOutput {
    pub plan_release: Option<PlanRelease>,
    pub safe_zone: Option<SafeZone>,
    pub plan_error: Option<PlanError>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct PlanExecutorConfig {
    pub grid_width: u32,
    pub grid_height: u32,
    pub grid_resolution: f32,
    pub grid_origin: Pose,
}

impl Default for PlanExecutorConfig {
    fn default() -> Self {
        let mut origin = Pose::default();
        origin.orientation.w = 1.0;
        Self {
            grid_width: 20,
            grid_height: 20,
            grid_resolution: 1.0,
            grid_origin: origin,
        }
    }
}

impl PlanExecutorConfig {
    pub fn new(grid_width: u32, grid_height: u32, grid_resolution: f32, grid_origin: Pose) -> Self {
        Self {
            grid_width,
            grid_height,
            grid_resolution,
            grid_origin,
        }
    }

    pub fn with_grid_size(mut self, width: u32, height: u32) -> Self {
        self.grid_width = width;
        self.grid_height = height;
        self
    }

    pub fn with_resolution(mut self, resolution: f32) -> Self {
        self.grid_resolution = resolution;
        self
    }

    pub fn with_origin(mut self, origin: Pose) -> Self {
        self.grid_origin = origin;
        self
    }
}

pub struct PlanExecutor {
    // TODO(arjoc) The use of BTreeMap here is to make sure robots are ordered by their names
    // this helps with maintaining correspondence between their `mapf_post` ids and the input.
    // We should re-visit this after re-designing the mapf-post API a little.
    pub active_robots: BTreeMap<String, RobotState>,
    pub grid: Arc<Grid2D>,
    pub grid_width: u32,
    pub grid_height: u32,
    pub grid_resolution: f32,
    pub grid_origin: Pose,
    pub latest_map: Option<OccupancyGrid>,
    plan_registry: PlanRegistry,
}

impl Default for PlanExecutor {
    fn default() -> Self {
        Self::new()
    }
}

fn target_yaw(plan: &Plan, target_idx: usize) -> f32 {
    let Some(target) = plan.waypoints.get(target_idx) else {
        return 0.0;
    };

    for node in &target.arrival_constraints.nodes {
        if let Some(target_ori) = node.orientations.first() {
            return target_ori.orientation_radians;
        }
    }

    for region in &target.arrival_constraints.regions {
        if let Some(target_ori) = region.orientations.first() {
            return target_ori.orientation_radians;
        }
    }

    let [target_x, target_y] = target.position;

    for waypoint in plan.waypoints[..target_idx].iter().rev() {
        let dx = target_x - waypoint.position[0];
        let dy = target_y - waypoint.position[1];
        if dx.hypot(dy) > 1e-3 {
            return dy.atan2(dx);
        }
    }

    for waypoint in plan.waypoints.iter().skip(target_idx + 1) {
        let dx = waypoint.position[0] - target_x;
        let dy = waypoint.position[1] - target_y;
        if dx.hypot(dy) > 1e-3 {
            return dy.atan2(dx);
        }
    }

    0.0
}

fn is_constrained_orientation(ori: &TargetOrientation) -> bool {
    let effective_tol = 0.5 * ori.spread_radians.max(0.0) + ori.tolerance_radians.max(0.0);
    effective_tol < std::f32::consts::PI - 1e-3
}

fn target_orientation(plan: &Plan, target_idx: usize) -> TargetOrientation {
    if let Some(target) = plan.waypoints.get(target_idx) {
        let explicit_ori = target
            .arrival_constraints
            .nodes
            .iter()
            .find_map(|n| n.orientations.first())
            .or_else(|| {
                target
                    .arrival_constraints
                    .regions
                    .iter()
                    .find_map(|r| r.orientations.first())
            });
        if let Some(ori) = explicit_ori {
            return ori.clone();
        }
    }

    TargetOrientation {
        orientation_radians: target_yaw(plan, target_idx),
        spread_radians: 0.0,
        tolerance_radians: std::f32::consts::PI,
    }
}

impl PlanExecutor {
    pub fn new() -> Self {
        Self::with_config(PlanExecutorConfig::default())
    }

    pub fn new_with_config(config: PlanExecutorConfig) -> Self {
        Self::with_config(config)
    }

    pub fn with_config(config: PlanExecutorConfig) -> Self {
        let grid = Arc::new(Grid2D::new(
            vec![vec![0; config.grid_height as usize]; config.grid_width as usize],
            config.grid_resolution,
        ));
        Self {
            active_robots: BTreeMap::new(),
            grid,
            grid_width: config.grid_width,
            grid_height: config.grid_height,
            grid_resolution: config.grid_resolution,
            grid_origin: config.grid_origin,
            latest_map: None,
            plan_registry: PlanRegistry::default(),
        }
    }

    pub fn handle_robot_added(&mut self, robot_id: &str, radius: f32) -> bool {
        self.plan_registry.add_robot(robot_id.to_string(), radius);

        if !self.active_robots.contains_key(robot_id) {
            self.active_robots.insert(
                robot_id.to_string(),
                RobotState {
                    radius,
                    latest_status: None,
                    plan: None,
                    waypoint_follower: None,
                    safe_zone_version: 0,
                    last_incremental_target_wp: None,
                    blockage_monitor: BlockageMonitor::default(),
                },
            );
            return true;
        }
        false
    }

    pub fn handle_robot_removed(&mut self, robot_id: &str) -> bool {
        self.plan_registry.remove_robot(robot_id);
        self.active_robots.remove(robot_id).is_some()
    }

    fn get_agent_index(&self, robot_id: &str) -> Option<usize> {
        self.active_robots.keys().position(|k| k == robot_id)
    }

    pub fn handle_plan(&mut self, robot_id: &str, msg: Plan) -> bool {
        let Some(agent_idx) = self.get_agent_index(robot_id) else {
            return false;
        };

        let Some(state) = self.active_robots.get_mut(robot_id) else {
            return false;
        };

        if msg.waypoints.is_empty() {
            return false;
        }

        self.plan_registry
            .add_robot_plan(robot_id.to_string(), msg.clone());

        let poses: Vec<Isometry2<f32>> = msg
            .waypoints
            .iter()
            .map(|wp| Isometry2::new(Vector2::new(wp.position[0], wp.position[1]), 0.0))
            .collect();
        let mut follower = WaypointFollower::from_trajectory(agent_idx, Trajectory { poses });
        if let Some(status) = &state.latest_status {
            let current_x = status.pose.pose.position.x as f32;
            let current_y = status.pose.pose.position.y as f32;
            let position = Isometry2::new(Vector2::new(current_x, current_y), 0.0);
            follower.update_position_estimate(&position, 0.5);
        }

        state.plan = Some(msg);
        state.waypoint_follower = Some(follower);
        state.safe_zone_version = 0;
        state.last_incremental_target_wp = None;
        state.blockage_monitor.begin_plan();
        true
    }

    pub fn handle_map(&mut self, msg: OccupancyGrid) -> Vec<(String, PlanError)> {
        if msg.info.width > 0 && msg.info.height > 0 && msg.info.resolution > 0.0 {
            self.grid_width = msg.info.width;
            self.grid_height = msg.info.height;
            self.grid_resolution = msg.info.resolution;
            self.grid_origin = msg.info.origin.clone();
            self.grid = Arc::new(Grid2D::new(
                vec![vec![0; self.grid_height as usize]; self.grid_width as usize],
                self.grid_resolution,
            ));
        }

        self.latest_map = Some(msg);
        let robot_ids: Vec<_> = self.active_robots.keys().cloned().collect();
        let mut errors = Vec::new();
        for robot_id in robot_ids {
            if let Some(error) = self.update_route_blockage(&robot_id) {
                errors.push((robot_id, error));
            }
        }
        errors
    }

    /// Decide whether a traffic dependency has been satisfied.
    ///
    /// We prefer the blocker's own reported `Progress` in `RobotStatus`, because
    /// that is its own statement of how far it has got. We only fall back to
    /// projecting its pose onto its plan when it has not reported progress against
    /// the plan we are depending on.
    fn blocker_is_satisfied(
        &self,
        blocker: &TrafficDependency,
        semantic_waypoints: &HashMap<String, SemanticWaypoint>,
    ) -> bool {
        if let Some(blocker_state) = self.active_robots.get(&blocker.name) {
            if let Some(status) = &blocker_state.latest_status {
                if let Some(rmw_prog) = status.progress.first() {
                    if rmw_prog.plan_id.destination_session.uuid
                        == blocker.plan_id.destination_session.uuid
                        && rmw_prog.plan_id.plan_version == blocker.plan_id.plan_version
                    {
                        return rmw_prog.progress >= blocker.required_progress;
                    }
                }
            }
            if let Some(active_plan) = &blocker_state.plan {
                if blocker.plan_id != PlanId::default() && active_plan.plan_id != blocker.plan_id {
                    // The blocker robot has moved on to a different plan.
                    return true;
                }
            }
        }

        match semantic_waypoints.get(&blocker.name) {
            Some(sem) => sem.trajectory_index >= blocker.required_progress.round() as usize,
            // We know nothing about this participant, so we must assume it is in
            // the way.
            None => false,
        }
    }

    pub fn handle_status(&mut self, robot_id: &str, msg: RobotStatus) -> ExecutionStepOutput {
        let current_x = msg.pose.pose.position.x as f32;
        let current_y = msg.pose.pose.position.y as f32;
        let position = Isometry2::new(Vector2::new(current_x, current_y), 0.0);

        self.plan_registry
            .update_stationary_position(robot_id, position);

        {
            let Some(state) = self.active_robots.get_mut(robot_id) else {
                return ExecutionStepOutput::default();
            };

            state.latest_status = Some(msg.clone());

            if let (Some(fw), Some(plan)) = (&mut state.waypoint_follower, &state.plan) {
                if msg.mode == RobotStatus::MODE_ACTION {
                    if let Some(rmw_prog) = msg.progress.first() {
                        if rmw_prog.plan_id.destination_session.uuid
                            == plan.plan_id.destination_session.uuid
                            && rmw_prog.plan_id.plan_version == plan.plan_id.plan_version
                        {
                            let reached = rmw_prog.reached_waypoint as usize;
                            if reached > fw.get_semantic_waypoint().trajectory_index {
                                if let Some(wp) = plan.waypoints.get(reached) {
                                    let wp_pos = Isometry2::new(
                                        Vector2::new(wp.position[0], wp.position[1]),
                                        0.0,
                                    );
                                    fw.update_position_estimate(&wp_pos, 0.1);
                                }
                            }
                        }
                    }
                } else {
                    fw.update_position_estimate(&position, 0.5);
                }
                let after = fw.get_semantic_waypoint().trajectory_index;
                self.plan_registry
                    .update_progress(robot_id, after, plan.plan_id.clone());
            }
        }

        let plan_error = self.update_route_blockage(robot_id);

        if !self.ready_to_execute(robot_id) {
            return ExecutionStepOutput {
                plan_release: None,
                safe_zone: None,
                plan_error,
            };
        }

        // Cache all semantic waypoints first because get_semantic_waypoint requires &mut self.
        // Doing this first avoids borrowing active_robots as mutable during later immutable reads.
        let mut semantic_waypoints = HashMap::new();
        for (robot_index, (name, r_state)) in self.active_robots.iter_mut().enumerate() {
            if let Some(fw) = &mut r_state.waypoint_follower {
                semantic_waypoints.insert(
                    name.clone(),
                    SemanticWaypoint {
                        agent: robot_index,
                        trajectory_index: fw.get_semantic_waypoint().trajectory_index,
                    },
                );
            }
        }

        // 1. Calculate PlanRelease
        let robot_state = &self.active_robots[robot_id];
        let plan = robot_state.plan.as_ref().unwrap();
        let curr_wp_idx = semantic_waypoints
            .get(robot_id)
            .map(|w| w.trajectory_index)
            .unwrap_or(0);

        let mut released_wp_idx = curr_wp_idx;
        while released_wp_idx < plan.waypoints.len() {
            let wp = &plan.waypoints[released_wp_idx];
            if released_wp_idx > curr_wp_idx
                && (!wp.arrival_constraints.nodes.is_empty()
                    || wp
                        .arrival_constraints
                        .regions
                        .iter()
                        .any(|r| r.orientations.iter().any(is_constrained_orientation))
                    || !wp.arrival_action.is_empty()
                    || !wp.departure_action.is_empty())
            {
                break;
            }
            let mut blocked = false;
            for blocker in &wp.departure_blockers {
                if !self.blocker_is_satisfied(blocker, &semantic_waypoints) {
                    blocked = true;
                    break;
                }
            }
            if blocked {
                break;
            }
            released_wp_idx += 1;
        }

        if released_wp_idx < plan.waypoints.len() {
            let wp_pos = &plan.waypoints[released_wp_idx].position;
            for (r_name, r_state) in &self.active_robots {
                if r_name == robot_id {
                    continue;
                }
                if let Some(status) = &r_state.latest_status {
                    let dx = wp_pos[0] - status.pose.pose.position.x as f32;
                    let dy = wp_pos[1] - status.pose.pose.position.y as f32;
                    if dx.hypot(dy) < 0.8 {
                        released_wp_idx = released_wp_idx.saturating_sub(1);
                        break;
                    }
                }
            }
        }

        if released_wp_idx >= plan.waypoints.len() {
            released_wp_idx = plan.waypoints.len() - 1;
        }

        let plan_release = PlanRelease {
            waypoint_id: released_wp_idx as u64,
            plan_id: plan.plan_id.clone(),
        };

        // --- Dynamic grid resizing ---
        let mut min_x = f32::MAX;
        let mut min_y = f32::MAX;
        let mut max_x = f32::MIN;
        let mut max_y = f32::MIN;

        for r_state in self.active_robots.values() {
            if let Some(r_status) = &r_state.latest_status {
                min_x = min_x.min(r_status.pose.pose.position.x as f32);
                min_y = min_y.min(r_status.pose.pose.position.y as f32);
                max_x = max_x.max(r_status.pose.pose.position.x as f32);
                max_y = max_y.max(r_status.pose.pose.position.y as f32);
            }
            if let Some(r_plan) = &r_state.plan {
                for wp in &r_plan.waypoints {
                    min_x = min_x.min(wp.position[0]);
                    min_y = min_y.min(wp.position[1]);
                    max_x = max_x.max(wp.position[0]);
                    max_y = max_y.max(wp.position[1]);
                }
            }
        }

        let current_min_x = self.grid_origin.position.x as f32;
        let current_min_y = self.grid_origin.position.y as f32;
        let current_max_x = current_min_x + (self.grid_width as f32 * self.grid_resolution);
        let current_max_y = current_min_y + (self.grid_height as f32 * self.grid_resolution);

        if min_x < current_min_x
            || min_y < current_min_y
            || max_x > current_max_x
            || max_y > current_max_y
        {
            let new_min_x = min_x - 20.0;
            let new_min_y = min_y - 20.0;
            let new_max_x = max_x + 20.0;
            let new_max_y = max_y + 20.0;

            self.grid_origin.position.x = new_min_x as f64;
            self.grid_origin.position.y = new_min_y as f64;
            self.grid_origin.orientation.w = 1.0;

            self.grid_width = ((new_max_x - new_min_x) / self.grid_resolution).ceil() as u32;
            self.grid_height = ((new_max_y - new_min_y) / self.grid_resolution).ceil() as u32;

            self.grid = Arc::new(Grid2D::new(
                vec![vec![0; self.grid_height as usize]; self.grid_width as usize],
                self.grid_resolution,
            ));
        }
        // --- End dynamic grid resizing ---

        // 2. Generate SafeZone using the DFS-reconstructed multi-plan snapshot
        let mut snapshot = self.plan_registry.get_mapf_snapshot();
        let origin_offset = Vector2::new(
            self.grid_origin.position.x as f32,
            self.grid_origin.position.y as f32,
        );
        for traj in &mut snapshot.mapf_result.trajectories {
            for pose in &mut traj.poses {
                pose.translation.vector -= origin_offset;
            }
        }

        let mut current_positions = Vec::new();
        for (idx, r_name) in snapshot.included_robots.iter().enumerate() {
            let r_state = &self.active_robots[r_name];
            let real_position = if let Some(r_status) = &r_state.latest_status {
                (
                    r_status.pose.pose.position.x as f32 - self.grid_origin.position.x as f32,
                    r_status.pose.pose.position.y as f32 - self.grid_origin.position.y as f32,
                )
            } else {
                (0.0, 0.0)
            };

            let semantic_position = semantic_waypoints
                .get(r_name)
                .map(|sem| {
                    let global_idx = snapshot
                        .get_global_waypoint_index(r_name, sem.trajectory_index)
                        .unwrap_or(sem.trajectory_index);
                    SemanticWaypoint {
                        agent: idx,
                        trajectory_index: global_idx,
                    }
                })
                .unwrap_or(SemanticWaypoint {
                    agent: idx,
                    trajectory_index: 0,
                });
            current_positions.push(CurrentPosition {
                semantic_position,
                real_position,
            });
        }

        let allocation_field = self
            .grid
            .allocate_trajectory(&snapshot.mapf_result, &current_positions);

        let snapshot_agent_idx = snapshot.get_agent_index(robot_id).unwrap_or(0);
        let positions = allocation_field
            .get_alloc_for_agent(snapshot_agent_idx)
            .unwrap_or_default();

        let target_wp = &plan.waypoints[released_wp_idx];
        let target_x = target_wp.position[0];
        let target_y = target_wp.position[1];
        let target_ori = target_orientation(plan, released_wp_idx);
        let target_tolerance = target_wp
            .arrival_constraints
            .regions
            .first()
            .map(|r| r.tolerance)
            .filter(|&t| t > 0.0)
            .unwrap_or(0.2);
        let mut target_nodes = target_wp.arrival_constraints.nodes.clone();
        if is_constrained_orientation(&target_ori) {
            for node in &mut target_nodes {
                if node.orientations.is_empty() {
                    node.orientations.push(target_ori.clone());
                }
            }
        }
        let target_progress = target_wp.progress;

        let costmap = Self::to_costmap_msg(
            &positions,
            self.grid_width,
            self.grid_height,
            self.grid_resolution,
            self.grid_origin.clone(),
            msg.pose.header.stamp.clone(),
        );

        let plan_id = plan.plan_id.clone();

        // We need to update safezones all the time.
        // TODO(arjoc) see if we can update safezones only when there are changes in the
        // shape of the safe zone or target to limit network traffic.
        let state = self.active_robots.get_mut(robot_id).unwrap();
        state.safe_zone_version += 1;

        let safe_zone = SafeZone {
            incremental_target: DestinationConstraints {
                regions: vec![rmf_prototype_msgs::msg::TargetRegion {
                    tolerance: target_tolerance,
                    region: rmf_prototype_msgs::msg::Region {
                        points: vec![target_x, target_y],
                        hint: rmf_prototype_msgs::msg::Region::HINT_POINT,
                    },
                    orientations: vec![target_ori],
                }],
                nodes: target_nodes,
            },
            costmap,
            target_waypoint: vec![released_wp_idx as u64].try_into().unwrap(),
            last_waypoint: curr_wp_idx as u64,
            target_progress,
            id: SafeZoneId {
                plan_id,
                safe_zone_version: state.safe_zone_version,
            },
        };

        ExecutionStepOutput {
            plan_release: Some(plan_release),
            safe_zone: Some(safe_zone),
            plan_error,
        }
    }

    fn update_route_blockage(&mut self, robot_id: &str) -> Option<PlanError> {
        let map = self.latest_map.as_ref()?;
        let state = self.active_robots.get_mut(robot_id)?;
        let (Some(plan), Some(status), Some(follower)) = (
            state.plan.as_ref(),
            state.latest_status.as_ref(),
            state.waypoint_follower.as_mut(),
        ) else {
            return None;
        };

        let remaining = follower.remaining_trajectory();
        let mut route = Vec::with_capacity(remaining.len() + 1);
        route.push((
            status.pose.pose.position.x as f32,
            status.pose.pose.position.y as f32,
        ));
        route.extend(remaining);

        let blocked = route_intersects_map(map, &route, state.radius);
        let plan_id = plan.plan_id.clone();
        if !state
            .blockage_monitor
            .observe(blocked, &plan_id, Instant::now())
        {
            return None;
        }

        Some(PlanError {
            error: rmf_prototype_msgs::msg::Error {
                code: PlanError::CODE_PATH_BLOCKED,
                message: format!("Updated map blocks the remaining route for {robot_id}"),
                parameters: String::new(),
            },
            plan_id,
        })
    }

    /// Whether we can generate a release and safe zone for `robot_id`.
    fn ready_to_execute(&self, robot_id: &str) -> bool {
        self.active_robots.get(robot_id).is_some_and(|state| {
            state.latest_status.is_some()
                && state.plan.is_some()
                && state.waypoint_follower.is_some()
        })
    }

    pub fn to_costmap_msg(
        positions: &[(usize, usize)],
        width: u32,
        height: u32,
        resolution: f32,
        origin: Pose,
        stamp: builtin_interfaces::msg::Time,
    ) -> Costmap {
        let mut data = vec![254u8; (width * height) as usize];
        for &(x, y) in positions {
            if x < width as usize && y < height as usize {
                // Grid2D from mapf_post returns y with top-left origin (i.e. flipped)
                // We need to invert y back for ROS Costmap which expects bottom-left origin
                let inverted_y = (height - 1) as usize - y;
                let index = inverted_y * (width as usize) + x;
                data[index] = 0;
            }
        }

        Costmap {
            header: std_msgs::msg::Header {
                stamp: stamp.clone(),
                frame_id: "map".to_string(),
            },
            metadata: nav2_msgs::msg::CostmapMetaData {
                map_load_time: stamp.clone(),
                update_time: stamp,
                layer: "safe_zone".to_string(),
                resolution,
                size_x: width,
                size_y: height,
                origin,
            },
            data,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::{target_orientation, target_yaw, PlanExecutor, PlanExecutorConfig};
    use mapf_post::na::{Isometry2, Vector2};
    use mapf_post::{Trajectory, WaypointFollower};
    use ros_env::{
        geometry_msgs::msg::Pose,
        nav_msgs::msg::OccupancyGrid,
        rmf_prototype_msgs::msg::{Plan, PlanId, RobotStatus, TrafficDependency, Waypoint},
    };

    fn plan_with_positions(positions: &[[f32; 2]]) -> Plan {
        Plan {
            waypoints: positions
                .iter()
                .enumerate()
                .map(|(i, position)| Waypoint {
                    position: *position,
                    progress: i as f32,
                    ..Default::default()
                })
                .collect(),
            ..Default::default()
        }
    }

    fn status_at(x: f64, y: f64) -> RobotStatus {
        let mut status = RobotStatus::default();
        status.pose.pose.position.x = x;
        status.pose.pose.position.y = y;
        status
    }

    #[test]
    fn target_yaw_ignores_stationary_wait_waypoints() {
        let plan = plan_with_positions(&[[3.0, 4.0], [3.0, 4.0], [3.0, 4.0], [3.0, 5.0]]);

        assert!((target_yaw(&plan, 3) - std::f32::consts::FRAC_PI_2).abs() < 1e-6);
    }

    #[test]
    fn target_yaw_uses_departure_direction_at_trajectory_start() {
        let plan = plan_with_positions(&[[3.0, 4.0], [3.0, 4.0], [2.0, 4.0]]);

        assert!((target_yaw(&plan, 0) - std::f32::consts::PI).abs() < 1e-6);
    }

    #[test]
    fn stationary_trajectory_has_neutral_yaw() {
        let plan = plan_with_positions(&[[3.0, 4.0], [3.0, 4.0]]);

        assert_eq!(target_yaw(&plan, 1), 0.0);
    }

    #[test]
    fn target_yaw_prefers_explicit_arrival_constraint_orientation() {
        use ros_env::rmf_prototype_msgs::msg::{TargetOrientation, TargetRegion};

        let mut plan = plan_with_positions(&[[0.0, 0.0], [5.0, 0.0]]);
        let mut region = TargetRegion::default();
        region.orientations.push(TargetOrientation {
            orientation_radians: -std::f32::consts::FRAC_PI_2,
            ..Default::default()
        });
        plan.waypoints[1].arrival_constraints.regions.push(region);

        assert!((target_yaw(&plan, 1) - (-std::f32::consts::FRAC_PI_2)).abs() < 1e-6);

        // Unconstrained waypoint 0 gets travel heading 0.0 and tolerance PI
        let ori0 = target_orientation(&plan, 0);
        assert!((ori0.orientation_radians - 0.0).abs() < 1e-6);
        assert!((ori0.tolerance_radians - std::f32::consts::PI).abs() < 1e-6);

        // Constrained waypoint 1 gets explicit -PI/2 and preserves the input tolerance (0.0)
        let ori1 = target_orientation(&plan, 1);
        assert!((ori1.orientation_radians - (-std::f32::consts::FRAC_PI_2)).abs() < 1e-6);
        assert!((ori1.tolerance_radians - 0.0).abs() < 1e-6);
    }

    #[test]
    fn executor_releases_and_generates_safe_zone_across_plans_with_stationary_peer() {
        let mut executor = PlanExecutor::new();
        executor.handle_robot_added("robot_1", 0.5);
        executor.handle_robot_added("robot_2", 0.5);
        executor.handle_robot_added("robot_3", 0.5);

        let mut plan_1 = plan_with_positions(&[[1.0, 1.0], [2.0, 1.0], [3.0, 1.0]]);
        plan_1.plan_id = PlanId {
            plan_version: 1,
            ..Default::default()
        };
        plan_1.plan_id.destination_session.uuid = [1; 16];
        let plan_1_id = plan_1.plan_id.clone();

        let mut plan_2 = plan_with_positions(&[[5.0, 5.0], [6.0, 5.0]]);
        plan_2.plan_id = PlanId {
            plan_version: 1,
            ..Default::default()
        };
        plan_2.plan_id.destination_session.uuid = [2; 16];
        plan_2.waypoints[0].departure_blockers = vec![TrafficDependency {
            name: "robot_1".to_string(),
            plan_id: plan_1_id.clone(),
            required_progress: 2.0,
        }];

        executor.handle_plan("robot_1", plan_1);
        executor.handle_plan("robot_2", plan_2);

        // Localize all three robots (robot_3 has no plan and remains stationary)
        executor.handle_status("robot_1", status_at(1.0, 1.0));
        executor.handle_status("robot_3", status_at(10.0, 10.0));

        // robot_2 is blocked at wp 0 until robot_1 reaches progress 2.0
        let out_blocked = executor.handle_status("robot_2", status_at(5.0, 5.0));
        assert_eq!(out_blocked.plan_release.unwrap().waypoint_id, 0);
        assert!(out_blocked.safe_zone.is_some());

        // Report robot_1 progress = 2.0 on its plan via RobotStatus
        let mut status_r1_progressed = status_at(1.0, 1.0);
        let mut rmw_prog = ros_env::rmf_prototype_msgs::msg::rmw::Progress::default();
        rmw_prog.plan_id.destination_session.uuid = plan_1_id.destination_session.uuid;
        rmw_prog.plan_id.plan_version = plan_1_id.plan_version;
        rmw_prog.progress = 2.0;
        rmw_prog.reached_waypoint = 2;
        rmw_prog.target_waypoint = 2;
        status_r1_progressed.progress = vec![rmw_prog].try_into().unwrap();
        executor.handle_status("robot_1", status_r1_progressed);

        let out_released = executor.handle_status("robot_2", status_at(5.0, 5.0));
        assert_eq!(out_released.plan_release.unwrap().waypoint_id, 1);
        assert!(out_released.safe_zone.is_some());
    }

    #[test]
    fn handle_plan_does_not_reset_peer_action_follower() {
        let mut executor = PlanExecutor::new();
        executor.handle_robot_added("robot_1", 0.5);
        executor.handle_robot_added("robot_2", 0.5);

        let mut plan_1 = plan_with_positions(&[[0.0, 0.0], [1.0, 0.0], [2.0, 0.0]]);
        plan_1.plan_id.destination_session.uuid = [1; 16];
        let plan_1_id = plan_1.plan_id.clone();

        executor.handle_plan("robot_1", plan_1);
        executor.handle_status("robot_2", status_at(5.0, 5.0));

        // Advance robot_1 to waypoint 2 via RobotStatus (MODE_ACTION + Progress) while pose is off-path
        let mut action_status = status_at(0.0, 0.0);
        action_status.mode = RobotStatus::MODE_ACTION;
        let mut rmw_prog = ros_env::rmf_prototype_msgs::msg::rmw::Progress::default();
        rmw_prog.plan_id.destination_session.uuid = plan_1_id.destination_session.uuid;
        rmw_prog.plan_id.plan_version = plan_1_id.plan_version;
        rmw_prog.progress = 2.0;
        rmw_prog.reached_waypoint = 2;
        rmw_prog.target_waypoint = 2;
        action_status.progress = vec![rmw_prog].try_into().unwrap();
        executor.handle_status("robot_1", action_status.clone());
        assert_eq!(
            executor
                .active_robots
                .get_mut("robot_1")
                .unwrap()
                .waypoint_follower
                .as_mut()
                .unwrap()
                .get_semantic_waypoint()
                .trajectory_index,
            2
        );

        // Now send a new plan for robot_2 while robot_1 is still in MODE_ACTION at (0.0, 0.0)
        executor.handle_status("robot_1", action_status);
        let plan_2 = plan_with_positions(&[[5.0, 5.0], [6.0, 5.0]]);
        executor.handle_plan("robot_2", plan_2);

        // robot_1's follower must NOT have been reset back to 0!
        assert_eq!(
            executor
                .active_robots
                .get_mut("robot_1")
                .unwrap()
                .waypoint_follower
                .as_mut()
                .unwrap()
                .get_semantic_waypoint()
                .trajectory_index,
            2
        );
    }

    #[test]
    fn test_robot_2_follower() {
        let poses = vec![
            Isometry2::new(Vector2::new(3.0, 0.0), 0.0),
            Isometry2::new(Vector2::new(4.0, 0.0), 0.0),
            Isometry2::new(Vector2::new(5.0, 0.0), 0.0),
            Isometry2::new(Vector2::new(6.0, 0.0), 0.0),
            Isometry2::new(Vector2::new(7.0, 0.0), 0.0),
            Isometry2::new(Vector2::new(8.0, 0.0), 0.0),
            Isometry2::new(Vector2::new(9.0, 0.0), 0.0),
            Isometry2::new(Vector2::new(10.0, 0.0), 0.0),
            Isometry2::new(Vector2::new(10.0, 0.0), 0.0),
            Isometry2::new(Vector2::new(10.0, 0.0), 0.0),
        ];
        let mut follower = WaypointFollower::from_trajectory(0, Trajectory { poses });

        // Simulating the robot moving through the path:
        follower.update_position_estimate(&Isometry2::new(Vector2::new(3.0, 0.0), 0.0), 0.5);
        assert_eq!(follower.get_semantic_waypoint().trajectory_index, 0);

        follower.update_position_estimate(&Isometry2::new(Vector2::new(4.0, 0.0), 0.0), 0.5);
        assert_eq!(follower.get_semantic_waypoint().trajectory_index, 1);

        follower.update_position_estimate(&Isometry2::new(Vector2::new(5.0, 0.0), 0.0), 0.5);
        assert_eq!(follower.get_semantic_waypoint().trajectory_index, 2);

        follower.update_position_estimate(&Isometry2::new(Vector2::new(6.0, 0.0), 0.0), 0.5);
        assert_eq!(follower.get_semantic_waypoint().trajectory_index, 3);

        follower.update_position_estimate(&Isometry2::new(Vector2::new(7.0, 0.0), 0.0), 0.5);
        assert_eq!(follower.get_semantic_waypoint().trajectory_index, 4);

        follower.update_position_estimate(&Isometry2::new(Vector2::new(8.0, 0.0), 0.0), 0.5);
        assert_eq!(follower.get_semantic_waypoint().trajectory_index, 5);

        follower.update_position_estimate(&Isometry2::new(Vector2::new(9.0, 0.0), 0.0), 0.5);
        assert_eq!(follower.get_semantic_waypoint().trajectory_index, 6);

        follower.update_position_estimate(&Isometry2::new(Vector2::new(10.0, 0.0), 0.0), 0.5);
        assert!(follower.get_semantic_waypoint().trajectory_index >= 7);
    }

    #[test]
    fn test_plan_executor_config_default_and_builders() {
        let config = PlanExecutorConfig::default();
        assert_eq!(config.grid_width, 20);
        assert_eq!(config.grid_height, 20);
        assert_eq!(config.grid_resolution, 1.0);
        assert_eq!(config.grid_origin.orientation.w, 1.0);

        let mut custom_origin = Pose::default();
        custom_origin.position.x = -10.0;
        custom_origin.position.y = -5.0;
        custom_origin.orientation.w = 1.0;

        let custom_config = PlanExecutorConfig::new(50, 60, 0.5, custom_origin.clone());
        assert_eq!(custom_config.grid_width, 50);
        assert_eq!(custom_config.grid_height, 60);
        assert_eq!(custom_config.grid_resolution, 0.5);
        assert_eq!(custom_config.grid_origin.position.x, -10.0);

        let builder_config = PlanExecutorConfig::default()
            .with_grid_size(100, 200)
            .with_resolution(0.2)
            .with_origin(custom_origin.clone());
        assert_eq!(builder_config.grid_width, 100);
        assert_eq!(builder_config.grid_height, 200);
        assert_eq!(builder_config.grid_resolution, 0.2);
        assert_eq!(builder_config.grid_origin, custom_origin);
    }

    #[test]
    fn test_handle_map() {
        let mut plan_executor = PlanExecutor::new();
        assert_eq!(plan_executor.grid_width, 20);
        assert_eq!(plan_executor.grid_height, 20);
        assert_eq!(plan_executor.grid_resolution, 1.0);

        let mut map_msg = OccupancyGrid::default();
        map_msg.info.width = 150;
        map_msg.info.height = 200;
        map_msg.info.resolution = 0.05;
        map_msg.info.origin.position.x = -15.0;
        map_msg.info.origin.position.y = -10.0;
        map_msg.info.origin.orientation.w = 1.0;

        let errors = plan_executor.handle_map(map_msg);
        assert!(errors.is_empty());

        assert_eq!(plan_executor.grid_width, 150);
        assert_eq!(plan_executor.grid_height, 200);
        assert_eq!(plan_executor.grid_resolution, 0.05);
        assert_eq!(plan_executor.grid_origin.position.x, -15.0);
        assert_eq!(plan_executor.grid_origin.position.y, -10.0);
    }

    #[test]
    fn test_uncommanded_robot_execution() {
        let mut plan_executor = PlanExecutor::new();

        // Add two robots
        assert!(plan_executor.handle_robot_added("robot_1", 0.5));
        assert!(plan_executor.handle_robot_added("robot_2", 0.5));

        // Send status for both robots
        let mut status1 = RobotStatus::default();
        status1.pose.pose.position.x = 0.0;
        status1.pose.pose.position.y = 0.0;
        let out1 = plan_executor.handle_status("robot_1", status1.clone());
        assert!(out1.plan_release.is_none());
        assert!(out1.safe_zone.is_none());

        let mut status2 = RobotStatus::default();
        status2.pose.pose.position.x = 5.0;
        status2.pose.pose.position.y = 5.0;
        let out2 = plan_executor.handle_status("robot_2", status2.clone());
        assert!(out2.plan_release.is_none());
        assert!(out2.safe_zone.is_none());

        // Only give a plan to robot_1. robot_2 remains uncommanded!
        let mut plan1 = Plan::default();
        let mut wp0 = Waypoint::default();
        wp0.position = [0.0, 0.0];
        let mut wp1 = Waypoint::default();
        wp1.position = [2.0, 0.0];
        plan1.waypoints = vec![wp0, wp1];

        assert!(plan_executor.handle_plan("robot_1", plan1));

        // robot_1 status arrives: it should execute without error even though robot_2 has no plan!
        let out1 = plan_executor.handle_status("robot_1", status1);
        assert!(out1.plan_release.is_some());
        assert_eq!(out1.plan_release.unwrap().waypoint_id, 1);
        assert!(out1.safe_zone.is_some());

        // robot_2 has no plan, so its status produces no release or safe zone
        let out2 = plan_executor.handle_status("robot_2", status2);
        assert!(out2.plan_release.is_none());
        assert!(out2.safe_zone.is_none());
    }

    #[test]
    fn test_blocker_holds_and_releases_plan() {
        let mut plan_executor = PlanExecutor::new();
        plan_executor.handle_robot_added("robot_1", 0.5);
        plan_executor.handle_robot_added("robot_2", 0.5);

        // robot_1 moves from (0,0) -> (1,0) -> (2,0), blocked at wp 1 until robot_2 reaches wp 1
        let mut wp0_r1 = Waypoint::default();
        wp0_r1.position = [0.0, 0.0];
        let mut wp1_r1 = Waypoint::default();
        wp1_r1.position = [1.0, 0.0];
        wp1_r1.departure_blockers = vec![TrafficDependency {
            name: "robot_2".to_string(),
            required_progress: 1.0,
            ..Default::default()
        }];
        let mut wp2_r1 = Waypoint::default();
        wp2_r1.position = [2.0, 0.0];

        let mut plan1 = Plan::default();
        plan1.waypoints = vec![wp0_r1, wp1_r1, wp2_r1];

        let plan2 = plan_with_positions(&[[0.0, 5.0], [2.0, 5.0]]);

        plan_executor.handle_plan("robot_1", plan1);
        plan_executor.handle_plan("robot_2", plan2);

        let mut status2 = RobotStatus::default();
        status2.pose.pose.position.x = 0.0;
        status2.pose.pose.position.y = 5.0;
        plan_executor.handle_status("robot_2", status2);

        let mut status1 = RobotStatus::default();
        status1.pose.pose.position.x = 0.0;
        status1.pose.pose.position.y = 0.0;
        let out1 = plan_executor.handle_status("robot_1", status1.clone());
        // Held at waypoint 1 because robot_2 is still at waypoint 0
        assert_eq!(out1.plan_release.unwrap().waypoint_id, 1);

        // Advance robot_2 to waypoint 1 (2.0, 5.0)
        let mut status2_reached = RobotStatus::default();
        status2_reached.pose.pose.position.x = 2.0;
        status2_reached.pose.pose.position.y = 5.0;
        plan_executor.handle_status("robot_2", status2_reached);

        // Now robot_1 should be released up to waypoint 2
        let out1_released = plan_executor.handle_status("robot_1", status1);
        assert_eq!(out1_released.plan_release.unwrap().waypoint_id, 2);
    }

    #[test]
    fn test_safe_zone_forwards_target_nodes_and_tolerance() {
        use ros_env::rmf_prototype_msgs::msg::{
            GraphElementKey, Region, TargetNode, TargetOrientation, TargetRegion,
        };

        let mut plan_executor = PlanExecutor::new();
        plan_executor.handle_robot_added("robot_1", 0.5);

        let mut plan = plan_with_positions(&[[0.0, 0.0], [2.0, 0.0]]);
        plan.waypoints[1].arrival_constraints.regions = vec![TargetRegion {
            tolerance: 0.42,
            region: Region {
                points: vec![2.0, 0.0],
                hint: Region::HINT_POINT,
            },
            orientations: vec![],
        }];
        let mut key = GraphElementKey::default();
        key.key = vec![42i64].try_into().unwrap();
        key.name = vec!["station_alpha".to_string().into()].try_into().unwrap();
        plan.waypoints[1]
            .arrival_constraints
            .nodes
            .push(TargetNode {
                key,
                orientations: vec![TargetOrientation {
                    orientation_radians: std::f32::consts::FRAC_PI_2,
                    spread_radians: 0.0,
                    tolerance_radians: 0.12,
                }],
            });

        assert!(plan_executor.handle_plan("robot_1", plan));

        let mut status1 = RobotStatus::default();
        status1.pose.pose.position.x = 0.0;
        status1.pose.pose.position.y = 0.0;
        let out = plan_executor.handle_status("robot_1", status1);
        let sz = out.safe_zone.expect("SafeZone should be emitted");
        assert_eq!(sz.incremental_target.regions.len(), 1);
        assert!((sz.incremental_target.regions[0].tolerance - 0.42).abs() < 1e-5);
        assert_eq!(sz.incremental_target.nodes.len(), 1);
        assert_eq!(sz.incremental_target.nodes[0].key.key.as_slice(), &[42]);
        assert!(
            (sz.incremental_target.regions[0].orientations[0].orientation_radians
                - std::f32::consts::FRAC_PI_2)
                .abs()
                < 1e-5
        );
        assert!(
            (sz.incremental_target.regions[0].orientations[0].tolerance_radians - 0.12).abs()
                < 1e-5
        );
    }

    #[test]
    fn test_intermediate_target_nodes_and_orientations_are_gated_and_forwarded() {
        use ros_env::rmf_prototype_msgs::msg::{
            GraphElementKey, TargetNode, TargetOrientation, TargetRegion,
        };

        let mut plan_executor = PlanExecutor::new();
        plan_executor.handle_robot_added("robot_1", 0.5);

        let mut plan = plan_with_positions(&[[0.0, 0.0], [2.0, 0.0], [4.0, 0.0], [6.0, 0.0]]);

        // Intermediate wp[1] has a TargetNode with an orientation constraint (-pi/2)
        let mut staging_key = GraphElementKey::default();
        staging_key.key = vec![13i64].try_into().unwrap();
        staging_key.name = vec!["staging_node".to_string().into()].try_into().unwrap();
        plan.waypoints[1]
            .arrival_constraints
            .nodes
            .push(TargetNode {
                key: staging_key,
                orientations: vec![TargetOrientation {
                    orientation_radians: -std::f32::consts::FRAC_PI_2,
                    spread_radians: 0.0,
                    tolerance_radians: 0.05,
                }],
            });

        // Intermediate wp[2] has a region orientation constraint (pi) without a TargetNode
        let mut reg = TargetRegion::default();
        reg.tolerance = 0.35;
        reg.orientations.push(TargetOrientation {
            orientation_radians: std::f32::consts::PI,
            spread_radians: 0.0,
            tolerance_radians: 0.1,
        });
        plan.waypoints[2].arrival_constraints.regions = vec![reg];

        assert!(plan_executor.handle_plan("robot_1", plan));

        // 1. At start (0, 0), execution gates at wp[1] and forwards its node + orientation (-pi/2)
        let mut status1 = RobotStatus::default();
        status1.pose.pose.position.x = 0.0;
        status1.pose.pose.position.y = 0.0;
        let out1 = plan_executor.handle_status("robot_1", status1);
        assert_eq!(out1.plan_release.unwrap().waypoint_id, 1);
        let sz1 = out1.safe_zone.expect("SafeZone should be emitted at start");
        assert_eq!(sz1.target_waypoint.as_slice(), &[1]);
        assert!(
            (sz1.incremental_target.regions[0].orientations[0].orientation_radians
                - (-std::f32::consts::FRAC_PI_2))
                .abs()
                < 1e-5
        );
        assert!(
            (sz1.incremental_target.regions[0].orientations[0].tolerance_radians - 0.05).abs()
                < 1e-5
        );
        assert_eq!(sz1.incremental_target.nodes.len(), 1);
        assert_eq!(sz1.incremental_target.nodes[0].key.key.as_slice(), &[13]);

        // 2. At wp[1] (2, 0), execution gates at wp[2] and forwards its region orientation (pi) and tolerance (0.35)
        let mut status2 = RobotStatus::default();
        status2.pose.pose.position.x = 2.0;
        status2.pose.pose.position.y = 0.0;
        let out2 = plan_executor.handle_status("robot_1", status2);
        assert_eq!(out2.plan_release.unwrap().waypoint_id, 2);
        let sz2 = out2.safe_zone.expect("SafeZone should be emitted at wp[1]");
        assert_eq!(sz2.target_waypoint.as_slice(), &[2]);
        assert!((sz2.incremental_target.regions[0].tolerance - 0.35).abs() < 1e-5);
        assert!(
            (sz2.incremental_target.regions[0].orientations[0].orientation_radians
                - std::f32::consts::PI)
                .abs()
                < 1e-5
        );
        assert!(
            (sz2.incremental_target.regions[0].orientations[0].tolerance_radians - 0.1).abs()
                < 1e-5
        );

        // 3. At wp[2] (4, 0), execution releases to final wp[3]
        let mut status3 = RobotStatus::default();
        status3.pose.pose.position.x = 4.0;
        status3.pose.pose.position.y = 0.0;
        let out3 = plan_executor.handle_status("robot_1", status3);
        assert_eq!(out3.plan_release.unwrap().waypoint_id, 3);
    }
}
