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

use mapf_post::{
    na::{Isometry2, Vector2},
    spatial_allocation::{CurrentPosition, Grid2D},
    MapfResult, WaypointFollower,
};
use rclrs::{IntoPrimitiveOptions, Node};
use ros_env::builtin_interfaces;
use ros_env::geometry_msgs::msg::Pose;
use ros_env::nav2_msgs;
use ros_env::nav2_msgs::msg::Costmap;
use ros_env::nav_msgs::msg::{OccupancyGrid, Odometry};
use ros_env::rmf_prototype_msgs;
use ros_env::rmf_prototype_msgs::msg::{
    DestinationConstraints, Plan, PlanError, PlanId, PlanRelease, SafeZone, SafeZoneId,
    TargetOrientation,
};
use ros_env::std_msgs;
use std::{
    collections::{BTreeMap, HashMap},
    sync::Arc,
    time::{Duration, Instant},
};

const BLOCKAGE_DEBOUNCE: Duration = Duration::from_millis(300);
const REPLAN_COOLDOWN: Duration = Duration::from_secs(2);
const OCCUPIED_THRESHOLD: i8 = 50;

pub struct RobotState {
    pub radius: f32,
    pub latest_odom: Option<Odometry>,
    pub plan: Option<Plan>,
    pub waypoint_follower: Option<WaypointFollower>,
    pub safe_zone_version: u64,
    pub last_incremental_target_wp: Option<usize>,
    blockage_monitor: BlockageMonitor,
}

#[derive(Default)]
struct BlockageMonitor {
    blocked_since: Option<Instant>,
    reported_plan: Option<PlanId>,
    last_reported_at: Option<Instant>,
}

impl BlockageMonitor {
    fn begin_plan(&mut self) {
        self.blocked_since = None;
    }

    fn observe(&mut self, blocked: bool, plan_id: &PlanId, now: Instant) -> bool {
        if !blocked {
            self.blocked_since = None;
            return false;
        }

        if self.reported_plan.as_ref() == Some(plan_id) {
            return false;
        }

        let blocked_since = self.blocked_since.get_or_insert(now);
        if now.duration_since(*blocked_since) < BLOCKAGE_DEBOUNCE {
            return false;
        }

        if self
            .last_reported_at
            .is_some_and(|last| now.duration_since(last) < REPLAN_COOLDOWN)
        {
            return false;
        }

        self.reported_plan = Some(plan_id.clone());
        self.last_reported_at = Some(now);
        self.blocked_since = None;
        true
    }
}

pub struct PlanExecutor {
    pub node: Node,
    // TODO(arjoc) The use of BTreeMap here is to make sure robts are ordered by their names
    // this helps with maintaining correspondance between their `mapf_post` ids and the input.
    // We should re-visit this after re-designing the mapf-post API a little.
    pub active_robots: BTreeMap<String, RobotState>,
    pub plan_release_publishers: HashMap<String, rclrs::Publisher<PlanRelease>>,
    pub safezone_publishers: HashMap<String, rclrs::Publisher<SafeZone>>,
    pub plan_error_publishers: HashMap<String, rclrs::Publisher<PlanError>>,
    pub grid: Arc<Grid2D>,
    pub grid_width: u32,
    pub grid_height: u32,
    pub grid_resolution: f32,
    pub grid_origin: Pose,
    pub latest_map: Option<OccupancyGrid>,
}

fn target_yaw(plan: &Plan, target_idx: usize) -> f32 {
    let Some(target) = plan.waypoints.get(target_idx) else {
        return 0.0;
    };
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

impl PlanExecutor {
    pub fn new(node: Node) -> Self {
        let mut origin = Pose::default();
        origin.orientation.w = 1.0;
        Self {
            node,
            active_robots: BTreeMap::new(),
            plan_release_publishers: HashMap::new(),
            safezone_publishers: HashMap::new(),
            plan_error_publishers: HashMap::new(),
            grid: Arc::new(Grid2D::new(vec![vec![0; 20]; 20], 1.0)),
            grid_width: 20,
            grid_height: 20,
            grid_resolution: 1.0,
            grid_origin: origin,
            latest_map: None,
        }
    }

    pub fn handle_robot_added(&mut self, robot_id: &str, radius: f32) {
        if !self.active_robots.contains_key(robot_id) {
            rclrs::log!(
                self.node.logger(),
                "PlanExecutor adding participant: {} with radius {}",
                robot_id,
                radius
            );
            self.active_robots.insert(
                robot_id.to_string(),
                RobotState {
                    radius,
                    latest_odom: None,
                    plan: None,
                    waypoint_follower: None,
                    safe_zone_version: 0,
                    last_incremental_target_wp: None,
                    blockage_monitor: BlockageMonitor::default(),
                },
            );
            self.reindex_followers();
        }
    }

    pub fn handle_robot_removed(&mut self, robot_id: &str) {
        if self.active_robots.remove(robot_id).is_some() {
            rclrs::log!(
                self.node.logger(),
                "PlanExecutor removing participant: {}",
                robot_id
            );
            self.plan_release_publishers.remove(robot_id);
            self.safezone_publishers.remove(robot_id);
            self.plan_error_publishers.remove(robot_id);
            self.reindex_followers();
        }
    }

    fn get_agent_index(&self, robot_id: &str) -> Option<usize> {
        self.active_robots.keys().position(|k| k == robot_id) // TODO(arjoc): Peak tier AI-SLOP code.
    }

    fn reindex_followers(&mut self) {
        let sorted_names: Vec<String> = self.active_robots.keys().cloned().collect();
        for (agent_idx, name) in sorted_names.iter().enumerate() {
            let robot_state = self.active_robots.get_mut(name).unwrap();
            if let Some(plan) = &robot_state.plan {
                let traj_poses: Vec<Isometry2<f32>> = plan
                    .waypoints
                    .iter()
                    .map(|wp| Isometry2::new(Vector2::new(wp.position[0], wp.position[1]), 0.0))
                    .collect();

                let mut follower = WaypointFollower::from_trajectory(
                    agent_idx,
                    mapf_post::Trajectory { poses: traj_poses },
                );

                if let Some(odom) = &robot_state.latest_odom {
                    let current_x = odom.pose.pose.position.x as f32;
                    let current_y = odom.pose.pose.position.y as f32;
                    let position = Isometry2::new(Vector2::new(current_x, current_y), 0.0);
                    follower.update_position_estimate(&position, 0.5);
                }

                robot_state.waypoint_follower = Some(follower);
            }
        }
    }

    pub fn handle_plan(&mut self, robot_id: &str, msg: Plan) {
        rclrs::log!(
            self.node.logger(),
            "Received plan version {} with {} waypoints for robot {}",
            msg.plan_id.plan_version,
            msg.waypoints.len(),
            robot_id
        );

        let Some(state) = self.active_robots.get_mut(robot_id) else {
            return;
        };

        if msg.waypoints.len() == 0 {
            // TODO(arjoc): publish an error message
            rclrs::log_error!(self.node.logger(), "Received empty plan. Ignoring.");
            return;
        }

        state.plan = Some(msg);
        state.safe_zone_version = 0;
        state.last_incremental_target_wp = None;
        state.blockage_monitor.begin_plan();

        // Reindex because we updated the plan
        self.reindex_followers();
    }

    pub fn handle_map(&mut self, msg: OccupancyGrid) {
        self.latest_map = Some(msg);
        let robot_ids: Vec<_> = self.active_robots.keys().cloned().collect();
        for robot_id in robot_ids {
            self.update_route_blockage(&robot_id);
        }
    }

    pub fn handle_odometry(&mut self, robot_id: &str, msg: Odometry) {
        let current_x = msg.pose.pose.position.x as f32;
        let current_y = msg.pose.pose.position.y as f32;

        {
            let Some(state) = self.active_robots.get_mut(robot_id) else {
                return;
            };

            state.latest_odom = Some(msg.clone());

            if let Some(fw) = &mut state.waypoint_follower {
                let before = fw.get_semantic_waypoint().trajectory_index;
                let position = Isometry2::new(Vector2::new(current_x, current_y), 0.0);
                fw.update_position_estimate(&position, 0.5);
                let after = fw.get_semantic_waypoint().trajectory_index;
                rclrs::log_debug!(
                    self.node.logger(),
                    "[Executor Debug] Robot {} pos=({}, {}) index before={}, after={}",
                    robot_id,
                    current_x,
                    current_y,
                    before,
                    after
                );
            }
        }

        self.update_route_blockage(robot_id);

        if !self.ready_to_execute() {
            return;
        }

        // Cache all semantic waypoints first because get_semantic_waypoint requires &mut self.
        // Doing this first avoids borrowing active_robots as mutable during later immutable reads.
        let mut semantic_waypoints = HashMap::new();
        for (name, r_state) in &mut self.active_robots {
            if let Some(fw) = &mut r_state.waypoint_follower {
                semantic_waypoints.insert(name.clone(), fw.get_semantic_waypoint());
            }
        }

        // 1. Calculate PlanRelease
        let agent_idx = self.get_agent_index(robot_id).unwrap();
        let robot_state = &self.active_robots[robot_id];
        let plan = robot_state.plan.as_ref().unwrap();
        let curr_wp_idx = semantic_waypoints
            .get(robot_id)
            .map(|w| w.trajectory_index)
            .unwrap_or(0);

        rclrs::log_debug!(
            self.node.logger(),
            "[Executor Debug] Robot {} curr_wp_idx={} (semantic waypoint trajectory index)",
            robot_id,
            curr_wp_idx
        );

        let mut released_wp_idx = curr_wp_idx;
        while released_wp_idx < plan.waypoints.len() {
            let wp = &plan.waypoints[released_wp_idx];
            let mut blocked = false;
            for blocker in &wp.departure_blockers {
                if let Some(blocker_sem) = semantic_waypoints.get(&blocker.name) {
                    let blocker_progress = blocker_sem.trajectory_index;
                    let required_progress = blocker.required_progress.round() as usize;
                    rclrs::log_debug!(
                        self.node.logger(),
                        "[Executor Debug] Blocker check for robot {} wp {}: blocker={} blocker_progress={} required={}",
                        robot_id,
                        released_wp_idx,
                        blocker.name,
                        blocker_progress,
                        required_progress
                    );
                    if blocker_progress < required_progress {
                        blocked = true;
                        break;
                    }
                } else {
                    rclrs::log_debug!(
                        self.node.logger(),
                        "[Executor Debug] Blocker check for robot {} wp {}: blocker={} is missing semantic waypoint",
                        robot_id,
                        released_wp_idx,
                        blocker.name
                    );
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
                if let Some(odom) = &r_state.latest_odom {
                    let dx = wp_pos[0] - odom.pose.pose.position.x as f32;
                    let dy = wp_pos[1] - odom.pose.pose.position.y as f32;
                    if dx.hypot(dy) < 0.8 {
                        released_wp_idx = released_wp_idx.saturating_sub(1);
                        break;
                    }
                }
            }
        }

        if released_wp_idx >= plan.waypoints.len() {
            // Plan validation takes place in the ros2 callback so it should
            // be safe to subtract the waypoint length.
            released_wp_idx = plan.waypoints.len() - 1;
        }

        let pr = PlanRelease {
            waypoint_id: released_wp_idx as u64,
            plan_id: plan.plan_id.clone(),
        };

        // Publish PlanRelease
        if let Some(plan_release_pub) = self.plan_release_publishers.get_mut(robot_id) {
            let _ = plan_release_pub.publish(pr);
        } else {
            let publisher = self
                .node
                .create_publisher(
                    format!("{}/plan/release", robot_id)
                        .as_str()
                        .transient_local()
                        .reliable(),
                )
                .unwrap();
            let _ = publisher.publish(pr);
            self.plan_release_publishers
                .insert(robot_id.to_string(), publisher);
        }

        // --- Dynamic grid resizing ---
        let mut min_x = f32::MAX;
        let mut min_y = f32::MAX;
        let mut max_x = f32::MIN;
        let mut max_y = f32::MIN;

        for r_state in self.active_robots.values() {
            let r_plan = r_state.plan.as_ref().unwrap();
            let r_odom = r_state.latest_odom.as_ref().unwrap();

            min_x = min_x.min(r_odom.pose.pose.position.x as f32);
            min_y = min_y.min(r_odom.pose.pose.position.y as f32);
            max_x = max_x.max(r_odom.pose.pose.position.x as f32);
            max_y = max_y.max(r_odom.pose.pose.position.y as f32);

            for wp in &r_plan.waypoints {
                min_x = min_x.min(wp.position[0]);
                min_y = min_y.min(wp.position[1]);
                max_x = max_x.max(wp.position[0]);
                max_y = max_y.max(wp.position[1]);
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

        // 2. Generate and publish SafeZone
        let mut trajectories = Vec::new();
        let mut footprints = Vec::new();
        let mut current_positions = Vec::new();

        for (r_name, r_state) in &self.active_robots {
            let r_plan = r_state.plan.as_ref().unwrap();
            let r_odom = r_state.latest_odom.as_ref().unwrap();

            let traj_poses: Vec<Isometry2<f32>> = r_plan
                .waypoints
                .iter()
                .map(|wp| {
                    Isometry2::new(
                        Vector2::new(
                            wp.position[0] - self.grid_origin.position.x as f32,
                            wp.position[1] - self.grid_origin.position.y as f32,
                        ),
                        0.0,
                    )
                })
                .collect();
            trajectories.push(mapf_post::Trajectory { poses: traj_poses });
            footprints.push(Arc::new(mapf_post::shape::Ball::new(r_state.radius))
                as Arc<dyn mapf_post::shape::Shape>);

            let semantic_position = semantic_waypoints.get(r_name).cloned().unwrap();
            current_positions.push(CurrentPosition {
                semantic_position,
                real_position: (
                    r_odom.pose.pose.position.x as f32 - self.grid_origin.position.x as f32,
                    r_odom.pose.pose.position.y as f32 - self.grid_origin.position.y as f32,
                ),
            });
        }

        let mapf_result = MapfResult {
            trajectories,
            footprints,
            discretization_timestep: 1.0,
        };

        let allocation_field = self
            .grid
            .allocate_trajectory(&mapf_result, &current_positions);
        let positions = allocation_field
            .get_alloc_for_agent(agent_idx)
            .unwrap_or_default();

        let target_x = plan.waypoints[released_wp_idx].position[0];
        let target_y = plan.waypoints[released_wp_idx].position[1];
        let target_yaw = target_yaw(plan, released_wp_idx);

        let costmap = Self::to_costmap_msg(
            &positions,
            self.grid_width,
            self.grid_height,
            self.grid_resolution,
            self.grid_origin.clone(),
            msg.header.stamp.clone(),
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
                    tolerance: 0.2,
                    region: rmf_prototype_msgs::msg::Region {
                        points: vec![target_x, target_y],
                        hint: rmf_prototype_msgs::msg::Region::HINT_POINT,
                    },
                    orientations: vec![TargetOrientation {
                        // Avoid turning in place at hold points.
                        orientation_radians: target_yaw,
                        spread_radians: 0.0,
                        tolerance_radians: 0.0,
                    }],
                }],
                nodes: vec![],
            },
            costmap,
            target_waypoint: vec![released_wp_idx as u64].try_into().unwrap(),
            last_waypoint: released_wp_idx as u64,
            target_progress: 0.0,
            id: SafeZoneId {
                plan_id,
                safe_zone_version: state.safe_zone_version,
            },
        };

        if let Some(safe_zone_pub) = self.safezone_publishers.get_mut(robot_id) {
            let _ = safe_zone_pub.publish(safe_zone);
        } else {
            let publisher = self
                .node
                .create_publisher(
                    format!("{}/plan/safe_zone", robot_id)
                        .as_str()
                        .transient_local()
                        .reliable(),
                )
                .unwrap();
            let _ = publisher.publish(safe_zone);
            self.safezone_publishers
                .insert(robot_id.to_string(), publisher);
        }
    }

    fn update_route_blockage(&mut self, robot_id: &str) {
        let Some(map) = self.latest_map.as_ref() else {
            return;
        };

        let Some(state) = self.active_robots.get_mut(robot_id) else {
            return;
        };
        let (Some(plan), Some(odom), Some(follower)) = (
            state.plan.as_ref(),
            state.latest_odom.as_ref(),
            state.waypoint_follower.as_mut(),
        ) else {
            return;
        };

        let remaining = follower.remaining_trajectory();
        let mut route = Vec::with_capacity(remaining.len() + 1);
        route.push((
            odom.pose.pose.position.x as f32,
            odom.pose.pose.position.y as f32,
        ));
        route.extend(remaining);

        let blocked = route_intersects_map(map, &route, state.radius);
        let plan_id = plan.plan_id.clone();
        if !state
            .blockage_monitor
            .observe(blocked, &plan_id, Instant::now())
        {
            return;
        }

        let publisher = match self.plan_error_publishers.entry(robot_id.to_string()) {
            std::collections::hash_map::Entry::Occupied(entry) => entry.into_mut(),
            std::collections::hash_map::Entry::Vacant(entry) => {
                let topic = format!("{robot_id}/plan/error");
                match self.node.create_publisher(topic.as_str()) {
                    Ok(publisher) => entry.insert(publisher),
                    Err(error) => {
                        rclrs::log_error!(
                            self.node.logger(),
                            "Failed to create plan error publisher for {}: {:?}",
                            robot_id,
                            error
                        );
                        return;
                    }
                }
            }
        };

        let error = PlanError {
            error: rmf_prototype_msgs::msg::Error {
                code: PlanError::CODE_PATH_BLOCKED,
                message: format!("Updated map blocks the remaining route for {robot_id}"),
                parameters: String::new(),
            },
            plan_id,
        };
        if let Err(error) = publisher.publish(error) {
            rclrs::log_error!(
                self.node.logger(),
                "Failed to publish path blockage for {}: {:?}",
                robot_id,
                error
            );
        } else {
            rclrs::log_warn!(
                self.node.logger(),
                "Updated map blocks the remaining route for {}. Requesting a replan.",
                robot_id
            );
        }
    }

    fn ready_to_execute(&self) -> bool {
        if self.active_robots.is_empty() {
            return false;
        }
        for state in self.active_robots.values() {
            if state.plan.is_none()
                || state.latest_odom.is_none()
                || state.waypoint_follower.is_none()
            {
                return false;
            }
        }
        true
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

fn route_intersects_map(map: &OccupancyGrid, route: &[(f32, f32)], radius: f32) -> bool {
    if route.len() < 2 || map.info.resolution <= 0.0 {
        return false;
    }

    let width = map.info.width as isize;
    let height = map.info.height as isize;
    if width == 0 || height == 0 {
        return false;
    }

    let resolution = map.info.resolution;
    let q = &map.info.origin.orientation;
    let yaw = (2.0 * (q.w * q.z + q.x * q.y)).atan2(1.0 - 2.0 * (q.y * q.y + q.z * q.z)) as f32;
    let cos_yaw = yaw.cos();
    let sin_yaw = yaw.sin();
    let origin_x = map.info.origin.position.x as f32;
    let origin_y = map.info.origin.position.y as f32;
    let clearance = radius.max(0.0) + resolution * std::f32::consts::FRAC_1_SQRT_2;
    let clearance_squared = clearance * clearance;

    let to_map = |(x, y): (f32, f32)| {
        let dx = x - origin_x;
        let dy = y - origin_y;
        (cos_yaw * dx + sin_yaw * dy, -sin_yaw * dx + cos_yaw * dy)
    };

    for segment in route.windows(2) {
        let start = to_map(segment[0]);
        let end = to_map(segment[1]);
        let min_x = (((start.0.min(end.0) - clearance) / resolution).floor() as isize).max(0);
        let max_x =
            (((start.0.max(end.0) + clearance) / resolution).floor() as isize).min(width - 1);
        let min_y = (((start.1.min(end.1) - clearance) / resolution).floor() as isize).max(0);
        let max_y =
            (((start.1.max(end.1) + clearance) / resolution).floor() as isize).min(height - 1);

        for y in min_y..=max_y {
            for x in min_x..=max_x {
                let index = y as usize * width as usize + x as usize;
                if map.data.get(index).copied().unwrap_or(-1) <= OCCUPIED_THRESHOLD {
                    continue;
                }

                let center = ((x as f32 + 0.5) * resolution, (y as f32 + 0.5) * resolution);
                if distance_squared_to_segment(center, start, end) <= clearance_squared {
                    return true;
                }
            }
        }
    }

    false
}

fn distance_squared_to_segment(point: (f32, f32), start: (f32, f32), end: (f32, f32)) -> f32 {
    let segment = (end.0 - start.0, end.1 - start.1);
    let length_squared = segment.0 * segment.0 + segment.1 * segment.1;
    if length_squared <= f32::EPSILON {
        return (point.0 - start.0).powi(2) + (point.1 - start.1).powi(2);
    }

    let offset = (point.0 - start.0, point.1 - start.1);
    let t = ((offset.0 * segment.0 + offset.1 * segment.1) / length_squared).clamp(0.0, 1.0);
    let closest = (start.0 + t * segment.0, start.1 + t * segment.1);
    (point.0 - closest.0).powi(2) + (point.1 - closest.1).powi(2)
}

#[cfg(test)]
mod tests {
    use super::{
        route_intersects_map, target_yaw, BlockageMonitor, BLOCKAGE_DEBOUNCE, REPLAN_COOLDOWN,
    };
    use mapf_post::na::{Isometry2, Vector2};
    use mapf_post::{Trajectory, WaypointFollower};
    use ros_env::{
        nav_msgs::msg::OccupancyGrid,
        rmf_prototype_msgs::msg::{Plan, PlanId, Waypoint},
    };
    use std::time::Instant;

    fn plan_with_positions(positions: &[[f32; 2]]) -> Plan {
        Plan {
            waypoints: positions
                .iter()
                .map(|position| Waypoint {
                    position: *position,
                    ..Default::default()
                })
                .collect(),
            ..Default::default()
        }
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
    fn occupied_cell_on_remaining_route_is_blocked() {
        let mut map = OccupancyGrid::default();
        map.info.resolution = 1.0;
        map.info.width = 10;
        map.info.height = 10;
        map.info.origin.orientation.w = 1.0;
        map.data = vec![0; 100];
        map.data[5 * 10 + 5] = 100;

        assert!(route_intersects_map(&map, &[(1.5, 5.5), (8.5, 5.5)], 0.25));
        assert!(!route_intersects_map(&map, &[(1.5, 8.5), (8.5, 8.5)], 0.25));
    }

    #[test]
    fn blockage_must_persist_before_reporting() {
        let mut monitor = BlockageMonitor::default();
        let plan_id = PlanId::default();
        let start = Instant::now();

        assert!(!monitor.observe(true, &plan_id, start));
        assert!(!monitor.observe(true, &plan_id, start + BLOCKAGE_DEBOUNCE / 2));
        assert!(monitor.observe(true, &plan_id, start + BLOCKAGE_DEBOUNCE));
        let later = start + BLOCKAGE_DEBOUNCE + REPLAN_COOLDOWN;
        assert!(!monitor.observe(true, &plan_id, later));
    }

    #[test]
    fn clear_route_resets_the_debounce_window() {
        let mut monitor = BlockageMonitor::default();
        let plan_id = PlanId::default();
        let start = Instant::now();

        assert!(!monitor.observe(true, &plan_id, start));
        assert!(!monitor.observe(false, &plan_id, start + BLOCKAGE_DEBOUNCE));
        assert!(!monitor.observe(true, &plan_id, start + BLOCKAGE_DEBOUNCE));
        let before_debounce = start + BLOCKAGE_DEBOUNCE * 3 / 2;
        assert!(!monitor.observe(true, &plan_id, before_debounce));
        assert!(monitor.observe(true, &plan_id, start + BLOCKAGE_DEBOUNCE * 2));
    }

    #[test]
    fn cooldown_delays_a_blockage_on_the_next_plan() {
        let mut monitor = BlockageMonitor::default();
        let first_plan = PlanId::default();
        let second_plan = PlanId {
            plan_version: 1,
            ..Default::default()
        };
        let start = Instant::now();

        assert!(!monitor.observe(true, &first_plan, start));
        assert!(monitor.observe(true, &first_plan, start + BLOCKAGE_DEBOUNCE));
        monitor.begin_plan();
        let during_cooldown = start + BLOCKAGE_DEBOUNCE * 2;
        assert!(!monitor.observe(true, &second_plan, during_cooldown));
        let after_debounce = start + BLOCKAGE_DEBOUNCE * 3;
        assert!(!monitor.observe(true, &second_plan, after_debounce));
        assert!(monitor.observe(
            true,
            &second_plan,
            start + BLOCKAGE_DEBOUNCE + REPLAN_COOLDOWN,
        ));
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
        println!(
            "After reaching 10.0: index is {}",
            follower.get_semantic_waypoint().trajectory_index
        );

        follower.update_position_estimate(&Isometry2::new(Vector2::new(10.0, 0.0), 0.0), 0.5);
        println!(
            "After waiting at 10.0 (1st time): index is {}",
            follower.get_semantic_waypoint().trajectory_index
        );

        follower.update_position_estimate(&Isometry2::new(Vector2::new(10.0, 0.0), 0.0), 0.5);
        println!(
            "After waiting at 10.0 (2nd time): index is {}",
            follower.get_semantic_waypoint().trajectory_index
        );
    }
}
