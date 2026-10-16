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
    shape::{Ball, Shape},
    MapfResult, Trajectory,
};
use ros_env::rmf_prototype_msgs::msg::{Plan, PlanId};
use std::{
    collections::{BTreeMap, HashMap, HashSet},
    sync::Arc,
};

pub(crate) struct ExecutionSnapshot {
    pub(crate) mapf_result: MapfResult,
    /// Ordered list of robot IDs corresponding to `mapf_result.trajectories`.
    pub(crate) included_robots: Vec<String>,
    /// Maps `(robot_id, local_waypoint_index)` to the global trajectory index
    /// in `mapf_result` reconstructed by DFS over `departure_blockers`.
    waypoint_arrival_times: HashMap<(String, usize), usize>,
}

impl ExecutionSnapshot {
    pub(crate) fn get_global_waypoint_index(
        &self,
        robot: &str,
        local_waypoint_index: usize,
    ) -> Option<usize> {
        self.waypoint_arrival_times
            .get(&(robot.to_string(), local_waypoint_index))
            .copied()
    }

    pub(crate) fn get_agent_index(&self, robot: &str) -> Option<usize> {
        self.included_robots.iter().position(|r| r == robot)
    }
}

/// Tracks active robot plans and reconstructs a unified [`ExecutionSnapshot`]
/// across plans by performing a depth-first search (DFS) over the
/// [`TrafficDependency`] edges stored in each [`Plan`]'s `departure_blockers`.
#[derive(Default, Clone)]
pub(crate) struct PlanRegistry {
    /// Active plans keyed by robot ID (`BTreeMap` ensures deterministic agent ordering).
    active_plans: BTreeMap<String, Plan>,
    /// Mapping between robot and its current waypoint index along its active plan.
    robot_to_progress: HashMap<String, usize>,
    /// Robots with no active plan and their stationary position (if known).
    inactive_robots: BTreeMap<String, Option<Isometry2<f32>>>,
    /// Footprint radius for each registered robot.
    robot_radii: HashMap<String, f32>,
}

impl PlanRegistry {
    pub(crate) fn add_robot(&mut self, robot: String, radius: f32) {
        self.robot_radii.insert(robot.clone(), radius);
        self.inactive_robots.insert(robot, None);
    }

    pub(crate) fn remove_robot(&mut self, robot: &str) {
        self.active_plans.remove(robot);
        self.robot_to_progress.remove(robot);
        self.inactive_robots.remove(robot);
        self.robot_radii.remove(robot);
    }

    pub(crate) fn update_stationary_position(&mut self, robot: &str, position: Isometry2<f32>) {
        if let Some(pos) = self.inactive_robots.get_mut(robot) {
            *pos = Some(position);
        }
    }

    pub(crate) fn add_robot_plan(&mut self, robot: String, plan: Plan) {
        self.inactive_robots.remove(&robot);
        self.robot_to_progress.insert(robot.clone(), 0);
        self.active_plans.insert(robot, plan);
    }

    pub(crate) fn update_progress(&mut self, robot: &str, progress: usize, plan_id: PlanId) {
        let Some(plan) = self.active_plans.get(robot) else {
            return;
        };
        if plan.plan_id != plan_id {
            return;
        }
        self.robot_to_progress.insert(robot.to_string(), progress);

        if progress >= plan.waypoints.len() {
            let last_pose = plan
                .waypoints
                .last()
                .map(|wp| Isometry2::new(Vector2::new(wp.position[0], wp.position[1]), 0.0));
            self.active_plans.remove(robot);
            self.inactive_robots.insert(robot.to_string(), last_pose);
        }
    }

    fn required_waypoint_index(plan: &Plan, required_progress: f32) -> usize {
        if plan.waypoints.is_empty() {
            return 0;
        }
        if plan.waypoints.last().is_some_and(|w| w.progress > 0.0) {
            plan.waypoints
                .iter()
                .position(|w| w.progress + 1e-4 >= required_progress)
                .unwrap_or(plan.waypoints.len() - 1)
        } else {
            (required_progress.round().max(0.0) as usize).min(plan.waypoints.len() - 1)
        }
    }

    /// Recursively resolves `(arrival_time, departure_time)` for `(robot, wp_idx)`
    /// via DFS over sequential plan edges and cross-plan `departure_blockers`.
    fn resolve_waypoint(
        &self,
        robot: &str,
        wp_idx: usize,
        memo: &mut HashMap<(String, usize), (usize, usize)>,
        visiting: &mut HashSet<(String, usize)>,
    ) -> (usize, usize) {
        let key = (robot.to_string(), wp_idx);
        if let Some(&times) = memo.get(&key) {
            return times;
        }
        if !visiting.insert(key.clone()) {
            // Cycle guard: break any malformed cyclic dependency gracefully.
            return (0, 0);
        }

        let arrival = if wp_idx == 0 {
            0
        } else {
            let (_prev_arr, prev_dep) = self.resolve_waypoint(robot, wp_idx - 1, memo, visiting);
            prev_dep + 1
        };

        let mut departure = arrival;
        if let Some(plan) = self.active_plans.get(robot) {
            if let Some(wp) = plan.waypoints.get(wp_idx) {
                for blocker in &wp.departure_blockers {
                    if let Some(blocker_plan) = self.active_plans.get(&blocker.name) {
                        if blocker_plan.plan_id == blocker.plan_id
                            && !blocker_plan.waypoints.is_empty()
                        {
                            let dep_wp_idx = Self::required_waypoint_index(
                                blocker_plan,
                                blocker.required_progress,
                            );
                            let (blocker_arrival, _blocker_departure) =
                                self.resolve_waypoint(&blocker.name, dep_wp_idx, memo, visiting);
                            departure = departure.max(blocker_arrival);
                        }
                    }
                }
            }
        }

        visiting.remove(&key);
        memo.insert(key, (arrival, departure));
        (arrival, departure)
    }

    pub(crate) fn get_mapf_snapshot(&self) -> ExecutionSnapshot {
        let mut memo: HashMap<(String, usize), (usize, usize)> = HashMap::new();
        let mut visiting: HashSet<(String, usize)> = HashSet::new();

        for (robot, plan) in &self.active_plans {
            for wp_idx in 0..plan.waypoints.len() {
                self.resolve_waypoint(robot, wp_idx, &mut memo, &mut visiting);
            }
        }

        let mut trajectories_by_robot: BTreeMap<String, Vec<Isometry2<f32>>> = BTreeMap::new();
        let mut waypoint_arrival_times: HashMap<(String, usize), usize> = HashMap::new();

        for (robot, plan) in &self.active_plans {
            if plan.waypoints.is_empty() {
                continue;
            }
            let mut poses = Vec::new();
            for (wp_idx, wp) in plan.waypoints.iter().enumerate() {
                let (arr, dep) = memo
                    .get(&(robot.clone(), wp_idx))
                    .copied()
                    .unwrap_or((poses.len(), poses.len()));
                waypoint_arrival_times.insert((robot.clone(), wp_idx), arr);
                let pose = Isometry2::new(Vector2::new(wp.position[0], wp.position[1]), 0.0);
                let hold_steps = dep.saturating_sub(arr) + 1;
                poses.extend(std::iter::repeat_n(pose, hold_steps));
            }
            trajectories_by_robot.insert(robot.clone(), poses);
        }

        let max_active_len = trajectories_by_robot
            .values()
            .map(|poses| poses.len())
            .max()
            .unwrap_or(0);
        let total_len = max_active_len.max(1);

        for poses in trajectories_by_robot.values_mut() {
            if let Some(&last_pose) = poses.last() {
                poses.resize(total_len, last_pose);
            }
        }

        for (robot, maybe_pos) in &self.inactive_robots {
            if let Some(pos) = *maybe_pos {
                trajectories_by_robot.insert(robot.clone(), vec![pos; total_len]);
            }
        }

        let mut trajectories = Vec::with_capacity(trajectories_by_robot.len());
        let mut footprints = Vec::with_capacity(trajectories_by_robot.len());
        let mut included_robots = Vec::with_capacity(trajectories_by_robot.len());

        for (robot, poses) in trajectories_by_robot {
            let radius = self.robot_radii.get(&robot).copied().unwrap_or(0.5);
            included_robots.push(robot);
            trajectories.push(Trajectory { poses });
            footprints.push(Arc::new(Ball::new(radius)) as Arc<dyn Shape>);
        }

        ExecutionSnapshot {
            mapf_result: MapfResult {
                trajectories,
                footprints,
                discretization_timestep: 1.0,
            },
            included_robots,
            waypoint_arrival_times,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use ros_env::rmf_prototype_msgs::msg::{TrafficDependency, Waypoint};

    fn make_plan(uuid_byte: u8, plan_version: u64, coords: &[[f32; 2]]) -> Plan {
        let mut plan_id = PlanId {
            plan_version,
            ..Default::default()
        };
        plan_id.destination_session.uuid = [uuid_byte; 16];
        Plan {
            plan_id,
            waypoints: coords
                .iter()
                .enumerate()
                .map(|(i, &position)| Waypoint {
                    position,
                    progress: i as f32,
                    ..Default::default()
                })
                .collect(),
            ..Default::default()
        }
    }

    fn xy_coords(traj: &Trajectory) -> Vec<[f32; 2]> {
        traj.poses
            .iter()
            .map(|p| [p.translation.x, p.translation.y])
            .collect()
    }

    #[test]
    fn dfs_reconstructs_cross_plan_timeline_from_traffic_dependencies_and_stationary_holds() {
        let mut registry = PlanRegistry::default();
        registry.add_robot("robot_1".to_string(), 0.5);
        registry.add_robot("robot_2".to_string(), 0.5);
        registry.add_robot("robot_3".to_string(), 0.5);

        registry.update_stationary_position("robot_3", Isometry2::new(Vector2::new(9.0, 9.0), 0.0));

        // Plan 1: robot_1 moves across 3 waypoints (0,0) -> (1,0) -> (2,0)
        let plan_1 = make_plan(1, 1, &[[0.0, 0.0], [1.0, 0.0], [2.0, 0.0]]);
        let plan_1_id = plan_1.plan_id.clone();
        registry.add_robot_plan("robot_1".to_string(), plan_1);

        // Plan 2: robot_2 moves across 2 waypoints (5,0) -> (5,1), blocked at wp 0
        // until robot_1 reaches progress 2.0 (waypoint 2) on plan_1.
        let mut plan_2 = make_plan(2, 1, &[[5.0, 0.0], [5.0, 1.0]]);
        plan_2.waypoints[0].departure_blockers = vec![TrafficDependency {
            name: "robot_1".to_string(),
            plan_id: plan_1_id,
            required_progress: 2.0,
        }];
        registry.add_robot_plan("robot_2".to_string(), plan_2);

        let snapshot = registry.get_mapf_snapshot();
        assert_eq!(snapshot.mapf_result.trajectories.len(), 3);
        assert_eq!(snapshot.get_global_waypoint_index("robot_1", 0), Some(0));
        assert_eq!(snapshot.get_global_waypoint_index("robot_1", 2), Some(2));
        assert_eq!(snapshot.get_global_waypoint_index("robot_2", 0), Some(0));
        assert_eq!(snapshot.get_global_waypoint_index("robot_2", 1), Some(3));

        // robot_1 moves through its 3 waypoints (steps 0, 1, 2), then holds at step 3
        assert_eq!(
            xy_coords(&snapshot.mapf_result.trajectories[0]),
            vec![[0.0, 0.0], [1.0, 0.0], [2.0, 0.0], [2.0, 0.0]]
        );

        // robot_2 holds its start position until robot_1 reaches wp 2 at step 2, then moves at step 3
        assert_eq!(
            xy_coords(&snapshot.mapf_result.trajectories[1]),
            vec![[5.0, 0.0], [5.0, 0.0], [5.0, 0.0], [5.0, 1.0]]
        );

        // robot_3 is stationary throughout all 4 steps
        assert_eq!(
            xy_coords(&snapshot.mapf_result.trajectories[2]),
            vec![[9.0, 9.0], [9.0, 9.0], [9.0, 9.0], [9.0, 9.0]]
        );
    }

    #[test]
    fn completed_plan_transitions_robot_to_stationary_at_final_pose() {
        let mut registry = PlanRegistry::default();
        registry.add_robot("robot_1".to_string(), 0.5);
        registry.add_robot("robot_2".to_string(), 0.5);

        let plan_1 = make_plan(1, 1, &[[0.0, 0.0], [1.0, 0.0], [2.0, 0.0]]);
        let plan_1_id = plan_1.plan_id.clone();
        registry.add_robot_plan("robot_1".to_string(), plan_1);

        let mut plan_2 = make_plan(2, 1, &[[5.0, 0.0], [5.0, 1.0]]);
        plan_2.waypoints[0].departure_blockers = vec![TrafficDependency {
            name: "robot_1".to_string(),
            plan_id: plan_1_id.clone(),
            required_progress: 2.0,
        }];
        registry.add_robot_plan("robot_2".to_string(), plan_2);

        // Mark robot_1's plan as completed
        registry.update_progress("robot_1", 3, plan_1_id);

        let snapshot = registry.get_mapf_snapshot();
        assert_eq!(snapshot.mapf_result.trajectories.len(), 2);
        assert_eq!(snapshot.get_global_waypoint_index("robot_1", 0), None);
        assert_eq!(snapshot.get_global_waypoint_index("robot_2", 0), Some(0));
        assert_eq!(snapshot.get_global_waypoint_index("robot_2", 1), Some(1));

        // Plan 1 is retired; robot_1 is stationary at [2.0, 0.0] while robot_2 runs immediately
        assert_eq!(
            xy_coords(&snapshot.mapf_result.trajectories[0]),
            vec![[2.0, 0.0], [2.0, 0.0]]
        );
        assert_eq!(
            xy_coords(&snapshot.mapf_result.trajectories[1]),
            vec![[5.0, 0.0], [5.0, 1.0]]
        );
    }
}
