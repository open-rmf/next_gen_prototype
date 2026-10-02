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
    collections::{BTreeMap, HashMap},
    hash::{Hash, Hasher},
    sync::Arc,
};

#[derive(Clone, Debug, PartialEq)]
pub(crate) struct PlanIdKey(pub(crate) PlanId);

impl Eq for PlanIdKey {}

impl Hash for PlanIdKey {
    fn hash<H: Hasher>(&self, state: &mut H) {
        self.0.destination_session.uuid.hash(state);
        self.0.mapf_session.hash(state);
        self.0.plan_version.hash(state);
    }
}

impl From<PlanId> for PlanIdKey {
    fn from(plan_id: PlanId) -> Self {
        Self(plan_id)
    }
}

impl From<&PlanId> for PlanIdKey {
    fn from(plan_id: &PlanId) -> Self {
        Self(plan_id.clone())
    }
}

pub(crate) struct ExecutionSnapshot {
    pub(crate) mapf_result: MapfResult,
    pub(crate) plan_offsets: HashMap<PlanIdKey, usize>,
}

impl ExecutionSnapshot {
    pub(crate) fn get_plan_offset(&self, plan_id: &PlanId) -> Option<usize> {
        self.plan_offsets.get(&PlanIdKey(plan_id.clone())).copied()
    }
}

/// Handles incoming plans. It is possible to have multiple active
/// MAPF plans. For instance, when a robot is docking, it will not be
/// responsive to any new plans and be executing its old plan.
/// The same logic could apply to other complex procedures where the
/// robot's responsibility is to perform some maneuver.
///
/// The way this is implmemented is that robots start of in an inactive state.
/// When a new plan comes in it gets assigned to a robot.
///
#[derive(Default, Clone)]
pub(crate) struct PlanRegistry {
    /// Contains all plans belonging to a session
    /// We use a BTreeMap so that we can iterate over plans in order of
    /// their creation.
    session_to_plans: BTreeMap<u64, HashMap<String, Plan>>,
    /// Mapping between robots and their sessions
    robot_to_session: HashMap<String, PlanId>,
    /// Mapping between robot progress along current plan
    robot_to_progress: HashMap<String, usize>,
    /// Robots with no active plan and their stationary position (if known)
    inactive_robots: HashMap<String, Option<Isometry2<f32>>>,
    /// Footprint radius for each registered robot
    robot_radii: HashMap<String, f32>,
}

impl PlanRegistry {
    pub(crate) fn add_robot(&mut self, robot: String, radius: f32) {
        self.robot_radii.insert(robot.clone(), radius);
        self.inactive_robots.insert(robot, None);
    }

    pub(crate) fn update_stationary_position(&mut self, robot: &str, position: Isometry2<f32>) {
        if let Some(pos) = self.inactive_robots.get_mut(robot) {
            *pos = Some(position);
        }
    }

    pub(crate) fn add_robot_plan(&mut self, robot: String, plan: Plan) {
        let session = plan.plan_id.mapf_session;
        let plan_id = plan.plan_id.clone();

        self.inactive_robots.remove(&robot);

        if let Some(old_plan) = self.robot_to_session.insert(robot.clone(), plan_id.clone()) {
            // A new plan has come in
            if old_plan.mapf_session < plan_id.mapf_session {
                self.invalidate_entry(&robot, &old_plan.mapf_session);
            }
        }

        if let Some(p) = self.session_to_plans.get_mut(&session) {
            p.insert(robot.clone(), plan);
        } else {
            let mut hashmap = HashMap::new();
            hashmap.insert(robot, plan);
            self.session_to_plans.insert(session, hashmap);
        }
    }

    fn invalidate_entry(&mut self, robot: &str, mapf_session: &u64) {
        let mut clear_session = false;
        // invalidate old plan.
        if let Some(robot_trajectories) = self.session_to_plans.get_mut(mapf_session) {
            robot_trajectories.remove_entry(robot);
            if robot_trajectories.len() == 0 {
                clear_session = true;
            }
        }

        if clear_session {
            self.session_to_plans.remove_entry(mapf_session);
        }
    }

    pub(crate) fn update_progress(&mut self, robot: &str, progress: usize, plan_id: PlanId) {
        let Some(correct_plan_id) = self.robot_to_session.get(robot) else {
            return;
        };
        if *correct_plan_id != plan_id {
            return;
        }
        self.robot_to_progress.insert(robot.to_string(), progress);

        let Some(p) = self.session_to_plans.get(&plan_id.mapf_session) else {
            panic!("PlanRegistry state has gone out of sync. This should never happen.");
        };

        let Some(plan) = p.get(robot) else {
            panic!("PlanRegistry state has gone out of sync. This should never happen.");
        };

        if plan.waypoints.len() == progress {
            let last_pose = plan
                .waypoints
                .last()
                .map(|wp| Isometry2::new(Vector2::new(wp.position[0], wp.position[1]), 0.0));
            self.invalidate_entry(robot, &plan_id.mapf_session);
            self.inactive_robots.insert(robot.to_string(), last_pose);
        }
    }

    pub(crate) fn get_mapf_snapshot(&self) -> ExecutionSnapshot {
        let mut trajectories_by_robot: BTreeMap<String, Vec<Isometry2<f32>>> = BTreeMap::new();
        let mut plan_offsets: HashMap<PlanIdKey, usize> = HashMap::new();
        let mut elapsed_steps = 0usize;

        for plans in self.session_to_plans.values() {
            let session_len = plans
                .values()
                .map(|plan| plan.waypoints.len())
                .max()
                .unwrap_or(0);
            if session_len == 0 {
                continue;
            }

            // Pad earlier sessions with each robot's last position while this
            // newer session executes.
            for poses in trajectories_by_robot.values_mut() {
                if let Some(&last_pose) = poses.last() {
                    poses.resize(poses.len() + session_len, last_pose);
                }
            }

            // Add trajectories for robots in the current session, holding their
            // starting position while older sessions execute.
            for (robot, plan) in plans {
                let plan_poses: Vec<Isometry2<f32>> = plan
                    .waypoints
                    .iter()
                    .map(|wp| Isometry2::new(Vector2::new(wp.position[0], wp.position[1]), 0.0))
                    .collect();

                let Some(&start_pose) = plan_poses.first() else {
                    continue;
                };

                plan_offsets.insert(PlanIdKey::from(&plan.plan_id), elapsed_steps);

                let mut poses = vec![start_pose; elapsed_steps];
                poses.extend(plan_poses);

                if let Some(&last_pose) = poses.last() {
                    poses.resize(elapsed_steps + session_len, last_pose);
                }

                trajectories_by_robot.insert(robot.clone(), poses);
            }

            elapsed_steps += session_len;
        }

        let total_len = elapsed_steps.max(1);
        for (robot, maybe_pos) in &self.inactive_robots {
            if let Some(pos) = *maybe_pos {
                trajectories_by_robot.insert(robot.clone(), vec![pos; total_len]);
            }
        }

        let mut trajectories = Vec::with_capacity(trajectories_by_robot.len());
        let mut footprints = Vec::with_capacity(trajectories_by_robot.len());

        for (robot, poses) in trajectories_by_robot {
            let radius = self.robot_radii.get(&robot).copied().unwrap_or(0.5);
            trajectories.push(Trajectory { poses });
            footprints.push(Arc::new(Ball::new(radius)) as Arc<dyn Shape>);
        }

        ExecutionSnapshot {
            mapf_result: MapfResult {
                trajectories,
                footprints,
                discretization_timestep: 1.0,
            },
            plan_offsets,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use ros_env::rmf_prototype_msgs::msg::Waypoint;

    fn make_plan(mapf_session: u64, plan_version: u64, coords: &[[f32; 2]]) -> Plan {
        Plan {
            plan_id: PlanId {
                mapf_session,
                plan_version,
                ..Default::default()
            },
            waypoints: coords
                .iter()
                .map(|&position| Waypoint {
                    position,
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
    fn older_session_completes_before_newer_session_and_stationary_holds() {
        let mut registry = PlanRegistry::default();
        registry.add_robot("robot_1".to_string(), 0.5);
        registry.add_robot("robot_2".to_string(), 0.5);
        registry.add_robot("robot_3".to_string(), 0.5);

        registry.update_stationary_position("robot_3", Isometry2::new(Vector2::new(9.0, 9.0), 0.0));

        // Session 1 (older): robot_1 moves across 3 waypoints
        let plan_1 = make_plan(1, 1, &[[0.0, 0.0], [1.0, 0.0], [2.0, 0.0]]);
        let plan_1_id = plan_1.plan_id.clone();
        registry.add_robot_plan("robot_1".to_string(), plan_1);

        // Session 2 (newer): robot_2 moves across 2 waypoints
        let plan_2 = make_plan(2, 1, &[[5.0, 0.0], [5.0, 1.0]]);
        let plan_2_id = plan_2.plan_id.clone();
        registry.add_robot_plan("robot_2".to_string(), plan_2);

        let snapshot = registry.get_mapf_snapshot();
        assert_eq!(snapshot.mapf_result.trajectories.len(), 3);
        assert_eq!(snapshot.get_plan_offset(&plan_1_id), Some(0));
        assert_eq!(snapshot.get_plan_offset(&plan_2_id), Some(3));

        // robot_1 executes session 1 (3 steps), then holds its last position (2 steps)
        assert_eq!(
            xy_coords(&snapshot.mapf_result.trajectories[0]),
            vec![[0.0, 0.0], [1.0, 0.0], [2.0, 0.0], [2.0, 0.0], [2.0, 0.0],]
        );

        // robot_2 holds its start position during session 1 (3 steps), then executes session 2 (2 steps)
        assert_eq!(
            xy_coords(&snapshot.mapf_result.trajectories[1]),
            vec![[5.0, 0.0], [5.0, 0.0], [5.0, 0.0], [5.0, 0.0], [5.0, 1.0],]
        );

        // robot_3 is stationary throughout all 5 steps
        assert_eq!(
            xy_coords(&snapshot.mapf_result.trajectories[2]),
            vec![[9.0, 9.0], [9.0, 9.0], [9.0, 9.0], [9.0, 9.0], [9.0, 9.0],]
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

        let plan_2 = make_plan(2, 1, &[[5.0, 0.0], [5.0, 1.0]]);
        let plan_2_id = plan_2.plan_id.clone();
        registry.add_robot_plan("robot_2".to_string(), plan_2);

        // Mark robot_1's plan in session 1 as completed
        registry.update_progress("robot_1", 3, plan_1_id.clone());

        let snapshot = registry.get_mapf_snapshot();
        assert_eq!(snapshot.mapf_result.trajectories.len(), 2);
        assert_eq!(snapshot.get_plan_offset(&plan_1_id), None);
        assert_eq!(snapshot.get_plan_offset(&plan_2_id), Some(0));

        // Session 1 is now retired; robot_1 is stationary at [2.0, 0.0] while robot_2 runs session 2
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
