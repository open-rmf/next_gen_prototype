// Copyright 2026 Open Source Robotics Foundation
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
    MapfResult, SemanticPlan, SemanticWaypoint, Trajectory,
};
use ros_env::{
    builtin_interfaces,
    rmf_prototype_msgs::msg::{
        Destination, Plan, PlanId, RobotStatus, TrafficDependency, Waypoint,
    },
};
use std::{
    collections::{BTreeMap, HashMap, HashSet},
    sync::Arc,
};

struct CollatedAgent {
    robot_id: String,
    plan_id: PlanId,
    /// For each global step `t`, the local waypoint index `wp_idx` of `plan`
    /// that the robot occupies at step `t`.
    step_to_local_wp: Vec<usize>,
    /// Global step index at which the robot's active trajectory finishes
    /// (any steps after this are stationary end-of-plan padding).
    active_end_step: usize,
    poses: Vec<Isometry2<f32>>,
}

/// Manages active [`Plan`]s, computes [`PlanId`] versions, answers plan-occupancy
/// queries, and collates retained plans via DFS over [`TrafficDependency`] edges
/// so [`mapf_post::mapf_post`] can compute [`TrafficDependency`] blockers across plans.
#[derive(Clone, Debug, Default)]
pub struct PlanRegistry {
    /// Active plans keyed by robot ID.
    active_plans: BTreeMap<String, Plan>,
}

impl PlanRegistry {
    pub fn new() -> Self {
        Self::default()
    }

    /// Returns the currently active [`Plan`] for `robot_id`, if any.
    pub fn get_plan(&self, robot_id: &str) -> Option<&Plan> {
        self.active_plans.get(robot_id)
    }

    /// Returns the currently active [`PlanId`] for `robot_id`, if any.
    pub fn get_plan_id(&self, robot_id: &str) -> Option<&PlanId> {
        self.get_plan(robot_id).map(|plan| &plan.plan_id)
    }

    /// Computes the next [`PlanId`] for `robot_id` given its target `dest`.
    /// Increments `plan_version` if `dest.session.uuid` matches the robot's
    /// active plan, or starts at version `0` for a new destination session.
    pub fn next_plan_id(&self, robot_id: &str, dest: &Destination) -> PlanId {
        let new_version = match self.get_plan_id(robot_id) {
            Some(p_id) if p_id.destination_session.uuid == dest.session.uuid => {
                p_id.plan_version + 1
            }
            _ => 0,
        };

        PlanId {
            destination_session: dest.session.clone(),
            plan_version: new_version,
        }
    }

    /// Records a newly generated `plan` for `robot_id`, replacing any older plan.
    pub fn insert_plan(&mut self, _session_id: u64, robot_id: String, plan: Plan) {
        self.active_plans.insert(robot_id, plan);
    }

    /// Records all plans produced in a planning session.
    pub fn record_session_plans(&mut self, session_id: u64, plans: &HashMap<String, Plan>) {
        for (robot_id, plan) in plans {
            self.insert_plan(session_id, robot_id.clone(), plan.clone());
        }
    }

    /// Removes a robot and its active plan from the registry.
    pub fn remove_robot(&mut self, robot_id: &str) {
        self.active_plans.remove(robot_id);
    }

    /// Returns true if `robot_id` is currently executing an action according to
    /// `status` and its active plan.
    pub fn is_executing_action(&self, robot_id: &str, status: &RobotStatus) -> bool {
        if status.mode == RobotStatus::MODE_ACTION {
            return true;
        }
        if status.mode != RobotStatus::MODE_IDLE {
            if let (Some(progress), Some(plan)) = (status.progress.first(), self.get_plan(robot_id))
            {
                if let Some(wp) = plan.waypoints.get(progress.target_waypoint as usize) {
                    if !wp.arrival_action.is_empty() || !wp.departure_action.is_empty() {
                        return true;
                    }
                }
                if let Some(wp) = plan.waypoints.get(progress.reached_waypoint as usize) {
                    if !wp.departure_action.is_empty() {
                        return true;
                    }
                }
            }
        }
        false
    }

    /// Returns the polyline path (`[reached_waypoint, current_pose, target_waypoint]`)
    /// currently occupied by `robot_id` based on `status` and its active plan.
    pub fn occupied_path_of(&self, robot_id: &str, status: &RobotStatus) -> Vec<[f32; 2]> {
        let current_pos = [
            status.pose.pose.position.x as f32,
            status.pose.pose.position.y as f32,
        ];
        let mut path = Vec::new();
        if let (Some(progress), Some(plan)) = (status.progress.first(), self.get_plan(robot_id)) {
            if let Some(reached_wp) = plan.waypoints.get(progress.reached_waypoint as usize) {
                path.push(reached_wp.position);
            }
            path.push(current_pos);
            if let Some(target_wp) = plan.waypoints.get(progress.target_waypoint as usize) {
                path.push(target_wp.position);
            }
        } else {
            path.push(current_pos);
        }
        path
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

    fn resolve_retained_waypoint(
        robot: &str,
        wp_idx: usize,
        retained_plans: &BTreeMap<String, &Plan>,
        memo: &mut HashMap<(String, usize), (usize, usize)>,
        visiting: &mut HashSet<(String, usize)>,
    ) -> (usize, usize) {
        let key = (robot.to_string(), wp_idx);
        if let Some(&times) = memo.get(&key) {
            return times;
        }
        if !visiting.insert(key.clone()) {
            return (0, 0);
        }

        let arrival = if wp_idx == 0 {
            0
        } else {
            let (_prev_arr, prev_dep) =
                Self::resolve_retained_waypoint(robot, wp_idx - 1, retained_plans, memo, visiting);
            prev_dep + 1
        };

        let mut departure = arrival;
        if let Some(plan) = retained_plans.get(robot) {
            if let Some(wp) = plan.waypoints.get(wp_idx) {
                for blocker in &wp.departure_blockers {
                    if let Some(blocker_plan) = retained_plans.get(&blocker.name) {
                        if blocker_plan.plan_id == blocker.plan_id
                            && !blocker_plan.waypoints.is_empty()
                        {
                            let dep_wp_idx = Self::required_waypoint_index(
                                blocker_plan,
                                blocker.required_progress,
                            );
                            let (blocker_arrival, _blocker_departure) =
                                Self::resolve_retained_waypoint(
                                    &blocker.name,
                                    dep_wp_idx,
                                    retained_plans,
                                    memo,
                                    visiting,
                                );
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

    /// Reconstructs the timeline of retained active plans via DFS over their
    /// `departure_blockers`, appends the newly planned cohort (`basic_plan` for
    /// `robot_ids`), and runs [`mapf_post::mapf_post`] to compute both intra-plan
    /// and cross-plan [`TrafficDependency`] blockers.
    pub fn build_session_plans(
        &self,
        basic_plan: &[Vec<Isometry2<f32>>],
        goals: &HashMap<String, Destination>,
        robot_ids: &[String],
        footprints: &HashMap<String, Arc<dyn Shape>>,
        timestep: f32,
    ) -> (HashMap<String, Plan>, MapfResult, SemanticPlan) {
        let mut collated_agents: Vec<CollatedAgent> = Vec::new();

        // 1. Reconstruct retained active plans using DFS over their departure_blockers.
        let retained_plans: BTreeMap<String, &Plan> = self
            .active_plans
            .iter()
            .filter(|(r_id, plan)| {
                !plan.waypoints.is_empty() && !robot_ids.iter().any(|id| id == *r_id)
            })
            .map(|(r_id, plan)| (r_id.clone(), plan))
            .collect();

        let mut memo: HashMap<(String, usize), (usize, usize)> = HashMap::new();
        let mut visiting: HashSet<(String, usize)> = HashSet::new();
        for (robot_id, plan) in &retained_plans {
            for wp_idx in 0..plan.waypoints.len() {
                Self::resolve_retained_waypoint(
                    robot_id,
                    wp_idx,
                    &retained_plans,
                    &mut memo,
                    &mut visiting,
                );
            }
        }

        for (robot_id, plan) in &retained_plans {
            let mut poses = Vec::new();
            let mut step_to_local_wp = Vec::new();
            for (wp_idx, wp) in plan.waypoints.iter().enumerate() {
                let (arr, dep) = memo
                    .get(&(robot_id.clone(), wp_idx))
                    .copied()
                    .unwrap_or((poses.len(), poses.len()));
                let hold_steps = dep.saturating_sub(arr) + 1;
                let pose = Isometry2::new(Vector2::new(wp.position[0], wp.position[1]), 0.0);
                poses.extend(std::iter::repeat_n(pose, hold_steps));
                step_to_local_wp.extend(std::iter::repeat_n(wp_idx, hold_steps));
            }
            let active_end_step = poses.len().saturating_sub(1);
            collated_agents.push(CollatedAgent {
                robot_id: robot_id.clone(),
                plan_id: plan.plan_id.clone(),
                step_to_local_wp,
                active_end_step,
                poses,
            });
        }

        let retained_len = collated_agents
            .iter()
            .map(|a| a.poses.len())
            .max()
            .unwrap_or(0);

        for agent in &mut collated_agents {
            if let Some(&last_pose) = agent.poses.last() {
                agent.poses.resize(retained_len, last_pose);
            }
            if let Some(&last_wp) = agent.step_to_local_wp.last() {
                agent.step_to_local_wp.resize(retained_len, last_wp);
            }
        }

        // 2. Append the newly planned cohort starting at `new_session_offset = retained_len`.
        let new_session_offset = retained_len;
        let new_session_len = basic_plan.iter().map(|t| t.len()).max().unwrap_or(0);

        for agent in &mut collated_agents {
            if let Some(&last_pose) = agent.poses.last() {
                agent
                    .poses
                    .resize(agent.poses.len() + new_session_len, last_pose);
            }
            if let Some(&last_wp) = agent.step_to_local_wp.last() {
                agent
                    .step_to_local_wp
                    .resize(agent.step_to_local_wp.len() + new_session_len, last_wp);
            }
        }

        let mut new_cohort_indices: Vec<(usize, usize)> = Vec::with_capacity(robot_ids.len());
        for (cohort_idx, robot_id) in robot_ids.iter().enumerate() {
            let Some(traj) = basic_plan.get(cohort_idx) else {
                continue;
            };
            let Some(dest) = goals.get(robot_id) else {
                continue;
            };
            let plan_id = self.next_plan_id(robot_id, dest);
            let mut poses = match traj.first() {
                Some(&start_pose) => vec![start_pose; new_session_offset],
                None => Vec::new(),
            };
            let mut step_to_local_wp = vec![0usize; new_session_offset];
            for (local_idx, &pose) in traj.iter().enumerate() {
                poses.push(pose);
                step_to_local_wp.push(local_idx);
            }
            let active_end_step = poses.len().saturating_sub(1);
            if let Some(&last_pose) = poses.last() {
                poses.resize(new_session_offset + new_session_len, last_pose);
            }
            if let Some(&last_wp) = step_to_local_wp.last() {
                step_to_local_wp.resize(new_session_offset + new_session_len, last_wp);
            }

            let collated_idx = collated_agents.len();
            collated_agents.push(CollatedAgent {
                robot_id: robot_id.clone(),
                plan_id,
                step_to_local_wp,
                active_end_step,
                poses,
            });
            new_cohort_indices.push((cohort_idx, collated_idx));
        }

        // 3. Build the combined MapfResult and compute traffic dependencies across all plans.
        let trajectories: Vec<Trajectory> = collated_agents
            .iter()
            .map(|a| Trajectory {
                poses: a.poses.clone(),
            })
            .collect();
        let collated_footprints: Vec<Arc<dyn Shape>> = collated_agents
            .iter()
            .map(|a| {
                footprints
                    .get(&a.robot_id)
                    .cloned()
                    .unwrap_or_else(|| Arc::new(Ball::new(0.49)) as Arc<dyn Shape>)
            })
            .collect();

        let mapf_result = MapfResult {
            trajectories,
            footprints: collated_footprints,
            discretization_timestep: timestep,
        };

        let traffic_dependencies = mapf_post::mapf_post(&mapf_result);

        // 4. Populate Plan messages for the newly planned cohort.
        let mut plans = HashMap::new();
        for (cohort_idx, collated_idx) in new_cohort_indices {
            let agent = &collated_agents[collated_idx];
            let traj = &basic_plan[cohort_idx];

            let mut waypoints = Vec::with_capacity(traj.len());
            for (i, pose) in traj.iter().enumerate() {
                waypoints.push(Waypoint {
                    position: [pose.translation.x, pose.translation.y],
                    arrival_constraints: Default::default(),
                    progress: i as f32 * timestep,
                    maps: Vec::new(),
                    departure_blockers: Vec::new(),
                    departure_trajectory: Vec::new(),
                    departure_action: String::new(),
                    arrival_action: String::new(),
                });
            }

            for i in 0..traj.len() {
                if let Some(dep_ids) = traffic_dependencies.comes_before(&SemanticWaypoint {
                    agent: collated_idx,
                    trajectory_index: new_session_offset + i,
                }) {
                    for &dep_id in dep_ids {
                        let dep_wp = &traffic_dependencies.waypoints[dep_id];
                        if dep_wp.agent == collated_idx {
                            continue;
                        }
                        let Some(dep_agent) = collated_agents.get(dep_wp.agent) else {
                            continue;
                        };
                        if dep_wp.trajectory_index < dep_agent.active_end_step {
                            let local_next_wp = dep_agent
                                .step_to_local_wp
                                .get(dep_wp.trajectory_index + 1)
                                .copied()
                                .unwrap_or(0);
                            if local_next_wp > 0 {
                                let req_progress = local_next_wp as f32 * timestep;
                                waypoints[i].departure_blockers.push(TrafficDependency {
                                    name: dep_agent.robot_id.clone(),
                                    plan_id: dep_agent.plan_id.clone(),
                                    required_progress: req_progress,
                                });
                            }
                        }
                    }
                }
            }

            if let Some(dest) = goals.get(&agent.robot_id) {
                if let Some(last_wp) = waypoints.last_mut() {
                    last_wp.arrival_constraints = dest.constraints.clone();
                }
            }

            plans.insert(
                agent.robot_id.clone(),
                Plan {
                    waypoints,
                    start_time: builtin_interfaces::msg::Time { sec: 0, nanosec: 0 },
                    plan_id: agent.plan_id.clone(),
                    workflow: String::new(),
                },
            );
        }

        (plans, mapf_result, traffic_dependencies)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_destination(uuid_byte: u8) -> Destination {
        let mut dest = Destination::default();
        dest.session.uuid = [uuid_byte; 16];
        dest
    }

    fn make_poses(coords: &[[f32; 2]]) -> Vec<Isometry2<f32>> {
        coords
            .iter()
            .map(|&[x, y]| Isometry2::new(Vector2::new(x, y), 0.0))
            .collect()
    }

    #[test]
    fn test_plan_version_increments_on_same_destination_and_resets_on_new_destination() {
        let mut registry = PlanRegistry::new();
        let dest_a = make_destination(1);
        let dest_b = make_destination(2);

        let id0 = registry.next_plan_id("robot1", &dest_a);
        assert_eq!(id0.plan_version, 0);
        assert_eq!(id0.destination_session.uuid, [1; 16]);

        registry.insert_plan(
            1,
            "robot1".to_string(),
            Plan {
                plan_id: id0,
                ..Default::default()
            },
        );

        // Same destination UUID -> version increments to 1
        let id1 = registry.next_plan_id("robot1", &dest_a);
        assert_eq!(id1.plan_version, 1);

        registry.insert_plan(
            2,
            "robot1".to_string(),
            Plan {
                plan_id: id1,
                ..Default::default()
            },
        );

        // Different destination UUID -> version resets to 0
        let id_new = registry.next_plan_id("robot1", &dest_b);
        assert_eq!(id_new.plan_version, 0);
        assert_eq!(id_new.destination_session.uuid, [2; 16]);
    }

    #[test]
    fn test_cross_plan_traffic_dependencies_across_sessions() {
        let mut registry = PlanRegistry::new();
        let mut footprints: HashMap<String, Arc<dyn Shape>> = HashMap::new();
        footprints.insert("robot1".to_string(), Arc::new(Ball::new(0.49)));
        footprints.insert("robot2".to_string(), Arc::new(Ball::new(0.49)));

        // Session 1: robot1 plans along (0, 0) -> (1, 0) -> (2, 0)
        let mut goals_s1 = HashMap::new();
        goals_s1.insert("robot1".to_string(), make_destination(10));
        let robot_ids_s1 = vec!["robot1".to_string()];
        let basic_plan_s1 = vec![make_poses(&[[0.0, 0.0], [1.0, 0.0], [2.0, 0.0]])];

        let (plans_s1, _, _) = registry.build_session_plans(
            &basic_plan_s1,
            &goals_s1,
            &robot_ids_s1,
            &footprints,
            1.0,
        );
        registry.record_session_plans(1, &plans_s1);

        let r1_plan = registry.get_plan("robot1").unwrap().clone();
        assert_eq!(r1_plan.plan_id.destination_session.uuid, [10; 16]);
        assert_eq!(r1_plan.plan_id.plan_version, 0);

        // Session 2: robot1 is executing an action (not replanned) and keeps its Session 1 plan.
        // robot2 plans across (1, -1) -> (1, 0) -> (1, 1), intersecting robot1 at (1, 0).
        let mut goals_s2 = HashMap::new();
        goals_s2.insert("robot2".to_string(), make_destination(20));
        let robot_ids_s2 = vec!["robot2".to_string()];
        let basic_plan_s2 = vec![make_poses(&[[1.0, -1.0], [1.0, 0.0], [1.0, 1.0]])];

        let (plans_s2, _, _) = registry.build_session_plans(
            &basic_plan_s2,
            &goals_s2,
            &robot_ids_s2,
            &footprints,
            1.0,
        );
        registry.record_session_plans(2, &plans_s2);

        let r2_plan = plans_s2.get("robot2").unwrap();
        assert_eq!(r2_plan.waypoints.len(), 3);

        // robot2's departure from waypoint 0 (1, -1) -> (1, 0) must depend on robot1's Session 1 plan!
        let wp0_blockers = &r2_plan.waypoints[0].departure_blockers;
        assert!(
            !wp0_blockers.is_empty(),
            "Expected cross-plan departure_blockers on robot2 waypoint 0"
        );
        for blocker in wp0_blockers {
            assert_eq!(blocker.name, "robot1");
            assert_eq!(blocker.plan_id, r1_plan.plan_id);
        }
        // robot1 clears (1, 0) when it reaches waypoint 2 at (2, 0) (progress 2.0)
        assert!(
            wp0_blockers
                .iter()
                .any(|b| (b.required_progress - 2.0).abs() < 1e-5),
            "Expected robot2 to wait for robot1 to reach progress 2.0, got: {:?}",
            wp0_blockers
        );
    }
}
