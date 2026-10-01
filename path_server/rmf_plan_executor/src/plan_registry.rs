use mapf_post::MapfResult;
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
use ros_env::rmf_prototype_msgs::msg::{Plan, PlanId};
use std::collections::{BTreeMap, HashMap, HashSet};

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
    /// List of robots with no active plan
    inactive_robots: HashSet<String>,
}

impl PlanRegistry {
    pub(crate) fn add_robot(&mut self, robot: String) {
        self.inactive_robots.insert(robot);
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
            self.invalidate_entry(robot, &plan_id.mapf_session);
            self.inactive_robots.insert(robot.to_string());
        }
    }

    //pub(crate) fn get_mapf_snapshot(&self) -> MapfResult {}
}
