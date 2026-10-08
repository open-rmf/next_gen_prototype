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

use crate::{PlanExecutor, PlanExecutorConfig};
use rclrs::{IntoPrimitiveOptions, Node};
use ros_env::nav_msgs::msg::{OccupancyGrid, Odometry};
use ros_env::rmf_prototype_msgs::msg::{Plan, PlanError, PlanRelease, SafeZone};
use std::collections::HashMap;

pub struct PlanExecutorRosNode {
    pub node: Node,
    pub executor: PlanExecutor,
    pub plan_release_publishers: HashMap<String, rclrs::Publisher<PlanRelease>>,
    pub safezone_publishers: HashMap<String, rclrs::Publisher<SafeZone>>,
    pub plan_error_publishers: HashMap<String, rclrs::Publisher<PlanError>>,
}

impl PlanExecutorRosNode {
    pub fn new(node: Node) -> Self {
        Self::with_config(node, PlanExecutorConfig::default())
    }

    pub fn new_with_config(node: Node, config: PlanExecutorConfig) -> Self {
        Self::with_config(node, config)
    }

    pub fn with_config(node: Node, config: PlanExecutorConfig) -> Self {
        Self {
            node,
            executor: PlanExecutor::with_config(config),
            plan_release_publishers: HashMap::new(),
            safezone_publishers: HashMap::new(),
            plan_error_publishers: HashMap::new(),
        }
    }

    pub fn handle_robot_added(&mut self, robot_id: &str, radius: f32) {
        if self.executor.handle_robot_added(robot_id, radius) {
            rclrs::log!(
                self.node.logger(),
                "PlanExecutor adding participant: {} with radius {}",
                robot_id,
                radius
            );
        }
    }

    pub fn handle_robot_removed(&mut self, robot_id: &str) {
        if self.executor.handle_robot_removed(robot_id) {
            rclrs::log!(
                self.node.logger(),
                "PlanExecutor removing participant: {}",
                robot_id
            );
            self.plan_release_publishers.remove(robot_id);
            self.safezone_publishers.remove(robot_id);
            self.plan_error_publishers.remove(robot_id);
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

        if msg.waypoints.is_empty() {
            // TODO(arjoc): publish an error message
            rclrs::log_error!(self.node.logger(), "Received empty plan. Ignoring.");
            return;
        }

        self.executor.handle_plan(robot_id, msg);
    }

    pub fn handle_map(&mut self, msg: OccupancyGrid) {
        if msg.info.width > 0 && msg.info.height > 0 && msg.info.resolution > 0.0 {
            rclrs::log!(
                self.node.logger(),
                "PlanExecutor reconfiguring grid from map: width={}, height={}, resolution={}, origin=({}, {})",
                msg.info.width,
                msg.info.height,
                msg.info.resolution,
                msg.info.origin.position.x,
                msg.info.origin.position.y
            );
        }

        let errors = self.executor.handle_map(msg);
        for (robot_id, error) in errors {
            self.publish_plan_error(&robot_id, error);
        }
    }

    pub fn handle_odometry(&mut self, robot_id: &str, msg: Odometry) {
        let output = self.executor.handle_odometry(robot_id, msg);

        if let Some(error) = output.plan_error {
            self.publish_plan_error(robot_id, error);
        }

        if let Some(pr) = output.plan_release {
            self.publish_plan_release(robot_id, pr);
        }

        if let Some(safe_zone) = output.safe_zone {
            self.publish_safe_zone(robot_id, safe_zone);
        }
    }

    fn publish_plan_release(&mut self, robot_id: &str, pr: PlanRelease) {
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
    }

    fn publish_safe_zone(&mut self, robot_id: &str, safe_zone: SafeZone) {
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

    fn publish_plan_error(&mut self, robot_id: &str, error: PlanError) {
        let publisher = match self.plan_error_publishers.entry(robot_id.to_string()) {
            std::collections::hash_map::Entry::Occupied(entry) => entry.into_mut(),
            std::collections::hash_map::Entry::Vacant(entry) => {
                let topic = format!("{robot_id}/plan/error");
                match self.node.create_publisher(topic.as_str()) {
                    Ok(publisher) => entry.insert(publisher),
                    Err(err) => {
                        rclrs::log_error!(
                            self.node.logger(),
                            "Failed to create plan error publisher for {}: {:?}",
                            robot_id,
                            err
                        );
                        return;
                    }
                }
            }
        };

        if let Err(err) = publisher.publish(error) {
            rclrs::log_error!(
                self.node.logger(),
                "Failed to publish path blockage for {}: {:?}",
                robot_id,
                err
            );
        } else {
            rclrs::log_warn!(
                self.node.logger(),
                "Updated map blocks the remaining route for {}. Requesting a replan.",
                robot_id
            );
        }
    }
}

#[cfg(test)]
mod tests {
    use super::PlanExecutorRosNode;
    use rclrs::{Context, CreateBasicExecutor};
    use ros_env::nav_msgs::msg::Odometry;
    use ros_env::rmf_prototype_msgs::msg::{Plan, Waypoint};

    #[test]
    fn ros_node_manages_publishers_on_add_and_remove() {
        if let Ok(context) = Context::default_from_env() {
            let executor = context.create_basic_executor();
            if let Ok(node) = executor.create_node("test_plan_executor_ros_node") {
                let mut ros_node = PlanExecutorRosNode::new(node);

                ros_node.handle_robot_added("robot_1", 0.5);
                ros_node.handle_robot_added("robot_2", 0.5);

                let mut odom1 = Odometry::default();
                odom1.pose.pose.position.x = 0.0;
                odom1.pose.pose.position.y = 0.0;
                ros_node.handle_odometry("robot_1", odom1.clone());

                let mut odom2 = Odometry::default();
                odom2.pose.pose.position.x = 5.0;
                odom2.pose.pose.position.y = 5.0;
                ros_node.handle_odometry("robot_2", odom2);

                let mut plan1 = Plan::default();
                let mut wp0 = Waypoint::default();
                wp0.position = [0.0, 0.0];
                let mut wp1 = Waypoint::default();
                wp1.position = [2.0, 0.0];
                plan1.waypoints = vec![wp0, wp1];

                ros_node.handle_plan("robot_1", plan1);
                ros_node.handle_odometry("robot_1", odom1);

                assert!(ros_node.plan_release_publishers.contains_key("robot_1"));
                assert!(ros_node.safezone_publishers.contains_key("robot_1"));
                assert!(!ros_node.plan_release_publishers.contains_key("robot_2"));

                ros_node.handle_robot_removed("robot_1");
                assert!(!ros_node.plan_release_publishers.contains_key("robot_1"));
                assert!(!ros_node.safezone_publishers.contains_key("robot_1"));
            }
        }
    }
}
