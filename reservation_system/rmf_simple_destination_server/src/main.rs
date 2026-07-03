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

use rclrs::{Context, CreateBasicExecutor, IntoPrimitiveOptions, SpinOptions};
use ros_env::rmf_prototype_msgs::msg::{
    Destination, DestinationConstraints, DestinationError, DestinationGoal, Error,
};
use ros_env::unique_identifier_msgs::msg::UUID;
use std::collections::HashMap;
use std::sync::Arc;

fn first_candidate(goal: &DestinationGoal) -> Option<&DestinationConstraints> {
    goal.one_of.first()
}

struct RobotPublishers {
    destination: rclrs::Publisher<Destination>,
    error: rclrs::Publisher<DestinationError>,
}

struct DestinationsServer {
    node: Arc<rclrs::Node>,
    robot_publishers: HashMap<String, RobotPublishers>,
}

impl DestinationsServer {
    fn new(node: Arc<rclrs::Node>) -> Self {
        Self {
            node,
            robot_publishers: HashMap::new(),
        }
    }

    fn register_robot(
        &mut self,
        robot_id: &str,
        destination: rclrs::Publisher<Destination>,
        error: rclrs::Publisher<DestinationError>,
    ) {
        self.robot_publishers
            .insert(robot_id.to_string(), RobotPublishers { destination, error });
    }

    fn deregister_robot(&mut self, robot_id: &str) {
        self.robot_publishers.remove(robot_id);
    }

    fn handle_goal(&self, robot_id: &str, goal: DestinationGoal) {
        let Some(publishers) = self.robot_publishers.get(robot_id) else {
            rclrs::log_warn!(
                self.node.logger(),
                "No publishers registered for {}; dropping destination goal",
                robot_id
            );
            return;
        };

        let Some(constraints) = first_candidate(&goal) else {
            self.publish_error(publishers, &goal, "No goals provided", "one_of is empty");
            return;
        };

        let destination = Destination {
            constraints: constraints.clone(),
            session: goal.session.clone(),
            detour_for_goal: UUID { uuid: [0; 16] },
            ..Default::default()
        };

        rclrs::log!(
            self.node.logger(),
            "Selected the first of {} candidates for {}",
            goal.one_of.len(),
            robot_id
        );
        let _ = publishers.destination.publish(&destination);
    }

    fn publish_error(
        &self,
        publishers: &RobotPublishers,
        goal: &DestinationGoal,
        message: &str,
        parameters: &str,
    ) {
        let error = DestinationError {
            error: Error {
                code: 0,
                message: message.to_string(),
                parameters: parameters.to_string(),
            },
            session: goal.session.clone(),
        };
        rclrs::log_error!(self.node.logger(), "{}", message);
        let _ = publishers.error.publish(&error);
    }
}

struct DiscoveryServer {
    node: Arc<rclrs::Node>,
    active_robots: HashMap<String, rclrs::WorkerSubscription<DestinationGoal, DestinationsServer>>,
    destinations_worker: Arc<rclrs::Worker<DestinationsServer>>,
}

impl DiscoveryServer {
    fn new(
        node: Arc<rclrs::Node>,
        destinations_worker: Arc<rclrs::Worker<DestinationsServer>>,
    ) -> Self {
        Self {
            node,
            active_robots: HashMap::new(),
            destinations_worker,
        }
    }
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();
    let node = Arc::new(executor.create_node("simple_destination_server")?);

    let destinations_worker =
        Arc::new(node.create_worker(DestinationsServer::new(Arc::clone(&node))));
    let discovery_worker = Arc::new(node.create_worker(DiscoveryServer::new(
        Arc::clone(&node),
        Arc::clone(&destinations_worker),
    )));

    let _discovery_subscription = rmf_participant_discovery::create_discovery_subscription(
        &discovery_worker,
        "/destination/discovery",
        |server: &mut DiscoveryServer, robot_id: &str| {
            if server.active_robots.contains_key(robot_id) {
                return;
            }

            rclrs::log!(
                server.node.logger(),
                "Discovered new participant: {}",
                robot_id
            );

            let destination = match server.node.create_publisher::<Destination>(
                (&(robot_id.to_string() + "/destination"))
                    .transient_local()
                    .reliable(),
            ) {
                Ok(publisher) => publisher,
                Err(error) => {
                    rclrs::log_error!(
                        server.node.logger(),
                        "Failed to create destination publisher for {}: {:?}",
                        robot_id,
                        error
                    );
                    return;
                }
            };
            let error = match server.node.create_publisher::<DestinationError>(
                (&(robot_id.to_string() + "/destination/error"))
                    .transient_local()
                    .reliable(),
            ) {
                Ok(publisher) => publisher,
                Err(error) => {
                    rclrs::log_error!(
                        server.node.logger(),
                        "Failed to create error publisher for {}: {:?}",
                        robot_id,
                        error
                    );
                    return;
                }
            };

            let register_id = robot_id.to_string();
            std::mem::drop(server.destinations_worker.run(
                move |destinations: &mut DestinationsServer| {
                    destinations.register_robot(&register_id, destination, error);
                },
            ));

            let callback_id = robot_id.to_string();
            let subscription = match server
                .destinations_worker
                .create_subscription::<DestinationGoal, _>(
                    (&(robot_id.to_string() + "/destination/goal"))
                        .transient_local()
                        .reliable(),
                    move |destinations: &mut DestinationsServer, goal: DestinationGoal| {
                        destinations.handle_goal(&callback_id, goal);
                    },
                ) {
                Ok(subscription) => subscription,
                Err(error) => {
                    rclrs::log_error!(
                        server.node.logger(),
                        "Failed to create goal subscription for {}: {:?}",
                        robot_id,
                        error
                    );
                    return;
                }
            };
            server
                .active_robots
                .insert(robot_id.to_string(), subscription);
        },
        |server: &mut DiscoveryServer, robot_id: &str| {
            if server.active_robots.remove(robot_id).is_none() {
                return;
            }
            let deregister_id = robot_id.to_string();
            std::mem::drop(server.destinations_worker.run(
                move |destinations: &mut DestinationsServer| {
                    destinations.deregister_robot(&deregister_id);
                },
            ));
        },
    )?;

    rclrs::log!(
        node.logger(),
        "Simple destination server started; forwarding the first candidate"
    );
    executor.spin(SpinOptions::default());
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn goal_with_candidates(count: usize) -> DestinationGoal {
        DestinationGoal {
            one_of: vec![DestinationConstraints::default(); count],
            ..Default::default()
        }
    }

    #[test]
    fn selects_first_candidate() {
        assert!(first_candidate(&goal_with_candidates(3)).is_some());
    }

    #[test]
    fn rejects_empty_candidate_list() {
        assert!(first_candidate(&goal_with_candidates(0)).is_none());
    }
}
