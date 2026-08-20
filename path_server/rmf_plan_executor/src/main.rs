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
use rmf_plan_executor::PlanExecutor;
use ros_env::nav_msgs::msg::{OccupancyGrid, Odometry};
use ros_env::rmf_prototype_msgs::msg::{ParticipantList, Plan};
use std::collections::HashMap;

struct RobotConnections {
    _odom_subscription: rclrs::WorkerSubscription<Odometry, PlanExecutor>,
    _plan_subscription: rclrs::WorkerSubscription<Plan, PlanExecutor>,
}

struct ExecutorDiscoveryServer {
    node: rclrs::Node,
    active_robots: HashMap<String, RobotConnections>,
    executor_worker: rclrs::Worker<PlanExecutor>,
}

impl ExecutorDiscoveryServer {
    fn new(node: rclrs::Node, executor_worker: rclrs::Worker<PlanExecutor>) -> Self {
        Self {
            node,
            active_robots: HashMap::new(),
            executor_worker,
        }
    }
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let context = Context::default_from_env().unwrap();
    let mut executor = context.create_basic_executor();
    let node = executor.create_node("plan_executor")?;

    // Create the executor worker
    let executor_worker = node.create_worker(PlanExecutor::new(node.clone()));

    // Create the discovery worker
    let discovery_worker = node.create_worker(ExecutorDiscoveryServer::new(
        node.clone(),
        executor_worker.clone(),
    ));

    // 1. Subscribe to discovery on the executor_worker manually to handle state and footprints
    let mut tracker = rmf_participant_discovery::ParticipantTracker::new();
    let _footprints_subscription = executor_worker.create_subscription::<ParticipantList, _>(
        "/destination/discovery"
            .transient_local()
            .reliable()
            .keep_last(10),
        move |executor: &mut PlanExecutor, msg: ParticipantList| {
            let (added, removed) = tracker.update(&msg);
            for robot_id in removed {
                executor.handle_robot_removed(&robot_id);
            }
            for p in msg.participants {
                if added.contains(&p.name) {
                    executor.handle_robot_added(&p.name, 0.49);
                }
            }
        },
    )?;

    let _map_subscription = executor_worker.create_subscription::<OccupancyGrid, _>(
        "/map".transient_local().reliable(),
        move |executor: &mut PlanExecutor, msg: OccupancyGrid| {
            executor.handle_map(msg);
        },
    )?;

    // 2. Subscribe to discovery on the discovery worker to manage odom/plan subscriptions
    let _discovery_subscription = rmf_participant_discovery::create_discovery_subscription(
        &discovery_worker,
        "/destination/discovery",
        |server: &mut ExecutorDiscoveryServer, robot_id: &str| {
            if !server.active_robots.contains_key(robot_id) {
                rclrs::log!(
                    server.node.logger(),
                    "Discovered new participant for execution: {}",
                    robot_id
                );

                let robot_id_clone = robot_id.to_string();
                let robot_id_clone2 = robot_id.to_string();
                let odom_topic = robot_id.to_string() + "/odom";
                let plan_topic = robot_id.to_string() + "/plan";

                let odom_sub = match server.executor_worker.create_subscription::<Odometry, _>(
                    odom_topic.as_str().reliable(),
                    move |executor: &mut PlanExecutor, msg: Odometry| {
                        executor.handle_odometry(&robot_id_clone, msg);
                    },
                ) {
                    Ok(sub) => sub,
                    Err(err) => {
                        rclrs::log_error!(
                            server.node.logger(),
                            "Failed to create odom subscription on executor_worker for {}: {:?}",
                            robot_id,
                            err
                        );
                        return;
                    }
                };

                let plan_sub = match server.executor_worker.create_subscription::<Plan, _>(
                    plan_topic.as_str().transient_local().reliable(),
                    move |executor: &mut PlanExecutor, msg: Plan| {
                        executor.handle_plan(&robot_id_clone2, msg);
                    },
                ) {
                    Ok(sub) => sub,
                    Err(err) => {
                        rclrs::log_error!(
                            server.node.logger(),
                            "Failed to create plan subscription on executor_worker for {}: {:?}",
                            robot_id,
                            err
                        );
                        return;
                    }
                };

                server.active_robots.insert(
                    robot_id.to_string(),
                    RobotConnections {
                        _odom_subscription: odom_sub,
                        _plan_subscription: plan_sub,
                    },
                );
            }
        },
        |server: &mut ExecutorDiscoveryServer, robot_id: &str| {
            if server.active_robots.remove(robot_id).is_some() {
                rclrs::log!(
                    server.node.logger(),
                    "Participant left execution tracking: {}",
                    robot_id
                );
            }
        },
    )?;

    rclrs::log!(node.logger(), "Plan executor started. Spinning...");
    executor.spin(SpinOptions::default());
    Ok(())
}
