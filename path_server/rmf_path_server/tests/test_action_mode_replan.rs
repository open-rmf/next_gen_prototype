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

use mapf_post::na::{Isometry2, Vector2};
use rclrs::{Context, CreateBasicExecutor, IntoPrimitiveOptions, SpinOptions};
use rmf_path_server::{start_path_server, Map, MapfPlanner};
use ros_env::nav_msgs::msg::OccupancyGrid;
use ros_env::rmf_prototype_msgs::{
    self,
    msg::{Destination, Participant, ParticipantList, RobotStatus},
};
use std::collections::HashMap;
use std::sync::atomic::AtomicBool;
use std::sync::{Arc, Mutex};

struct TestPlanner {
    received_map: Arc<Mutex<Option<Map>>>,
}

impl MapfPlanner for TestPlanner {
    fn plan(
        &self,
        _starts: &HashMap<String, RobotStatus>,
        _goals: &HashMap<String, Destination>,
        _footprints: &HashMap<String, Arc<dyn mapf_post::shape::Shape>>,
        robot_ids: &[String],
        map: &Map,
        _cancellation: Arc<AtomicBool>,
    ) -> Result<Vec<Vec<Isometry2<f32>>>, Box<dyn std::error::Error>> {
        if let Ok(mut guard) = self.received_map.lock() {
            *guard = Some(map.clone());
        }
        let mut plan = Vec::new();
        for id in robot_ids {
            if id == "robot1" {
                // robot1 moves horizontally: (0, 3) -> (1, 3) -> (2, 3)
                plan.push(vec![
                    Isometry2::new(Vector2::new(0.0, 3.0), 0.0),
                    Isometry2::new(Vector2::new(1.0, 3.0), 0.0),
                    Isometry2::new(Vector2::new(2.0, 3.0), 0.0),
                ]);
            } else {
                // robot2 moves vertically across (1, 3): (1, 2) -> (1, 3) -> (1, 4)
                plan.push(vec![
                    Isometry2::new(Vector2::new(1.0, 2.0), 0.0),
                    Isometry2::new(Vector2::new(1.0, 3.0), 0.0),
                    Isometry2::new(Vector2::new(1.0, 4.0), 0.0),
                ]);
            }
        }
        Ok(plan)
    }
}

#[test]
fn test_action_mode_robot_excluded_from_replan_and_cross_plan_dependencies_computed(
) -> Result<(), Box<dyn std::error::Error>> {
    let context = Context::default_from_env().unwrap();
    let mut executor = context.create_basic_executor();
    let test_node = Arc::new(executor.create_node("test_action_mode_node")?);
    let server_node = Arc::new(executor.create_node("path_server_action_mode")?);

    let received_map = Arc::new(Mutex::new(None));
    let planner = TestPlanner {
        received_map: Arc::clone(&received_map),
    };

    let path_server_guard = start_path_server(Arc::clone(&server_node), planner)?;

    let discovery_pub = test_node.create_publisher::<ParticipantList>(
        "/destination/discovery".transient_local().reliable(),
    )?;
    let map_pub =
        test_node.create_publisher::<OccupancyGrid>("/map".transient_local().reliable())?;

    let r1_status_pub =
        test_node.create_publisher::<RobotStatus>("robot1/status".transient_local().reliable())?;
    let r2_status_pub =
        test_node.create_publisher::<RobotStatus>("robot2/status".transient_local().reliable())?;
    let r1_dest_pub = test_node
        .create_publisher::<Destination>("robot1/destination".transient_local().reliable())?;
    let r2_dest_pub = test_node
        .create_publisher::<Destination>("robot2/destination".transient_local().reliable())?;

    let r1_plans = Arc::new(Mutex::new(Vec::new()));
    let r1_plans_clone = Arc::clone(&r1_plans);
    let _r1_plan_sub = test_node.create_subscription::<rmf_prototype_msgs::msg::Plan, _>(
        "robot1/plan".transient_local().reliable(),
        move |msg: rmf_prototype_msgs::msg::Plan| {
            r1_plans_clone.lock().unwrap().push(msg);
        },
    )?;

    let r2_plans = Arc::new(Mutex::new(Vec::new()));
    let r2_plans_clone = Arc::clone(&r2_plans);
    let _r2_plan_sub = test_node.create_subscription::<rmf_prototype_msgs::msg::Plan, _>(
        "robot2/plan".transient_local().reliable(),
        move |msg: rmf_prototype_msgs::msg::Plan| {
            r2_plans_clone.lock().unwrap().push(msg);
        },
    )?;

    let mut map_msg = OccupancyGrid::default();
    map_msg.info.resolution = 1.0;
    map_msg.info.width = 10;
    map_msg.info.height = 10;
    map_msg.data = vec![0; 100];
    map_pub.publish(&map_msg)?;

    let mut discovery_msg = ParticipantList::default();
    for id in ["robot1", "robot2"] {
        discovery_msg.participants.push(Participant {
            name: id.to_string(),
            components: vec![],
        });
    }
    discovery_pub.publish(&discovery_msg)?;

    // Session 1: robot1 is IDLE at (0.0, 3.0) and receives its initial plan to (2.0, 3.0)
    let mut r1_status = RobotStatus::default();
    r1_status.pose.pose.position.x = 0.0;
    r1_status.pose.pose.position.y = 3.0;
    r1_status.mode = RobotStatus::MODE_IDLE;
    r1_status_pub.publish(&r1_status)?;

    let mut r2_status = RobotStatus::default();
    r2_status.pose.pose.position.x = 1.0;
    r2_status.pose.pose.position.y = 2.0;
    r2_status.mode = RobotStatus::MODE_IDLE;
    r2_status_pub.publish(&r2_status)?;

    let mut r1_dest = Destination::default();
    r1_dest.session.uuid = [1; 16];
    r1_dest
        .constraints
        .regions
        .push(rmf_prototype_msgs::msg::TargetRegion {
            region: rmf_prototype_msgs::msg::Region {
                points: vec![2.0, 3.0],
                ..Default::default()
            },
            ..Default::default()
        });
    r1_dest_pub.publish(&r1_dest)?;

    let timeout = std::time::Duration::from_secs(5);
    let start = std::time::Instant::now();
    let mut r1_initial_plan = None;
    while start.elapsed() < timeout {
        r1_status_pub.publish(&r1_status)?;
        r2_status_pub.publish(&r2_status)?;
        executor.spin(SpinOptions::spin_once().timeout(std::time::Duration::from_millis(100)));

        if let Some(plan) = r1_plans.lock().unwrap().first().cloned() {
            r1_initial_plan = Some(plan);
            break;
        }
    }
    let r1_initial_plan = r1_initial_plan.expect("Timeout waiting for robot1 initial plan");
    assert_eq!(r1_initial_plan.plan_id.destination_session.uuid, [1; 16]);
    assert_eq!(r1_initial_plan.plan_id.plan_version, 0);

    // Session 2: robot1 enters MODE_ACTION at (2.0, 3.0).
    // Wait until the server has observed robot1's MODE_ACTION status before sending new destinations.
    r1_status.pose.pose.position.x = 2.0;
    r1_status.pose.pose.position.y = 3.0;
    r1_status.mode = RobotStatus::MODE_ACTION;
    let observed_action_mode = Arc::new(AtomicBool::new(false));
    let start = std::time::Instant::now();
    while start.elapsed() < timeout {
        r1_status_pub.publish(&r1_status)?;
        let observed_clone = Arc::clone(&observed_action_mode);
        let _ = path_server_guard.destinations_worker.run(move |server| {
            if server.is_executing_action("robot1") {
                observed_clone.store(true, std::sync::atomic::Ordering::Relaxed);
            }
        });
        executor.spin(SpinOptions::spin_once().timeout(std::time::Duration::from_millis(50)));
        if observed_action_mode.load(std::sync::atomic::Ordering::Relaxed) {
            break;
        }
    }
    assert!(
        observed_action_mode.load(std::sync::atomic::Ordering::Relaxed),
        "Timeout waiting for server to observe robot1 MODE_ACTION"
    );

    let mut r1_dest_deferred = Destination::default();
    r1_dest_deferred.session.uuid = [11; 16];
    r1_dest_deferred
        .constraints
        .regions
        .push(rmf_prototype_msgs::msg::TargetRegion {
            region: rmf_prototype_msgs::msg::Region {
                points: vec![2.0, 4.0],
                ..Default::default()
            },
            ..Default::default()
        });
    r1_dest_pub.publish(&r1_dest_deferred)?;

    let mut r2_dest = Destination::default();
    r2_dest.session.uuid = [2; 16];
    r2_dest
        .constraints
        .regions
        .push(rmf_prototype_msgs::msg::TargetRegion {
            region: rmf_prototype_msgs::msg::Region {
                points: vec![1.0, 4.0],
                ..Default::default()
            },
            ..Default::default()
        });
    r2_dest_pub.publish(&r2_dest)?;

    let start = std::time::Instant::now();
    let mut r2_planned = false;
    while start.elapsed() < timeout {
        r1_status_pub.publish(&r1_status)?;
        r2_status_pub.publish(&r2_status)?;
        executor.spin(SpinOptions::spin_once().timeout(std::time::Duration::from_millis(100)));

        if let Some(r2_plan) = r2_plans.lock().unwrap().first().cloned() {
            // robot1 is in MODE_ACTION so it must still only have its 1st plan
            assert_eq!(
                r1_plans.lock().unwrap().len(),
                1,
                "robot1 in MODE_ACTION should not be replanned"
            );
            // Cell (2, 3) should be marked occupied (100) in the map passed to the planner
            let map = received_map.lock().unwrap().clone().unwrap();
            assert_eq!(map.grid.data[3 * 10 + 2], 100);

            // And robot2's plan must have cross-plan traffic dependencies on robot1's Session 1 plan!
            let blockers = &r2_plan.waypoints[0].departure_blockers;
            assert!(
                !blockers.is_empty(),
                "Expected cross-plan departure_blockers on robot2 waypoint 0"
            );
            assert!(
                blockers.iter().any(|b| {
                    b.name == "robot1"
                        && b.plan_id == r1_initial_plan.plan_id
                        && (b.required_progress - 2.0).abs() < 1e-5
                }),
                "Expected robot2 to depend on robot1 plan {:?} reaching progress 2.0, got: {:?}",
                r1_initial_plan.plan_id,
                blockers
            );

            r2_planned = true;
            break;
        }
    }
    assert!(r2_planned, "Timeout waiting for robot2 plan");

    // Once robot1 finishes its action and reports MODE_IDLE, its deferred destination should be planned
    r1_status.mode = RobotStatus::MODE_IDLE;
    let start = std::time::Instant::now();
    while start.elapsed() < timeout {
        r1_status_pub.publish(&r1_status)?;
        r2_status_pub.publish(&r2_status)?;
        executor.spin(SpinOptions::spin_once().timeout(std::time::Duration::from_millis(100)));

        if r1_plans.lock().unwrap().len() >= 2 {
            return Ok(());
        }
    }

    Err("Timeout waiting for deferred robot1 plan after leaving MODE_ACTION".into())
}
