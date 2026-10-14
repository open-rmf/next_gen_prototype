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

use mapf_post::na::Isometry2;
use rclrs::{Context, CreateBasicExecutor, IntoPrimitiveOptions, SpinOptions};
use rmf_path_server::{start_path_server, Map, MapfPlanner};
use ros_env::nav_msgs::msg::OccupancyGrid;
use ros_env::rmf_prototype_msgs::{
    self,
    msg::{rmw, Destination, Participant, ParticipantList, RobotStatus},
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
        for _ in 0..robot_ids.len() {
            plan.push(vec![Isometry2::identity()]);
        }
        Ok(plan)
    }
}

#[test]
fn test_map_subscription() -> Result<(), Box<dyn std::error::Error>> {
    let context = Context::default_from_env().unwrap();
    let mut executor = context.create_basic_executor();
    let test_node = Arc::new(executor.create_node("test_node")?);
    let server_node = Arc::new(executor.create_node("path_server")?);

    let received_map = Arc::new(Mutex::new(None));
    let planner = TestPlanner {
        received_map: Arc::clone(&received_map),
    };

    let _path_server_guard = start_path_server(Arc::clone(&server_node), planner)?;

    // Publishers
    let discovery_pub = test_node.create_publisher::<ParticipantList>(
        "/destination/discovery".transient_local().reliable(),
    )?;
    let map_pub =
        test_node.create_publisher::<OccupancyGrid>("/map".transient_local().reliable())?;

    let robot_id = "robot1";
    let status_topic = format!("{}/status", robot_id);
    let status_pub = test_node
        .create_publisher::<RobotStatus>(status_topic.as_str().transient_local().reliable())?;
    let dest_topic = format!("{}/destination", robot_id);
    let dest_pub = test_node
        .create_publisher::<Destination>(dest_topic.as_str().transient_local().reliable())?;

    // Publish map
    let mut map_msg = OccupancyGrid::default();
    map_msg.info.resolution = 1.0;
    map_msg.info.width = 10;
    map_msg.info.height = 10;
    map_msg.data = vec![0; 100];
    map_pub.publish(&map_msg)?;
    println!("Published map");

    // Publish discovery
    let mut discovery_msg = ParticipantList::default();
    discovery_msg.participants.push(Participant {
        name: robot_id.to_string(),
        components: vec![],
    });
    discovery_pub.publish(&discovery_msg)?;
    println!("Published discovery");

    // Publish status
    let mut status_msg = RobotStatus::default();
    status_msg.pose.pose.position.x = 1.0;
    status_msg.pose.pose.position.y = 1.0;
    status_pub.publish(&status_msg)?;
    println!("Published status");

    // Publish destination
    let mut dest_msg = Destination::default();
    dest_msg
        .constraints
        .regions
        .push(rmf_prototype_msgs::msg::TargetRegion {
            region: rmf_prototype_msgs::msg::Region {
                points: vec![5.0, 5.0],
                ..Default::default()
            },
            ..Default::default()
        });
    dest_pub.publish(&dest_msg)?;
    println!("Published destination");

    // Wait for the planner to receive the map.
    let start = std::time::Instant::now();
    let timeout = std::time::Duration::from_secs(5);
    while start.elapsed() < timeout {
        status_pub.publish(&status_msg)?;
        executor.spin(SpinOptions::spin_once().timeout(std::time::Duration::from_millis(100)));

        if let Ok(guard) = received_map.lock() {
            if let Some(map) = guard.as_ref() {
                assert_eq!(map.grid.info.resolution, 1.0);
                assert_eq!(map.grid.info.width, 10);
                assert_eq!(map.grid.info.height, 10);
                assert_eq!(map.grid.data, vec![0; 100]);
                return Ok(());
            }
        }
    }

    Err("Timeout waiting for map in planner".into())
}

#[test]
fn test_status_subscription() -> Result<(), Box<dyn std::error::Error>> {
    let context = Context::default_from_env().unwrap();
    let mut executor = context.create_basic_executor();
    let test_node = Arc::new(executor.create_node("test_status_node")?);
    let server_node = Arc::new(executor.create_node("path_server_status")?);

    let received_map = Arc::new(Mutex::new(None));
    let planner = TestPlanner { received_map };

    let path_server_guard = start_path_server(Arc::clone(&server_node), planner)?;

    let discovery_pub = test_node.create_publisher::<ParticipantList>(
        "/destination/discovery".transient_local().reliable(),
    )?;

    let robot_id = "robot1";
    let status_topic = format!("{}/status", robot_id);
    let status_pub = test_node
        .create_publisher::<RobotStatus>(status_topic.as_str().transient_local().reliable())?;

    let mut discovery_msg = ParticipantList::default();
    discovery_msg.participants.push(Participant {
        name: robot_id.to_string(),
        components: vec![],
    });
    discovery_pub.publish(&discovery_msg)?;

    let mut progress_msg = rmw::Progress::default();
    progress_msg.progress = 2.5;
    progress_msg.reached_waypoint = 2;
    progress_msg.target_waypoint = 3;
    progress_msg.plan_id.plan_version = 4;

    let mut status_msg = RobotStatus::default();
    status_msg.pose.pose.position.x = 3.0;
    status_msg.pose.pose.position.y = 4.0;
    status_msg.mode = RobotStatus::MODE_NAVIGATING;
    status_msg.progress = vec![progress_msg].try_into().unwrap();

    let observed_status = Arc::new(Mutex::new(None));
    let start = std::time::Instant::now();
    let timeout = std::time::Duration::from_secs(5);
    while start.elapsed() < timeout {
        status_pub.publish(&status_msg)?;
        let observed_clone = Arc::clone(&observed_status);
        let _ = path_server_guard.destinations_worker.run(move |server| {
            if let Some(s) = server.latest_status.get(robot_id) {
                *observed_clone.lock().unwrap() = Some(s.clone());
            }
        });
        executor.spin(SpinOptions::spin_once().timeout(std::time::Duration::from_millis(100)));

        if let Some(s) = observed_status.lock().unwrap().clone() {
            assert!((s.pose.pose.position.x - 3.0).abs() < 1e-6);
            assert!((s.pose.pose.position.y - 4.0).abs() < 1e-6);
            assert_eq!(s.mode, RobotStatus::MODE_NAVIGATING);
            assert_eq!(s.progress.len(), 1);
            let p = &s.progress[0];
            assert!((p.progress - 2.5).abs() < 1e-6);
            assert_eq!(p.reached_waypoint, 2);
            assert_eq!(p.target_waypoint, 3);
            assert_eq!(p.plan_id.plan_version, 4);
            return Ok(());
        }
    }

    Err("Timeout waiting for status in PlanServer".into())
}
