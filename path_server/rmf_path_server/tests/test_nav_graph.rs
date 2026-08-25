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
use rmf_path_server::{start_path_server_with_nav_graph, Map, MapfPlanner, NavGraphData};
use ros_env::nav_msgs::msg::Odometry;
use ros_env::rmf_prototype_msgs::msg::{
    Destination, DestinationConstraints, GraphElementKey, Participant, ParticipantList, Plan,
    TargetNode,
};
use std::collections::HashMap;
use std::sync::atomic::AtomicBool;
use std::sync::{Arc, Mutex};

const SAMPLE_SITE_JSON: &str = r#"{
  "format_version": "0.1",
  "properties": {
    "name": "test_site"
  },
  "levels": {
    "1": {
      "properties": {
        "name": "L1",
        "elevation": 0.0
      },
      "anchors": {
        "12": {
          "Translate2D": [0.0, 1.5]
        },
        "13": {
          "Translate2D": [0.0, 2.0]
        },
        "17": {
          "Translate2D": [2.5, 1.5]
        },
        "18": {
          "Translate2D": [2.5, 2.0]
        }
      },
      "floors": {},
      "rankings": {
        "floors": []
      }
    }
  },
  "navigation": {
    "guided": {
      "graphs": {
        "1": {
          "name": "default",
          "color": [1.0, 0.5, 0.3]
        }
      },
      "ranking": [1],
      "lanes": {
        "16": {
          "anchors": [13, 12],
          "forward": {
            "orientation_constraint": "Forwards",
            "speed_limit": 0.3,
            "dock": {
              "name": "dock_conveyor_r1_c1",
              "duration": 3.0
            }
          },
          "graphs": "All"
        }
      },
      "locations": {
        "14": {
          "anchor": 12,
          "tags": ["HoldingPoint"],
          "name": "conveyor_r1_c1_dock",
          "graphs": "All"
        },
        "15": {
          "anchor": 13,
          "tags": [],
          "name": "conveyor_r1_c1_staging",
          "graphs": "All"
        }
      }
    }
  }
}"#;

#[test]
fn test_nav_graph_parsing() {
    let site: rmf_site_format::Site =
        serde_json::from_str(SAMPLE_SITE_JSON).expect("failed to parse site json");
    let nav_graph = NavGraphData::from_site(&site);

    assert_eq!(nav_graph.vertices_by_id.len(), 4);
    assert_eq!(nav_graph.vertices_by_name.len(), 2);

    // Look up by vertex ID
    let v12 = nav_graph
        .vertices_by_id
        .get(&12)
        .expect("vertex 12 missing");
    assert_eq!(v12.position, [0.0, 1.5]);
    assert_eq!(v12.name.as_deref(), Some("conveyor_r1_c1_dock"));

    let action = v12
        .arrival_action
        .as_ref()
        .expect("dock action missing on v12");
    assert_eq!(action.action_type, "dock");
    assert_eq!(action.name, "dock_conveyor_r1_c1");
    assert_eq!(action.duration, Some(3.0));

    // Look up by GraphElementKey with vertex ID
    let mut key_id = GraphElementKey::default();
    key_id.key = vec![12i64].try_into().unwrap();
    let matched_v = nav_graph
        .find_vertex(&key_id)
        .expect("failed to find by id");
    assert_eq!(matched_v.id, 12);
    assert_eq!(
        matched_v.arrival_action.as_ref().unwrap().name,
        "dock_conveyor_r1_c1"
    );

    // Look up by GraphElementKey with semantic name
    let mut key_name = GraphElementKey::default();
    key_name.name = vec!["conveyor_r1_c1_dock".to_string().into()]
        .try_into()
        .unwrap();
    let matched_v_name = nav_graph
        .find_vertex(&key_name)
        .expect("failed to find by name");
    assert_eq!(matched_v_name.id, 12);
}

struct MockPathPlanner;

impl MapfPlanner for MockPathPlanner {
    fn plan(
        &self,
        starts: &HashMap<String, Odometry>,
        goals: &HashMap<String, Destination>,
        _footprints: &HashMap<String, Arc<dyn mapf_post::shape::Shape>>,
        robot_ids: &[String],
        _map: &Map,
        _cancellation: Arc<AtomicBool>,
    ) -> Result<Vec<Vec<Isometry2<f32>>>, Box<dyn std::error::Error>> {
        let mut plans = Vec::new();
        for robot_id in robot_ids {
            let start = starts.get(robot_id).unwrap();
            let dest = goals.get(robot_id).unwrap();
            let sx = start.pose.pose.position.x as f32;
            let sy = start.pose.pose.position.y as f32;
            let region = dest.constraints.regions.first().unwrap();
            let gx = region.region.points[0];
            let gy = region.region.points[1];

            // 2-waypoint plan from start to goal
            plans.push(vec![
                Isometry2::translation(sx, sy),
                Isometry2::translation(gx, gy),
            ]);
        }
        Ok(plans)
    }
}

#[test]
fn test_path_server_graphkey_destination_dock_action() -> Result<(), Box<dyn std::error::Error>> {
    let context = Context::default_from_env().unwrap();
    let mut executor = context.create_basic_executor();
    let test_node = Arc::new(executor.create_node("test_graphkey_node")?);
    let server_node = Arc::new(executor.create_node("path_server_graphkey")?);

    let site: rmf_site_format::Site =
        serde_json::from_str(SAMPLE_SITE_JSON).expect("failed to parse site json");
    let nav_graph = Arc::new(NavGraphData::from_site(&site));

    let _path_server_guard = start_path_server_with_nav_graph(
        Arc::clone(&server_node),
        MockPathPlanner,
        Some(nav_graph),
    )?;

    let received_plan = Arc::new(Mutex::new(None));
    let received_plan_clone = Arc::clone(&received_plan);

    let robot_id = "test_mir_dock_graphkey";
    let plan_sub = test_node.create_subscription::<Plan, _>(
        format!("{}/plan", robot_id)
            .as_str()
            .transient_local()
            .reliable(),
        move |msg: Plan| {
            let mut guard = received_plan_clone.lock().unwrap();
            *guard = Some(msg);
        },
    )?;
    let _ = plan_sub;

    let discovery_pub = test_node.create_publisher::<ParticipantList>(
        "/destination/discovery".transient_local().reliable(),
    )?;
    let odom_pub =
        test_node.create_publisher::<Odometry>(format!("{}/odom", robot_id).as_str().reliable())?;
    let dest_pub = test_node.create_publisher::<Destination>(
        format!("{}/destination", robot_id)
            .as_str()
            .transient_local()
            .reliable(),
    )?;

    // Publish discovery
    let mut discovery_msg = ParticipantList::default();
    discovery_msg.participants.push(Participant {
        name: robot_id.to_string(),
        components: vec![],
    });
    discovery_pub.publish(&discovery_msg)?;

    // Publish odometry at staging area (0.0, 2.0)
    let mut odom_msg = Odometry::default();
    odom_msg.pose.pose.position.x = 0.0;
    odom_msg.pose.pose.position.y = 2.0;
    odom_pub.publish(&odom_msg)?;

    // Publish Destination using GraphElementKey (vertex 12 -> dock_conveyor_r1_c1)
    let mut dest_msg = Destination::default();
    let mut key = GraphElementKey::default();
    key.key = vec![12i64].try_into().unwrap();

    let mut constraints = DestinationConstraints::default();
    constraints.nodes.push(TargetNode {
        key,
        orientations: vec![],
    });
    dest_msg.constraints = constraints;

    dest_pub.publish(&dest_msg)?;

    // Spin until plan is received
    let start_time = std::time::Instant::now();
    while start_time.elapsed() < std::time::Duration::from_secs(5) {
        let _ = odom_pub.publish(&odom_msg);
        executor.spin(SpinOptions::spin_once().timeout(std::time::Duration::from_millis(100)));
        if let Ok(guard) = received_plan.lock() {
            if let Some(plan) = guard.as_ref() {
                assert!(!plan.waypoints.is_empty(), "Plan should have waypoints");
                let last_wp = plan.waypoints.last().unwrap();
                assert_eq!(
                    last_wp.arrival_action, "dock_conveyor_r1_c1",
                    "Expected arrival_action 'dock_conveyor_r1_c1' on the final waypoint"
                );
                assert_eq!(last_wp.position, [0.0, 1.5]);

                // Verify departure trajectory is populated
                assert!(
                    !last_wp.departure_trajectory.is_empty(),
                    "Expected departure_trajectory to be populated on docking waypoint"
                );
                let dep_traj = &last_wp.departure_trajectory[0];
                assert_eq!(dep_traj.curve.control_points.len(), 2);
                assert_eq!(dep_traj.curve.control_points[0].position, [0.0, 1.5]);
                assert_eq!(dep_traj.curve.control_points[1].position, [0.0, 2.0]);
                return Ok(());
            }
        }
    }

    panic!("Timed out waiting for generated plan with arrival_action");
}
