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
    Destination, DestinationConstraints, DockStatus, GraphElementKey, Participant, ParticipantList,
    Plan, Region, TargetNode, TargetOrientation, TargetRegion,
};
use std::collections::HashMap;
use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
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
    assert!(nav_graph.vertices_by_name.len() >= 2);

    // Look up by vertex ID
    let v12 = nav_graph
        .vertices_by_id
        .get(&12)
        .expect("vertex 12 missing");
    assert_eq!(v12.position, [0.0, 1.5]);
    assert_eq!(v12.name.as_deref(), Some("conveyor_r1_c1_dock"));
    assert_eq!(v12.orientation, Some(-std::f32::consts::FRAC_PI_2));

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

    // Look up by dock action name (from lane.forward.dock)
    let mut key_action = GraphElementKey::default();
    key_action.name = vec!["dock_conveyor_r1_c1".to_string().into()]
        .try_into()
        .unwrap();
    let matched_action = nav_graph
        .find_vertex(&key_action)
        .expect("failed to find by dock action name");
    assert_eq!(matched_action.id, 12);

    // Proximity matching: physical contact point at (0.0, 0.95) should snap to pre-dock at (0.0, 1.5)
    let snapped_dock = nav_graph.find_dock_vertex_by_proximity(0.0, 0.95, 0.8);
    assert!(snapped_dock.is_some());
    assert_eq!(snapped_dock.unwrap().id, 12);
    assert_eq!(snapped_dock.unwrap().position, [0.0, 1.5]);

    // Staging point at (0.0, 2.0) should NOT snap to dock
    let staging_snap = nav_graph.find_dock_vertex_by_proximity(0.0, 2.0, 0.8);
    assert!(staging_snap.is_none());

    // Pre-dock point at (0.0, 1.5) directly snaps to dock
    let predock_snap = nav_graph.find_dock_vertex_by_proximity(0.0, 1.5, 0.8);
    assert!(predock_snap.is_some());
    assert_eq!(predock_snap.unwrap().id, 12);
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
        _frozen_claims: &[Vec<[f32; 2]>],
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

/// Index of the single waypoint carrying `action`.
///
/// The dock is deliberately *not* the last waypoint. Plans are padded to the
/// length of the longest concurrent plan so that a robot which has arrived
/// keeps occupying its goal in the conflict analysis, and the action must fire
/// on arrival rather than on each padded repeat.
fn waypoint_with_action(plan: &Plan, action: &str) -> usize {
    let matches: Vec<usize> = plan
        .waypoints
        .iter()
        .enumerate()
        .filter(|(_, wp)| wp.arrival_action == action)
        .map(|(i, _)| i)
        .collect();
    assert_eq!(
        matches.len(),
        1,
        "expected exactly one waypoint with action '{action}', found {matches:?}"
    );
    matches[0]
}

/// Assert that everything beyond `idx` is the robot holding station.
///
/// These tests share a ROS domain, so the amount of padding depends on which
/// peers happen to be alive. What must hold regardless is that padding never
/// moves the robot and never carries an action.
fn assert_padding_after(plan: &Plan, idx: usize) {
    let held = plan.waypoints[idx].position;
    for (i, wp) in plan.waypoints.iter().enumerate().skip(idx + 1) {
        assert_eq!(wp.position, held, "padding at wp {i} moved the robot");
        assert!(
            wp.arrival_action.is_empty() && wp.departure_action.is_empty(),
            "padding at wp {i} carries an action"
        );
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

    // Publish odometry away from the dock, at the far staging vertex (2.5, 2.0),
    // so the resulting plan is not degenerate.
    let mut odom_msg = Odometry::default();
    odom_msg.pose.pose.position.x = 2.5;
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
        let _ = discovery_pub.publish(&discovery_msg);
        let _ = odom_pub.publish(&odom_msg);
        let _ = dest_pub.publish(&dest_msg);
        executor.spin(SpinOptions::spin_once().timeout(std::time::Duration::from_millis(100)));
        if let Ok(guard) = received_plan.lock() {
            if let Some(plan) = guard.as_ref() {
                assert!(!plan.waypoints.is_empty(), "Plan should have waypoints");

                // The destination resolves to the *staging* vertex, so the mock
                // planner emits [start, staging]. The dock lane is then appended
                // as a single open-ended segment: how long the robot stays in
                // the dock is not something the plan can know.
                let dock_idx = waypoint_with_action(plan, "dock_conveyor_r1_c1");
                assert_eq!(
                    dock_idx, 2,
                    "Expected 2 planned waypoints then the appended dock segment"
                );

                assert_eq!(plan.waypoints[0].position, [2.5, 2.0]);

                // The staging vertex is where the plan proper ends, so the
                // destination's constraints live here.
                let pre_dock = &plan.waypoints[1];
                assert_eq!(pre_dock.position, [0.0, 2.0]);
                assert!(
                    pre_dock.arrival_action.is_empty(),
                    "Staging is not the dock; it must not trigger the dock action"
                );

                // Nothing follows the dock but the robot sitting in it: the
                // plan does not model the dwell, only the occupancy.
                let arrival_wp = &plan.waypoints[dock_idx];
                assert_eq!(arrival_wp.position, [0.0, 1.5]);
                assert_padding_after(plan, dock_idx);

                // Verify departure trajectory is populated. It points back out
                // of the dock at the staging vertex.
                assert!(
                    !arrival_wp.departure_trajectory.is_empty(),
                    "Expected departure_trajectory to be populated on docking waypoint"
                );
                let dep_traj = &arrival_wp.departure_trajectory[0];
                assert_eq!(dep_traj.curve.control_points.len(), 2);
                assert_eq!(dep_traj.curve.control_points[0].position, [0.0, 1.5]);
                assert_eq!(dep_traj.curve.control_points[1].position, [0.0, 2.0]);
                return Ok(());
            }
        }
    }

    panic!("Timed out waiting for generated plan with arrival_action");
}

#[test]
fn test_path_server_dock_name_destination_routes_to_predock(
) -> Result<(), Box<dyn std::error::Error>> {
    let context = Context::default_from_env().unwrap();
    let mut executor = context.create_basic_executor();
    let test_node = Arc::new(executor.create_node("test_dock_name_node")?);
    let server_node = Arc::new(executor.create_node("path_server_dock_name")?);

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

    let robot_id = "test_mir_dock_name";
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

    let mut discovery_msg = ParticipantList::default();
    discovery_msg.participants.push(Participant {
        name: robot_id.to_string(),
        components: vec![],
    });
    discovery_pub.publish(&discovery_msg)?;

    let mut odom_msg = Odometry::default();
    odom_msg.pose.pose.position.x = 2.5;
    odom_msg.pose.pose.position.y = 2.0;
    odom_pub.publish(&odom_msg)?;

    // Destination specifies "conveyor_r1_c1_dock" (from site format locations) and raw contact coords (0.0, 0.95)
    let mut dest_msg = Destination::default();
    let mut key = GraphElementKey::default();
    key.name = vec!["conveyor_r1_c1_dock".to_string().into()]
        .try_into()
        .unwrap();

    let mut constraints = DestinationConstraints::default();
    constraints.nodes.push(TargetNode {
        key,
        orientations: vec![],
    });
    constraints.regions.push(TargetRegion {
        region: Region {
            points: vec![0.0, 0.95],
            hint: Region::HINT_POINT,
        },
        ..Default::default()
    });
    dest_msg.constraints = constraints;
    dest_pub.publish(&dest_msg)?;

    let start_time = std::time::Instant::now();
    while start_time.elapsed() < std::time::Duration::from_secs(5) {
        let _ = discovery_pub.publish(&discovery_msg);
        let _ = odom_pub.publish(&odom_msg);
        let _ = dest_pub.publish(&dest_msg);
        executor.spin(SpinOptions::spin_once().timeout(std::time::Duration::from_millis(100)));
        if let Ok(guard) = received_plan.lock() {
            if let Some(plan) = guard.as_ref() {
                assert!(!plan.waypoints.is_empty(), "Plan should have waypoints");
                // The dock action marks where the robot arrives. Verifies the
                // planner routed to the pre-dock pose and the dock lane was
                // appended down to (0.0, 1.5), NOT to contact point 0.95.
                let dock_idx = waypoint_with_action(plan, "dock_conveyor_r1_c1");
                let arrival_wp = &plan.waypoints[dock_idx];
                assert_eq!(arrival_wp.position, [0.0, 1.5]);
                assert_padding_after(plan, dock_idx);

                // The destination's constraints belong to the staging vertex,
                // which is where the planned motion actually ends. The approach
                // orientation (facing South towards the conveyor, -pi/2) is
                // therefore adopted before entering the corridor.
                let pre_dock = &plan.waypoints[dock_idx - 1];
                assert_eq!(pre_dock.position, [0.0, 2.0]);
                let region_ori = pre_dock
                    .arrival_constraints
                    .regions
                    .first()
                    .and_then(|r| r.orientations.first())
                    .map(|o| o.orientation_radians);
                assert!(
                    region_ori.is_some(),
                    "Arrival region should contain orientation"
                );
                let ori = region_ori.unwrap();
                assert!(
                    (ori - (-std::f32::consts::FRAC_PI_2)).abs() < 1e-3,
                    "Expected orientation -pi/2, got {}",
                    ori
                );

                assert!(!arrival_wp.departure_trajectory.is_empty());
                return Ok(());
            }
        }
    }

    panic!("Timed out waiting for generated plan with dock name destination");
}

#[test]
fn test_path_server_raw_contact_coordinates_snaps_to_predock(
) -> Result<(), Box<dyn std::error::Error>> {
    let context = Context::default_from_env().unwrap();
    let mut executor = context.create_basic_executor();
    let test_node = Arc::new(executor.create_node("test_rawcoord_node")?);
    let server_node = Arc::new(executor.create_node("path_server_rawcoord")?);

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

    let robot_id = "test_mir_raw_dock";
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

    let mut discovery_msg = ParticipantList::default();
    discovery_msg.participants.push(Participant {
        name: robot_id.to_string(),
        components: vec![],
    });
    discovery_pub.publish(&discovery_msg)?;

    let mut odom_msg = Odometry::default();
    odom_msg.pose.pose.position.x = 2.5;
    odom_msg.pose.pose.position.y = 2.0;
    odom_pub.publish(&odom_msg)?;

    // Destination specifies ONLY raw contact coordinates (0.0, 0.95), no graph node key!
    let mut dest_msg = Destination::default();
    let mut constraints = DestinationConstraints::default();
    constraints.regions.push(TargetRegion {
        region: Region {
            points: vec![0.0, 0.95],
            hint: Region::HINT_POINT,
        },
        ..Default::default()
    });
    dest_msg.constraints = constraints;
    dest_pub.publish(&dest_msg)?;

    let start_time = std::time::Instant::now();
    while start_time.elapsed() < std::time::Duration::from_secs(5) {
        let _ = discovery_pub.publish(&discovery_msg);
        let _ = odom_pub.publish(&odom_msg);
        let _ = dest_pub.publish(&dest_msg);
        executor.spin(SpinOptions::spin_once().timeout(std::time::Duration::from_millis(100)));
        if let Ok(guard) = received_plan.lock() {
            if let Some(plan) = guard.as_ref() {
                assert!(!plan.waypoints.is_empty(), "Plan should have waypoints");
                let dock_idx = waypoint_with_action(plan, "dock_conveyor_r1_c1");
                let arrival_wp = &plan.waypoints[dock_idx];
                // Verifies raw contact coordinate was snapped by proximity to
                // the dock vertex, and that the appended dock lane ends there.
                assert_eq!(arrival_wp.position, [0.0, 1.5]);
                assert_padding_after(plan, dock_idx);

                let pre_dock = &plan.waypoints[dock_idx - 1];
                assert_eq!(pre_dock.position, [0.0, 2.0]);
                let region_ori = pre_dock
                    .arrival_constraints
                    .regions
                    .first()
                    .and_then(|r| r.orientations.first())
                    .map(|o| o.orientation_radians);
                assert!(
                    region_ori.is_some(),
                    "Arrival region should contain orientation"
                );
                let ori = region_ori.unwrap();
                assert!(
                    (ori - (-std::f32::consts::FRAC_PI_2)).abs() < 1e-3,
                    "Expected orientation -pi/2, got {}",
                    ori
                );

                assert!(!arrival_wp.departure_trajectory.is_empty());
                return Ok(());
            }
        }
    }

    panic!("Timed out waiting for generated plan with raw contact coordinates");
}

/// The grid planner is still started at the undocked vertex, because the dock
/// lane is finer than its resolution. But the plan handed to the robot must
/// begin where the robot really is, so that the undock sweep is visible to
/// `mapf_post` and the dock corridor is not silently declared free.
///
/// The robot announces that it is docked. Sitting at the dock's coordinates is
/// not sufficient and deliberately so: a robot stopped beside a dock occupies
/// the same neighbourhood without being in it, and only the robot knows which
/// of the two it is.
#[test]
fn test_path_server_docked_start_splices_the_undock_back_in(
) -> Result<(), Box<dyn std::error::Error>> {
    let context = Context::default_from_env().unwrap();
    let mut executor = context.create_basic_executor();
    let test_node = Arc::new(executor.create_node("test_docked_start_node")?);
    let server_node = Arc::new(executor.create_node("path_server_docked_start")?);

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

    let robot_id = "test_mir_docked_start";
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
    // Transient local, matching the real publisher in rmf_nav2_traffic: dock
    // state is a latched fact, not an event stream.
    let dock_status_pub = test_node.create_publisher::<DockStatus>(
        format!("{}/dock_status", robot_id)
            .as_str()
            .transient_local()
            .reliable(),
    )?;

    let mut discovery_msg = ParticipantList::default();
    discovery_msg.participants.push(Participant {
        name: robot_id.to_string(),
        components: vec![],
    });
    discovery_pub.publish(&discovery_msg)?;

    // Robot is currently physically docked at (0.0, 0.95), and says so. The
    // dock_id names the lane it is in, which is how the path server finds the
    // vertex to back out through.
    let mut odom_msg = Odometry::default();
    odom_msg.pose.pose.position.x = 0.0;
    odom_msg.pose.pose.position.y = 0.95;
    odom_pub.publish(&odom_msg)?;

    let mut dock_status_msg = DockStatus::default();
    dock_status_msg.state = DockStatus::STATE_DOCKED;
    dock_status_msg.dock_id = "dock_conveyor_r1_c1".to_string();
    dock_status_pub.publish(&dock_status_msg)?;

    // Robot receives a destination to go to parking_spot at (2.5, 2.0)
    let mut dest_msg = Destination::default();
    let mut constraints = DestinationConstraints::default();
    constraints.regions.push(TargetRegion {
        region: Region {
            points: vec![2.5, 2.0],
            hint: Region::HINT_POINT,
        },
        ..Default::default()
    });
    dest_msg.constraints = constraints;
    dest_pub.publish(&dest_msg)?;

    let start_time = std::time::Instant::now();
    while start_time.elapsed() < std::time::Duration::from_secs(5) {
        let _ = discovery_pub.publish(&discovery_msg);
        let _ = odom_pub.publish(&odom_msg);
        let _ = dock_status_pub.publish(&dock_status_msg);
        let _ = dest_pub.publish(&dest_msg);
        executor.spin(SpinOptions::spin_once().timeout(std::time::Duration::from_millis(100)));
        if let Ok(guard) = received_plan.lock() {
            if let Some(plan) = guard.as_ref() {
                assert!(!plan.waypoints.is_empty(), "Plan should have waypoints");

                // [docked] [dock vertex] [staging] [goal] then any padding
                assert!(plan.waypoints.len() >= 4);

                // The plan starts where the robot physically is, not at the
                // staging vertex the planner was seeded with.
                let first_wp = plan.waypoints.first().unwrap();
                assert_eq!(first_wp.position, [0.0, 0.95]);
                assert_eq!(
                    first_wp.departure_action, "undock",
                    "Waypoint 0 should be marked as requiring an undock"
                );

                // Back out through the dock vertex and the staging vertex,
                // sweeping the corridor on the way. There are no repeated poses
                // before the goal: a projection-based progress report cannot
                // tell them apart, so any blocker aimed at one would never be
                // satisfied.
                assert_eq!(plan.waypoints[1].position, [0.0, 1.5]);
                assert_eq!(plan.waypoints[2].position, [0.0, 2.0]);
                assert_eq!(plan.waypoints[3].position, [2.5, 2.0]);

                // This robot is not docking, so anything past the goal is the
                // padding that keeps it visible while its peers finish.
                assert_padding_after(plan, 3);
                return Ok(());
            }
        }
    }

    panic!("Timed out waiting for generated plan starting from the docked pose");
}

/// A robot parked in a dock that nobody has asked to move must not be replanned
/// because one of its peers was given a task.
///
/// This is the defect the dock reporting exists to fix. A destination is never
/// retired once reached, so a robot that docked long ago is still a member of
/// every subsequent negotiation. Before the fix it would be handed a fresh plan
/// whenever any peer replanned, and since `rmf_nav2_traffic` undocks before
/// servicing a `NavigateToPose`, receiving that plan reversed it out of a dock
/// it had been sitting in quite happily.
///
/// The assertion is on the *number of plans published*, not their contents,
/// because the content is irrelevant: even republishing the identical plan the
/// robot is already executing is enough to undock it.
#[test]
fn test_path_server_parked_robot_is_not_replanned_for_a_peer(
) -> Result<(), Box<dyn std::error::Error>> {
    let context = Context::default_from_env().unwrap();
    let mut executor = context.create_basic_executor();
    let test_node = Arc::new(executor.create_node("test_parked_peer_node")?);
    let server_node = Arc::new(executor.create_node("path_server_parked_peer")?);

    let site: rmf_site_format::Site =
        serde_json::from_str(SAMPLE_SITE_JSON).expect("failed to parse site json");
    let nav_graph = Arc::new(NavGraphData::from_site(&site));

    let _path_server_guard = start_path_server_with_nav_graph(
        Arc::clone(&server_node),
        MockPathPlanner,
        Some(nav_graph),
    )?;

    let parked_id = "test_mir_parked_in_dock";
    let mover_id = "test_mir_peer_mover";

    let parked_plan_count = Arc::new(AtomicUsize::new(0));
    let parked_plan_count_clone = Arc::clone(&parked_plan_count);
    let _parked_plan_sub = test_node.create_subscription::<Plan, _>(
        format!("{}/plan", parked_id)
            .as_str()
            .transient_local()
            .reliable(),
        move |_msg: Plan| {
            parked_plan_count_clone.fetch_add(1, Ordering::SeqCst);
        },
    )?;

    let mover_plan = Arc::new(Mutex::new(None));
    let mover_plan_clone = Arc::clone(&mover_plan);
    let _mover_plan_sub = test_node.create_subscription::<Plan, _>(
        format!("{}/plan", mover_id)
            .as_str()
            .transient_local()
            .reliable(),
        move |msg: Plan| {
            *mover_plan_clone.lock().unwrap() = Some(msg);
        },
    )?;

    let discovery_pub = test_node.create_publisher::<ParticipantList>(
        "/destination/discovery".transient_local().reliable(),
    )?;
    let parked_odom_pub = test_node
        .create_publisher::<Odometry>(format!("{}/odom", parked_id).as_str().reliable())?;
    let mover_odom_pub =
        test_node.create_publisher::<Odometry>(format!("{}/odom", mover_id).as_str().reliable())?;
    let parked_dock_status_pub = test_node.create_publisher::<DockStatus>(
        format!("{}/dock_status", parked_id)
            .as_str()
            .transient_local()
            .reliable(),
    )?;
    let parked_dest_pub = test_node.create_publisher::<Destination>(
        format!("{}/destination", parked_id)
            .as_str()
            .transient_local()
            .reliable(),
    )?;
    let mover_dest_pub = test_node.create_publisher::<Destination>(
        format!("{}/destination", mover_id)
            .as_str()
            .transient_local()
            .reliable(),
    )?;

    let mut discovery_msg = ParticipantList::default();
    for name in [parked_id, mover_id] {
        discovery_msg.participants.push(Participant {
            name: name.to_string(),
            components: vec![],
        });
    }

    // The parked robot sits in the dock and says so.
    let mut parked_odom = Odometry::default();
    parked_odom.pose.pose.position.x = 0.0;
    parked_odom.pose.pose.position.y = 0.95;

    let mut parked_dock_status = DockStatus::default();
    parked_dock_status.state = DockStatus::STATE_DOCKED;
    parked_dock_status.dock_id = "dock_conveyor_r1_c1".to_string();

    // Both robots need odometry throughout: `replan` abandons the whole
    // negotiation if any participant's pose is missing.
    let mut mover_odom = Odometry::default();
    mover_odom.pose.pose.position.x = 2.5;
    mover_odom.pose.pose.position.y = 1.5;

    let point_destination = |x: f32, y: f32| {
        let mut dest = Destination::default();
        let mut constraints = DestinationConstraints::default();
        constraints.regions.push(TargetRegion {
            region: Region {
                points: vec![x, y],
                hint: Region::HINT_POINT,
            },
            ..Default::default()
        });
        dest.constraints = constraints;
        dest
    };

    // Phase 1: give the parked robot a destination so that it acquires an entry
    // in `active_destinations`. This is the state a robot is left in after it
    // has completed a task, and the reason it keeps turning up in negotiations.
    let parked_dest = point_destination(2.5, 2.0);
    // Everything the server needs to keep believing in both robots. Republished
    // every iteration because the subscriptions are created asynchronously as
    // the participants are discovered, so an early sample can be missed.
    let republish = || {
        let _ = discovery_pub.publish(&discovery_msg);
        let _ = parked_odom_pub.publish(&parked_odom);
        let _ = mover_odom_pub.publish(&mover_odom);
        let _ = parked_dock_status_pub.publish(&parked_dock_status);
        let _ = parked_dest_pub.publish(&parked_dest);
    };

    let start_time = std::time::Instant::now();
    while start_time.elapsed() < std::time::Duration::from_secs(5)
        && parked_plan_count.load(Ordering::SeqCst) == 0
    {
        republish();
        executor.spin(SpinOptions::spin_once().timeout(std::time::Duration::from_millis(100)));
    }
    assert!(
        parked_plan_count.load(Ordering::SeqCst) > 0,
        "Timed out waiting for the parked robot's initial plan"
    );

    // Let the first negotiation finish completely, so that anything counted
    // after this point is attributable to the peer's request.
    let settle = std::time::Instant::now();
    while settle.elapsed() < std::time::Duration::from_secs(1) {
        republish();
        executor.spin(SpinOptions::spin_once().timeout(std::time::Duration::from_millis(100)));
    }
    let plans_before_peer_request = parked_plan_count.load(Ordering::SeqCst);

    // Phase 2: an unrelated robot is given a task. The parked robot has not been
    // asked to go anywhere, so it must be left alone.
    let mover_dest = point_destination(0.0, 2.0);
    let start_time = std::time::Instant::now();
    while start_time.elapsed() < std::time::Duration::from_secs(5) {
        let _ = mover_dest_pub.publish(&mover_dest);
        republish();
        executor.spin(SpinOptions::spin_once().timeout(std::time::Duration::from_millis(100)));
        if mover_plan.lock().unwrap().is_some() {
            break;
        }
    }
    assert!(
        mover_plan.lock().unwrap().is_some(),
        "Timed out waiting for the peer's plan; the parked robot must not be blocking it"
    );

    // The peer's plan has been produced, so the negotiation that would have
    // swept the parked robot in has already run. Settle once more to catch a
    // late publication.
    let settle = std::time::Instant::now();
    while settle.elapsed() < std::time::Duration::from_secs(1) {
        let _ = mover_dest_pub.publish(&mover_dest);
        republish();
        executor.spin(SpinOptions::spin_once().timeout(std::time::Duration::from_millis(100)));
    }

    assert_eq!(
        parked_plan_count.load(Ordering::SeqCst),
        plans_before_peer_request,
        "A robot parked in a dock was sent a new plan because a peer replanned; \
         receiving any plan at all makes it reverse out of the dock"
    );

    Ok(())
}
