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

use mapf_post::shape::Ball;
use rmf_path_server::planner::{CcbsPlanner, Map, MapfPlanner};
use ros_env::nav_msgs::msg::{OccupancyGrid, Odometry};
use ros_env::rmf_prototype_msgs::msg::Destination;
use std::collections::HashMap;
use std::sync::atomic::AtomicBool;
use std::sync::Arc;

#[test]
fn test_ccbs_planner_with_map() {
    let mut occupancy_grid = OccupancyGrid::default();
    occupancy_grid.info.resolution = 1.0;
    occupancy_grid.info.width = 10;
    occupancy_grid.info.height = 10;
    occupancy_grid.info.origin.position.x = 0.0;
    occupancy_grid.info.origin.position.y = 0.0;
    occupancy_grid.data = vec![0; 100];

    // Put obstacle at (5,5) -> index = 5 * 10 + 5 = 55
    occupancy_grid.data[55] = 100; // 100 is occupied

    let map = Map {
        grid: occupancy_grid,
    };

    let mut starts = HashMap::new();
    let mut odom = Odometry::default();
    odom.pose.pose.position.x = 0.0;
    odom.pose.pose.position.y = 5.0;
    starts.insert("robot1".to_string(), odom);

    let mut goals = HashMap::new();
    let mut dest = Destination::default();
    dest.constraints
        .regions
        .push(ros_env::rmf_prototype_msgs::msg::TargetRegion {
            region: ros_env::rmf_prototype_msgs::msg::Region {
                points: vec![9.0, 5.0],
                ..Default::default()
            },
            ..Default::default()
        });
    goals.insert("robot1".to_string(), dest);

    let mut footprints = HashMap::new();
    footprints.insert(
        "robot1".to_string(),
        Arc::new(Ball::new(0.45)) as Arc<dyn mapf_post::shape::Shape>,
    );

    let robot_ids = vec!["robot1".to_string()];
    let planner = CcbsPlanner::default();

    let result = planner.plan(
        &starts,
        &goals,
        &footprints,
        &robot_ids,
        &map,
        Arc::new(AtomicBool::new(false)),
    );

    assert!(result.is_ok(), "Planning failed: {:?}", result.err());
    let trajectories = result.unwrap();
    assert_eq!(trajectories.len(), 1);

    let path = &trajectories[0];
    assert!(!path.is_empty(), "Path is empty");

    println!("Generated CCBS path:");
    for (i, pose) in path.iter().enumerate() {
        let x = pose.translation.x;
        let y = pose.translation.y;
        println!("  {}: ({}, {})", i, x, y);

        // Verify that the path does not hit the obstacle at (5.0, 5.0)
        let dx = (x - 5.0).abs();
        let dy = (y - 5.0).abs();
        assert!(
            dx > 0.1 || dy > 0.1,
            "Path hits obstacle at (5,5) at step {}",
            i
        );

        // Verify the path stays within map bounds [0, 9]
        assert!(x >= -0.1 && x <= 9.1, "Path x is out of bounds: {}", x);
        assert!(y >= -0.1 && y <= 9.1, "Path y is out of bounds: {}", y);
    }

    // Verify that start is aligned to ~0.0, 5.0 without half-cell offset
    let first_pose = path.first().unwrap();
    assert!(
        (first_pose.translation.x - 0.0).abs() < 0.1,
        "First pose x is {}, expected 0.0",
        first_pose.translation.x
    );
    assert!(
        (first_pose.translation.y - 5.0).abs() < 0.1,
        "First pose y is {}, expected 5.0",
        first_pose.translation.y
    );

    // Verify that it reached the goal (9.0, 5.0) exactly without half-cell offset
    let last_pose = path.last().unwrap();
    let lx = last_pose.translation.x;
    let ly = last_pose.translation.y;
    assert!(
        (lx - 9.0).abs() < 0.1,
        "Last pose x is {}, expected 9.0",
        lx
    );
    assert!(
        (ly - 5.0).abs() < 0.1,
        "Last pose y is {}, expected 5.0",
        ly
    );
}

#[test]
fn test_ccbs_planner_footprint_radius() {
    let mut occupancy_grid = OccupancyGrid::default();
    occupancy_grid.info.resolution = 1.0;
    occupancy_grid.info.width = 10;
    occupancy_grid.info.height = 10;
    occupancy_grid.data = vec![0; 100];

    let map = Map {
        grid: occupancy_grid,
    };

    let mut starts = HashMap::new();
    let mut odom = Odometry::default();
    odom.pose.pose.position.x = 1.0;
    odom.pose.pose.position.y = 1.0;
    starts.insert("robot_large".to_string(), odom);

    let mut goals = HashMap::new();
    let mut dest = Destination::default();
    dest.constraints
        .regions
        .push(ros_env::rmf_prototype_msgs::msg::TargetRegion {
            region: ros_env::rmf_prototype_msgs::msg::Region {
                points: vec![8.0, 8.0],
                ..Default::default()
            },
            ..Default::default()
        });
    goals.insert("robot_large".to_string(), dest);

    let mut footprints = HashMap::new();
    footprints.insert(
        "robot_large".to_string(),
        Arc::new(Ball::new(0.4)) as Arc<dyn mapf_post::shape::Shape>,
    );

    let robot_ids = vec!["robot_large".to_string()];
    let planner = CcbsPlanner::default();

    let result = planner.plan(
        &starts,
        &goals,
        &footprints,
        &robot_ids,
        &map,
        Arc::new(AtomicBool::new(false)),
    );

    assert!(result.is_ok(), "Planning failed: {:?}", result.err());
    let trajectories = result.unwrap();
    assert_eq!(trajectories.len(), 1);
    let path = &trajectories[0];
    assert!(!path.is_empty());
    let last_pose = path.last().unwrap();
    assert!((last_pose.translation.x - 8.0).abs() < 0.1);
    assert!((last_pose.translation.y - 8.0).abs() < 0.1);
}

#[test]
fn test_ccbs_planner_decoupled_resolution() {
    // 200x200 grid at 0.05m resolution = 10m x 10m map
    let mut occupancy_grid = OccupancyGrid::default();
    occupancy_grid.info.resolution = 0.05;
    occupancy_grid.info.width = 200;
    occupancy_grid.info.height = 200;
    occupancy_grid.info.origin.position.x = 0.0;
    occupancy_grid.info.origin.position.y = 0.0;
    occupancy_grid.data = vec![0; 40000];

    // Put obstacle around world (5.0, 5.0), which corresponds to grid (100, 100)
    for x in 98..=102 {
        for y in 98..=102 {
            occupancy_grid.data[y * 200 + x] = 100;
        }
    }

    let map = Map {
        grid: occupancy_grid,
    };

    let mut starts = HashMap::new();
    let mut odom = Odometry::default();
    odom.pose.pose.position.x = 0.0;
    odom.pose.pose.position.y = 5.0;
    starts.insert("robot1".to_string(), odom);

    let mut goals = HashMap::new();
    let mut dest = Destination::default();
    dest.constraints
        .regions
        .push(ros_env::rmf_prototype_msgs::msg::TargetRegion {
            region: ros_env::rmf_prototype_msgs::msg::Region {
                points: vec![9.0, 5.0],
                ..Default::default()
            },
            ..Default::default()
        });
    goals.insert("robot1".to_string(), dest);

    let footprints = HashMap::new();
    let robot_ids = vec!["robot1".to_string()];
    // Planner automatically decouples planning resolution (1.0m) from fine map resolution (0.05m)
    let planner = CcbsPlanner::default().with_cell_size(1.0);

    let result = planner.plan(
        &starts,
        &goals,
        &footprints,
        &robot_ids,
        &map,
        Arc::new(AtomicBool::new(false)),
    );

    assert!(
        result.is_ok(),
        "Planning on fine resolution map failed: {:?}",
        result.err()
    );
    let trajectories = result.unwrap();
    assert_eq!(trajectories.len(), 1);
    let path = &trajectories[0];
    assert!(!path.is_empty());

    let last_pose = path.last().unwrap();
    assert!(
        (last_pose.translation.x - 9.0).abs() < 0.1,
        "Expected 9.0, got {}",
        last_pose.translation.x
    );
    assert!(
        (last_pose.translation.y - 5.0).abs() < 0.1,
        "Expected 5.0, got {}",
        last_pose.translation.y
    );
}

#[test]
fn test_ccbs_planner_boundary_wall() {
    let mut occupancy_grid = OccupancyGrid::default();
    occupancy_grid.info.resolution = 1.0;
    occupancy_grid.info.width = 10;
    occupancy_grid.info.height = 10;
    occupancy_grid.info.origin.position.x = 0.0;
    occupancy_grid.info.origin.position.y = 0.0;
    occupancy_grid.data = vec![0; 100];

    // Place obstacle block in the center
    for x in 4..=6 {
        for y in 4..=6 {
            occupancy_grid.data[y * 10 + x] = 100;
        }
    }

    let map = Map {
        grid: occupancy_grid,
    };

    let mut starts = HashMap::new();
    let mut odom = Odometry::default();
    odom.pose.pose.position.x = 1.0;
    odom.pose.pose.position.y = 1.0;
    starts.insert("robot1".to_string(), odom);

    let mut goals = HashMap::new();
    let mut dest = Destination::default();
    dest.constraints
        .regions
        .push(ros_env::rmf_prototype_msgs::msg::TargetRegion {
            region: ros_env::rmf_prototype_msgs::msg::Region {
                points: vec![8.0, 8.0],
                ..Default::default()
            },
            ..Default::default()
        });
    goals.insert("robot1".to_string(), dest);

    let footprints = HashMap::new();
    let robot_ids = vec!["robot1".to_string()];
    let planner = CcbsPlanner::default();

    let result = planner.plan(
        &starts,
        &goals,
        &footprints,
        &robot_ids,
        &map,
        Arc::new(AtomicBool::new(false)),
    );

    assert!(result.is_ok(), "Planning failed: {:?}", result.err());
    let trajectories = result.unwrap();
    let path = &trajectories[0];

    // Check that every point stays strictly inside [0.0, 9.0]
    for pose in path {
        assert!(
            pose.translation.x >= -0.1 && pose.translation.x <= 9.1,
            "x {} exceeded boundary",
            pose.translation.x
        );
        assert!(
            pose.translation.y >= -0.1 && pose.translation.y <= 9.1,
            "y {} exceeded boundary",
            pose.translation.y
        );
    }
}

#[test]
fn test_ccbs_multi_agent_schedule() {
    let mut occupancy_grid = OccupancyGrid::default();
    occupancy_grid.info.resolution = 1.0;
    occupancy_grid.info.width = 10;
    occupancy_grid.info.height = 10;
    occupancy_grid.info.origin.position.x = 0.0;
    occupancy_grid.info.origin.position.y = 0.0;
    occupancy_grid.data = vec![0; 100];

    let map = Map {
        grid: occupancy_grid,
    };

    let mut starts = HashMap::new();
    let mut odom1 = Odometry::default();
    odom1.pose.pose.position.x = 0.0;
    odom1.pose.pose.position.y = 5.0;
    starts.insert("robot1".to_string(), odom1);

    let mut odom2 = Odometry::default();
    odom2.pose.pose.position.x = 5.0;
    odom2.pose.pose.position.y = 0.0;
    starts.insert("robot2".to_string(), odom2);

    let mut goals = HashMap::new();
    let mut dest1 = Destination::default();
    dest1
        .constraints
        .regions
        .push(ros_env::rmf_prototype_msgs::msg::TargetRegion {
            region: ros_env::rmf_prototype_msgs::msg::Region {
                points: vec![9.0, 5.0],
                ..Default::default()
            },
            ..Default::default()
        });
    goals.insert("robot1".to_string(), dest1);

    let mut dest2 = Destination::default();
    dest2
        .constraints
        .regions
        .push(ros_env::rmf_prototype_msgs::msg::TargetRegion {
            region: ros_env::rmf_prototype_msgs::msg::Region {
                points: vec![5.0, 9.0],
                ..Default::default()
            },
            ..Default::default()
        });
    goals.insert("robot2".to_string(), dest2);

    let footprints = HashMap::new();
    let robot_ids = vec!["robot1".to_string(), "robot2".to_string()];
    let planner = CcbsPlanner::default();

    let result = planner.plan(
        &starts,
        &goals,
        &footprints,
        &robot_ids,
        &map,
        Arc::new(AtomicBool::new(false)),
    );

    assert!(
        result.is_ok(),
        "Multi-agent CCBS failed: {:?}",
        result.err()
    );
    let trajectories = result.unwrap();
    assert_eq!(trajectories.len(), 2);

    // Both trajectories must have equal length for time-discretized mapf_post processing
    assert_eq!(
        trajectories[0].len(),
        trajectories[1].len(),
        "Trajectories should be discretized to matching lengths"
    );

    // Ensure agents don't collide at any time step
    for step in 0..trajectories[0].len() {
        let p1 = &trajectories[0][step].translation;
        let p2 = &trajectories[1][step].translation;
        let dist = ((p1.x - p2.x).powi(2) + (p1.y - p2.y).powi(2)).sqrt();
        assert!(
            dist > 0.5,
            "Agents collided at step {} with dist = {}",
            step,
            dist
        );
    }
}
