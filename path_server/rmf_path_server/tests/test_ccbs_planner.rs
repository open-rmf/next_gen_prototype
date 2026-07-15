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
    assert_eq!(trajectories.len(), 1);

    let path = &trajectories[0];
    assert!(!path.is_empty(), "Path is empty");

    println!("Generated CCBS path:");
    for (i, pose) in path.iter().enumerate() {
        let x = pose.translation.x;
        let y = pose.translation.y;
        println!("  {}: ({}, {})", i, x, y);
    }

    // Verify that it reached near the goal (9.0, 5.0)
    let last_pose = path.last().unwrap();
    let lx = last_pose.translation.x;
    let ly = last_pose.translation.y;
    assert!(
        (lx - 9.0).abs() < 1.0,
        "Last pose x is {}, expected ~9.0",
        lx
    );
    assert!(
        (ly - 5.0).abs() < 1.0,
        "Last pose y is {}, expected ~5.0",
        ly
    );
}
