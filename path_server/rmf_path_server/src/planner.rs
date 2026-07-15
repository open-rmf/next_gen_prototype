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

use hetpibt::external_tracks_pibt::PiBTWithExternalTracks;
use mapf::graph::occupancy::Cell;
use mapf::motion::se2::Point;
use mapf::negotiation::{negotiate, Agent, Scenario};
use mapf_post::na::Isometry2;
use ros_env::nav_msgs::msg::{OccupancyGrid, Odometry};
use ros_env::rmf_prototype_msgs::msg::Destination;
use std::collections::HashMap;

use std::sync::atomic::AtomicBool;
use std::sync::Arc;

#[derive(Clone, Debug, Default)]
pub struct Map {
    pub grid: OccupancyGrid,
}

/// Implement this trait to use your own custom MAPF
/// planner. The planner in this scenario will take in
/// starts and goals and assign a trajectory to the agents.
pub trait MapfPlanner: Send + Sync + 'static {
    fn plan(
        &self,
        starts: &HashMap<String, Odometry>,
        goals: &HashMap<String, Destination>,
        footprints: &HashMap<String, Arc<dyn mapf_post::shape::Shape>>,
        robot_ids: &[String],
        map: &Map,
        cancellation: Arc<AtomicBool>,
    ) -> Result<Vec<Vec<Isometry2<f32>>>, Box<dyn std::error::Error>>;
}

#[derive(Clone, Default)]
pub struct MockPlanner;

impl MapfPlanner for MockPlanner {
    fn plan(
        &self,
        _starts: &HashMap<String, Odometry>,
        _goals: &HashMap<String, Destination>,
        _footprints: &HashMap<String, Arc<dyn mapf_post::shape::Shape>>,
        robot_ids: &[String],
        _map: &Map,
        _cancellation: Arc<AtomicBool>,
    ) -> Result<Vec<Vec<Isometry2<f32>>>, Box<dyn std::error::Error>> {
        let mut plan = Vec::new();
        for _ in 0..robot_ids.len() {
            plan.push(vec![Isometry2::identity()]);
        }
        Ok(plan)
    }
}

#[derive(Clone)]
pub struct PibtPlanner {
    pub max_time: usize,
}

impl Default for PibtPlanner {
    fn default() -> Self {
        Self { max_time: 100 }
    }
}

impl PibtPlanner {
    pub fn new(max_time: usize) -> Self {
        Self { max_time }
    }
}

impl MapfPlanner for PibtPlanner {
    fn plan(
        &self,
        starts: &HashMap<String, Odometry>,
        goals: &HashMap<String, Destination>,
        _footprints: &HashMap<String, Arc<dyn mapf_post::shape::Shape>>,
        robot_ids: &[String],
        map: &Map,
        _cancellation: Arc<AtomicBool>,
    ) -> Result<Vec<Vec<Isometry2<f32>>>, Box<dyn std::error::Error>> {
        if robot_ids.is_empty() {
            return Ok(Vec::new());
        }

        let use_map =
            map.grid.info.width > 0 && map.grid.info.height > 0 && map.grid.info.resolution > 0.0;

        let (width, height, resolution, offset_x, offset_y, grid) = if use_map {
            let w = map.grid.info.width as usize;
            let h = map.grid.info.height as usize;
            let r = map.grid.info.resolution;
            let ox = map.grid.info.origin.position.x as f32;
            let oy = map.grid.info.origin.position.y as f32;

            let mut g = vec![vec![0; h]; w];
            for x in 0..w {
                for y in 0..h {
                    let ros_val = map.grid.data[y * w + x];
                    g[x][y] = if ros_val > 50 || ros_val == -1 { 1 } else { 0 };
                }
            }
            (w, h, r, ox, oy, g)
        } else {
            let mut min_x = f32::MAX;
            let mut min_y = f32::MAX;
            let mut max_x = f32::MIN;
            let mut max_y = f32::MIN;

            for id in robot_ids {
                if let Some(odom) = starts.get(id) {
                    let x = odom.pose.pose.position.x as f32;
                    let y = odom.pose.pose.position.y as f32;
                    if x < min_x {
                        min_x = x;
                    }
                    if y < min_y {
                        min_y = y;
                    }
                    if x > max_x {
                        max_x = x;
                    }
                    if y > max_y {
                        max_y = y;
                    }
                }
                if let Some(dest) = goals.get(id) {
                    if let Some(region) = dest.constraints.regions.first() {
                        if region.region.points.len() >= 2 {
                            let x = region.region.points[0];
                            let y = region.region.points[1];
                            if x < min_x {
                                min_x = x;
                            }
                            if y < min_y {
                                min_y = y;
                            }
                            if x > max_x {
                                max_x = x;
                            }
                            if y > max_y {
                                max_y = y;
                            }
                        }
                    }
                }
            }

            if min_x == f32::MAX {
                min_x = 0.0;
                min_y = 0.0;
                max_x = 0.0;
                max_y = 0.0;
            }

            let padding = 10.0;
            let ox = min_x.floor() - padding;
            let oy = min_y.floor() - padding;

            let w = (max_x.ceil() - ox + padding) as usize;
            let h = (max_y.ceil() - oy + padding) as usize;

            let w = w.max(1);
            let h = h.max(1);

            let g = vec![vec![0; h]; w];
            (w, h, 1.0f32, ox, oy, g)
        };

        let mut grid_starts = Vec::new();
        let mut grid_ends = Vec::new();

        for id in robot_ids {
            let odom = starts.get(id).ok_or_else(|| {
                Box::<dyn std::error::Error>::from(format!(
                    "Missing start odometry for agent {}",
                    id
                ))
            })?;
            let dest = goals.get(id).ok_or_else(|| {
                Box::<dyn std::error::Error>::from(format!(
                    "Missing goal destination for agent {}",
                    id
                ))
            })?;

            let sx = ((odom.pose.pose.position.x as f32 - offset_x) / resolution).round() as usize;
            let sy = ((odom.pose.pose.position.y as f32 - offset_y) / resolution).round() as usize;

            let gx_f32;
            let gy_f32;

            if let Some(region) = dest.constraints.regions.first() {
                if region.region.points.len() >= 2 {
                    gx_f32 = region.region.points[0];
                    gy_f32 = region.region.points[1];
                } else {
                    return Err(Box::<dyn std::error::Error>::from(format!(
                        "Destination for agent {} has invalid region points",
                        id
                    )));
                }
            } else {
                return Err(Box::<dyn std::error::Error>::from(format!(
                    "Destination for agent {} has no target regions",
                    id
                )));
            }

            let gx = ((gx_f32 - offset_x) / resolution).round() as usize;
            let gy = ((gy_f32 - offset_y) / resolution).round() as usize;

            let sx = sx.min(width - 1);
            let sy = sy.min(height - 1);
            let gx = gx.min(width - 1);
            let gy = gy.min(height - 1);

            grid_starts.push((sx, sy));
            grid_ends.push((gx, gy));
        }

        let mut solver = PiBTWithExternalTracks::init(grid);
        let external_tracks = Vec::new();
        let solved_paths =
            match solver.solve(&grid_starts, &grid_ends, &external_tracks, self.max_time) {
                Ok(paths) => paths,
                Err(_) => return Err(Box::<dyn std::error::Error>::from("PIBT failed to solve")),
            };

        let mut trajectories = vec![Vec::new(); robot_ids.len()];
        for time_step in solved_paths {
            for (agent_idx, pos) in time_step.iter().enumerate() {
                let world_x = pos.0 as f32 * resolution + offset_x;
                let world_y = pos.1 as f32 * resolution + offset_y;
                trajectories[agent_idx].push(Isometry2::translation(world_x, world_y));
            }
        }

        Ok(trajectories)
    }
}

impl MapfPlanner for Box<dyn MapfPlanner> {
    fn plan(
        &self,
        starts: &HashMap<String, Odometry>,
        goals: &HashMap<String, Destination>,
        footprints: &HashMap<String, Arc<dyn mapf_post::shape::Shape>>,
        robot_ids: &[String],
        map: &Map,
        cancellation: Arc<AtomicBool>,
    ) -> Result<Vec<Vec<Isometry2<f32>>>, Box<dyn std::error::Error>> {
        self.as_ref()
            .plan(starts, goals, footprints, robot_ids, map, cancellation)
    }
}

impl MapfPlanner for Arc<dyn MapfPlanner> {
    fn plan(
        &self,
        starts: &HashMap<String, Odometry>,
        goals: &HashMap<String, Destination>,
        footprints: &HashMap<String, Arc<dyn mapf_post::shape::Shape>>,
        robot_ids: &[String],
        map: &Map,
        cancellation: Arc<AtomicBool>,
    ) -> Result<Vec<Vec<Isometry2<f32>>>, Box<dyn std::error::Error>> {
        self.as_ref()
            .plan(starts, goals, footprints, robot_ids, map, cancellation)
    }
}

#[derive(Clone)]
pub struct CcbsPlanner {
    pub queue_length_limit: Option<usize>,
}

impl Default for CcbsPlanner {
    fn default() -> Self {
        Self {
            queue_length_limit: Some(100_000),
        }
    }
}

impl CcbsPlanner {
    pub fn new(queue_length_limit: Option<usize>) -> Self {
        Self { queue_length_limit }
    }
}

impl MapfPlanner for CcbsPlanner {
    fn plan(
        &self,
        starts: &HashMap<String, Odometry>,
        goals: &HashMap<String, Destination>,
        _footprints: &HashMap<String, Arc<dyn mapf_post::shape::Shape>>,
        robot_ids: &[String],
        map: &Map,
        _cancellation: Arc<AtomicBool>,
    ) -> Result<Vec<Vec<Isometry2<f32>>>, Box<dyn std::error::Error>> {
        if robot_ids.is_empty() {
            return Ok(Vec::new());
        }

        let cell_size = if map.grid.info.resolution > 0.0 {
            map.grid.info.resolution as f64
        } else {
            1.0
        };

        let origin_x = map.grid.info.origin.position.x as f64;
        let origin_y = map.grid.info.origin.position.y as f64;

        let mut occupancy: HashMap<i64, Vec<i64>> = HashMap::new();
        let w = map.grid.info.width as usize;
        let h = map.grid.info.height as usize;
        if w > 0 && h > 0 {
            for x in 0..w {
                for y in 0..h {
                    let ros_val = map.grid.data[y * w + x];
                    if ros_val > 50 || ros_val == -1 {
                        occupancy.entry(y as i64).or_default().push(x as i64);
                    }
                }
            }
            for row in occupancy.values_mut() {
                row.sort_unstable();
            }
        }

        let mut agents = std::collections::BTreeMap::new();

        for id in robot_ids {
            let odom = starts.get(id).ok_or_else(|| {
                Box::<dyn std::error::Error>::from(format!(
                    "Missing start odometry for agent {}",
                    id
                ))
            })?;
            let dest = goals.get(id).ok_or_else(|| {
                Box::<dyn std::error::Error>::from(format!(
                    "Missing goal destination for agent {}",
                    id
                ))
            })?;

            let sx_world = odom.pose.pose.position.x as f64;
            let sy_world = odom.pose.pose.position.y as f64;
            let sx_local = sx_world - origin_x;
            let sy_local = sy_world - origin_y;

            let q = &odom.pose.pose.orientation;
            let yaw = 2.0 * f64::atan2(q.z as f64, q.w as f64);

            let start_cell = Cell::from_point(Point::new(sx_local, sy_local), cell_size);

            let gx_world: f64;
            let gy_world: f64;

            if let Some(region) = dest.constraints.regions.first() {
                if region.region.points.len() >= 2 {
                    gx_world = region.region.points[0] as f64;
                    gy_world = region.region.points[1] as f64;
                } else {
                    return Err(Box::<dyn std::error::Error>::from(format!(
                        "Destination for agent {} has invalid region points",
                        id
                    )));
                }
            } else {
                return Err(Box::<dyn std::error::Error>::from(format!(
                    "Destination for agent {} has no target regions",
                    id
                )));
            }

            let gx_local = gx_world - origin_x;
            let gy_local = gy_world - origin_y;
            let goal_cell = Cell::from_point(Point::new(gx_local, gy_local), cell_size);

            agents.insert(
                id.clone(),
                Agent {
                    start: [start_cell.x, start_cell.y],
                    yaw,
                    goal: [goal_cell.x, goal_cell.y],
                    radius: 0.45,
                    speed: 0.75,
                    spin: 60.0_f64.to_radians(),
                },
            );
        }

        let scenario = Scenario {
            agents,
            obstacles: Vec::new(),
            occupancy,
            cell_size,
            camera_bounds: None,
        };

        let (solution_node, _node_history, name_map) =
            match negotiate(&scenario, self.queue_length_limit) {
                Ok(solution) => solution,
                Err(err) => {
                    return Err(Box::<dyn std::error::Error>::from(format!(
                        "CCBS negotiation failed: {:?}",
                        err
                    )));
                }
            };

        let mut trajectories = vec![Vec::new(); robot_ids.len()];

        for (agent_idx, id) in robot_ids.iter().enumerate() {
            let internal_idx = name_map
                .iter()
                .find(|(_, name)| *name == id)
                .map(|(idx, _)| *idx)
                .ok_or_else(|| {
                    Box::<dyn std::error::Error>::from(format!(
                        "Failed to find name map entry for agent {}",
                        id
                    ))
                })?;

            let proposal = solution_node.proposals.get(&internal_idx).ok_or_else(|| {
                Box::<dyn std::error::Error>::from(format!(
                    "Missing proposal solution for agent {}",
                    id
                ))
            })?;

            for wp in proposal.meta.trajectory.iter() {
                let world_x = (wp.position.translation.x + origin_x) as f32;
                let world_y = (wp.position.translation.y + origin_y) as f32;
                let angle = wp.position.rotation.angle() as f32;

                trajectories[agent_idx].push(Isometry2::new(
                    mapf_post::na::Vector2::new(world_x, world_y),
                    angle,
                ));
            }
        }

        Ok(trajectories)
    }
}
