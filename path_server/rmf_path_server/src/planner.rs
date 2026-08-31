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
use mapf::motion::{Motion, TimePoint};
use mapf::negotiation::{negotiate, Agent, Scenario};
use mapf_post::na::Isometry2;
use ros_env::nav_msgs::msg::{OccupancyGrid, Odometry};
use ros_env::rmf_prototype_msgs::msg::Destination;
use std::collections::HashMap;

use std::sync::atomic::AtomicBool;
use std::sync::Arc;

const MIN_PLANNING_RESOLUTION: f32 = 1.0;

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

fn planning_grid(grid: &OccupancyGrid) -> (usize, usize, f32, f32, f32, Vec<Vec<usize>>) {
    let source_width = grid.info.width as usize;
    let source_height = grid.info.height as usize;
    let source_resolution = grid.info.resolution;
    let resolution = source_resolution.max(MIN_PLANNING_RESOLUTION);
    let width = ((source_width as f32 * source_resolution) / resolution)
        .ceil()
        .max(1.0) as usize;
    let height = ((source_height as f32 * source_resolution) / resolution)
        .ceil()
        .max(1.0) as usize;
    let mut cells = vec![vec![0; height]; width];

    for source_x in 0..source_width {
        for source_y in 0..source_height {
            let value = grid
                .data
                .get(source_y * source_width + source_x)
                .copied()
                .unwrap_or(-1);
            if value > 50 || value == -1 {
                let x = ((source_x as f32 * source_resolution) / resolution).floor() as usize;
                let y = ((source_y as f32 * source_resolution) / resolution).floor() as usize;
                cells[x.min(width - 1)][y.min(height - 1)] = 1;
            }
        }
    }

    (
        width,
        height,
        resolution,
        grid.info.origin.position.x as f32,
        grid.info.origin.position.y as f32,
        cells,
    )
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
            planning_grid(&map.grid)
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
    pub cell_size: Option<f64>,
    pub discretization_timestep: Option<f64>,
}

impl Default for CcbsPlanner {
    fn default() -> Self {
        Self {
            queue_length_limit: Some(100_000),
            cell_size: None,
            discretization_timestep: Some(1.0),
        }
    }
}

impl CcbsPlanner {
    pub fn new(queue_length_limit: Option<usize>) -> Self {
        Self {
            queue_length_limit,
            cell_size: None,
            discretization_timestep: Some(1.0),
        }
    }

    pub fn with_cell_size(mut self, cell_size: f64) -> Self {
        self.cell_size = Some(cell_size);
        self
    }

    pub fn with_discretization_timestep(mut self, timestep: f64) -> Self {
        self.discretization_timestep = Some(timestep);
        self
    }
}

impl MapfPlanner for CcbsPlanner {
    fn plan(
        &self,
        starts: &HashMap<String, Odometry>,
        goals: &HashMap<String, Destination>,
        footprints: &HashMap<String, Arc<dyn mapf_post::shape::Shape>>,
        robot_ids: &[String],
        map: &Map,
        _cancellation: Arc<AtomicBool>,
    ) -> Result<Vec<Vec<Isometry2<f32>>>, Box<dyn std::error::Error>> {
        if robot_ids.is_empty() {
            return Ok(Vec::new());
        }

        let cell_size = self.cell_size.unwrap_or_else(|| {
            if map.grid.info.resolution > 0.0 {
                map.grid.info.resolution as f64
            } else {
                1.0
            }
        });

        let map_res = if map.grid.info.resolution > 0.0 {
            map.grid.info.resolution as f64
        } else {
            cell_size
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
                        let wx = origin_x + x as f64 * map_res;
                        let wy = origin_y + y as f64 * map_res;
                        let cx = ((wx - origin_x) / cell_size).round() as i64;
                        let cy = ((wy - origin_y) / cell_size).round() as i64;
                        occupancy.entry(cy).or_default().push(cx);
                    }
                }
            }

            // Mark a boundary ring of cells just outside the grid bounds as occupied
            // so agents do not navigate outside the mapped area.
            let max_cx = ((w as f64 * map_res) / cell_size).round() as i64 - 1;
            let max_cy = ((h as f64 * map_res) / cell_size).round() as i64 - 1;
            let min_cx = 0i64;
            let min_cy = 0i64;

            for cx in (min_cx - 1)..=(max_cx + 1) {
                occupancy.entry(min_cy - 1).or_default().push(cx);
                occupancy.entry(max_cy + 1).or_default().push(cx);
            }
            for cy in min_cy..=max_cy {
                occupancy.entry(cy).or_default().push(min_cx - 1);
                occupancy.entry(cy).or_default().push(max_cx + 1);
            }

            for row in occupancy.values_mut() {
                row.sort_unstable();
                row.dedup();
            }
        }

        let mut agents = std::collections::BTreeMap::new();
        let (min_cx, max_cx, min_cy, max_cy) = if w > 0 && h > 0 {
            (
                0i64,
                ((w as f64 * map_res) / cell_size).round() as i64 - 1,
                0i64,
                ((h as f64 * map_res) / cell_size).round() as i64 - 1,
            )
        } else {
            (i64::MIN, i64::MAX, i64::MIN, i64::MAX)
        };

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
            let mut start_cx = ((sx_world - origin_x) / cell_size).round() as i64;
            let mut start_cy = ((sy_world - origin_y) / cell_size).round() as i64;

            if w > 0 && h > 0 {
                start_cx = start_cx.clamp(min_cx, max_cx);
                start_cy = start_cy.clamp(min_cy, max_cy);

                // If the robot's current position coincides with an occupied cell,
                // unmark it so the robot can plan out of the collision state.
                if let Some(row) = occupancy.get_mut(&start_cy) {
                    row.retain(|&x| x != start_cx);
                }
            }

            let q = &odom.pose.pose.orientation;
            let yaw = 2.0 * f64::atan2(q.z as f64, q.w as f64);

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

            let mut goal_cx = ((gx_world - origin_x) / cell_size).round() as i64;
            let mut goal_cy = ((gy_world - origin_y) / cell_size).round() as i64;

            if w > 0 && h > 0 {
                goal_cx = goal_cx.clamp(min_cx, max_cx);
                goal_cy = goal_cy.clamp(min_cy, max_cy);

                // If the goal is occupied, search for the nearest free cell.
                let is_occupied = |x: i64, y: i64| -> bool {
                    occupancy.get(&y).map_or(false, |row| row.contains(&x))
                };

                if is_occupied(goal_cx, goal_cy) {
                    'search: for r in 1i64..=10i64 {
                        for dx in -r..=r {
                            for dy in -r..=r {
                                if dx.abs() == r || dy.abs() == r {
                                    let nx = (goal_cx + dx).clamp(min_cx, max_cx);
                                    let ny = (goal_cy + dy).clamp(min_cy, max_cy);
                                    if !is_occupied(nx, ny) {
                                        goal_cx = nx;
                                        goal_cy = ny;
                                        break 'search;
                                    }
                                }
                            }
                        }
                    }
                }
            }

            let radius = footprints
                .get(id)
                .and_then(|shape| {
                    if let Some(ball) = shape.as_ball() {
                        Some(ball.radius as f64)
                    } else {
                        let aabb = shape.compute_local_aabb();
                        Some((aabb.maxs.x.max(aabb.maxs.y)) as f64)
                    }
                })
                .filter(|r| *r > 0.0)
                .unwrap_or(0.45);

            agents.insert(
                id.clone(),
                Agent {
                    start: [start_cx, start_cy],
                    yaw,
                    goal: [goal_cx, goal_cy],
                    radius,
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

        let time_step = self.discretization_timestep.unwrap_or(1.0);
        let max_finish_time = solution_node
            .proposals
            .values()
            .map(|p| p.meta.trajectory.finish_motion_time().as_secs_f64())
            .fold(0.0f64, f64::max);

        let num_steps = if max_finish_time > 0.0 {
            (max_finish_time / time_step).ceil() as usize + 1
        } else {
            1
        }
        .max(2);

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

            let motion = proposal.meta.trajectory.motion();

            for step in 0..num_steps {
                let t = (step as f64 * time_step).min(max_finish_time);
                let time_point = TimePoint::from_secs_f64(t);
                let pos = motion
                    .compute_position(&time_point)
                    .unwrap_or_else(|_| proposal.meta.trajectory.finish_motion().position);

                let world_x = (pos.translation.x + origin_x - 0.5 * cell_size) as f32;
                let world_y = (pos.translation.y + origin_y - 0.5 * cell_size) as f32;
                let angle = pos.rotation.angle() as f32;

                trajectories[agent_idx].push(Isometry2::new(
                    mapf_post::na::Vector2::new(world_x, world_y),
                    angle,
                ));
            }
        }

        Ok(trajectories)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fine_occupancy_cells_are_conservatively_downsampled() {
        let mut map = OccupancyGrid::default();
        map.info.resolution = 0.25;
        map.info.width = 8;
        map.info.height = 4;
        map.info.origin.position.x = -1.0;
        map.info.origin.position.y = -2.0;
        map.data = vec![0; 32];
        map.data[2 * 8 + 5] = 100;

        let (width, height, resolution, offset_x, offset_y, cells) = planning_grid(&map);

        assert_eq!(width, 2);
        assert_eq!(height, 1);
        assert_eq!(resolution, 1.0);
        assert_eq!(offset_x, -1.0);
        assert_eq!(offset_y, -2.0);
        assert_eq!(cells, vec![vec![0], vec![1]]);
    }

    #[test]
    fn native_grid_is_kept_when_it_is_already_coarse() {
        let mut map = OccupancyGrid::default();
        map.info.resolution = 2.0;
        map.info.width = 2;
        map.info.height = 2;
        map.data = vec![0, 100, 0, 0];

        let (width, height, resolution, _, _, cells) = planning_grid(&map);

        assert_eq!(width, 2);
        assert_eq!(height, 2);
        assert_eq!(resolution, 2.0);
        assert_eq!(cells, vec![vec![0, 0], vec![1, 0]]);
    }
}
