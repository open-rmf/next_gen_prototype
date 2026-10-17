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
use mapf_post::na::Isometry2;
use ros_env::nav_msgs::msg::OccupancyGrid;
use ros_env::rmf_prototype_msgs::msg::{Destination, RobotStatus};
use std::collections::HashMap;

use std::sync::atomic::AtomicBool;
use std::sync::Arc;

pub(crate) const DEFAULT_PLANNING_GRID_RESOLUTION: f64 = 1.0;

#[derive(Clone, Debug, Default)]
pub struct Map {
    pub grid: OccupancyGrid,
}

impl Map {
    pub fn is_usable(&self) -> bool {
        self.grid.info.width > 0 && self.grid.info.height > 0 && self.grid.info.resolution > 0.0
    }

    fn cell(&self, point: [f32; 2]) -> Option<(usize, usize)> {
        let resolution = self.grid.info.resolution;
        let x = (point[0] - self.grid.info.origin.position.x as f32) / resolution;
        let y = (point[1] - self.grid.info.origin.position.y as f32) / resolution;
        if x < 0.0 || y < 0.0 {
            return None;
        }
        let (x, y) = (x.round() as usize, y.round() as usize);
        if x >= self.grid.info.width as usize || y >= self.grid.info.height as usize {
            return None;
        }
        Some((x, y))
    }

    pub fn mark(&mut self, path: &[[f32; 2]]) {
        if !self.is_usable() {
            return;
        }
        let Some(first) = path.first() else {
            return;
        };

        self.occupy(*first);
        let resolution = self.grid.info.resolution;
        for pair in path.windows(2) {
            let (from, to) = (pair[0], pair[1]);
            let (dx, dy) = (to[0] - from[0], to[1] - from[1]);
            let steps = ((dx.hypot(dy) / (resolution * 0.25)).ceil() as usize).max(1);
            for step in 1..=steps {
                let t = step as f32 / steps as f32;
                self.occupy([from[0] + dx * t, from[1] + dy * t]);
            }
        }
    }

    fn occupy(&mut self, point: [f32; 2]) {
        let Some((x, y)) = self.cell(point) else {
            return;
        };
        let width = self.grid.info.width as usize;
        if let Some(cell) = self.grid.data.get_mut(y * width + x) {
            *cell = 100;
        }
    }
}

/// Implement this trait to use your own custom MAPF
/// planner. The planner in this scenario will take in
/// starts and goals and assign a trajectory to the agents.
pub trait MapfPlanner: Send + Sync + 'static {
    fn plan(
        &self,
        starts: &HashMap<String, RobotStatus>,
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
        _starts: &HashMap<String, RobotStatus>,
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
    pub grid_resolution: f32,
}

impl Default for PibtPlanner {
    fn default() -> Self {
        Self {
            max_time: 100,
            grid_resolution: DEFAULT_PLANNING_GRID_RESOLUTION as f32,
        }
    }
}

impl PibtPlanner {
    pub fn new(max_time: usize) -> Self {
        Self {
            max_time,
            ..Default::default()
        }
    }

    pub fn with_grid_resolution(max_time: usize, grid_resolution: f64) -> Result<Self, String> {
        if !grid_resolution.is_finite()
            || grid_resolution <= 0.0
            || grid_resolution > f32::MAX as f64
        {
            return Err(format!(
                "planning_grid_resolution must be finite and positive, got {grid_resolution}"
            ));
        }

        Ok(Self {
            max_time,
            grid_resolution: grid_resolution as f32,
        })
    }
}

fn planning_grid(
    grid: &OccupancyGrid,
    minimum_resolution: f32,
) -> (usize, usize, f32, f32, f32, Vec<Vec<usize>>) {
    let source_width = grid.info.width as usize;
    let source_height = grid.info.height as usize;
    let source_resolution = grid.info.resolution;
    let resolution = source_resolution.max(minimum_resolution);
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
        starts: &HashMap<String, RobotStatus>,
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
            planning_grid(&map.grid, self.grid_resolution)
        } else {
            let mut min_x = f32::MAX;
            let mut min_y = f32::MAX;
            let mut max_x = f32::MIN;
            let mut max_y = f32::MIN;

            for id in robot_ids {
                if let Some(status) = starts.get(id) {
                    let x = status.pose.pose.position.x as f32;
                    let y = status.pose.pose.position.y as f32;
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

            let resolution = self.grid_resolution;
            let padding = 10.0;
            let ox = (min_x / resolution).floor() * resolution - padding;
            let oy = (min_y / resolution).floor() * resolution - padding;

            let w = ((max_x - ox + padding) / resolution).ceil() as usize;
            let h = ((max_y - oy + padding) / resolution).ceil() as usize;

            let w = w.max(1);
            let h = h.max(1);

            let g = vec![vec![0; h]; w];
            (w, h, resolution, ox, oy, g)
        };

        let mut grid_starts = Vec::new();
        let mut grid_ends = Vec::new();

        for id in robot_ids {
            let status = starts.get(id).ok_or_else(|| {
                Box::<dyn std::error::Error>::from(format!("Missing start status for agent {}", id))
            })?;
            let dest = goals.get(id).ok_or_else(|| {
                Box::<dyn std::error::Error>::from(format!(
                    "Missing goal destination for agent {}",
                    id
                ))
            })?;

            let sx =
                ((status.pose.pose.position.x as f32 - offset_x) / resolution).round() as usize;
            let sy =
                ((status.pose.pose.position.y as f32 - offset_y) / resolution).round() as usize;

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

        // Clamp the final waypoint to exact goal coordinates to prevent coarse grid discretization errors
        for (agent_idx, id) in robot_ids.iter().enumerate() {
            if let Some(dest) = goals.get(id) {
                if let Some(region) = dest.constraints.regions.first() {
                    if region.region.points.len() >= 2 {
                        let gx_f32 = region.region.points[0];
                        let gy_f32 = region.region.points[1];
                        if let Some(last_pose) = trajectories[agent_idx].last_mut() {
                            last_pose.translation.vector[0] = gx_f32;
                            last_pose.translation.vector[1] = gy_f32;
                        }
                    }
                }
            }
        }

        Ok(trajectories)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use ros_env::rmf_prototype_msgs::msg::{Region, TargetRegion};

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

        let (width, height, resolution, offset_x, offset_y, cells) =
            planning_grid(&map, DEFAULT_PLANNING_GRID_RESOLUTION as f32);

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

        let (width, height, resolution, _, _, cells) =
            planning_grid(&map, DEFAULT_PLANNING_GRID_RESOLUTION as f32);

        assert_eq!(width, 2);
        assert_eq!(height, 2);
        assert_eq!(resolution, 2.0);
        assert_eq!(cells, vec![vec![0, 0], vec![1, 0]]);
    }

    #[test]
    fn configured_grid_resolution_is_used() {
        let mut map = OccupancyGrid::default();
        map.info.resolution = 0.25;
        map.info.width = 8;
        map.info.height = 8;
        map.data = vec![0; 64];

        let (width, height, resolution, _, _, _) = planning_grid(&map, 0.5);

        assert_eq!(width, 4);
        assert_eq!(height, 4);
        assert_eq!(resolution, 0.5);
    }

    #[test]
    fn grid_resolution_must_be_positive() {
        assert!(PibtPlanner::with_grid_resolution(100, 0.5).is_ok());
        assert!(PibtPlanner::with_grid_resolution(100, 0.0).is_err());
        assert!(PibtPlanner::with_grid_resolution(100, -0.5).is_err());
    }

    #[test]
    fn final_waypoint_clamped_to_exact_goal_coordinates() {
        let planner = PibtPlanner::default();
        let mut starts = HashMap::new();
        let mut status = RobotStatus::default();
        status.pose.pose.position.x = 0.0;
        status.pose.pose.position.y = 0.0;
        starts.insert("robot_1".to_string(), status);

        let mut goals = HashMap::new();
        let mut dest = Destination::default();
        dest.constraints.regions.push(TargetRegion {
            region: Region {
                points: vec![2.35, 1.75],
                hint: Region::HINT_POINT,
            },
            ..Default::default()
        });
        goals.insert("robot_1".to_string(), dest);

        let footprints = HashMap::new();
        let robot_ids = vec!["robot_1".to_string()];
        let map = Map::default();
        let cancellation = Arc::new(AtomicBool::new(false));

        let trajectories = planner
            .plan(&starts, &goals, &footprints, &robot_ids, &map, cancellation)
            .unwrap();
        let last_pose = trajectories[0].last().unwrap();
        assert!((last_pose.translation.x - 2.35).abs() < 1e-6);
        assert!((last_pose.translation.y - 1.75).abs() < 1e-6);
    }

    #[test]
    fn marked_path_is_routed_around_by_pibt() {
        let mut map = Map::default();
        map.grid.info.resolution = 1.0;
        map.grid.info.width = 5;
        map.grid.info.height = 3;
        map.grid.data = vec![0; 15];

        // Mark action segment from (2.0, 1.0) to (2.0, 2.0)
        map.mark(&[[2.0, 1.0], [2.0, 2.0]]);

        let mut starts = HashMap::new();
        let mut status = RobotStatus::default();
        status.pose.pose.position.x = 0.0;
        status.pose.pose.position.y = 1.0;
        starts.insert("mover".to_string(), status);

        let mut goals = HashMap::new();
        let mut dest = Destination::default();
        dest.constraints.regions.push(TargetRegion {
            region: Region {
                points: vec![4.0, 1.0],
                hint: Region::HINT_POINT,
            },
            ..Default::default()
        });
        goals.insert("mover".to_string(), dest);

        let trajectories = PibtPlanner::new(50)
            .plan(
                &starts,
                &goals,
                &HashMap::new(),
                &["mover".to_string()],
                &map,
                Arc::new(AtomicBool::new(false)),
            )
            .expect("planner should route around the marked path");

        let path = &trajectories[0];
        for pose in &path[..path.len() - 1] {
            let at_mark =
                (pose.translation.x - 2.0).abs() < 0.01 && (pose.translation.y - 1.0).abs() < 0.01;
            assert!(!at_mark, "mover drove through marked space: {:?}", path);
        }

        let end = path.last().unwrap();
        assert!((end.translation.x - 4.0).abs() < 0.01);
        assert!((end.translation.y - 1.0).abs() < 0.01);
    }
}
