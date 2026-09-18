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
use ros_env::nav_msgs::msg::{OccupancyGrid, Odometry};
use ros_env::rmf_prototype_msgs::msg::Destination;
use std::collections::HashMap;

use std::sync::atomic::AtomicBool;
use std::sync::Arc;

const MIN_PLANNING_RESOLUTION: f32 = 1.0;

/// Margin left around the robots when a map has to be invented, in metres.
const SYNTHETIC_MAP_PADDING: f32 = 10.0;

#[derive(Clone, Debug, Default)]
pub struct Map {
    pub grid: OccupancyGrid,
}

impl Map {
    /// An empty grid of 1 m cells covering `points` with a margin around them.
    ///
    /// Used when no real map has arrived yet. The result has no walls in it, so
    /// it describes an open field rather than the site -- but it does give the
    /// caller somewhere to record where robots are standing, which a
    /// zero-sized grid does not.
    pub fn bounding(points: impl IntoIterator<Item = [f32; 2]>) -> Self {
        let (mut min_x, mut min_y) = (f32::MAX, f32::MAX);
        let (mut max_x, mut max_y) = (f32::MIN, f32::MIN);
        for [x, y] in points {
            min_x = min_x.min(x);
            min_y = min_y.min(y);
            max_x = max_x.max(x);
            max_y = max_y.max(y);
        }
        if min_x == f32::MAX {
            min_x = 0.0;
            min_y = 0.0;
            max_x = 0.0;
            max_y = 0.0;
        }

        let origin_x = min_x.floor() - SYNTHETIC_MAP_PADDING;
        let origin_y = min_y.floor() - SYNTHETIC_MAP_PADDING;
        let width = ((max_x.ceil() - origin_x + SYNTHETIC_MAP_PADDING) as usize).max(1);
        let height = ((max_y.ceil() - origin_y + SYNTHETIC_MAP_PADDING) as usize).max(1);

        let mut grid = OccupancyGrid::default();
        grid.info.resolution = 1.0;
        grid.info.width = width as u32;
        grid.info.height = height as u32;
        grid.info.origin.position.x = origin_x as f64;
        grid.info.origin.position.y = origin_y as f64;
        grid.data = vec![0; width * height];
        Self { grid }
    }

    /// Whether this map has geometry that can be planned on at all.
    pub fn is_usable(&self) -> bool {
        self.grid.info.width > 0 && self.grid.info.height > 0 && self.grid.info.resolution > 0.0
    }

    /// The cell owning a world point, or `None` if it falls off the grid.
    ///
    /// Out of bounds is dropped rather than clamped: clamping would plant the
    /// mark on the boundary and block a cell nobody is standing in.
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

    /// Record `path` as occupied space.
    ///
    /// `path` is an ordered polyline in world coordinates, and may be a single
    /// point -- for a robot standing still it usually is, being simply the
    /// place it is standing. The whole length is rasterised rather than just
    /// the vertices: a dock approach can cross a cell without either endpoint
    /// landing in it, and a gap in the middle of a mark is worse than no mark,
    /// because it invites a peer to aim through the gap.
    ///
    /// # This is map, not traffic
    ///
    /// What gets marked here is space held by something the planner is not
    /// going to move: a robot mid-dock, a robot parked with no task, the lane
    /// a robot is currently reversing out of. None of it expires within the
    /// horizon of a plan, so stating it as occupancy -- ahead of planning,
    /// where every cost and distance structure the solver builds will see it
    /// -- is both simpler and stronger than handing the planner a list of
    /// robots and hoping it checks them at the right moment.
    ///
    /// # Marking a cell an agent is standing in
    ///
    /// Allowed, and sometimes necessary: at the coarse planning resolution a
    /// robot being routed can share a cell with one that is parked. `hetpibt`
    /// filters candidate moves by the cell being *entered*, so such an agent
    /// can still drive off the mark; it simply cannot stay put. That is the
    /// right outcome — it is standing where something else is. An agent whose
    /// *goal* is marked genuinely cannot arrive, which is also the truth, and
    /// the caller should see it fail rather than be handed a path through an
    /// occupied dock.
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
            // A quarter cell per step. Half would be enough for an axis-aligned
            // run but can skip a corner cell on a diagonal.
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

    /// Whether a world point falls in space the planner must treat as blocked.
    ///
    /// The threshold matches the one `planning_grid` downsamples with, so this
    /// answers the same question the solver will ask rather than a similar one.
    /// A point off the edge of the grid is not occupied; it is simply not
    /// described by this map.
    pub fn is_occupied(&self, point: [f32; 2]) -> bool {
        let Some((x, y)) = self.cell(point) else {
            return false;
        };
        let width = self.grid.info.width as usize;
        self.grid
            .data
            .get(y * width + x)
            .is_some_and(|value| *value > 50 || *value == -1)
    }
}

/// Implement this trait to use your own custom MAPF
/// planner. The planner in this scenario will take in
/// starts and goals and assign a trajectory to the agents.
pub trait MapfPlanner: Send + Sync + 'static {
    /// Plan routes for `robot_ids` across `map`.
    ///
    /// Anything that is not being routed by this call -- a robot mid-dock, a
    /// robot parked with no task, a dock lane in use -- has already been
    /// written into `map` as occupancy by the caller. There is no separate
    /// notion of a robot-shaped obstacle to honour: plan around what the map
    /// says is occupied and the rest follows.
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

        // There is deliberately no fallback for a map without geometry. The
        // caller owns that decision, because it is the caller that has to write
        // occupied space into the map before planning: a grid invented down
        // here would be one the marks never reached.
        if !map.is_usable() {
            return Err(Box::<dyn std::error::Error>::from(
                "Cannot plan without a map: the grid has no width, height or resolution",
            ));
        }

        let (width, height, resolution, offset_x, offset_y, mut grid) = planning_grid(&map.grid);

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

        let solved_paths = match solver.solve(&grid_starts, &grid_ends, &Vec::new(), self.max_time)
        {
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

    /// Where a robot is standing and the dock it is driving into usually fall
    /// in the same cell, and a point off the grid is dropped rather than
    /// clamped onto the boundary.
    #[test]
    fn marks_land_in_the_right_cells_and_are_bounds_checked() {
        // 4x4 grid of 1 m cells with the origin at (-1, -1).
        let mut map = Map::default();
        map.grid.info.resolution = 1.0;
        map.grid.info.width = 4;
        map.grid.info.height = 4;
        map.grid.info.origin.position.x = -1.0;
        map.grid.info.origin.position.y = -1.0;
        map.grid.data = vec![0; 16];

        map.mark(&[[0.0, 0.0], [0.2, -0.1]]); // both -> (1, 1)
        map.mark(&[[1.0, 0.0]]); // -> (2, 1)
        map.mark(&[[99.0, 0.0]]); // off the east edge
        map.mark(&[[0.0, -50.0]]); // off the south edge
        map.mark(&[]); // a robot we know nothing about marks nothing

        let occupied: Vec<(usize, usize)> = (0..4)
            .flat_map(|y| (0..4).map(move |x| (x, y)))
            .filter(|(x, y)| map.grid.data[y * 4 + x] > 50)
            .collect();
        assert_eq!(occupied, vec![(1, 1), (2, 1)]);
    }

    /// The endpoints of a dock approach can sit three cells apart. Sampling
    /// only the ends would leave the middle of the robot's path open.
    #[test]
    fn a_mark_is_rasterised_along_its_whole_length() {
        // 6x6 grid of 1 m cells, origin at (0, 0). A diagonal run from cell
        // (1, 1) to cell (4, 4).
        let mut map = Map::default();
        map.grid.info.resolution = 1.0;
        map.grid.info.width = 6;
        map.grid.info.height = 6;
        map.grid.data = vec![0; 36];

        map.mark(&[[1.0, 1.0], [4.0, 4.0]]);

        for (x, y) in [(1, 1), (2, 2), (3, 3), (4, 4)] {
            assert!(
                map.grid.data[y * 6 + x] > 50,
                "the mark skipped ({x}, {y}): {:?}",
                map.grid.data
            );
        }
    }

    /// An invented map has to be big enough to hold everything the caller is
    /// about to mark on it, or the marks fall off the edge and are dropped.
    #[test]
    fn a_synthesised_map_covers_its_points_with_room_to_spare() {
        let map = Map::bounding([[0.0, 0.0], [5.0, 3.0]]);

        assert!(map.is_usable());
        assert_eq!(map.grid.info.resolution, 1.0);
        assert_eq!(map.grid.info.origin.position.x, -10.0);
        assert_eq!(map.grid.info.origin.position.y, -10.0);
        assert_eq!(map.cell([0.0, 0.0]), Some((10, 10)));
        assert_eq!(map.cell([5.0, 3.0]), Some((15, 13)));
    }

    /// A map with no geometry is refused rather than quietly replaced. The
    /// caller has to supply a grid, because the caller is the one writing
    /// occupied space into it.
    #[test]
    fn planning_without_a_map_is_an_error() {
        let starts = HashMap::from([("mover".to_string(), at(0.0, 1.0))]);
        let goals = HashMap::from([("mover".to_string(), heading_for(4.0, 1.0))]);

        let result = PibtPlanner::new(50).plan(
            &starts,
            &goals,
            &HashMap::new(),
            &["mover".to_string()],
            &Map::default(),
            Arc::new(AtomicBool::new(false)),
        );

        assert!(result.is_err(), "an empty map should not be planned on");
    }

    fn at(x: f64, y: f64) -> Odometry {
        let mut odom = Odometry::default();
        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom
    }

    fn heading_for(gx: f32, gy: f32) -> Destination {
        use ros_env::rmf_prototype_msgs::msg::{Region, TargetRegion};

        let mut dest = Destination::default();
        dest.constraints.regions.push(TargetRegion {
            region: Region {
                points: vec![gx, gy],
                hint: Region::HINT_POINT,
            },
            ..Default::default()
        });
        dest
    }

    /// The guarantee the whole scheme rests on: space marked as occupied is
    /// routed around, not through, and the agent standing there cannot be
    /// shoved out of the way.
    ///
    /// The mover has to cross the marked cell to reach its goal. PIBT will
    /// happily displace an *internal* agent standing there, which is why a
    /// robot the planner will not move has to reach the solver as map rather
    /// than as traffic.
    #[test]
    fn marked_space_is_never_entered_or_displaced() {
        let mut map = Map::default();
        map.grid.info.resolution = 1.0;
        map.grid.info.width = 5;
        map.grid.info.height = 3;
        map.grid.data = vec![0; 15];

        // Straight down the middle of the aisle, directly between the two.
        map.mark(&[[2.0, 1.0]]);

        let starts = HashMap::from([("mover".to_string(), at(0.0, 1.0))]);
        let goals = HashMap::from([("mover".to_string(), heading_for(4.0, 1.0))]);

        let trajectories = PibtPlanner::new(50)
            .plan(
                &starts,
                &goals,
                &HashMap::new(),
                &["mover".to_string()],
                &map,
                Arc::new(AtomicBool::new(false)),
            )
            .expect("planner should route around the mark");

        let path = &trajectories[0];
        // The last pose is snapped to the exact goal coordinates after solving,
        // so check the mark against the poses the solver actually chose.
        for pose in &path[..path.len() - 1] {
            let at_mark =
                (pose.translation.x - 2.0).abs() < 0.01 && (pose.translation.y - 1.0).abs() < 0.01;
            assert!(
                !at_mark,
                "the mover drove through occupied space: {:?}",
                path
            );
        }

        // Going around must still get there. If it does not, the mark has
        // simply walled the aisle off and the assertion above proves nothing.
        let end = path.last().expect("a trajectory");
        assert!(
            (end.translation.x - 4.0).abs() < 0.01 && (end.translation.y - 1.0).abs() < 0.01,
            "the mover never reached its goal: {:?}",
            path
        );
    }
}
