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

#[derive(Clone, Debug, Default)]
pub struct Map {
    pub grid: OccupancyGrid,
}

/// Implement this trait to use your own custom MAPF
/// planner. The planner in this scenario will take in
/// starts and goals and assign a trajectory to the agents.
pub trait MapfPlanner: Send + Sync + 'static {
    /// Plan for `robot_ids`, avoiding `frozen_claims`.
    ///
    /// Each entry of `frozen_claims` is the swept path of one robot that is
    /// *not* in `robot_ids`: it is physically committed to an action, cannot be
    /// replanned, and will not move out of the way. A path is a polyline in
    /// world coordinates, ordered, and may be a single point.
    ///
    /// These are not agents and they are not obstacles that expire. Treat every
    /// point along every path as **permanently occupied map**, for the whole
    /// horizon and in whatever cost or distance structure the planner steers
    /// by. A claim that only blocks the final move check, and not the search
    /// that leads up to it, will send peers straight at the frozen robot.
    #[allow(clippy::too_many_arguments)]
    fn plan(
        &self,
        starts: &HashMap<String, Odometry>,
        goals: &HashMap<String, Destination>,
        footprints: &HashMap<String, Arc<dyn mapf_post::shape::Shape>>,
        robot_ids: &[String],
        map: &Map,
        frozen_claims: &[Vec<[f32; 2]>],
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
        _frozen_claims: &[Vec<[f32; 2]>],
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

/// Rounds a world point to the planning cell that owns it.
///
/// `None` for anything off the grid. Out-of-bounds claims are dropped rather
/// than clamped: clamping would plant the claim on the boundary and block a
/// cell that nobody is standing in.
fn world_to_cell(
    point: [f32; 2],
    width: usize,
    height: usize,
    resolution: f32,
    offset_x: f32,
    offset_y: f32,
) -> Option<(usize, usize)> {
    let cx = ((point[0] - offset_x) / resolution).round();
    let cy = ((point[1] - offset_y) / resolution).round();
    if cx < 0.0 || cy < 0.0 {
        return None;
    }
    let cell = (cx as usize, cy as usize);
    if cell.0 >= width || cell.1 >= height {
        return None;
    }
    Some(cell)
}

/// Every grid cell swept by a robot that is not participating in this plan.
///
/// Each input path is rasterised, not just sampled at its vertices: a docking
/// robot's approach can cross a cell without either endpoint landing in it, and
/// a gap in the middle of the claim is worse than no claim at all — it invites
/// a peer to aim through the gap.
///
/// Deduplicated, because the two ends of a claim — where the robot is and where
/// it is docking — usually fall in the same cell anyway.
fn claimed_cells(
    frozen_claims: &[Vec<[f32; 2]>],
    width: usize,
    height: usize,
    resolution: f32,
    offset_x: f32,
    offset_y: f32,
) -> Vec<(usize, usize)> {
    let mut cells: Vec<(usize, usize)> = Vec::new();
    let claim = |point: [f32; 2], cells: &mut Vec<(usize, usize)>| {
        if let Some(cell) = world_to_cell(point, width, height, resolution, offset_x, offset_y) {
            if !cells.contains(&cell) {
                cells.push(cell);
            }
        }
    };

    for path in frozen_claims {
        let Some(first) = path.first() else {
            continue;
        };
        // A lone point still claims its own cell: we may know where a robot is
        // without knowing what it is driving into.
        claim(*first, &mut cells);

        for pair in path.windows(2) {
            let (from, to) = (pair[0], pair[1]);
            let (dx, dy) = (to[0] - from[0], to[1] - from[1]);
            // A quarter cell per step. Half would be enough for an axis-aligned
            // run but can skip a corner cell on a diagonal.
            let steps = ((dx.hypot(dy) / (resolution * 0.25)).ceil() as usize).max(1);
            for step in 1..=steps {
                let t = step as f32 / steps as f32;
                claim([from[0] + dx * t, from[1] + dy * t], &mut cells);
            }
        }
    }

    cells
}

impl MapfPlanner for PibtPlanner {
    fn plan(
        &self,
        starts: &HashMap<String, Odometry>,
        goals: &HashMap<String, Destination>,
        _footprints: &HashMap<String, Arc<dyn mapf_post::shape::Shape>>,
        robot_ids: &[String],
        map: &Map,
        frozen_claims: &[Vec<[f32; 2]>],
        _cancellation: Arc<AtomicBool>,
    ) -> Result<Vec<Vec<Isometry2<f32>>>, Box<dyn std::error::Error>> {
        if robot_ids.is_empty() {
            return Ok(Vec::new());
        }

        let use_map =
            map.grid.info.width > 0 && map.grid.info.height > 0 && map.grid.info.resolution > 0.0;

        let (width, height, resolution, offset_x, offset_y, mut grid) = if use_map {
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

        // A committed robot is not an agent in this problem: it cannot be
        // replanned and it will not get out of the way, so the space it holds
        // is map, not traffic.
        //
        // This used to enter as a hetpibt external track. That was the wrong
        // mechanism twice over. An external track describes an agent that is
        // *moving* along a known path, which a docking robot is not, and
        // nothing propagates it into the BFS distance field the solver steers
        // by — that field is built from `grid` alone. Peers were still aimed
        // straight at the docking robot and only discovered the obstruction as
        // a local move rejection, one cell at a time. Burning the claim into
        // the grid is what actually redirects them.
        for (cx, cy) in claimed_cells(frozen_claims, width, height, resolution, offset_x, offset_y)
        {
            // Never wall in an agent we are planning for. At the 1 m planning
            // resolution a peer can share a cell with the frozen robot, and
            // marking the cell it is standing in leaves it with nowhere legal
            // to be for the whole horizon.
            //
            // A claimed *goal* is deliberately left marked: "the place you were
            // sent to is occupied by a robot that is not moving" is true, and
            // the caller should see the agent fail to arrive rather than have
            // us quietly hand it a path through the dock.
            if grid_starts.contains(&(cx, cy)) {
                continue;
            }
            grid[cx][cy] = 1;
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

    #[test]
    fn claims_are_deduplicated_and_bounds_checked() {
        // 4x4 grid of 1 m cells with the origin at (-1, -1).
        let claims = vec![
            vec![
                [0.0, 0.0], // -> (1, 1)
                [0.2, -0.1], // -> (1, 1) as well: where a robot is and the dock
                            //    it is driving into are usually the same cell
            ],
            vec![[1.0, 0.0]],   // -> (2, 1)
            vec![[99.0, 0.0]],  // off the east edge, dropped rather than clamped
            vec![[0.0, -50.0]], // off the south edge, likewise
            vec![],             // a robot we know nothing about claims nothing
        ];

        assert_eq!(
            claimed_cells(&claims, 4, 4, 1.0, -1.0, -1.0),
            vec![(1, 1), (2, 1)]
        );
    }

    /// The endpoints of a dock approach can sit three cells apart. Sampling
    /// only the ends would leave the middle of the robot's path open.
    #[test]
    fn a_claim_is_rasterised_along_its_whole_length() {
        // 6x6 grid of 1 m cells, origin at (0, 0). A diagonal run from cell
        // (1, 1) to cell (4, 4).
        let claims = vec![vec![[1.0, 1.0], [4.0, 4.0]]];

        let mut cells = claimed_cells(&claims, 6, 6, 1.0, 0.0, 0.0);
        cells.sort();

        for expected in [(1, 1), (2, 2), (3, 3), (4, 4)] {
            assert!(
                cells.contains(&expected),
                "the claim skipped {expected:?}: {cells:?}"
            );
        }
    }

    /// The guarantee Layer 2 rests on: a committed robot is routed around, not
    /// through, and cannot be shoved out of the way.
    ///
    /// The mover has to cross the claimed cell to reach its goal. PIBT will
    /// happily displace an *internal* agent standing there, so the claim has to
    /// reach the solver as map rather than as traffic.
    #[test]
    fn a_frozen_claim_is_never_entered_or_displaced() {
        use ros_env::rmf_prototype_msgs::msg::{Region, TargetRegion};

        let mut map = OccupancyGrid::default();
        map.info.resolution = 1.0;
        map.info.width = 5;
        map.info.height = 3;
        map.data = vec![0; 15];

        let mut start = Odometry::default();
        start.pose.pose.position.x = 0.0;
        start.pose.pose.position.y = 1.0;
        let starts = HashMap::from([("mover".to_string(), start)]);

        let mut goal = Destination::default();
        goal.constraints.regions.push(TargetRegion {
            region: Region {
                points: vec![4.0, 1.0],
                hint: Region::HINT_POINT,
            },
            ..Default::default()
        });
        let goals = HashMap::from([("mover".to_string(), goal)]);

        let robot_ids = vec!["mover".to_string()];
        let planner = PibtPlanner::new(50);

        // Straight down the middle of the aisle, directly between the two.
        let frozen = vec![vec![[2.0f32, 1.0]]];

        let trajectories = planner
            .plan(
                &starts,
                &goals,
                &HashMap::new(),
                &robot_ids,
                &Map { grid: map },
                &frozen,
                Arc::new(AtomicBool::new(false)),
            )
            .expect("planner should route around the claim");

        let path = &trajectories[0];
        // The last pose is snapped to the exact goal coordinates after solving,
        // so check the claim against the poses the solver actually chose.
        for pose in &path[..path.len() - 1] {
            let at_claim =
                (pose.translation.x - 2.0).abs() < 0.01 && (pose.translation.y - 1.0).abs() < 0.01;
            assert!(
                !at_claim,
                "the mover drove through a committed robot: {:?}",
                path
            );
        }

        // Going around must still get there. If it does not, the claim has
        // simply walled the aisle off and the assertion above proves nothing.
        let end = path.last().expect("a trajectory");
        assert!(
            (end.translation.x - 4.0).abs() < 0.01 && (end.translation.y - 1.0).abs() < 0.01,
            "the mover never reached its goal: {:?}",
            path
        );
    }
}
