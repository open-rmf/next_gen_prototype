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

use rclrs::{IntoPrimitiveOptions, Node, PrimitiveOptions};
use ros_env::{
    geometry_msgs::msg::Pose,
    nav_msgs::msg::OccupancyGrid,
    rmf_layered_map_msgs::msg::{
        MapObservationSource, MapRegionPatch, MapRegionUpdate, MapSourceContribution,
        MapSourceSnapshot,
    },
    rmf_prototype_msgs::msg::Region,
};
use std::{
    collections::{HashMap, HashSet, VecDeque},
    time::Duration,
};

const NANOS_PER_SECOND: i128 = 1_000_000_000;
const MAP_QOS_DEPTH: u32 = 10;
const MAX_PENDING_REGION_UPDATES: usize = 1024;

#[derive(Clone, Copy, Debug, Hash, PartialEq, Eq)]
enum DynamicUpdateType {
    Obstacle,
    Clear,
}

#[derive(Clone, Debug, Hash, PartialEq, Eq)]
struct DynamicCellKey {
    update_type: DynamicUpdateType,
    cell_index: usize,
}

#[derive(Clone, Debug)]
struct DynamicCell {
    occupancy_value: i8,
    expires_at_nsec: i128,
    sequence: u64,
}

#[derive(Clone, Debug, Hash, PartialEq, Eq)]
struct ObservationSourceKey {
    source_id: String,
    map_name: String,
}

#[derive(Clone, Debug)]
pub struct LayeredMap {
    static_grid: Option<OccupancyGrid>,
    dynamic_cells: HashMap<ObservationSourceKey, HashMap<DynamicCellKey, Vec<DynamicCell>>>,
    source_metadata: HashMap<ObservationSourceKey, MapObservationSource>,
    latest_source_stamps: HashMap<ObservationSourceKey, i128>,
    default_ttl_nsec: i128,
    next_sequence: u64,
    revision: u64,
}

impl LayeredMap {
    pub fn new(default_ttl: Duration) -> Self {
        Self {
            static_grid: None,
            dynamic_cells: HashMap::new(),
            source_metadata: HashMap::new(),
            latest_source_stamps: HashMap::new(),
            default_ttl_nsec: duration_to_nsec(default_ttl),
            next_sequence: 0,
            revision: 0,
        }
    }

    pub fn revision(&self) -> u64 {
        self.revision
    }

    pub fn dynamic_observation_count(&self) -> usize {
        self.dynamic_cells.values().map(HashMap::len).sum()
    }

    pub fn set_static_map(&mut self, mut grid: OccupancyGrid) {
        normalize_grid_data(&mut grid);
        if self
            .static_grid
            .as_ref()
            .is_some_and(|current| !same_grid_geometry(current, &grid))
        {
            self.dynamic_cells.clear();
        }
        self.static_grid = Some(grid);
        self.revision = self.revision.wrapping_add(1);
    }

    pub fn ingest_region_update(&mut self, update: MapRegionUpdate, now_nsec: i128) -> bool {
        if source_validation_error(&update, self.static_frame_id()).is_some() {
            return false;
        }
        let Some(static_grid) = self.static_grid.as_ref() else {
            return false;
        };

        let reset_source = update.reset_source;
        let source_metadata = update.source.clone();
        let key = ObservationSourceKey {
            source_id: update.source.source_id.clone(),
            map_name: update.source.map_name.clone(),
        };
        let stamp_nsec = i128::from(update.source.header.stamp.sec) * NANOS_PER_SECOND
            + i128::from(update.source.header.stamp.nanosec);

        if self
            .latest_source_stamps
            .get(&key)
            .is_some_and(|latest_stamp| stamp_nsec < *latest_stamp)
        {
            return false;
        }

        let mut changed = false;

        if reset_source {
            changed |= self.dynamic_cells.remove(&key).is_some();
        }

        let default_ttl_sec = update.source.default_ttl_sec;
        let robot_pose = update.source.robot_pose;
        let mut patches = update.patches;
        patches.sort_by_key(|patch| match patch.update_type {
            MapRegionPatch::UPDATE_CLEAR => 0,
            MapRegionPatch::UPDATE_OBSTACLE => 1,
            _ => 2,
        });

        for patch in patches {
            let update_type = match patch.update_type {
                MapRegionPatch::UPDATE_OBSTACLE => DynamicUpdateType::Obstacle,
                MapRegionPatch::UPDATE_CLEAR => DynamicUpdateType::Clear,
                _ => continue,
            };

            let ttl_nsec = positive_seconds_to_nsec(patch.ttl_sec)
                .or_else(|| positive_seconds_to_nsec(default_ttl_sec))
                .unwrap_or(self.default_ttl_nsec);
            if ttl_nsec <= 0 {
                continue;
            }

            let expires_at_nsec = stamp_nsec + ttl_nsec;
            if expires_at_nsec <= now_nsec {
                continue;
            }

            let occupancy_value = match update_type {
                DynamicUpdateType::Obstacle if patch.occupancy_value <= 0 => 100,
                DynamicUpdateType::Obstacle => patch.occupancy_value.clamp(1, 100),
                DynamicUpdateType::Clear if patch.occupancy_value < 0 => 0,
                DynamicUpdateType::Clear => patch.occupancy_value.clamp(0, 100),
            };

            let cell_indices = patch
                .regions
                .into_iter()
                .filter(|region| region_validation_error(region).is_none())
                .map(|region| transform_region(region, &robot_pose))
                .flat_map(|region| rasterized_indices(static_grid, &region))
                .collect::<HashSet<_>>();
            if cell_indices.is_empty() {
                continue;
            }

            let sequence = self.next_sequence;
            self.next_sequence = self.next_sequence.wrapping_add(1);
            let source_cells = self.dynamic_cells.entry(key.clone()).or_default();
            for cell_index in cell_indices {
                if update_type == DynamicUpdateType::Clear {
                    // Clear cells replace older obstacles from the same source.
                    source_cells.remove(&DynamicCellKey {
                        update_type: DynamicUpdateType::Obstacle,
                        cell_index,
                    });
                }

                let history = source_cells
                    .entry(DynamicCellKey {
                        update_type,
                        cell_index,
                    })
                    .or_default();
                // Keep older entries only when they may outlive the new one.
                history.retain(|cell| cell.expires_at_nsec > expires_at_nsec);
                history.push(DynamicCell {
                    occupancy_value,
                    expires_at_nsec,
                    sequence,
                });
            }
            changed = true;
        }

        if changed || reset_source {
            self.source_metadata.insert(key.clone(), source_metadata);
            self.latest_source_stamps.insert(key, stamp_nsec);
        }

        if changed {
            self.revision = self.revision.wrapping_add(1);
        }
        changed
    }

    pub fn prune_expired(&mut self, now_nsec: i128) -> bool {
        let mut changed = false;
        self.dynamic_cells.retain(|_, cells| {
            cells.retain(|_, history| {
                history.retain(|cell| {
                    let active = cell.expires_at_nsec > now_nsec;
                    changed |= !active;
                    active
                });
                !history.is_empty()
            });
            !cells.is_empty()
        });
        if changed {
            self.revision = self.revision.wrapping_add(1);
        }
        changed
    }

    pub fn compose(&self) -> Option<OccupancyGrid> {
        let mut composed = self.static_grid.clone()?;
        normalize_grid_data(&mut composed);

        for update_type in [DynamicUpdateType::Clear, DynamicUpdateType::Obstacle] {
            let mut cells = self
                .dynamic_cells
                .values()
                .flat_map(|source_cells| source_cells.iter())
                .filter_map(|(key, history)| {
                    if key.update_type != update_type {
                        return None;
                    }
                    history.last().map(|cell| (key, cell))
                })
                .collect::<Vec<_>>();
            cells.sort_by_key(|(_, cell)| cell.sequence);

            for (key, cell) in cells {
                if let Some(value) = composed.data.get_mut(key.cell_index) {
                    *value = cell.occupancy_value;
                }
            }
        }

        Some(composed)
    }

    pub fn source_snapshot(&self) -> Option<MapSourceSnapshot> {
        let static_grid = self.static_grid.as_ref()?;
        let mut source_entries = self.dynamic_cells.iter().collect::<Vec<_>>();
        source_entries.sort_by(|(left, _), (right, _)| {
            (&left.source_id, &left.map_name).cmp(&(&right.source_id, &right.map_name))
        });

        let mut sources = Vec::new();
        for (source_key, source_cells) in source_entries {
            // Clear cells can be omitted because each snapshot replaces the previous one.
            let mut cells = source_cells
                .iter()
                .filter_map(|(cell_key, history)| {
                    if cell_key.update_type != DynamicUpdateType::Obstacle {
                        return None;
                    }
                    history
                        .last()
                        .map(|cell| (cell_key.cell_index, cell.occupancy_value))
                })
                .collect::<Vec<_>>();
            cells.sort_by_key(|(cell_index, _)| *cell_index);
            let mut contribution = MapSourceContribution {
                source: self
                    .source_metadata
                    .get(source_key)
                    .cloned()
                    .unwrap_or_default(),
                ..Default::default()
            };
            for (cell_index, occupancy_value) in cells {
                if let Ok(cell_index) = u64::try_from(cell_index) {
                    contribution.cell_indices.push(cell_index);
                    contribution.occupancy_values.push(occupancy_value);
                }
            }
            if !contribution.cell_indices.is_empty() {
                sources.push(contribution);
            }
        }

        Some(MapSourceSnapshot {
            header: static_grid.header.clone(),
            info: static_grid.info.clone(),
            sources,
        })
    }

    fn static_frame_id(&self) -> Option<&str> {
        self.static_grid
            .as_ref()
            .map(|grid| grid.header.frame_id.as_str())
    }
}

impl Default for LayeredMap {
    fn default() -> Self {
        Self::new(Duration::from_secs(30))
    }
}

#[derive(Clone, Debug)]
pub struct LayeredMapServerConfig {
    pub static_map_topic: String,
    pub region_updates_topic: String,
    pub composed_map_topic: String,
    pub source_contributions_topic: String,
    pub default_ttl: Duration,
    pub publish_period: Duration,
}

impl Default for LayeredMapServerConfig {
    fn default() -> Self {
        Self {
            static_map_topic: "/map/static".to_string(),
            region_updates_topic: "/map/region_updates".to_string(),
            composed_map_topic: "/map".to_string(),
            source_contributions_topic: "/map/source_contributions".to_string(),
            default_ttl: Duration::from_secs(30),
            publish_period: Duration::from_millis(250),
        }
    }
}

fn push_pending_region_update(
    updates: &mut VecDeque<MapRegionUpdate>,
    update: MapRegionUpdate,
) -> Option<MapRegionUpdate> {
    let dropped = if updates.len() >= MAX_PENDING_REGION_UPDATES {
        updates.pop_front()
    } else {
        None
    };
    updates.push_back(update);
    dropped
}

pub struct LayeredMapServer {
    node: Node,
    map: LayeredMap,
    pending_region_updates: VecDeque<MapRegionUpdate>,
    map_publisher: rclrs::Publisher<OccupancyGrid>,
    source_contributions_publisher: rclrs::Publisher<MapSourceSnapshot>,
    last_published_revision: Option<u64>,
}

impl LayeredMapServer {
    pub fn new(
        node: Node,
        config: &LayeredMapServerConfig,
    ) -> Result<Self, Box<dyn std::error::Error>> {
        let map_publisher =
            node.create_publisher::<OccupancyGrid>(composed_map_qos(&config.composed_map_topic))?;
        let source_contributions_publisher = node.create_publisher::<MapSourceSnapshot>(
            source_contributions_qos(&config.source_contributions_topic),
        )?;

        Ok(Self {
            node,
            map: LayeredMap::new(config.default_ttl),
            pending_region_updates: VecDeque::new(),
            map_publisher,
            source_contributions_publisher,
            last_published_revision: None,
        })
    }

    fn handle_static_map(&mut self, msg: OccupancyGrid) {
        self.map.set_static_map(msg);
        let pending_updates = std::mem::take(&mut self.pending_region_updates);
        let pending_count = pending_updates.len();
        if pending_count > 0 {
            let now_nsec = self.now_nsec();
            let mut changed_count = 0;
            for update in pending_updates {
                log_region_update_errors(&self.node, &update, self.map.static_frame_id());
                if self.map.ingest_region_update(update, now_nsec) {
                    changed_count += 1;
                }
            }
            rclrs::log!(
                self.node.logger(),
                "Rasterized {} queued region updates; {} changed the map",
                pending_count,
                changed_count
            );
        }

        rclrs::log!(
            self.node.logger(),
            "Received static map, revision {}",
            self.map.revision()
        );
        self.publish_if_changed();
    }

    fn handle_region_update(&mut self, msg: MapRegionUpdate) {
        if self.map.static_grid.is_none() {
            log_region_update_errors(&self.node, &msg, None);
            if source_validation_error(&msg, None).is_some() {
                return;
            }

            let source_id = msg.source.source_id.clone();
            if let Some(dropped) = push_pending_region_update(&mut self.pending_region_updates, msg)
            {
                rclrs::log_warn!(
                    self.node.logger(),
                    "Region update queue is full; dropped oldest update from '{}'",
                    dropped.source.source_id
                );
            }
            rclrs::log!(
                self.node.logger(),
                "Queued region update from '{}' until a static map is available ({} queued)",
                source_id,
                self.pending_region_updates.len()
            );
            return;
        }

        log_region_update_errors(&self.node, &msg, self.map.static_frame_id());
        let now_nsec = self.now_nsec();
        if self.map.ingest_region_update(msg, now_nsec) {
            rclrs::log!(
                self.node.logger(),
                "Accepted region update, revision {}, active rasterized cells {}",
                self.map.revision(),
                self.map.dynamic_observation_count()
            );
            self.publish_if_changed();
        }
    }

    fn prune_expired(&mut self) {
        let now_nsec = self.now_nsec();
        if self.map.prune_expired(now_nsec) {
            rclrs::log!(
                self.node.logger(),
                "Pruned expired observations, revision {}, active rasterized cells {}",
                self.map.revision(),
                self.map.dynamic_observation_count()
            );
            self.publish_if_changed();
        }
    }

    fn publish_if_changed(&mut self) {
        if self.last_published_revision == Some(self.map.revision()) {
            return;
        }

        let Some(composed) = self.map.compose() else {
            return;
        };
        let source_snapshot = self.map.source_snapshot();

        match self.map_publisher.publish(&composed) {
            Ok(()) => {
                if let Some(source_snapshot) = source_snapshot {
                    if let Err(err) = self
                        .source_contributions_publisher
                        .publish(&source_snapshot)
                    {
                        rclrs::log_error!(
                            self.node.logger(),
                            "Failed to publish source contributions: {:?}",
                            err
                        );
                        return;
                    }
                }
                self.last_published_revision = Some(self.map.revision());
                rclrs::log!(
                    self.node.logger(),
                    "Published composed map {}x{} @ {}m/cell",
                    composed.info.width,
                    composed.info.height,
                    composed.info.resolution
                );
            }
            Err(err) => {
                rclrs::log_error!(
                    self.node.logger(),
                    "Failed to publish composed map: {:?}",
                    err
                );
            }
        }
    }

    fn now_nsec(&self) -> i128 {
        i128::from(self.node.get_clock().now().nsec)
    }
}

pub struct LayeredMapServerRunning {
    pub worker: rclrs::Worker<LayeredMapServer>,
    // Keep the ROS handles alive for as long as the server is running.
    pub static_map_subscription: rclrs::WorkerSubscription<OccupancyGrid, LayeredMapServer>,
    pub region_update_subscription: rclrs::WorkerSubscription<MapRegionUpdate, LayeredMapServer>,
    pub prune_timer: rclrs::WorkerTimer<LayeredMapServer>,
}

pub fn start_layered_map_server(
    node: Node,
    config: LayeredMapServerConfig,
) -> Result<LayeredMapServerRunning, Box<dyn std::error::Error>> {
    let static_map_topic = config.static_map_topic.clone();
    let region_updates_topic = config.region_updates_topic.clone();
    let publish_period = config.publish_period;
    let worker = node.create_worker(LayeredMapServer::new(node.clone(), &config)?);

    let static_map_subscription = worker.create_subscription::<OccupancyGrid, _>(
        static_map_qos(&static_map_topic),
        |server: &mut LayeredMapServer, msg: OccupancyGrid| {
            server.handle_static_map(msg);
        },
    )?;

    let region_update_subscription = worker.create_subscription::<MapRegionUpdate, _>(
        region_update_qos(&region_updates_topic),
        |server: &mut LayeredMapServer, msg: MapRegionUpdate| {
            server.handle_region_update(msg);
        },
    )?;

    let prune_timer =
        worker.create_timer_repeating(publish_period, |server: &mut LayeredMapServer| {
            server.prune_expired();
        })?;

    Ok(LayeredMapServerRunning {
        worker,
        static_map_subscription,
        region_update_subscription,
        prune_timer,
    })
}

fn static_map_qos(topic: &str) -> PrimitiveOptions<'_> {
    topic.keep_last(MAP_QOS_DEPTH).transient_local().reliable()
}

fn composed_map_qos(topic: &str) -> PrimitiveOptions<'_> {
    topic.keep_last(MAP_QOS_DEPTH).transient_local().reliable()
}

fn source_contributions_qos(topic: &str) -> PrimitiveOptions<'_> {
    topic.keep_last(MAP_QOS_DEPTH).transient_local().reliable()
}

fn region_update_qos(topic: &str) -> PrimitiveOptions<'_> {
    topic.keep_last(MAP_QOS_DEPTH).reliable()
}

fn log_region_update_errors(node: &Node, update: &MapRegionUpdate, expected_frame: Option<&str>) {
    if let Some(error) = source_validation_error(update, expected_frame) {
        rclrs::log_error!(
            node.logger(),
            "Ignoring map region update from '{}': {}",
            update.source.source_id,
            error
        );
    }

    for patch in &update.patches {
        if !matches!(
            patch.update_type,
            MapRegionPatch::UPDATE_OBSTACLE | MapRegionPatch::UPDATE_CLEAR
        ) {
            rclrs::log_error!(
                node.logger(),
                "Ignoring map region patch with unsupported update_type {}",
                patch.update_type
            );
            continue;
        }

        for region in &patch.regions {
            if let Some(error) = region_validation_error(region) {
                rclrs::log_error!(node.logger(), "Ignoring map region: {}", error);
            }
        }
    }
}

fn source_validation_error(
    update: &MapRegionUpdate,
    expected_frame: Option<&str>,
) -> Option<String> {
    let stamp = &update.source.header.stamp;
    if stamp.sec == 0 && stamp.nanosec == 0 {
        return Some("source timestamp must be non-zero".to_string());
    }

    let frame_id = update.source.header.frame_id.as_str();
    if frame_id.is_empty() {
        return Some("source frame must not be empty".to_string());
    }

    if let Some(expected) = expected_frame {
        if !expected.is_empty() && expected != frame_id {
            return Some(format!(
                "source frame '{}' does not match map frame '{}'",
                frame_id, expected
            ));
        }
    }

    let pose = &update.source.robot_pose;
    if ![
        pose.position.x,
        pose.position.y,
        pose.position.z,
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w,
    ]
    .into_iter()
    .all(f64::is_finite)
    {
        return Some("source pose must contain finite values".to_string());
    }

    let orientation_norm = pose.orientation.x * pose.orientation.x
        + pose.orientation.y * pose.orientation.y
        + pose.orientation.z * pose.orientation.z
        + pose.orientation.w * pose.orientation.w;
    if orientation_norm <= f64::EPSILON {
        return Some("source pose must contain a valid orientation".to_string());
    }

    None
}

fn region_validation_error(region: &Region) -> Option<String> {
    if region.points.len() < 2 {
        return Some("region must contain at least one x/y point pair".to_string());
    }

    if region.points.len() % 2 != 0 {
        return Some("region points must contain complete x/y pairs".to_string());
    }

    match region.hint {
        Region::HINT_POINT if region.points.len() != 2 => {
            Some("point region must contain exactly one x/y pair".to_string())
        }
        Region::HINT_AXIS_ALIGNED_RECTANGLE if region.points.len() < 4 => {
            Some("axis-aligned rectangle must contain at least two x/y pairs".to_string())
        }
        Region::HINT_CONVEX_POLYGON if region.points.len() < 6 => {
            Some("convex polygon must contain at least three x/y pairs".to_string())
        }
        Region::HINT_POINT
        | Region::HINT_AXIS_ALIGNED_RECTANGLE
        | Region::HINT_CONVEX_POLYGON => None,
        hint => Some(format!(
            "unsupported region hint {}; expected a point, axis-aligned rectangle, or convex polygon",
            hint
        )),
    }
}

fn is_rasterizable_region_hint(hint: u8) -> bool {
    matches!(
        hint,
        Region::HINT_POINT | Region::HINT_AXIS_ALIGNED_RECTANGLE | Region::HINT_CONVEX_POLYGON
    )
}

fn transform_region(mut region: Region, pose: &Pose) -> Region {
    let points = point_pairs(&region);
    let points = if region.hint == Region::HINT_AXIS_ALIGNED_RECTANGLE {
        let (min_x, min_y, max_x, max_y) = bounds(&points);
        region.hint = Region::HINT_CONVEX_POLYGON;
        vec![
            (min_x, min_y),
            (max_x, min_y),
            (max_x, max_y),
            (min_x, max_y),
        ]
    } else {
        points
    };

    let (cos_yaw, sin_yaw) = planar_rotation(pose);
    region.points = points
        .into_iter()
        .flat_map(|(x, y)| {
            let map_x = pose.position.x + cos_yaw * x - sin_yaw * y;
            let map_y = pose.position.y + sin_yaw * x + cos_yaw * y;
            [map_x as f32, map_y as f32]
        })
        .collect();
    region
}

fn planar_rotation(pose: &Pose) -> (f64, f64) {
    let q = &pose.orientation;
    let norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
    let sin_yaw = 2.0 * (q.w * q.z + q.x * q.y) / norm;
    let cos_yaw = 1.0 - 2.0 * (q.y * q.y + q.z * q.z) / norm;
    let yaw = sin_yaw.atan2(cos_yaw);
    (yaw.cos(), yaw.sin())
}

fn normalize_grid_data(grid: &mut OccupancyGrid) {
    let Some(expected_len) = grid_len(grid) else {
        grid.data.clear();
        return;
    };

    if grid.data.len() < expected_len {
        grid.data.resize(expected_len, -1);
    } else if grid.data.len() > expected_len {
        grid.data.truncate(expected_len);
    }
}

fn grid_len(grid: &OccupancyGrid) -> Option<usize> {
    let width = usize::try_from(grid.info.width).ok()?;
    let height = usize::try_from(grid.info.height).ok()?;
    width.checked_mul(height)
}

fn same_grid_geometry(lhs: &OccupancyGrid, rhs: &OccupancyGrid) -> bool {
    lhs.header.frame_id == rhs.header.frame_id
        && lhs.info.width == rhs.info.width
        && lhs.info.height == rhs.info.height
        && lhs.info.resolution == rhs.info.resolution
        && lhs.info.origin == rhs.info.origin
}

fn rasterized_indices(grid: &OccupancyGrid, region: &Region) -> Vec<usize> {
    let Some((width, height, resolution, origin_x, origin_y)) = grid_geometry(grid) else {
        return Vec::new();
    };

    if region.points.len() < 2 || region.points.len() % 2 != 0 {
        return Vec::new();
    }

    if !is_rasterizable_region_hint(region.hint) {
        return Vec::new();
    }

    if region.hint == Region::HINT_POINT || region.points.len() == 2 {
        return point_index(
            region.points[0],
            region.points[1],
            width,
            height,
            resolution,
            origin_x,
            origin_y,
        )
        .into_iter()
        .collect();
    }

    if region.hint == Region::HINT_AXIS_ALIGNED_RECTANGLE && region.points.len() >= 4 {
        let pairs = point_pairs(region);
        let (min_x, min_y, max_x, max_y) = bounds(&pairs);
        return indices_in_bounds(
            width, height, resolution, origin_x, origin_y, min_x, min_y, max_x, max_y,
        );
    }

    let pairs = point_pairs(region);
    if pairs.len() < 3 {
        let (min_x, min_y, max_x, max_y) = bounds(&pairs);
        return indices_in_bounds(
            width, height, resolution, origin_x, origin_y, min_x, min_y, max_x, max_y,
        );
    }

    let (min_x, min_y, max_x, max_y) = bounds(&pairs);
    indices_in_bounds(
        width, height, resolution, origin_x, origin_y, min_x, min_y, max_x, max_y,
    )
    .into_iter()
    .filter(|index| {
        let x = index % width;
        let y = index / width;
        let world_x = origin_x + (x as f64 + 0.5) * resolution;
        let world_y = origin_y + (y as f64 + 0.5) * resolution;
        point_in_polygon(world_x, world_y, &pairs)
    })
    .collect()
}

fn grid_geometry(grid: &OccupancyGrid) -> Option<(usize, usize, f64, f64, f64)> {
    if grid.info.width == 0 || grid.info.height == 0 || grid.info.resolution <= 0.0 {
        return None;
    }

    Some((
        usize::try_from(grid.info.width).ok()?,
        usize::try_from(grid.info.height).ok()?,
        f64::from(grid.info.resolution),
        grid.info.origin.position.x,
        grid.info.origin.position.y,
    ))
}

fn point_index(
    world_x: f32,
    world_y: f32,
    width: usize,
    height: usize,
    resolution: f64,
    origin_x: f64,
    origin_y: f64,
) -> Option<usize> {
    let cell_x = ((f64::from(world_x) - origin_x) / resolution).floor() as isize;
    let cell_y = ((f64::from(world_y) - origin_y) / resolution).floor() as isize;
    if cell_x < 0 || cell_y < 0 {
        return None;
    }

    let cell_x = usize::try_from(cell_x).ok()?;
    let cell_y = usize::try_from(cell_y).ok()?;
    if cell_x >= width || cell_y >= height {
        return None;
    }

    Some(cell_y * width + cell_x)
}

fn point_pairs(region: &Region) -> Vec<(f64, f64)> {
    region
        .points
        .chunks_exact(2)
        .map(|point| (f64::from(point[0]), f64::from(point[1])))
        .collect()
}

fn bounds(points: &[(f64, f64)]) -> (f64, f64, f64, f64) {
    let mut min_x = f64::INFINITY;
    let mut min_y = f64::INFINITY;
    let mut max_x = f64::NEG_INFINITY;
    let mut max_y = f64::NEG_INFINITY;

    for (x, y) in points {
        min_x = min_x.min(*x);
        min_y = min_y.min(*y);
        max_x = max_x.max(*x);
        max_y = max_y.max(*y);
    }

    (min_x, min_y, max_x, max_y)
}

fn indices_in_bounds(
    width: usize,
    height: usize,
    resolution: f64,
    origin_x: f64,
    origin_y: f64,
    min_x: f64,
    min_y: f64,
    max_x: f64,
    max_y: f64,
) -> Vec<usize> {
    if min_x > max_x || min_y > max_y {
        return Vec::new();
    }

    let start_x = (((min_x - origin_x) / resolution).floor() as isize).max(0) as usize;
    let start_y = (((min_y - origin_y) / resolution).floor() as isize).max(0) as usize;
    let end_x = (((max_x - origin_x) / resolution).ceil() as isize)
        .max(0)
        .min(width as isize) as usize;
    let end_y = (((max_y - origin_y) / resolution).ceil() as isize)
        .max(0)
        .min(height as isize) as usize;

    let mut indices = Vec::new();
    for y in start_y..end_y {
        for x in start_x..end_x {
            indices.push(y * width + x);
        }
    }
    indices
}

fn point_in_polygon(x: f64, y: f64, polygon: &[(f64, f64)]) -> bool {
    let mut inside = false;
    let mut j = polygon.len() - 1;

    for i in 0..polygon.len() {
        let (xi, yi) = polygon[i];
        let (xj, yj) = polygon[j];
        let crosses = (yi > y) != (yj > y);
        if crosses {
            let x_intersection = (xj - xi) * (y - yi) / (yj - yi) + xi;
            if x < x_intersection {
                inside = !inside;
            }
        }
        j = i;
    }

    inside
}

fn duration_to_nsec(duration: Duration) -> i128 {
    i128::from(duration.as_secs()) * NANOS_PER_SECOND + i128::from(duration.subsec_nanos())
}

fn positive_seconds_to_nsec(seconds: f64) -> Option<i128> {
    if !seconds.is_finite() || seconds <= 0.0 {
        return None;
    }

    Some((seconds * NANOS_PER_SECOND as f64).round() as i128)
}

#[cfg(test)]
mod tests {
    use super::*;
    use ros_env::{
        geometry_msgs::msg::{Point, Pose, Quaternion},
        nav_msgs::msg::MapMetaData,
        rmf_layered_map_msgs::msg::{MapObservationSource, MapRegionPatch, MapRegionUpdate},
        rmf_prototype_msgs::msg::Region,
        std_msgs::msg::Header,
    };

    fn static_grid(width: u32, height: u32, value: i8) -> OccupancyGrid {
        OccupancyGrid {
            header: Header {
                frame_id: "map".to_string(),
                ..Default::default()
            },
            info: MapMetaData {
                resolution: 1.0,
                width,
                height,
                origin: Pose {
                    position: Point {
                        x: 0.0,
                        y: 0.0,
                        z: 0.0,
                    },
                    orientation: Quaternion {
                        w: 1.0,
                        ..Default::default()
                    },
                },
                ..Default::default()
            },
            data: vec![value; (width * height) as usize],
            ..Default::default()
        }
    }

    fn source() -> MapObservationSource {
        source_with_id("robot_1/local_costmap")
    }

    fn source_with_id(source_id: &str) -> MapObservationSource {
        let mut source = MapObservationSource {
            header: Header {
                frame_id: "map".to_string(),
                ..Default::default()
            },
            source_id: source_id.to_string(),
            robot_name: "robot_1".to_string(),
            map_name: "test_map".to_string(),
            default_ttl_sec: 10.0,
            ..Default::default()
        };
        source.header.stamp.sec = 1;
        source.robot_pose.orientation.w = 1.0;
        source
    }

    fn rectangle(min_x: f32, min_y: f32, max_x: f32, max_y: f32) -> Region {
        Region {
            hint: Region::HINT_AXIS_ALIGNED_RECTANGLE,
            points: vec![min_x, min_y, max_x, max_y],
        }
    }

    fn point(x: f32, y: f32) -> Region {
        Region {
            hint: Region::HINT_POINT,
            points: vec![x, y],
        }
    }

    fn convex_polygon(points: Vec<f32>) -> Region {
        Region {
            hint: Region::HINT_CONVEX_POLYGON,
            points,
        }
    }

    fn patch(update_type: u8, regions: Vec<Region>) -> MapRegionPatch {
        MapRegionPatch {
            update_type,
            regions,
            ..Default::default()
        }
    }

    fn update(update_type: u8, regions: Vec<Region>) -> MapRegionUpdate {
        MapRegionUpdate {
            source: source(),
            patches: vec![patch(update_type, regions)],
            ..Default::default()
        }
    }

    fn update_from(source_id: &str, update_type: u8, regions: Vec<Region>) -> MapRegionUpdate {
        MapRegionUpdate {
            source: source_with_id(source_id),
            patches: vec![patch(update_type, regions)],
            ..Default::default()
        }
    }

    #[test]
    fn obstacle_regions_are_composed_over_static_map() {
        let mut map = LayeredMap::default();
        map.set_static_map(static_grid(5, 5, 0));

        assert!(map.ingest_region_update(
            update(
                MapRegionPatch::UPDATE_OBSTACLE,
                vec![rectangle(2.0, 1.0, 3.0, 3.0)]
            ),
            1_000,
        ));

        let composed = map.compose().unwrap();
        assert_eq!(composed.data[1 * 5 + 2], 100);
        assert_eq!(composed.data[2 * 5 + 2], 100);
        assert_eq!(composed.data[0], 0);
    }

    #[test]
    fn point_regions_respect_map_origin_and_resolution() {
        let mut static_map = static_grid(4, 4, 0);
        static_map.info.resolution = 0.5;
        static_map.info.origin.position.x = -1.0;
        static_map.info.origin.position.y = -1.0;

        let mut map = LayeredMap::default();
        map.set_static_map(static_map);

        assert!(map.ingest_region_update(
            update(MapRegionPatch::UPDATE_OBSTACLE, vec![point(-0.25, 0.25)]),
            0,
        ));

        let composed = map.compose().unwrap();
        assert_eq!(composed.data[2 * 4 + 1], 100);
        assert_eq!(composed.data[0], 0);
    }

    #[test]
    fn convex_clear_regions_are_composed_over_static_map() {
        let mut map = LayeredMap::default();
        map.set_static_map(static_grid(5, 5, 100));

        assert!(map.ingest_region_update(
            update(
                MapRegionPatch::UPDATE_CLEAR,
                vec![convex_polygon(vec![0.0, 0.0, 4.0, 0.0, 4.0, 4.0])]
            ),
            0,
        ));

        let composed = map.compose().unwrap();
        assert_eq!(composed.data[1 * 5 + 3], 0);
        assert_eq!(composed.data[3 * 5 + 1], 100);
    }

    #[test]
    fn robot_local_regions_are_transformed_into_the_map_frame() {
        let mut map = LayeredMap::default();
        map.set_static_map(static_grid(5, 5, 0));

        let mut update = update(MapRegionPatch::UPDATE_OBSTACLE, vec![point(1.0, 0.0)]);
        update.source.robot_pose.position.x = 2.25;
        update.source.robot_pose.position.y = 1.25;
        update.source.robot_pose.orientation.z = std::f64::consts::FRAC_1_SQRT_2;
        update.source.robot_pose.orientation.w = std::f64::consts::FRAC_1_SQRT_2;

        assert!(map.ingest_region_update(update, 0));

        let composed = map.compose().unwrap();
        assert_eq!(composed.data[2 * 5 + 2], 100);
        assert_eq!(composed.data[0], 0);
    }

    #[test]
    fn out_of_bounds_regions_do_not_touch_the_grid() {
        let mut map = LayeredMap::default();
        map.set_static_map(static_grid(3, 3, 0));

        assert!(!map.ingest_region_update(
            update(
                MapRegionPatch::UPDATE_OBSTACLE,
                vec![rectangle(10.0, 10.0, 11.0, 11.0)]
            ),
            0,
        ));
        assert_eq!(map.dynamic_observation_count(), 0);

        let composed = map.compose().unwrap();
        assert!(composed.data.iter().all(|cell| *cell == 0));
    }

    #[test]
    fn invalid_region_points_are_ignored() {
        let mut map = LayeredMap::default();
        map.set_static_map(static_grid(3, 3, 0));

        assert!(!map.ingest_region_update(
            update(
                MapRegionPatch::UPDATE_OBSTACLE,
                vec![Region {
                    hint: Region::HINT_POINT,
                    points: vec![1.0, 1.0, 2.0, 2.0],
                }]
            ),
            0,
        ));

        let composed = map.compose().unwrap();
        assert!(composed.data.iter().all(|cell| *cell == 0));
    }

    #[test]
    fn unsupported_region_types_are_ignored() {
        let mut map = LayeredMap::default();
        map.set_static_map(static_grid(3, 3, 0));

        assert!(!map.ingest_region_update(
            update(
                MapRegionPatch::UPDATE_OBSTACLE,
                vec![Region {
                    hint: Region::HINT_POLYGON,
                    points: vec![0.0, 0.0, 2.0, 0.0, 1.0, 2.0],
                }]
            ),
            0,
        ));
        assert_eq!(map.dynamic_observation_count(), 0);
    }

    #[test]
    fn updates_without_a_timestamp_are_rejected() {
        let mut map = LayeredMap::default();
        map.set_static_map(static_grid(3, 3, 0));

        let mut unstamped = update(MapRegionPatch::UPDATE_OBSTACLE, vec![point(1.0, 1.0)]);
        unstamped.source.header.stamp = Default::default();

        assert!(!map.ingest_region_update(unstamped, NANOS_PER_SECOND));
        assert_eq!(map.dynamic_observation_count(), 0);
    }

    #[test]
    fn updates_in_a_different_global_frame_are_rejected() {
        let mut map = LayeredMap::default();
        map.set_static_map(static_grid(3, 3, 0));

        let mut update = update(MapRegionPatch::UPDATE_OBSTACLE, vec![point(1.0, 1.0)]);
        update.source.header.frame_id = "odom".to_string();

        assert!(!map.ingest_region_update(update, NANOS_PER_SECOND));
        assert_eq!(map.dynamic_observation_count(), 0);
    }

    #[test]
    fn expired_observations_are_removed() {
        let mut map = LayeredMap::default();
        map.set_static_map(static_grid(3, 3, 0));

        assert!(map.ingest_region_update(
            update(
                MapRegionPatch::UPDATE_OBSTACLE,
                vec![rectangle(1.0, 1.0, 2.0, 2.0)]
            ),
            0,
        ));
        assert_eq!(map.dynamic_observation_count(), 1);

        assert!(map.prune_expired(11 * NANOS_PER_SECOND));
        assert_eq!(map.dynamic_observation_count(), 0);

        let composed = map.compose().unwrap();
        assert_eq!(composed.data[1 * 3 + 1], 0);
    }

    #[test]
    fn obstacles_win_over_clear_regions() {
        let mut map = LayeredMap::default();
        map.set_static_map(static_grid(5, 5, -1));

        assert!(map.ingest_region_update(
            MapRegionUpdate {
                source: source(),
                patches: vec![
                    patch(
                        MapRegionPatch::UPDATE_CLEAR,
                        vec![rectangle(1.0, 1.0, 4.0, 4.0)]
                    ),
                    patch(
                        MapRegionPatch::UPDATE_OBSTACLE,
                        vec![rectangle(2.0, 2.0, 3.0, 3.0)]
                    ),
                ],
                ..Default::default()
            },
            0,
        ));

        let composed = map.compose().unwrap();
        assert_eq!(composed.data[1 * 5 + 1], 0);
        assert_eq!(composed.data[2 * 5 + 2], 100);
        assert_eq!(composed.data[0], -1);
    }

    #[test]
    fn clear_regions_replace_only_observed_same_source_obstacles() {
        let mut map = LayeredMap::default();
        map.set_static_map(static_grid(5, 5, 0));

        assert!(map.ingest_region_update(
            update(
                MapRegionPatch::UPDATE_OBSTACLE,
                vec![point(1.0, 1.0), point(3.0, 3.0)]
            ),
            0,
        ));

        let mut clear = update(
            MapRegionPatch::UPDATE_CLEAR,
            vec![rectangle(0.0, 0.0, 2.0, 2.0)],
        );
        clear.source.header.stamp.sec = 2;
        assert!(map.ingest_region_update(clear, NANOS_PER_SECOND));

        let composed = map.compose().unwrap();
        assert_eq!(composed.data[1 * 5 + 1], 0);
        assert_eq!(composed.data[3 * 5 + 3], 100);

        assert!(map.prune_expired(11 * NANOS_PER_SECOND));
        let composed = map.compose().unwrap();
        assert_eq!(composed.data[3 * 5 + 3], 0);
    }

    #[test]
    fn clear_regions_do_not_remove_other_source_obstacles() {
        let mut map = LayeredMap::default();
        map.set_static_map(static_grid(5, 5, 0));

        assert!(map.ingest_region_update(
            update_from(
                "robot_2/local_costmap",
                MapRegionPatch::UPDATE_OBSTACLE,
                vec![point(1.0, 1.0)]
            ),
            0,
        ));

        let mut clear = update(
            MapRegionPatch::UPDATE_CLEAR,
            vec![rectangle(0.0, 0.0, 2.0, 2.0)],
        );
        clear.source.header.stamp.sec = 2;
        assert!(map.ingest_region_update(clear, NANOS_PER_SECOND));

        let composed = map.compose().unwrap();
        assert_eq!(composed.data[1 * 5 + 1], 100);
    }

    #[test]
    fn source_snapshot_reports_active_obstacle_contributions() {
        let mut map = LayeredMap::default();
        map.set_static_map(static_grid(5, 5, 0));

        assert!(map.ingest_region_update(
            update(
                MapRegionPatch::UPDATE_OBSTACLE,
                vec![point(1.0, 1.0), point(3.0, 3.0)]
            ),
            0,
        ));
        assert!(map.ingest_region_update(
            update_from(
                "robot_2/local_costmap",
                MapRegionPatch::UPDATE_OBSTACLE,
                vec![point(2.0, 2.0)]
            ),
            0,
        ));

        let mut clear = update(
            MapRegionPatch::UPDATE_CLEAR,
            vec![rectangle(0.0, 0.0, 2.0, 2.0)],
        );
        clear.source.header.stamp.sec = 2;
        assert!(map.ingest_region_update(clear, NANOS_PER_SECOND));

        let snapshot = map.source_snapshot().unwrap();
        assert_eq!(snapshot.info.width, 5);
        assert_eq!(snapshot.info.height, 5);
        assert_eq!(snapshot.sources.len(), 2);

        let robot_1 = snapshot
            .sources
            .iter()
            .find(|source| source.source.source_id == "robot_1/local_costmap")
            .unwrap();
        assert_eq!(robot_1.cell_indices, vec![18]);
        assert_eq!(robot_1.occupancy_values, vec![100]);

        let robot_2 = snapshot
            .sources
            .iter()
            .find(|source| source.source.source_id == "robot_2/local_costmap")
            .unwrap();
        assert_eq!(robot_2.cell_indices, vec![12]);
        assert_eq!(robot_2.occupancy_values, vec![100]);
    }

    #[test]
    fn newer_obstacles_survive_older_clear_updates_received_late() {
        let mut map = LayeredMap::default();
        map.set_static_map(static_grid(5, 5, -1));

        let mut obstacle = update(
            MapRegionPatch::UPDATE_OBSTACLE,
            vec![rectangle(2.0, 2.0, 3.0, 3.0)],
        );
        obstacle.source.header.stamp.sec = 2;

        let mut clear = update(
            MapRegionPatch::UPDATE_CLEAR,
            vec![rectangle(1.0, 1.0, 4.0, 4.0)],
        );
        clear.reset_source = true;
        clear.source.header.stamp.sec = 1;

        assert!(map.ingest_region_update(obstacle, 3 * NANOS_PER_SECOND));
        assert!(!map.ingest_region_update(clear, 3 * NANOS_PER_SECOND));

        let composed = map.compose().unwrap();
        assert_eq!(composed.data[1 * 5 + 1], -1);
        assert_eq!(composed.data[2 * 5 + 2], 100);
        assert_eq!(composed.data[0], -1);
    }

    #[test]
    fn reset_removes_observations_from_same_source_and_map() {
        let mut map = LayeredMap::default();
        map.set_static_map(static_grid(3, 3, 0));

        assert!(map.ingest_region_update(
            update(
                MapRegionPatch::UPDATE_OBSTACLE,
                vec![rectangle(1.0, 1.0, 2.0, 2.0)]
            ),
            0,
        ));

        assert!(map.ingest_region_update(
            MapRegionUpdate {
                source: source(),
                reset_source: true,
                ..Default::default()
            },
            0,
        ));

        assert_eq!(map.dynamic_observation_count(), 0);
        let composed = map.compose().unwrap();
        assert_eq!(composed.data[1 * 3 + 1], 0);
    }

    #[test]
    fn multiple_robot_sources_are_stitched_into_one_composed_grid() {
        let mut map = LayeredMap::default();
        map.set_static_map(static_grid(5, 5, 0));

        assert!(map.ingest_region_update(
            update_from(
                "robot_1/local_costmap",
                MapRegionPatch::UPDATE_OBSTACLE,
                vec![rectangle(1.0, 1.0, 2.0, 2.0)]
            ),
            0,
        ));
        assert!(map.ingest_region_update(
            update_from(
                "robot_2/local_costmap",
                MapRegionPatch::UPDATE_OBSTACLE,
                vec![rectangle(3.0, 3.0, 4.0, 4.0)]
            ),
            0,
        ));

        let composed = map.compose().unwrap();
        assert_eq!(composed.data[1 * 5 + 1], 100);
        assert_eq!(composed.data[3 * 5 + 3], 100);

        assert!(map.ingest_region_update(
            MapRegionUpdate {
                source: source_with_id("robot_1/local_costmap"),
                reset_source: true,
                ..Default::default()
            },
            0,
        ));

        let composed = map.compose().unwrap();
        assert_eq!(map.dynamic_observation_count(), 1);
        assert_eq!(composed.data[1 * 5 + 1], 0);
        assert_eq!(composed.data[3 * 5 + 3], 100);
    }

    #[test]
    fn repeated_regions_refresh_rasterized_cells_without_stacking() {
        let mut map = LayeredMap::default();
        map.set_static_map(static_grid(3, 3, 0));

        let mut first = update(MapRegionPatch::UPDATE_OBSTACLE, vec![point(1.0, 1.0)]);
        first.patches[0].occupancy_value = 40;
        first.patches[0].ttl_sec = 1.0;
        assert!(map.ingest_region_update(first, 0));
        assert_eq!(map.dynamic_observation_count(), 1);

        let mut refreshed = update(MapRegionPatch::UPDATE_OBSTACLE, vec![point(1.0, 1.0)]);
        refreshed.source.header.stamp.sec = 2;
        refreshed.patches[0].occupancy_value = 80;
        refreshed.patches[0].ttl_sec = 10.0;
        assert!(map.ingest_region_update(refreshed, 0));
        assert_eq!(map.dynamic_observation_count(), 1);
        assert!(!map.prune_expired(3 * NANOS_PER_SECOND));

        let composed = map.compose().unwrap();
        assert_eq!(composed.data[1 * 3 + 1], 80);

        assert!(map.prune_expired(13 * NANOS_PER_SECOND));
        assert_eq!(map.dynamic_observation_count(), 0);
    }

    #[test]
    fn longer_lived_cell_evidence_resurfaces_after_newer_evidence_expires() {
        let mut map = LayeredMap::default();
        map.set_static_map(static_grid(3, 3, 0));

        let mut first = update(MapRegionPatch::UPDATE_OBSTACLE, vec![point(1.0, 1.0)]);
        first.patches[0].occupancy_value = 40;
        first.patches[0].ttl_sec = 10.0;
        assert!(map.ingest_region_update(first, 0));

        let mut newer = update(MapRegionPatch::UPDATE_OBSTACLE, vec![point(1.0, 1.0)]);
        newer.source.header.stamp.sec = 2;
        newer.patches[0].occupancy_value = 80;
        newer.patches[0].ttl_sec = 1.0;
        assert!(map.ingest_region_update(newer, 0));
        assert_eq!(map.dynamic_observation_count(), 1);
        assert_eq!(map.compose().unwrap().data[1 * 3 + 1], 80);

        assert!(map.prune_expired(4 * NANOS_PER_SECOND));
        assert_eq!(map.dynamic_observation_count(), 1);
        assert_eq!(map.compose().unwrap().data[1 * 3 + 1], 40);

        assert!(map.prune_expired(12 * NANOS_PER_SECOND));
        assert_eq!(map.dynamic_observation_count(), 0);
    }

    #[test]
    fn most_recent_source_wins_between_same_type_cell_contributions() {
        let mut map = LayeredMap::default();
        map.set_static_map(static_grid(3, 3, 0));

        let mut first = update_from(
            "robot_1/local_costmap",
            MapRegionPatch::UPDATE_OBSTACLE,
            vec![point(1.0, 1.0)],
        );
        first.patches[0].occupancy_value = 40;
        assert!(map.ingest_region_update(first, 0));

        let mut second = update_from(
            "robot_2/local_costmap",
            MapRegionPatch::UPDATE_OBSTACLE,
            vec![point(1.0, 1.0)],
        );
        second.patches[0].occupancy_value = 80;
        second.patches[0].ttl_sec = 1.0;
        assert!(map.ingest_region_update(second, 0));
        assert_eq!(map.dynamic_observation_count(), 2);

        let composed = map.compose().unwrap();
        assert_eq!(composed.data[1 * 3 + 1], 80);

        assert!(map.prune_expired(3 * NANOS_PER_SECOND));
        let composed = map.compose().unwrap();
        assert_eq!(composed.data[1 * 3 + 1], 40);
    }

    #[test]
    fn static_map_updates_preserve_cells_only_when_geometry_matches() {
        let mut map = LayeredMap::default();
        map.set_static_map(static_grid(3, 3, 0));
        assert!(map.ingest_region_update(
            update(MapRegionPatch::UPDATE_OBSTACLE, vec![point(1.0, 1.0)]),
            0,
        ));

        map.set_static_map(static_grid(3, 3, -1));

        assert_eq!(map.dynamic_observation_count(), 1);
        let composed = map.compose().unwrap();
        assert_eq!(composed.data[1 * 3 + 1], 100);
        assert_eq!(composed.data[0], -1);

        map.set_static_map(static_grid(4, 4, 0));

        assert_eq!(map.dynamic_observation_count(), 0);
        assert!(map.compose().unwrap().data.iter().all(|cell| *cell == 0));
    }

    #[test]
    fn updates_are_rejected_until_a_static_grid_can_rasterize_them() {
        let mut map = LayeredMap::default();

        assert!(!map.ingest_region_update(
            update(MapRegionPatch::UPDATE_OBSTACLE, vec![point(1.0, 1.0)]),
            0,
        ));
        assert_eq!(map.dynamic_observation_count(), 0);
    }

    #[test]
    fn pending_region_update_queue_is_bounded_and_fifo() {
        let mut pending = VecDeque::new();

        for index in 0..=MAX_PENDING_REGION_UPDATES {
            let mut update = update(MapRegionPatch::UPDATE_OBSTACLE, vec![point(1.0, 1.0)]);
            update.source.source_id = format!("source_{index}");
            let dropped = push_pending_region_update(&mut pending, update);
            if index < MAX_PENDING_REGION_UPDATES {
                assert!(dropped.is_none());
            } else {
                assert_eq!(dropped.unwrap().source.source_id, "source_0");
            }
        }

        assert_eq!(pending.len(), MAX_PENDING_REGION_UPDATES);
        assert_eq!(pending.front().unwrap().source.source_id, "source_1");
        assert_eq!(
            pending.back().unwrap().source.source_id,
            format!("source_{MAX_PENDING_REGION_UPDATES}")
        );
    }
}
