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
    nav_msgs::msg::OccupancyGrid,
    rmf_layered_map_msgs::msg::{MapRegionPatch, MapRegionUpdate},
    rmf_prototype_msgs::msg::Region,
};
use std::{collections::HashMap, time::Duration};

const NANOS_PER_SECOND: i128 = 1_000_000_000;
const MAP_QOS_DEPTH: u32 = 10;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum DynamicUpdateType {
    Obstacle,
    Clear,
}

#[derive(Clone, Debug)]
struct DynamicObservation {
    source_id: String,
    map_name: String,
    update_type: DynamicUpdateType,
    occupancy_value: i8,
    regions: Vec<Region>,
    expires_at_nsec: i128,
}

#[derive(Clone, Debug, Hash, PartialEq, Eq)]
struct ObservationSourceKey {
    source_id: String,
    map_name: String,
}

#[derive(Clone, Debug)]
pub struct LayeredMap {
    static_grid: Option<OccupancyGrid>,
    dynamic_observations: Vec<DynamicObservation>,
    latest_source_stamps: HashMap<ObservationSourceKey, i128>,
    default_ttl_nsec: i128,
    revision: u64,
}

impl LayeredMap {
    pub fn new(default_ttl: Duration) -> Self {
        Self {
            static_grid: None,
            dynamic_observations: Vec::new(),
            latest_source_stamps: HashMap::new(),
            default_ttl_nsec: duration_to_nsec(default_ttl),
            revision: 0,
        }
    }

    pub fn revision(&self) -> u64 {
        self.revision
    }

    pub fn dynamic_observation_count(&self) -> usize {
        self.dynamic_observations.len()
    }

    pub fn set_static_map(&mut self, mut grid: OccupancyGrid) {
        normalize_grid_data(&mut grid);
        self.static_grid = Some(grid);
        self.revision = self.revision.wrapping_add(1);
    }

    pub fn ingest_region_update(&mut self, update: MapRegionUpdate, now_nsec: i128) -> bool {
        if update.source.header.stamp.sec == 0 && update.source.header.stamp.nanosec == 0 {
            return false;
        }

        let reset_source = update.reset_source;
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
            let before = self.dynamic_observations.len();
            self.dynamic_observations
                .retain(|obs| obs.source_id != key.source_id || obs.map_name != key.map_name);
            changed |= before != self.dynamic_observations.len();
        }

        let default_ttl_sec = update.source.default_ttl_sec;
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

            let regions: Vec<_> = patch
                .regions
                .into_iter()
                .filter(|region| region_validation_error(region).is_none())
                .collect();
            if regions.is_empty() {
                continue;
            }

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

            self.dynamic_observations.push(DynamicObservation {
                source_id: key.source_id.clone(),
                map_name: key.map_name.clone(),
                update_type,
                occupancy_value,
                regions,
                expires_at_nsec,
            });
            changed = true;
        }

        if changed || reset_source {
            self.latest_source_stamps.insert(key, stamp_nsec);
        }

        if changed {
            self.revision = self.revision.wrapping_add(1);
        }
        changed
    }

    pub fn prune_expired(&mut self, now_nsec: i128) -> bool {
        let before = self.dynamic_observations.len();
        self.dynamic_observations
            .retain(|obs| obs.expires_at_nsec > now_nsec);
        let changed = before != self.dynamic_observations.len();
        if changed {
            self.revision = self.revision.wrapping_add(1);
        }
        changed
    }

    pub fn compose(&self) -> Option<OccupancyGrid> {
        let mut composed = self.static_grid.clone()?;
        normalize_grid_data(&mut composed);

        for observation in self
            .dynamic_observations
            .iter()
            .filter(|obs| obs.update_type == DynamicUpdateType::Clear)
        {
            rasterize_observation(&mut composed, observation);
        }

        for observation in self
            .dynamic_observations
            .iter()
            .filter(|obs| obs.update_type == DynamicUpdateType::Obstacle)
        {
            rasterize_observation(&mut composed, observation);
        }

        Some(composed)
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
    pub default_ttl: Duration,
    pub publish_period: Duration,
}

impl Default for LayeredMapServerConfig {
    fn default() -> Self {
        Self {
            static_map_topic: "/map/static".to_string(),
            region_updates_topic: "/map/region_updates".to_string(),
            composed_map_topic: "/map".to_string(),
            default_ttl: Duration::from_secs(30),
            publish_period: Duration::from_millis(250),
        }
    }
}

pub struct LayeredMapServer {
    node: Node,
    map: LayeredMap,
    map_publisher: rclrs::Publisher<OccupancyGrid>,
    last_published_revision: Option<u64>,
}

impl LayeredMapServer {
    pub fn new(
        node: Node,
        config: &LayeredMapServerConfig,
    ) -> Result<Self, Box<dyn std::error::Error>> {
        let map_publisher =
            node.create_publisher::<OccupancyGrid>(composed_map_qos(&config.composed_map_topic))?;

        Ok(Self {
            node,
            map: LayeredMap::new(config.default_ttl),
            map_publisher,
            last_published_revision: None,
        })
    }

    fn handle_static_map(&mut self, msg: OccupancyGrid) {
        self.map.set_static_map(msg);
        rclrs::log!(
            self.node.logger(),
            "Received static map, revision {}",
            self.map.revision()
        );
        self.publish_if_changed();
    }

    fn handle_region_update(&mut self, msg: MapRegionUpdate) {
        log_region_update_errors(&self.node, &msg);
        let now_nsec = self.now_nsec();
        if self.map.ingest_region_update(msg, now_nsec) {
            rclrs::log!(
                self.node.logger(),
                "Accepted region update, revision {}, active observations {}",
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
                "Pruned expired observations, revision {}, active observations {}",
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

        match self.map_publisher.publish(&composed) {
            Ok(()) => {
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

fn region_update_qos(topic: &str) -> PrimitiveOptions<'_> {
    topic.keep_last(MAP_QOS_DEPTH).reliable()
}

fn log_region_update_errors(node: &Node, update: &MapRegionUpdate) {
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
        Region::HINT_POINT | Region::HINT_AXIS_ALIGNED_RECTANGLE => None,
        hint => Some(format!(
            "unsupported region hint {}; expected a point or axis-aligned rectangle",
            hint
        )),
    }
}

fn is_supported_region_hint(hint: u8) -> bool {
    matches!(
        hint,
        Region::HINT_POINT | Region::HINT_AXIS_ALIGNED_RECTANGLE
    )
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

fn rasterize_observation(grid: &mut OccupancyGrid, observation: &DynamicObservation) {
    for region in &observation.regions {
        for index in rasterized_indices(grid, region) {
            if let Some(cell) = grid.data.get_mut(index) {
                *cell = observation.occupancy_value;
            }
        }
    }
}

fn rasterized_indices(grid: &OccupancyGrid, region: &Region) -> Vec<usize> {
    let Some((width, height, resolution, origin_x, origin_y)) = grid_geometry(grid) else {
        return Vec::new();
    };

    if region.points.len() < 2 || region.points.len() % 2 != 0 {
        return Vec::new();
    }

    if !is_supported_region_hint(region.hint) {
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
    fn out_of_bounds_regions_do_not_touch_the_grid() {
        let mut map = LayeredMap::default();
        map.set_static_map(static_grid(3, 3, 0));

        assert!(map.ingest_region_update(
            update(
                MapRegionPatch::UPDATE_OBSTACLE,
                vec![rectangle(10.0, 10.0, 11.0, 11.0)]
            ),
            0,
        ));

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
}
