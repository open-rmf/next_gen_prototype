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

use rmf_layered_map_server::LayeredMap;
use ros_env::{
    geometry_msgs::msg::{Point, Pose, Quaternion},
    nav_msgs::msg::{MapMetaData, OccupancyGrid},
    rmf_layered_map_msgs::msg::{MapObservationSource, MapRegionPatch, MapRegionUpdate},
    rmf_prototype_msgs::msg::Region,
    std_msgs::msg::Header,
};
use std::{
    collections::hash_map::DefaultHasher,
    env, fs,
    hash::{Hash, Hasher},
    hint::black_box,
    io::{self, Write},
    time::{Duration, Instant},
};

const NANOS_PER_SECOND: i128 = 1_000_000_000;
const UPDATES: usize = 240;
const ROUNDS: usize = 8;
const WARMUP_ROUNDS: usize = 2;
const BEAMS: usize = 90;
const SOURCES: usize = 2;
const EVENT_PERIOD_NSEC: i128 = 250_000_000;
const TTL_SEC: f64 = 10.0;
const MAP_WIDTH: u32 = 240;
const MAP_HEIGHT: u32 = 180;
const MAP_RESOLUTION: f32 = 0.1;
const SCAN_RADIUS: f64 = 4.0;

#[derive(Clone, Copy)]
enum Scenario {
    Reset,
    RollingOverlap,
    RollingMoving,
}

impl Scenario {
    fn parse(value: &str) -> Result<Self, String> {
        match value {
            "reset" => Ok(Self::Reset),
            "rolling-overlap" => Ok(Self::RollingOverlap),
            "rolling-moving" => Ok(Self::RollingMoving),
            _ => Err(format!(
                "unsupported scenario '{value}'; expected reset, rolling-overlap, or rolling-moving"
            )),
        }
    }

    fn name(self) -> &'static str {
        match self {
            Self::Reset => "reset",
            Self::RollingOverlap => "rolling-overlap",
            Self::RollingMoving => "rolling-moving",
        }
    }
}

struct Config {
    label: String,
    scenario: Scenario,
}

#[derive(Clone)]
struct TraceEvent {
    update: MapRegionUpdate,
    now_nsec: i128,
}

#[derive(Default)]
struct Samples {
    ingest: Vec<Duration>,
    prune: Vec<Duration>,
    compose: Vec<Duration>,
    total: Vec<Duration>,
}

struct ReplayResult {
    samples: Samples,
    checksum: u64,
}

fn main() -> Result<(), String> {
    let config = parse_args()?;
    println!(
        "Running layered map benchmark ({})...",
        config.scenario.name()
    );
    io::stdout()
        .flush()
        .map_err(|error| format!("failed to flush benchmark output: {error}"))?;

    let trace = make_trace(&config);

    for _ in 0..WARMUP_ROUNDS {
        black_box(replay(&trace, false).checksum);
    }

    let mut samples = Samples::default();
    let mut expected_checksum = None;
    for _ in 0..ROUNDS {
        let result = replay(&trace, true);
        if let Some(expected) = expected_checksum {
            if result.checksum != expected {
                return Err(format!(
                    "non-deterministic output checksum: expected {expected}, got {}",
                    result.checksum
                ));
            }
        } else {
            expected_checksum = Some(result.checksum);
        }
        samples.ingest.extend(result.samples.ingest);
        samples.prune.extend(result.samples.prune);
        samples.compose.extend(result.samples.compose);
        samples.total.extend(result.samples.total);
    }

    let measured_updates = UPDATES * ROUNDS;
    let measured_seconds = samples.total.iter().map(Duration::as_secs_f64).sum::<f64>();
    let throughput = measured_updates as f64 / measured_seconds;
    let peak_rss_kib = peak_rss_kib().unwrap_or(0);

    println!();
    println!("Results");
    println!("  Label       {}", config.label);
    println!("  Scenario    {}", config.scenario.name());
    println!("  Workload    {UPDATES} updates x {ROUNDS} rounds ({WARMUP_ROUNDS} warm-up)");
    println!("  Scan        {BEAMS} beams x {SOURCES} sources");
    println!("  Throughput  {throughput:.1} updates/s");
    println!("  Peak RSS    {:.2} MiB", peak_rss_kib as f64 / 1024.0);
    println!("  Checksum    {}", expected_checksum.unwrap_or(0));
    println!();
    println!("Latency (ms)");
    println!(
        "  {:<10} {:>10} {:>10} {:>10} {:>10}",
        "Phase", "Mean", "P50", "P95", "P99"
    );
    print_stats("ingest", samples.ingest);
    print_stats("prune", samples.prune);
    print_stats("compose", samples.compose);
    print_stats("total", samples.total);

    Ok(())
}

fn parse_args() -> Result<Config, String> {
    let mut config = Config {
        label: "unlabeled".to_string(),
        scenario: Scenario::Reset,
    };

    let mut args = env::args().skip(1);
    while let Some(flag) = args.next() {
        let value = args
            .next()
            .ok_or_else(|| format!("missing value for argument '{flag}'"))?;
        match flag.as_str() {
            "--label" => config.label = value,
            "--scenario" => config.scenario = Scenario::parse(&value)?,
            _ => return Err(format!("unsupported argument '{flag}'")),
        }
    }

    Ok(config)
}

fn make_trace(config: &Config) -> Vec<TraceEvent> {
    (0..UPDATES)
        .map(|event_index| {
            let source_index = event_index % SOURCES;
            let source_scan_index = event_index / SOURCES;
            let now_nsec = NANOS_PER_SECOND + event_index as i128 * EVENT_PERIOD_NSEC;
            TraceEvent {
                update: make_update(config.scenario, source_index, source_scan_index, now_nsec),
                now_nsec,
            }
        })
        .collect()
}

fn make_update(
    scenario: Scenario,
    source_index: usize,
    scan_index: usize,
    stamp_nsec: i128,
) -> MapRegionUpdate {
    let mut source = MapObservationSource {
        header: Header {
            frame_id: "map".to_string(),
            ..Default::default()
        },
        source_id: format!("robot_{source_index}/scan"),
        robot_name: format!("robot_{source_index}"),
        map_name: "benchmark_map".to_string(),
        default_ttl_sec: TTL_SEC,
        ..Default::default()
    };
    source.header.stamp.sec = (stamp_nsec / NANOS_PER_SECOND) as i32;
    source.header.stamp.nanosec = (stamp_nsec % NANOS_PER_SECOND) as u32;
    source.robot_pose.position.x = match scenario {
        Scenario::RollingMoving => 7.0 + (scan_index % 40) as f64 * 0.1,
        Scenario::Reset | Scenario::RollingOverlap => 9.0,
    };
    let source_fraction = source_index as f64 / SOURCES.saturating_sub(1).max(1) as f64;
    source.robot_pose.position.y = 5.0 + 7.0 * source_fraction;
    source.robot_pose.orientation.w = 1.0;

    let mut clear_patch = MapRegionPatch {
        update_type: MapRegionPatch::UPDATE_CLEAR,
        occupancy_value: 0,
        ttl_sec: TTL_SEC,
        ..Default::default()
    };
    let mut obstacle_patch = MapRegionPatch {
        update_type: MapRegionPatch::UPDATE_OBSTACLE,
        occupancy_value: 100,
        ttl_sec: TTL_SEC,
        ..Default::default()
    };

    let angle_step = std::f64::consts::TAU / BEAMS as f64;
    for beam in 0..BEAMS {
        let start_angle = beam as f64 * angle_step;
        let end_angle = (beam + 1) as f64 * angle_step;
        let varying_radius = SCAN_RADIUS - 0.35 * ((beam + scan_index) % 7) as f64 / 6.0;
        let start = (
            varying_radius * start_angle.cos(),
            varying_radius * start_angle.sin(),
        );
        let end = (
            varying_radius * end_angle.cos(),
            varying_radius * end_angle.sin(),
        );
        clear_patch.regions.push(Region {
            hint: Region::HINT_CONVEX_POLYGON,
            points: vec![
                0.0,
                0.0,
                start.0 as f32,
                start.1 as f32,
                end.0 as f32,
                end.1 as f32,
            ],
        });

        if beam % 6 == 0 {
            obstacle_patch.regions.push(Region {
                hint: Region::HINT_POINT,
                points: vec![start.0 as f32, start.1 as f32],
            });
        }
    }

    MapRegionUpdate {
        source,
        reset_source: matches!(scenario, Scenario::Reset),
        patches: vec![clear_patch, obstacle_patch],
    }
}

fn replay(trace: &[TraceEvent], record_samples: bool) -> ReplayResult {
    let mut map = LayeredMap::new(Duration::from_secs(TTL_SEC as u64));
    map.set_static_map(static_grid());
    let mut samples = Samples::default();
    let mut hasher = DefaultHasher::new();

    for event in trace.iter().cloned() {
        let total_start = Instant::now();

        let prune_start = Instant::now();
        black_box(map.prune_expired(event.now_nsec));
        let prune_elapsed = prune_start.elapsed();

        let ingest_start = Instant::now();
        black_box(map.ingest_region_update(event.update, event.now_nsec));
        let ingest_elapsed = ingest_start.elapsed();

        let compose_start = Instant::now();
        let grid = map.compose().expect("benchmark always has a static map");
        let compose_elapsed = compose_start.elapsed();

        let total_elapsed = total_start.elapsed();
        grid.data.hash(&mut hasher);
        black_box(&grid);

        if record_samples {
            samples.prune.push(prune_elapsed);
            samples.ingest.push(ingest_elapsed);
            samples.compose.push(compose_elapsed);
            samples.total.push(total_elapsed);
        }
    }

    ReplayResult {
        samples,
        checksum: hasher.finish(),
    }
}

fn static_grid() -> OccupancyGrid {
    OccupancyGrid {
        header: Header {
            frame_id: "map".to_string(),
            ..Default::default()
        },
        info: MapMetaData {
            resolution: MAP_RESOLUTION,
            width: MAP_WIDTH,
            height: MAP_HEIGHT,
            origin: Pose {
                position: Point::default(),
                orientation: Quaternion {
                    w: 1.0,
                    ..Default::default()
                },
            },
            ..Default::default()
        },
        data: vec![0; (MAP_WIDTH * MAP_HEIGHT) as usize],
        ..Default::default()
    }
}

fn print_stats(name: &str, mut values: Vec<Duration>) {
    values.sort_unstable();
    let mean_ms =
        values.iter().map(Duration::as_secs_f64).sum::<f64>() * 1000.0 / values.len() as f64;
    println!(
        "  {name:<10} {:>10.3} {:>10.3} {:>10.3} {:>10.3}",
        mean_ms,
        percentile(&values, 50).as_secs_f64() * 1000.0,
        percentile(&values, 95).as_secs_f64() * 1000.0,
        percentile(&values, 99).as_secs_f64() * 1000.0,
    );
}

fn percentile(values: &[Duration], percentile: usize) -> Duration {
    let index = ((values.len() - 1) * percentile).div_ceil(100);
    values[index]
}

fn peak_rss_kib() -> Option<u64> {
    fs::read_to_string("/proc/self/status")
        .ok()?
        .lines()
        .find_map(|line| {
            let value = line.strip_prefix("VmHWM:")?;
            value.split_whitespace().next()?.parse().ok()
        })
}
