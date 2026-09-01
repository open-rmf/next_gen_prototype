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

use rclrs::{Context, CreateBasicExecutor, ParameterRange, SpinOptions};
use rmf_path_server::{start_path_server, PibtPlanner};
use std::sync::Arc;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let context = Context::default_from_env().unwrap();
    let mut executor = context.create_basic_executor();
    let node = Arc::new(executor.create_node("path_server")?);

    let planning_grid_resolution = node
        .declare_parameter("planning_grid_resolution")
        .default(PibtPlanner::default().grid_resolution as f64)
        .range(ParameterRange {
            lower: Some(f32::MIN_POSITIVE as f64),
            upper: Some(f32::MAX as f64),
            step: None,
        })
        .description("Minimum PiBT grid cell size in meters")
        .read_only()?;
    let planner = PibtPlanner::with_grid_resolution(100, planning_grid_resolution.get())?;

    let _path_server_guard = start_path_server(Arc::clone(&node), planner)?;

    rclrs::log!(
        node.logger(),
        "Path server started with multi-worker separation (DiscoveryWorker + DestinationsWorker). Spinning..."
    );
    executor.spin(SpinOptions::default());
    Ok(())
}
