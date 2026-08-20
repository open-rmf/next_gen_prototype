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

use rclrs::{Context, CreateBasicExecutor, SpinOptions};
use rmf_layered_map_server::{start_layered_map_server, LayeredMapServerConfig};
use std::sync::Arc;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let context = Context::default_from_env().unwrap();
    let mut executor = context.create_basic_executor();
    let node = Arc::new(executor.create_node("layered_map_server")?);

    let _server = start_layered_map_server(Arc::clone(&node), LayeredMapServerConfig::default())?;

    rclrs::log!(
        node.logger(),
        "Layered map server started. Composing /map/static and /map/region_updates into /map."
    );
    executor.spin(SpinOptions::default());
    Ok(())
}
