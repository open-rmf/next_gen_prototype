/*
 * Copyright (C) 2026 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

use bevy::prelude::*;
use bevy_ros2::RclrsPlugin;
use crossflow::CrossflowPlugin;

pub mod agent;
pub use agent::*;

pub mod destination;
pub use destination::*;

pub mod inner_navigation_client;
pub use inner_navigation_client::*;

pub mod navigation_server;
pub use navigation_server::*;

pub mod safe_zone;
pub use safe_zone::*;

#[derive(Default)]
pub struct Nav2TrafficPlugin {}

impl Plugin for Nav2TrafficPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins((CrossflowPlugin::default(), RclrsPlugin::default()))
            .add_plugins((
                DestinationGoalPublisherPlugin::default(),
                SafeZoneSubscriptionPlugin::default(),
                InnerNavigationClientPlugin::default(),
                NavigationServerPlugin::default(),
                Nav2AgentPlugin::default(),
            ));

        // Spawn agents last
        for name in resolve_agent_names() {
            app.world_mut().spawn(Nav2Agent::new(name));
        }
    }
}

fn resolve_agent_names() -> Vec<String> {
    if let Ok(env_agents) = std::env::var("RMF_NAV2_AGENTS") {
        let names: Vec<String> = env_agents
            .split(|c: char| c == ',' || c.is_whitespace())
            .map(str::trim)
            .filter(|s| !s.is_empty())
            .map(ToOwned::to_owned)
            .collect();
        if !names.is_empty() {
            return names;
        }
    }

    let cli_names: Vec<String> = std::env::args()
        .skip(1)
        .take_while(|arg| arg != "--ros-args")
        .filter(|arg| !arg.starts_with('-') && !arg.starts_with('_') && !arg.contains(":="))
        .collect();
    if !cli_names.is_empty() {
        return cli_names;
    }

    vec!["robot0".to_string(), "robot1".to_string()]
}
