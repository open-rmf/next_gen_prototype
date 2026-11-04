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

pub mod inner_navigation_client;
pub use inner_navigation_client::*;

pub mod safe_zone;
pub use safe_zone::*;

fn parse_agent_names() -> Vec<String> {
    if let Ok(val) = std::env::var("RMF_NAV2_AGENTS") {
        let names: Vec<String> = val
            .split(',')
            .map(|s| s.trim().to_string())
            .filter(|s| !s.is_empty())
            .collect();
        if !names.is_empty() {
            return names;
        }
    }

    let args: Vec<String> = std::env::args().collect();
    for i in 1..args.len() {
        if args[i] == "--agents" && i + 1 < args.len() {
            let names: Vec<String> = args[i + 1]
                .split(',')
                .map(|s| s.trim().to_string())
                .filter(|s| !s.is_empty())
                .collect();
            if !names.is_empty() {
                return names;
            }
        } else if let Some(rest) = args[i].strip_prefix("--agents=") {
            let names: Vec<String> = rest
                .split(',')
                .map(|s| s.trim().to_string())
                .filter(|s| !s.is_empty())
                .collect();
            if !names.is_empty() {
                return names;
            }
        }
    }

    vec!["robot0".to_string(), "robot1".to_string()]
}

#[derive(Default)]
pub struct Nav2TrafficPlugin {}

impl Plugin for Nav2TrafficPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins((CrossflowPlugin::default(), RclrsPlugin::default()))
            .add_plugins((
                SafeZoneSubscriptionPlugin::default(),
                InnerNavigationClientPlugin::default(),
                Nav2AgentPlugin::default(),
            ));

        for name in parse_agent_names() {
            app.world_mut().spawn(Nav2Agent::new(name));
        }
    }
}
