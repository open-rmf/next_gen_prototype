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
use bevy_ros2::{RclrsNode, RclrsPlugin};
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

use std::sync::Arc;

#[derive(Clone, Debug)]
pub struct Nav2TrafficPlugin {
    pub agent_names: Vec<String>,
    pub enable_destination_goal_publisher: Option<bool>,
    pub enable_safe_zone_subscription: Option<bool>,
    pub enable_inner_navigation_client: Option<bool>,
    pub enable_navigation_server: Option<bool>,
    pub enable_nav2_agent: Option<bool>,
}

impl Default for Nav2TrafficPlugin {
    fn default() -> Self {
        Self {
            agent_names: Vec::new(),
            enable_destination_goal_publisher: None,
            enable_safe_zone_subscription: None,
            enable_inner_navigation_client: None,
            enable_navigation_server: None,
            enable_nav2_agent: None,
        }
    }
}

impl Plugin for Nav2TrafficPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins((CrossflowPlugin::default(), RclrsPlugin::default()));

        let (
            agents_to_spawn,
            enable_dest_pub,
            enable_safe_zone,
            enable_inner_nav,
            enable_nav_server,
            enable_agent_plugin,
        ) = {
            let node = app.world().resource::<RclrsNode>();

            // 1. Determine agent names (defaults to 1 robot per plugin instance via `agent_name` ROS param)
            let mut agents = self.agent_names.clone();
            if agents.is_empty() {
                if let Ok(param) = node
                    .declare_parameter::<Arc<str>>("agent_name")
                    .default(Arc::from("robot0"))
                    .optional()
                {
                    if let Some(name) = param.get() {
                        let name_str = name.to_string();
                        if !name_str.is_empty() {
                            agents.push(name_str);
                        }
                    }
                } else if let Ok(param) = node
                    .declare_parameter::<Arc<str>>("agent_names")
                    .default(Arc::from("robot0"))
                    .optional()
                {
                    if let Some(names) = param.get() {
                        for name in names.replace(',', " ").split_whitespace() {
                            agents.push(name.to_string());
                        }
                    }
                }
            }
            if agents.is_empty() {
                agents.push("robot0".to_string());
            }

            // 2. Determine which plugins to enable
            let plugins_param: Option<String> = node
                .declare_parameter::<Arc<str>>("plugins")
                .default(Arc::from(""))
                .optional()
                .ok()
                .and_then(|param| param.get().map(|s| s.to_string()))
                .filter(|s| !s.is_empty());

            let is_plugin_enabled = |plugin_name: &str, struct_setting: Option<bool>, param_flag_name: &str| -> bool {
                if let Some(enabled) = struct_setting {
                    return enabled;
                }
                if let Some(ref plugins_list) = plugins_param {
                    return plugins_list.contains(plugin_name);
                }
                if let Ok(param) = node.declare_parameter::<bool>(param_flag_name).default(true).optional() {
                    if let Some(enabled) = param.get() {
                        return enabled;
                    }
                }
                true
            };

            (
                agents,
                is_plugin_enabled(
                    "destination_goal_publisher",
                    self.enable_destination_goal_publisher,
                    "enable_destination_goal_publisher",
                ),
                is_plugin_enabled(
                    "safe_zone_subscription",
                    self.enable_safe_zone_subscription,
                    "enable_safe_zone_subscription",
                ),
                is_plugin_enabled(
                    "inner_navigation_client",
                    self.enable_inner_navigation_client,
                    "enable_inner_navigation_client",
                ),
                is_plugin_enabled(
                    "navigation_server",
                    self.enable_navigation_server,
                    "enable_navigation_server",
                ),
                is_plugin_enabled(
                    "nav2_agent",
                    self.enable_nav2_agent,
                    "enable_nav2_agent",
                ),
            )
        };

        if enable_dest_pub {
            app.add_plugins(DestinationGoalPublisherPlugin::default());
        }
        if enable_safe_zone {
            app.add_plugins(SafeZoneSubscriptionPlugin::default());
        }
        if enable_inner_nav {
            app.add_plugins(InnerNavigationClientPlugin::default());
        }
        if enable_nav_server {
            app.add_plugins(NavigationServerPlugin::default());
        }
        if enable_agent_plugin {
            app.add_plugins(Nav2AgentPlugin::default());
        }

        // Spawn agents
        for name in agents_to_spawn {
            app.world_mut().spawn(Nav2Agent::new(name));
        }
    }
}

