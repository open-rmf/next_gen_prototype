use crate::Nav2Agent;
use bevy::prelude::*;
use bevy_ros2::RclrsNode;
use std::collections::HashSet;

#[derive(Resource, Default)]
pub struct AgentTracker {
    pub known_agents: HashSet<String>,
}

#[derive(Default)]
pub struct AgentDiscoveryPlugin {}

impl Plugin for AgentDiscoveryPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<AgentTracker>()
            .add_systems(PreUpdate, discover_agents_via_graph);
    }
}

// Discover agents via existing Nav2 nodes
fn discover_agents_via_graph(
    mut commands: Commands,
    node: Res<RclrsNode>,
    mut tracker: ResMut<AgentTracker>,
) {
    let Ok(node_names) = node.get_node_names() else {
        return;
    };

    for info in node_names {
        if info.name == "bt_navigator" {
            // Remove /inner suffix
            let namespace = info.namespace.trim_start_matches('/');
            let namespace = namespace.strip_suffix("/inner").unwrap_or(namespace);

            if namespace.is_empty() {
                continue;
            }

            let robot_name = namespace.to_string();

            if !tracker.known_agents.contains(&robot_name) {
                info!("Discovered new Nav2 agent from ROS graph: {}", robot_name);
                tracker.known_agents.insert(robot_name.clone());
                commands.spawn(Nav2Agent::new(robot_name));
            }
        }
    }
}
