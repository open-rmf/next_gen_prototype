use bevy::{log::LogPlugin, prelude::*};
use nav2_traffic::*;
use std::fs;

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let config_path = args
        .windows(2)
        .find(|w| w[0] == "-c" || w[0] == "--config")
        .map(|w| w[1].clone())
        .expect("Missing -c or --config argument");

    let config_contents = fs::read_to_string(&config_path).expect("Failed to read config file");
    let config: AgentConfig = serde_yaml::from_str(&config_contents).expect("Failed to parse config YAML");

    let mut configured_agents = ConfiguredAgents { names: vec![] };
    for agent_name in config.agents.keys() {
        configured_agents.names.push(agent_name.clone());
    }

    let mut app = App::new();
    app.insert_resource(configured_agents);
    app.add_plugins((MinimalPlugins, LogPlugin::default()));
    app.add_plugins(Nav2TrafficPlugin::default());
    app.run();
}
