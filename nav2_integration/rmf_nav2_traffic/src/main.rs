use bevy::{log::LogPlugin, prelude::*};
use clap::Parser;
use nav2_traffic::*;
use std::fs;

#[derive(Parser)]
#[command(author, version, about = "RMF Nav2 Traffic manager")]
struct Cli {
    /// Path to the YAML config file
    #[arg(short = 'c', long)]
    config: std::path::PathBuf,

    /// Absorb any trailing arguments appended by ROS 2 launch (e.g. --ros-args)
    #[arg(trailing_var_arg = true, allow_hyphen_values = true, hide = true)]
    _ros_args: Vec<String>,
}

fn main() {
    let cli = Cli::parse();

    let config_contents = fs::read_to_string(&cli.config).expect("Failed to read config file");
    let config: AgentConfig =
        serde_yaml::from_str(&config_contents).expect("Failed to parse config YAML");

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
