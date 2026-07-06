use bevy::{log::LogPlugin, prelude::*};
use nav2_traffic::*;

fn main() {
    let mut app = App::new();
    app.add_plugins((MinimalPlugins, LogPlugin::default()));
    app.add_plugins(Nav2TrafficPlugin::default());
    app.run();
}
