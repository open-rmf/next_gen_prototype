use bevy::prelude::*;
use nav2_traffic::*;

fn main() {
    let mut app = App::new();
    app.add_plugins(DefaultPlugins);
    app.add_plugins(Nav2TrafficPlugin::default());
    app.run();
}
