use bevy::prelude::*;
use nav2_traffic::*;

fn main() {
    let mut app = App::new();
    app.add_plugins(DefaultPlugins);
    // .set(WindowPlugin {
    //     primary_window: None,
    //     ..default()
    // }));
    app.add_plugins(Nav2TrafficPlugin::default());
    app.run();
}
