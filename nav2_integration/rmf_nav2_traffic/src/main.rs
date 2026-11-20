use bevy::{app::ScheduleRunnerPlugin, log::LogPlugin, prelude::*};
use nav2_traffic::*;
use std::time::Duration;

fn main() {
    let mut app = App::new();
    app.add_plugins((
        MinimalPlugins.set(ScheduleRunnerPlugin::run_loop(Duration::from_millis(20))),
        LogPlugin::default(),
    ));
    app.add_plugins(Nav2TrafficPlugin::default());
    app.run();
}
