use bevy::prelude::*;

pub mod destination_server;
pub use destination_server::*;

pub mod mapf_bridge;
pub use mapf_bridge::*;

pub mod safe_zone;
pub use safe_zone::*;

#[derive(Default)]
pub struct TestingPlugin {}

impl Plugin for TestingPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins((
            MockSafeZonePlugin::default(),
            MockDestinationServerPlugin::default(),
            MapfBridgePlugin::default(),
        ));
    }
}
