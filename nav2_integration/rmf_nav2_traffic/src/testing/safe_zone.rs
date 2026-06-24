use crate::{Nav2Agent, RclrsNode, RosPublisher};
use bevy::prelude::*;
use rmf_prototype_msgs::msg::SafeZone;
use std::sync::Arc;

#[derive(Component)]
pub struct SafeZonePublisher {
    pub publisher: Arc<RosPublisher<SafeZone>>,
}

#[derive(Debug, Clone, Event)]
pub struct SafeZoneReceived {
    pub agent: Entity,
    pub safe_zone: SafeZone,
}

#[derive(Default)]
pub struct MockSafeZonePlugin {}

impl Plugin for MockSafeZonePlugin {
    fn build(&self, app: &mut App) {
        app.add_event::<SafeZoneReceived>()
            .add_observer(create_safe_zone_publisher)
            .add_observer(publish_safe_zone);
    }
}

fn create_safe_zone_publisher(
    trigger: Trigger<OnAdd, Nav2Agent>,
    mut commands: Commands,
    agents: Query<&Nav2Agent>,
    node: Res<RclrsNode>,
) {
    let e = trigger.target();
    let Ok(agent_name) = agents.get(e).map(|agent| agent.name.clone()) else {
        return;
    };
    let topic = agent_name + "/plan/safe_zone";
    let publisher = Arc::new(RosPublisher::<SafeZone>::new(&node, topic.clone()));
    commands.entity(e).insert(SafeZonePublisher {
        publisher: Arc::clone(&publisher),
    });
}

fn publish_safe_zone(
    trigger: Trigger<SafeZoneReceived>,
    safe_zone_publisher: Query<&SafeZonePublisher>,
) {
    let Ok(publisher) = safe_zone_publisher.get(trigger.event().agent) else {
        return;
    };
    let safe_zone = trigger.event().safe_zone.clone();
    let _ = publisher.publisher.publish(safe_zone);
}
