use crate::{Nav2Agent, RclrsNode, RosPublisher};
use bevy::prelude::*;
use rmf_prototype_msgs::msg::DestinationGoal;
use std::sync::Arc;

#[derive(Component)]
pub struct DestinationGoalPublisher {
    pub publisher: Arc<RosPublisher<DestinationGoal>>,
}

#[derive(Default)]
pub struct DestinationGoalPublisherPlugin {}

impl Plugin for DestinationGoalPublisherPlugin {
    fn build(&self, app: &mut App) {
        app.add_observer(create_destination_goal_publisher);
    }
}

fn create_destination_goal_publisher(
    trigger: Trigger<OnAdd, Nav2Agent>,
    mut commands: Commands,
    agents: Query<&Nav2Agent>,
    node: Res<RclrsNode>,
) {
    let e = trigger.target();
    let Ok(agent_name) = agents.get(e).map(|agent| agent.name.clone()) else {
        return;
    };
    let topic = agent_name + "/destination/goal";
    let publisher = Arc::new(RosPublisher::<DestinationGoal>::new_transient_local(
        &node, topic,
    ));
    commands.entity(e).insert(DestinationGoalPublisher {
        publisher: Arc::clone(&publisher),
    });
}
