use crate::{Nav2Agent, RclrsNode, RosSubscription};
use bevy::prelude::*;
use ros_env::rmf_prototype_msgs::msg::DestinationGoal;
use std::sync::Arc;

#[derive(Event)]
pub struct RequestPlan(pub Entity);

#[derive(Component)]
pub struct DestinationGoalSubscription {
    pub subscriber: Arc<RosSubscription<DestinationGoal>>,
    pub last_msg: Option<DestinationGoal>,
}

#[derive(Default)]
pub struct MockDestinationServerPlugin {}

impl Plugin for MockDestinationServerPlugin {
    fn build(&self, app: &mut App) {
        app.add_event::<RequestPlan>()
            .add_systems(PreUpdate, receive_destination_goal)
            .add_observer(create_destination_goal_subscription);
    }
}

fn create_destination_goal_subscription(
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
    let subscriber = Arc::new(RosSubscription::<DestinationGoal>::new(
        &node,
        topic.clone(),
    ));
    commands.entity(e).insert(DestinationGoalSubscription {
        subscriber: Arc::clone(&subscriber),
        last_msg: None,
    });
}

fn receive_destination_goal(
    mut commands: Commands,
    mut subscriptions: Query<(Entity, &mut DestinationGoalSubscription)>,
) {
    for (e, mut destination_goal_sub) in subscriptions.iter_mut() {
        let Some(destination_goal) = destination_goal_sub.subscriber.data_callback() else {
            continue;
        };
        if destination_goal_sub
            .last_msg
            .as_ref()
            .is_some_and(|msg| *msg == destination_goal)
        {
            continue;
        }
        destination_goal_sub.last_msg = Some(destination_goal);

        // Received new destination goal, trigger a new request
        commands.trigger(RequestPlan(e));
    }
}
