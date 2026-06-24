use crate::{RclrsNode, RosPublisher, RosSubscription};
use bevy::prelude::*;
use mapf::negotiation::scenario::Agent;
use {
    geometry_msgs::msg::{PoseWithCovarianceStamped, TwistWithCovariance},
    nav_msgs::msg::Odometry,
    rmf_prototype_msgs::msg::{Participant, ParticipantList, SafeZoneId},
};
use std::sync::Arc;

#[derive(Component, Clone, Debug)]
pub struct Nav2Agent {
    pub agent: Agent,
    pub id: i32,
    pub name: String,
    pub localized: bool,
    pub last_safe_zone_id: Option<SafeZoneId>,
}

impl Nav2Agent {
    pub fn new(name: String) -> Self {
        // TODO(@xiyuoh) review this - danger of duplicate IDs
        // Maybe have an accumulator mapping id to agent name as a resource
        let id = name
            .chars()
            .last()
            .and_then(|id| id.to_digit(10))
            .unwrap_or(0) as i32;

        Self {
            agent: Agent {
                start: [0, 0],
                yaw: 0.0,
                goal: [10, 10],
                radius: 0.5,
                speed: 1.0,
                spin: 1.0,
            },
            id,
            name,
            localized: false,
            last_safe_zone_id: None,
        }
    }
}

#[derive(Component)]
pub struct AmclPose(pub PoseWithCovarianceStamped);

#[derive(Component)]
pub struct AmclPoseSubscription {
    pub subscriber: Arc<RosSubscription<PoseWithCovarianceStamped>>,
}

#[derive(Component)]
pub struct OdomPublisher {
    pub publisher: Arc<RosPublisher<Odometry>>,
}

#[derive(Resource)]
pub struct DiscoveryPublisher {
    pub publisher: Arc<RosPublisher<ParticipantList>>,
}

impl FromWorld for DiscoveryPublisher {
    fn from_world(world: &mut World) -> Self {
        let node = world.resource::<RclrsNode>();
        let publisher = Arc::new(RosPublisher::<ParticipantList>::new_transient_local(
            &node,
            "/destination/discovery".to_string(),
        ));
        Self { publisher }
    }
}

#[derive(Default)]
pub struct Nav2AgentPlugin {}

impl Plugin for Nav2AgentPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<DiscoveryPublisher>()
            .add_systems(PreUpdate, update_amcl_pose)
            .add_observer(create_amcl_pose_subscriber)
            .add_observer(publish_discovery);
    }
}

fn create_amcl_pose_subscriber(
    trigger: Trigger<OnAdd, Nav2Agent>,
    mut commands: Commands,
    agents: Query<&Nav2Agent>,
    node: Res<RclrsNode>,
) {
    let e = trigger.target();
    let Ok(agent_name) = agents.get(e).map(|agent| agent.name.clone()) else {
        return;
    };
    let pose_topic = agent_name.clone() + "/inner/amcl_pose";
    let pose_subscriber = Arc::new(RosSubscription::<PoseWithCovarianceStamped>::new(
        &node,
        pose_topic.clone(),
    ));
    let odom_topic = agent_name.clone() + "/odom";
    let odom_publisher = Arc::new(RosPublisher::<Odometry>::new(&node, odom_topic));

    commands.entity(e).insert((
        AmclPoseSubscription {
            subscriber: Arc::clone(&pose_subscriber),
        },
        OdomPublisher {
            publisher: Arc::clone(&odom_publisher),
        },
        AmclPose(PoseWithCovarianceStamped::default()),
    ));
}

fn update_amcl_pose(mut agents: Query<(&mut AmclPose, &AmclPoseSubscription, &OdomPublisher)>) {
    for (mut amcl_pose, amcl_pose_sub, odom_pub) in agents.iter_mut() {
        if let Some(amcl_pose_msg) = amcl_pose_sub.subscriber.data_callback() {
            amcl_pose.0 = amcl_pose_msg.clone();
        }

        let mut odom = Odometry::default();
        odom.header = amcl_pose.0.header.clone();
        odom.child_frame_id = "odom".to_string();
        odom.pose = amcl_pose.0.pose.clone();
        odom.twist = TwistWithCovariance::default();
        let _ = odom_pub.publisher.publish(odom);
    }
}

fn publish_discovery(
    _trigger: Trigger<OnAdd, Nav2Agent>,
    publisher: Res<DiscoveryPublisher>,
    agents: Query<&Nav2Agent>,
) {
    let mut msg = ParticipantList::default();
    for agent in agents.iter() {
        msg.participants.push(Participant {
            name: agent.name.clone(),
            components: vec![],
        });
    }
    let _ = publisher.publisher.publish(msg);
}
