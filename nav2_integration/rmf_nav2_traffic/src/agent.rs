use crate::{
    inner_navigation_client::InnerNavigationClient,
    safe_zone::{
        build_progress_status, CostmapHandshakeState, CurrentSafeZone, LatestProgress,
        ProgressPublisher, ProgressTracker,
    },
};
use bevy::prelude::*;
use bevy_ros2::{RclrsNode, RosPublisher, RosSubscription};
use mapf::negotiation::scenario::Agent;
use ros_env::{
    geometry_msgs::msg::{PoseStamped, PoseWithCovarianceStamped},
    rmf_prototype_msgs::msg::{Participant, ParticipantList, Progress, SafeZoneId},
};
use std::sync::Arc;

#[derive(Component, Clone, Debug)]
pub struct AgentPose(pub PoseStamped);

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

impl AmclPose {
    pub fn xy_yaw(&self) -> (f32, f32, f32) {
        let pos = &self.0.pose.pose.position;
        let q = &self.0.pose.pose.orientation;
        let siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
        let cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
        let yaw = siny_cosp.atan2(cosy_cosp) as f32;
        (pos.x as f32, pos.y as f32, yaw)
    }
}

#[derive(Component)]
pub struct AmclPoseSubscription {
    pub subscriber: Arc<RosSubscription<PoseWithCovarianceStamped>>,
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
            .add_systems(PreUpdate, (publish_discovery, update_amcl_pose))
            .add_observer(create_amcl_pose_subscriber);
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
    let pose_topic = format!("/{}/amcl_pose", agent_name);
    let pose_subscriber = Arc::new(RosSubscription::<PoseWithCovarianceStamped>::new(
        &node, pose_topic,
    ));

    commands.entity(e).insert((
        AmclPoseSubscription {
            subscriber: Arc::clone(&pose_subscriber),
        },
        AmclPose(PoseWithCovarianceStamped::default()),
    ));
}

fn update_amcl_pose(
    mut commands: Commands,
    mut agents: Query<(
        Entity,
        &mut Nav2Agent,
        &mut AmclPose,
        &AmclPoseSubscription,
        Option<&ProgressPublisher>,
        Option<&mut LatestProgress>,
        Option<&ProgressTracker>,
        Option<&CurrentSafeZone>,
        Option<&CostmapHandshakeState>,
        Option<&InnerNavigationClient>,
    )>,
) {
    for (
        entity,
        mut agent,
        mut amcl_pose,
        amcl_pose_sub,
        progress_pub,
        mut latest_progress,
        progress_tracker,
        current_safe_zone,
        handshake,
        inner_nav_client,
    ) in agents.iter_mut()
    {
        let mut received_amcl = false;
        while let Some(amcl_pose_msg) = amcl_pose_sub.subscriber.take_data() {
            amcl_pose.0 = amcl_pose_msg;
            agent.localized = true;
            received_amcl = true;
        }
        if !agent.localized || !received_amcl {
            continue;
        }

        let pose = PoseStamped {
            header: amcl_pose.0.header.clone(),
            pose: amcl_pose.0.pose.pose.clone(),
        };
        commands.entity(entity).insert(AgentPose(pose.clone()));

        if let (Some(tracker), Some(latest_prog)) = (progress_tracker, latest_progress.as_mut()) {
            if tracker.plan_id.is_some() {
                let current_pose = Some(amcl_pose.xy_yaw());
                let updated_prog = tracker.compute_progress(current_pose, latest_prog.0.as_ref());
                latest_prog.0 = Some(updated_prog);
            }
        }

        if let Some(progress_pub) = progress_pub {
            let mode = if progress_tracker.is_some_and(|t| t.is_holding_at_pre_waypoint()) {
                Progress::MODE_ACTION
            } else if inner_nav_client.is_some_and(|c| c.has_active_goal())
                || handshake.is_some_and(|h| h.pending_target.is_some())
            {
                Progress::MODE_NAVIGATING
            } else {
                Progress::MODE_IDLE
            };
            let prog = latest_progress.as_ref().and_then(|lp| lp.0.as_ref());
            let sz_id = current_safe_zone.and_then(|csz| csz.0.as_ref().map(|sz| &sz.id));
            let progress_msg = build_progress_status(pose, mode, prog, sz_id);
            if let Some(ref mut latest_prog) = latest_progress {
                latest_prog.0 = Some(progress_msg.clone());
            }
            let _ = progress_pub.publisher.publish(progress_msg);
        }
    }
}

fn publish_discovery(
    added: Query<(), Added<Nav2Agent>>,
    publisher: Res<DiscoveryPublisher>,
    agents: Query<&Nav2Agent>,
) {
    if added.is_empty() {
        return;
    }
    let mut msg = ParticipantList::default();
    for agent in agents.iter() {
        msg.participants.push(Participant {
            name: agent.name.clone(),
            components: vec![],
        });
    }
    let _ = publisher.publisher.publish(msg);
}
