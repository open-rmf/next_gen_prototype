use crate::{
    navigation_server::CurrentNavigationRequest,
    safe_zone::{
        build_robot_status, CurrentSafeZone, LatestProgress, ProgressPublisher, ProgressTracker,
        RobotStatusPublisher,
    },
};
use bevy::prelude::*;
use bevy_ros2::{RclrsNode, RosPublisher, RosSubscription};
use mapf::negotiation::scenario::Agent;
use ros_env::{
    geometry_msgs::msg::{PoseStamped, PoseWithCovarianceStamped, TwistWithCovariance},
    nav_msgs::msg::Odometry,
    rmf_prototype_msgs::msg::{Participant, ParticipantList, RobotStatus, SafeZoneId},
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

fn update_amcl_pose(
    mut agents: Query<(
        &mut Nav2Agent,
        &mut AmclPose,
        &AmclPoseSubscription,
        &OdomPublisher,
        Option<&RobotStatusPublisher>,
        Option<&ProgressPublisher>,
        Option<&mut LatestProgress>,
        Option<&ProgressTracker>,
        Option<&CurrentSafeZone>,
        Option<&CurrentNavigationRequest>,
    )>,
) {
    for (
        mut agent,
        mut amcl_pose,
        amcl_pose_sub,
        odom_pub,
        status_pub,
        progress_pub,
        mut latest_progress,
        progress_tracker,
        current_safe_zone,
        current_nav_req,
    ) in agents.iter_mut()
    {
        if let Some(amcl_pose_msg) = amcl_pose_sub.subscriber.data_callback() {
            amcl_pose.0 = amcl_pose_msg.clone();
            agent.localized = true;
        }
        if !agent.localized {
            continue;
        }

        let mut odom = Odometry::default();
        odom.header = amcl_pose.0.header.clone();
        odom.child_frame_id = "odom".to_string();
        odom.pose = amcl_pose.0.pose.clone();
        odom.twist = TwistWithCovariance::default();
        let _ = odom_pub.publisher.publish(odom);

        let current_pose = Some(amcl_pose.xy_yaw());
        if let (Some(tracker), Some(ref mut latest_prog)) =
            (progress_tracker, latest_progress.as_mut())
        {
            if tracker.plan_id.is_some() {
                let updated_prog = tracker.compute_progress(current_pose, latest_prog.0.as_ref());
                if latest_prog.0.as_ref() != Some(&updated_prog) {
                    if let Some(prog_pub) = progress_pub {
                        let _ = prog_pub.publisher.publish(updated_prog.clone());
                    }
                    latest_prog.0 = Some(updated_prog);
                }
            }
        }

        if let Some(status_pub) = status_pub {
            let pose = PoseStamped {
                header: amcl_pose.0.header.clone(),
                pose: amcl_pose.0.pose.pose.clone(),
            };
            let mode = if current_nav_req.is_some() {
                RobotStatus::MODE_NAVIGATING
            } else {
                RobotStatus::MODE_IDLE
            };
            let prog = latest_progress.as_ref().and_then(|lp| lp.0.as_ref());
            let sz_id = current_safe_zone.and_then(|csz| csz.0.as_ref().map(|sz| &sz.id));
            let status_msg = build_robot_status(pose, mode, prog, sz_id);
            let _ = status_pub.publisher.publish(status_msg);
        }
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
