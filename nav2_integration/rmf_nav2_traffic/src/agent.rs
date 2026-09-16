use bevy::prelude::*;
use bevy_ros2::{RclrsNode, RosPublisher, RosSubscription};
use mapf::negotiation::scenario::Agent;
use ros_env::{
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

#[derive(Component, Clone, Debug, Default, PartialEq)]
pub struct AgentDockState {
    pub is_docked: bool,
    pub dock_id: Option<String>,
    pub undock_pose: Option<[f64; 2]>,
}

impl AgentDockState {
    pub fn new(is_docked: bool, dock_id: Option<String>, undock_pose: Option<[f64; 2]>) -> Self {
        Self {
            is_docked,
            dock_id,
            undock_pose,
        }
    }

    pub fn docked(dock_id: impl Into<String>, undock_pose: Option<[f64; 2]>) -> Self {
        Self {
            is_docked: true,
            dock_id: Some(dock_id.into()),
            undock_pose,
        }
    }

    pub fn undocked() -> Self {
        Self {
            is_docked: false,
            dock_id: None,
            undock_pose: None,
        }
    }

    pub fn is_docked(&self) -> bool {
        self.is_docked
    }

    pub fn set_docked(&mut self, dock_id: impl Into<String>, undock_pose: Option<[f64; 2]>) {
        self.is_docked = true;
        self.dock_id = Some(dock_id.into());
        self.undock_pose = undock_pose;
    }

    pub fn set_undocked(&mut self) {
        self.is_docked = false;
        self.dock_id = None;
        self.undock_pose = None;
    }
}

#[derive(Clone, Debug, Event, PartialEq)]
pub struct AgentDockStateChanged {
    pub agent: Entity,
    pub state: AgentDockState,
}

/// What this agent's executor is currently busy with.
///
/// This is reported verbatim on `~/plan/progress` so that a traffic planner can
/// tell the difference between a robot that has merely stopped moving and one
/// that is physically committed to an action and must not be replanned. Without
/// it the two are indistinguishable, since both simply report
/// `reached_waypoint == target_waypoint`.
#[derive(Component, Clone, Debug, Default, PartialEq)]
pub struct AgentExecutionState {
    /// Set while an arrival_action / departure_action is in flight, and named
    /// after that action so the recipient can correlate it with the plan it
    /// posted. `None` whenever the agent is free to be given a new plan.
    pub active_action: Option<String>,
    /// Progress level to report instead of the odometry projection.
    ///
    /// A dock maneuver deliberately drives off the plan polyline: the contact
    /// pose is not a waypoint, and nav2's `DockRobot` controls the approach. A
    /// nearest-segment projection therefore stops describing the robot the
    /// moment the maneuver starts, and the direction it fails in is the
    /// dangerous one — it saturates at the end of the plan, telling every
    /// dependent robot that the corridor has been vacated.
    ///
    /// So while committed, progress is latched to the level the robot had
    /// genuinely reached and the maneuver reports its own completion.
    pub pinned_progress: Option<f32>,
}

impl AgentExecutionState {
    pub fn begin_action(&mut self, action: impl Into<String>) {
        self.active_action = Some(action.into());
    }

    pub fn end_action(&mut self) {
        self.active_action = None;
    }

    /// Latch the reported progress. Ignored if a pin is already held, so that
    /// the earliest (most conservative) level survives a multi-step workflow.
    pub fn pin_progress(&mut self, progress: f32) {
        self.pinned_progress.get_or_insert(progress);
    }

    /// Finish an action.
    ///
    /// On success the maneuver is over and the plan's own progress model
    /// describes the robot again. On failure it does not: the robot is
    /// somewhere in the corridor, having neither docked nor backed out, so the
    /// pin is kept and the space stays claimed until a replan supersedes it.
    pub fn complete_action(&mut self, success: bool) {
        self.active_action = None;
        if success {
            self.pinned_progress = None;
        }
    }

    /// Drop the pin because it is no longer meaningful.
    ///
    /// Progress levels are only comparable within one plan, so a pin latched
    /// against a superseded plan says nothing about the new one.
    pub fn clear_pin(&mut self) {
        self.pinned_progress = None;
    }

    pub fn is_executing_action(&self) -> bool {
        self.active_action.is_some()
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
        AgentExecutionState::default(),
    ));
}

fn update_amcl_pose(
    mut agents: Query<(
        &mut Nav2Agent,
        &mut AmclPose,
        &AmclPoseSubscription,
        &OdomPublisher,
    )>,
) {
    for (mut agent, mut amcl_pose, amcl_pose_sub, odom_pub) in agents.iter_mut() {
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
