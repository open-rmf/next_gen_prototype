use crate::{
    agent::AmclPose,
    navigation_server::{
        cleanup_navigation, request_publish_feedback_service, AgentPose,
        CancelledNavigationSessions,
    },
    safe_zone::{
        build_progress_status, update_incremental_target, CurrentSafeZone, LatestProgress,
        PlanErrorPublisher, ProgressPublisher, ProgressTracker,
    },
    Nav2Agent,
};
use bevy::prelude::*;
use bevy_ros2::{RclrsExecutorCommands, RclrsNode, RosActionClient};
use crossbeam::channel::{unbounded as unbounded_crossbeam, Receiver, Sender};
use futures::{
    channel::mpsc::{unbounded as unbounded_mpsc, UnboundedReceiver, UnboundedSender},
    future::{select, Either},
    StreamExt,
};
use nalgebra::UnitQuaternion;
use rclrs::*;
use ros_env::{
    builtin_interfaces::msg::Time as RosTime,
    geometry_msgs::msg::{Point, Pose, PoseStamped, Quaternion},
    nav2_msgs::action::{
        DockRobot, DockRobot_Goal, NavigateToPose, NavigateToPose_Feedback, NavigateToPose_Goal,
        UndockRobot, UndockRobot_Goal,
    },
    rmf_prototype_msgs::msg::{Progress, SafeZoneId},
    std_msgs::msg::Header,
};
use std::{pin::pin, sync::Arc};

#[derive(Clone, Debug, PartialEq, Eq, Default)]
pub enum InnerNavigationAction {
    #[default]
    NavigateToPose,
    DockRobot {
        use_dock_id: bool,
        dock_id: String,
        dock_type: String,
        navigate_to_staging_pose: bool,
    },
    UndockRobot {
        dock_type: String,
    },
}

#[derive(Clone, Debug, Event)]
pub struct InnerNavigationTarget {
    pub agent: Entity,
    pub safe_zone_id: SafeZoneId,
    pub x: f64,
    pub y: f64,
    pub yaw: f64,
    pub action: InnerNavigationAction,
}

impl InnerNavigationTarget {
    pub fn new(agent: Entity, safe_zone_id: SafeZoneId, x: f64, y: f64, yaw: f64) -> Self {
        Self {
            agent,
            safe_zone_id,
            x,
            y,
            yaw,
            action: InnerNavigationAction::NavigateToPose,
        }
    }

    pub fn with_action(
        agent: Entity,
        safe_zone_id: SafeZoneId,
        x: f64,
        y: f64,
        yaw: f64,
        action: InnerNavigationAction,
    ) -> Self {
        Self {
            agent,
            safe_zone_id,
            x,
            y,
            yaw,
            action,
        }
    }
}

#[derive(Clone)]
pub enum InnerGoalClient {
    NavigateToPose(GoalClient<NavigateToPose>),
    DockRobot(GoalClient<DockRobot>),
    UndockRobot(GoalClient<UndockRobot>),
}

impl InnerGoalClient {
    pub async fn is_cancellation_accepted(&self) -> bool {
        match self {
            Self::NavigateToPose(c) => {
                let accepted = c.cancellation.cancel().await.is_accepted();
                if accepted {
                    let _ = c.result.clone().await;
                }
                accepted
            }
            Self::DockRobot(c) => {
                let accepted = c.cancellation.cancel().await.is_accepted();
                if accepted {
                    let _ = c.result.clone().await;
                }
                accepted
            }
            Self::UndockRobot(c) => {
                let accepted = c.cancellation.cancel().await.is_accepted();
                if accepted {
                    let _ = c.result.clone().await;
                }
                accepted
            }
        }
    }
}

#[derive(Clone, Debug)]
pub struct ActiveInnerGoal {
    pub safe_zone_id: SafeZoneId,
    pub action: InnerNavigationAction,
    superseded: bool,
}

impl ActiveInnerGoal {
    pub fn new(safe_zone_id: SafeZoneId, action: InnerNavigationAction) -> Self {
        Self {
            safe_zone_id,
            action,
            superseded: false,
        }
    }

    pub fn id(&self) -> &SafeZoneId {
        &self.safe_zone_id
    }

    pub fn action(&self) -> &InnerNavigationAction {
        &self.action
    }

    pub fn is_superseded(&self) -> bool {
        self.superseded
    }

    pub fn mark_superseded(&mut self) {
        self.superseded = true;
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct InnerNavigationRequest {
    pub agent: Entity,
    pub safe_zone_id: SafeZoneId,
    pub target_pose: PoseStamped,
    pub action: InnerNavigationAction,
}

#[derive(Clone, Debug, PartialEq)]
pub(crate) enum AgentNavCommand {
    Dispatch(InnerNavigationRequest),
    Cancel,
}

#[derive(Clone, Debug)]
pub(crate) enum AgentNavOutcome {
    Feedback {
        agent: Entity,
        safe_zone_id: SafeZoneId,
        feedback: NavigateToPose_Feedback,
    },
    Succeeded {
        request: InnerNavigationRequest,
    },
    Aborted {
        request: InnerNavigationRequest,
        superseded: bool,
    },
    Cancelled {
        agent: Entity,
        request: Option<InnerNavigationRequest>,
        external: bool,
    },
}

#[derive(Component, Clone)]
pub struct InnerNavigationClient {
    pub action_client: Arc<RosActionClient<NavigateToPose>>,
    pub dock_action_client: Arc<RosActionClient<DockRobot>>,
    pub undock_action_client: Arc<RosActionClient<UndockRobot>>,
    // The ongoing action goal (if any)
    pub active_goal: Option<ActiveInnerGoal>,
    // The action type of the most recently dispatched goal
    pub last_action: Option<InnerNavigationAction>,
    command_tx: UnboundedSender<AgentNavCommand>,
    outcome_rx: Receiver<AgentNavOutcome>,
}

impl InnerNavigationClient {
    pub(crate) fn new(
        action_client: Arc<RosActionClient<NavigateToPose>>,
        dock_action_client: Arc<RosActionClient<DockRobot>>,
        undock_action_client: Arc<RosActionClient<UndockRobot>>,
        command_tx: UnboundedSender<AgentNavCommand>,
        outcome_rx: Receiver<AgentNavOutcome>,
    ) -> Self {
        Self {
            action_client,
            dock_action_client,
            undock_action_client,
            active_goal: None,
            last_action: None,
            command_tx,
            outcome_rx,
        }
    }

    pub fn goal(&self) -> &Option<ActiveInnerGoal> {
        &self.active_goal
    }

    pub fn goal_mut(&mut self) -> &mut Option<ActiveInnerGoal> {
        &mut self.active_goal
    }

    pub fn reset_goal(&mut self) {
        self.active_goal = None;
    }
}

#[derive(Clone, Debug, Event)]
pub struct CancelInnerForAgent {
    pub agent: Entity,
}

#[derive(Clone, Debug, Component)]
pub struct CancellingInnerNavigation {
    pub success: bool,
}

#[derive(Clone, Event)]
pub struct InnerNavigationFeedback {
    pub agent: Entity,
    pub safe_zone_id: SafeZoneId,
    pub feedback: NavigateToPose_Feedback,
}

impl InnerNavigationFeedback {
    pub fn new(agent: Entity, safe_zone_id: SafeZoneId, feedback: NavigateToPose_Feedback) -> Self {
        Self {
            agent,
            safe_zone_id,
            feedback,
        }
    }
}

#[derive(Component, Clone, Default)]
pub struct LatestInnerFeedbackPose(pub Option<PoseStamped>);

#[derive(Default)]
pub struct InnerNavigationClientPlugin {}

impl Plugin for InnerNavigationClientPlugin {
    fn build(&self, app: &mut App) {
        app.add_event::<InnerNavigationTarget>()
            .add_event::<InnerNavigationFeedback>()
            .add_event::<CancelInnerForAgent>()
            .add_systems(
                PreUpdate,
                (
                    dispatch_inner_navigation_commands,
                    process_inner_navigation_outcomes,
                )
                    .chain()
                    .after(update_incremental_target)
                    .after(request_publish_feedback_service),
            )
            .add_systems(
                PostUpdate,
                dispatch_inner_navigation_commands.before(cleanup_navigation),
            )
            .add_observer(create_inner_navigation_client)
            .add_observer(record_inner_navigation_feedback);
    }
}

fn create_inner_navigation_client(
    trigger: Trigger<OnAdd, Nav2Agent>,
    mut commands: Commands,
    agents: Query<&Nav2Agent>,
    node: Res<RclrsNode>,
    executor_commands: Res<RclrsExecutorCommands>,
) {
    let e = trigger.target();
    let Ok(agent_name) = agents.get(e).map(|agent| agent.name.clone()) else {
        return;
    };
    let action_name = agent_name.clone() + "/inner/navigate_to_pose";
    let dock_action_name = agent_name.clone() + "/inner/dock_robot";
    let undock_action_name = agent_name + "/inner/undock_robot";

    let inner_action_client = Arc::new(RosActionClient::<NavigateToPose>::new(&node, action_name));
    let inner_dock_client = Arc::new(RosActionClient::<DockRobot>::new(&node, dock_action_name));
    let inner_undock_client = Arc::new(RosActionClient::<UndockRobot>::new(
        &node,
        undock_action_name,
    ));

    let (command_tx, command_rx) = unbounded_mpsc::<AgentNavCommand>();
    let (outcome_tx, outcome_rx) = unbounded_crossbeam::<AgentNavOutcome>();

    let nav_client_for_actor = Arc::clone(&inner_action_client);
    let dock_client_for_actor = Arc::clone(&inner_dock_client);
    let undock_client_for_actor = Arc::clone(&inner_undock_client);

    let _ = executor_commands.run(run_agent_navigation_actor(
        e,
        nav_client_for_actor,
        dock_client_for_actor,
        undock_client_for_actor,
        command_rx,
        outcome_tx,
    ));

    commands.entity(e).insert(InnerNavigationClient::new(
        inner_action_client,
        inner_dock_client,
        inner_undock_client,
        command_tx,
        outcome_rx,
    ));
}

fn record_inner_navigation_feedback(
    trigger: Trigger<InnerNavigationFeedback>,
    mut commands: Commands,
    inner_nav_clients: Query<&InnerNavigationClient>,
) {
    let event = trigger.event();
    if let Ok(inner_nav_client) = inner_nav_clients.get(event.agent) {
        if let Some(active_goal) = inner_nav_client.goal() {
            if active_goal.id() != &event.safe_zone_id {
                return;
            }
        }
    }
    commands
        .entity(event.agent)
        .insert(LatestInnerFeedbackPose(Some(
            event.feedback.current_pose.clone(),
        )));
}

pub fn is_safe_zone_newer(candidate: &SafeZoneId, current: &SafeZoneId) -> bool {
    if candidate.plan_id.destination_session.uuid != current.plan_id.destination_session.uuid {
        return true;
    }
    if candidate.plan_id.plan_version != current.plan_id.plan_version {
        return candidate.plan_id.plan_version > current.plan_id.plan_version;
    }
    candidate.safe_zone_version > current.safe_zone_version
}

pub fn should_preempt_goal(
    candidate_id: &SafeZoneId,
    candidate_action: &InnerNavigationAction,
    current_id: &SafeZoneId,
    current_action: &InnerNavigationAction,
) -> bool {
    is_safe_zone_newer(candidate_id, current_id)
        || (candidate_id == current_id && candidate_action != current_action)
}

pub(crate) fn fold_agent_commands(
    current: AgentNavCommand,
    next: AgentNavCommand,
) -> AgentNavCommand {
    match (current, next) {
        (_, AgentNavCommand::Cancel) => AgentNavCommand::Cancel,
        (AgentNavCommand::Cancel, AgentNavCommand::Dispatch(req)) => AgentNavCommand::Dispatch(req),
        (AgentNavCommand::Dispatch(curr_req), AgentNavCommand::Dispatch(next_req)) => {
            if should_preempt_goal(
                &next_req.safe_zone_id,
                &next_req.action,
                &curr_req.safe_zone_id,
                &curr_req.action,
            ) || next_req.safe_zone_id == curr_req.safe_zone_id
            {
                AgentNavCommand::Dispatch(next_req)
            } else {
                AgentNavCommand::Dispatch(curr_req)
            }
        }
    }
}

fn coalesce_agent_command(
    first: AgentNavCommand,
    command_rx: &mut UnboundedReceiver<AgentNavCommand>,
) -> AgentNavCommand {
    let mut current = first;
    while let Ok(next) = command_rx.try_recv() {
        current = fold_agent_commands(current, next);
    }
    current
}

async fn cancel_action_server_goals(
    action: &InnerNavigationAction,
    nav_action_client: &Arc<RosActionClient<NavigateToPose>>,
    dock_action_client: &Arc<RosActionClient<DockRobot>>,
    undock_action_client: &Arc<RosActionClient<UndockRobot>>,
) -> bool {
    match action {
        InnerNavigationAction::NavigateToPose => {
            let mut cancel_req = nav_action_client.cancel_all_goals();
            (&mut *cancel_req).await.is_ok()
        }
        InnerNavigationAction::DockRobot { .. } => {
            let mut cancel_req = dock_action_client.cancel_all_goals();
            (&mut *cancel_req).await.is_ok()
        }
        InnerNavigationAction::UndockRobot { .. } => {
            let mut cancel_req = undock_action_client.cancel_all_goals();
            (&mut *cancel_req).await.is_ok()
        }
    }
}

async fn request_action_goal(
    nav_action_client: &Arc<RosActionClient<NavigateToPose>>,
    dock_action_client: &Arc<RosActionClient<DockRobot>>,
    undock_action_client: &Arc<RosActionClient<UndockRobot>>,
    request: &InnerNavigationRequest,
) -> Option<InnerGoalClient> {
    let mut pose = request.target_pose.clone();
    pose.header.frame_id = "map".to_string();

    match &request.action {
        InnerNavigationAction::NavigateToPose => {
            debug!(
                "[{:?}] Requesting new NavigateToPose goal to pose [{}, {}]",
                request.agent.index(),
                pose.pose.position.x,
                pose.pose.position.y
            );
            let req = nav_action_client.request_goal(NavigateToPose_Goal { pose, ..default() });
            req.await.map(InnerGoalClient::NavigateToPose)
        }
        InnerNavigationAction::DockRobot {
            use_dock_id,
            dock_id,
            dock_type,
            navigate_to_staging_pose,
        } => {
            info!(
                "[{:?}] Requesting new DockRobot goal (use_dock_id={}, dock_id='{}', dock_type='{}', navigate_to_staging_pose={}) to pose [{}, {}]",
                request.agent.index(),
                use_dock_id,
                dock_id,
                dock_type,
                navigate_to_staging_pose,
                pose.pose.position.x,
                pose.pose.position.y
            );
            let dock_goal = DockRobot_Goal {
                use_dock_id: *use_dock_id,
                dock_id: dock_id.clone(),
                dock_type: dock_type.clone(),
                dock_pose: pose,
                max_staging_time: 1000.0,
                navigate_to_staging_pose: *navigate_to_staging_pose,
            };
            let req = dock_action_client.request_goal(dock_goal);
            req.await.map(InnerGoalClient::DockRobot)
        }
        InnerNavigationAction::UndockRobot { dock_type } => {
            info!(
                "[{:?}] Requesting new UndockRobot goal (dock_type='{}')",
                request.agent.index(),
                dock_type
            );
            let undock_goal = UndockRobot_Goal {
                dock_type: dock_type.clone(),
                max_undocking_time: 30.0,
            };
            let req = undock_action_client.request_goal(undock_goal);
            req.await.map(InnerGoalClient::UndockRobot)
        }
    }
}

enum GoalMonitorOutcome {
    Interrupted(AgentNavCommand),
    ChannelClosed,
    Succeeded,
    Aborted,
    Cancelled,
}

async fn monitor_goal_until_command_or_completion(
    agent: Entity,
    safe_zone_id: &SafeZoneId,
    goal_client: InnerGoalClient,
    command_rx: &mut UnboundedReceiver<AgentNavCommand>,
    outcome_tx: &Sender<AgentNavOutcome>,
) -> GoalMonitorOutcome {
    match goal_client {
        InnerGoalClient::NavigateToPose(client) => {
            let mut stream = client.stream();
            loop {
                let next_cmd_fut = command_rx.next();
                let next_ev_fut = stream.next();
                match select(pin!(next_cmd_fut), pin!(next_ev_fut)).await {
                    Either::Left((Some(cmd), _)) => return GoalMonitorOutcome::Interrupted(cmd),
                    Either::Left((None, _)) => return GoalMonitorOutcome::ChannelClosed,
                    Either::Right((Some(event), _)) => match event {
                        GoalEvent::Feedback(feedback) => {
                            let _ = outcome_tx.send(AgentNavOutcome::Feedback {
                                agent,
                                safe_zone_id: safe_zone_id.clone(),
                                feedback,
                            });
                        }
                        GoalEvent::Status(s) => {
                            debug!(
                                "[{:?}] [inner nav2pose] Status: {:?}",
                                agent.index(),
                                s.code
                            );
                        }
                        GoalEvent::Result((status, result)) => {
                            debug!(
                                "[{:?}] [inner nav2pose] Result: {:?}",
                                agent.index(),
                                result
                            );
                            match status {
                                GoalStatusCode::Succeeded => return GoalMonitorOutcome::Succeeded,
                                GoalStatusCode::Aborted => return GoalMonitorOutcome::Aborted,
                                GoalStatusCode::Cancelled => return GoalMonitorOutcome::Cancelled,
                                _ => {}
                            }
                        }
                    },
                    Either::Right((None, _)) => return GoalMonitorOutcome::Aborted,
                }
            }
        }
        InnerGoalClient::DockRobot(client) => {
            let mut stream = client.stream();
            loop {
                let next_cmd_fut = command_rx.next();
                let next_ev_fut = stream.next();
                match select(pin!(next_cmd_fut), pin!(next_ev_fut)).await {
                    Either::Left((Some(cmd), _)) => return GoalMonitorOutcome::Interrupted(cmd),
                    Either::Left((None, _)) => return GoalMonitorOutcome::ChannelClosed,
                    Either::Right((Some(event), _)) => match event {
                        GoalEvent::Feedback(feedback) => {
                            debug!(
                                "[{:?}] [inner dock_robot] state={}, retries={}",
                                agent.index(),
                                feedback.state,
                                feedback.num_retries
                            );
                        }
                        GoalEvent::Status(s) => {
                            debug!(
                                "[{:?}] [inner dock_robot] Status: {:?}",
                                agent.index(),
                                s.code
                            );
                        }
                        GoalEvent::Result((status, result)) => {
                            info!(
                                "[{:?}] [inner dock_robot] Result: status={:?}, success={}, error_code={}, msg='{}'",
                                agent.index(),
                                status,
                                result.success,
                                result.error_code,
                                result.error_msg
                            );
                            match status {
                                GoalStatusCode::Succeeded if result.success => {
                                    return GoalMonitorOutcome::Succeeded
                                }
                                GoalStatusCode::Succeeded | GoalStatusCode::Aborted => {
                                    return GoalMonitorOutcome::Aborted
                                }
                                GoalStatusCode::Cancelled => return GoalMonitorOutcome::Cancelled,
                                _ => {}
                            }
                        }
                    },
                    Either::Right((None, _)) => return GoalMonitorOutcome::Aborted,
                }
            }
        }
        InnerGoalClient::UndockRobot(client) => {
            let mut stream = client.stream();
            loop {
                let next_cmd_fut = command_rx.next();
                let next_ev_fut = stream.next();
                match select(pin!(next_cmd_fut), pin!(next_ev_fut)).await {
                    Either::Left((Some(cmd), _)) => return GoalMonitorOutcome::Interrupted(cmd),
                    Either::Left((None, _)) => return GoalMonitorOutcome::ChannelClosed,
                    Either::Right((Some(event), _)) => match event {
                        GoalEvent::Feedback(_feedback) => {}
                        GoalEvent::Status(s) => {
                            debug!(
                                "[{:?}] [inner undock_robot] Status: {:?}",
                                agent.index(),
                                s.code
                            );
                        }
                        GoalEvent::Result((status, result)) => {
                            info!(
                                "[{:?}] [inner undock_robot] Result: status={:?}, success={}, error_code={}, msg='{}'",
                                agent.index(),
                                status,
                                result.success,
                                result.error_code,
                                result.error_msg
                            );
                            match status {
                                GoalStatusCode::Succeeded if result.success => {
                                    return GoalMonitorOutcome::Succeeded
                                }
                                GoalStatusCode::Succeeded | GoalStatusCode::Aborted => {
                                    return GoalMonitorOutcome::Aborted
                                }
                                GoalStatusCode::Cancelled => return GoalMonitorOutcome::Cancelled,
                                _ => {}
                            }
                        }
                    },
                    Either::Right((None, _)) => return GoalMonitorOutcome::Aborted,
                }
            }
        }
    }
}

async fn run_agent_navigation_actor(
    agent: Entity,
    nav_action_client: Arc<RosActionClient<NavigateToPose>>,
    dock_action_client: Arc<RosActionClient<DockRobot>>,
    undock_action_client: Arc<RosActionClient<UndockRobot>>,
    mut command_rx: UnboundedReceiver<AgentNavCommand>,
    outcome_tx: Sender<AgentNavOutcome>,
) {
    let mut active_goal: Option<(InnerNavigationRequest, InnerGoalClient)> = None;
    let mut last_action: Option<InnerNavigationAction> = None;
    let mut pending_cmd: Option<AgentNavCommand> = None;

    loop {
        let raw_cmd = if let Some(cmd) = pending_cmd.take() {
            Some(cmd)
        } else if active_goal.is_none() {
            let Some(cmd) = command_rx.next().await else {
                break;
            };
            Some(cmd)
        } else {
            None
        };

        if let Some(raw_cmd) = raw_cmd {
            let cmd = coalesce_agent_command(raw_cmd, &mut command_rx);
            match cmd {
                AgentNavCommand::Cancel => {
                    let mut accepted = false;
                    let prev_req = if let Some((req, client)) = active_goal.take() {
                        accepted = client.is_cancellation_accepted().await;
                        if cancel_action_server_goals(
                            &req.action,
                            &nav_action_client,
                            &dock_action_client,
                            &undock_action_client,
                        )
                        .await
                        {
                            accepted = true;
                        }
                        Some(req)
                    } else {
                        if let Some(ref prev_act) = last_action {
                            if cancel_action_server_goals(
                                prev_act,
                                &nav_action_client,
                                &dock_action_client,
                                &undock_action_client,
                            )
                            .await
                            {
                                accepted = true;
                            }
                        }
                        None
                    };

                    if !accepted {
                        warn!(
                            "[{}] Cancellation request rejected for inner navigation, it may have already completed.",
                            agent.index()
                        );
                    } else {
                        info!(
                            "[{}] Cancellation request finished for inner navigation",
                            agent.index()
                        );
                    }
                    let _ = outcome_tx.send(AgentNavOutcome::Cancelled {
                        agent,
                        request: prev_req,
                        external: true,
                    });
                    continue;
                }
                AgentNavCommand::Dispatch(request) => {
                    if let Some((ref curr_req, ref curr_client)) = active_goal {
                        if !should_preempt_goal(
                            &request.safe_zone_id,
                            &request.action,
                            &curr_req.safe_zone_id,
                            &curr_req.action,
                        ) {
                            debug!(
                                "[{:?}] Ignoring duplicate or outdated inner navigation request {:?}",
                                agent.index(),
                                request.safe_zone_id
                            );
                        } else {
                            let existing_is_nav =
                                matches!(curr_req.action, InnerNavigationAction::NavigateToPose);
                            let next_is_nav =
                                matches!(request.action, InnerNavigationAction::NavigateToPose);
                            if existing_is_nav && next_is_nav {
                                // Send the new NavigateToPose goal directly so Nav2's
                                // SimpleActionServer smoothly preempts the active goal in
                                // accept_pending_goal() without calling terminate_all().
                                active_goal = None;
                            } else {
                                let prev_client = curr_client.clone();
                                let prev_action = curr_req.action.clone();
                                active_goal = None;
                                let _ = prev_client.is_cancellation_accepted().await;
                                let _ = cancel_action_server_goals(
                                    &prev_action,
                                    &nav_action_client,
                                    &dock_action_client,
                                    &undock_action_client,
                                )
                                .await;
                            }
                        }
                    } else if let Some(ref prev_act) = last_action {
                        if std::mem::discriminant(prev_act)
                            != std::mem::discriminant(&request.action)
                        {
                            let _ = cancel_action_server_goals(
                                prev_act,
                                &nav_action_client,
                                &dock_action_client,
                                &undock_action_client,
                            )
                            .await;
                        }
                    }

                    if active_goal.is_none() {
                        last_action = Some(request.action.clone());
                        match request_action_goal(
                            &nav_action_client,
                            &dock_action_client,
                            &undock_action_client,
                            &request,
                        )
                        .await
                        {
                            Some(goal_client) => {
                                active_goal = Some((request, goal_client));
                            }
                            None => {
                                error!(
                                    "[{:?}] Failed to request new inner navigation goal {:?}",
                                    agent.index(),
                                    request.safe_zone_id
                                );
                                let _ = outcome_tx.send(AgentNavOutcome::Aborted {
                                    request,
                                    superseded: false,
                                });
                                continue;
                            }
                        }
                    }
                }
            }
        }

        let Some((curr_req, curr_client)) = active_goal.take() else {
            continue;
        };

        // Check if another command arrived while request_action_goal was in flight.
        if let Ok(cmd) = command_rx.try_recv() {
            active_goal = Some((curr_req, curr_client));
            pending_cmd = Some(cmd);
            continue;
        }

        match monitor_goal_until_command_or_completion(
            agent,
            &curr_req.safe_zone_id,
            curr_client.clone(),
            &mut command_rx,
            &outcome_tx,
        )
        .await
        {
            GoalMonitorOutcome::Interrupted(cmd) => {
                active_goal = Some((curr_req, curr_client));
                pending_cmd = Some(cmd);
            }
            GoalMonitorOutcome::ChannelClosed => break,
            GoalMonitorOutcome::Succeeded => {
                let _ = outcome_tx.send(AgentNavOutcome::Succeeded { request: curr_req });
            }
            GoalMonitorOutcome::Aborted => {
                let _ = outcome_tx.send(AgentNavOutcome::Aborted {
                    request: curr_req,
                    superseded: false,
                });
            }
            GoalMonitorOutcome::Cancelled => {
                let _ = outcome_tx.send(AgentNavOutcome::Cancelled {
                    agent,
                    request: Some(curr_req),
                    external: false,
                });
            }
        }
    }
}

fn dispatch_inner_navigation_commands(
    mut commands: Commands,
    mut nav_targets: EventReader<InnerNavigationTarget>,
    mut cancel_requests: EventReader<CancelInnerForAgent>,
    mut inner_nav_clients: Query<&mut InnerNavigationClient>,
    cancelled_sessions: Query<&CancelledNavigationSessions>,
    node: Res<RclrsNode>,
) {
    for request in cancel_requests.read() {
        info!(
            "Received external cancellation request for agent {:?}",
            request.agent.index()
        );
        let Ok(mut inner_client) = inner_nav_clients.get_mut(request.agent) else {
            continue;
        };
        if inner_client.goal().is_none() && inner_client.last_action.is_none() {
            continue;
        }
        if let Some(existing_goal) = inner_client.goal_mut().as_mut() {
            existing_goal.mark_superseded();
        }
        inner_client.reset_goal();
        commands
            .entity(request.agent)
            .insert(CancellingInnerNavigation { success: false });
        let _ = inner_client
            .command_tx
            .unbounded_send(AgentNavCommand::Cancel);
    }

    for target in nav_targets.read() {
        if cancelled_sessions.get(target.agent).is_ok_and(|cancelled| {
            cancelled.contains(&target.safe_zone_id.plan_id.destination_session)
        }) {
            continue;
        }

        let Ok(mut inner_client) = inner_nav_clients.get_mut(target.agent) else {
            continue;
        };

        if let Some(existing_goal) = inner_client.goal() {
            if !should_preempt_goal(
                &target.safe_zone_id,
                &target.action,
                existing_goal.id(),
                existing_goal.action(),
            ) {
                continue;
            }
        }

        let now = node.get_clock().now();
        let quat = UnitQuaternion::from_euler_angles(0.0, 0.0, target.yaw);
        let goal_pose = PoseStamped {
            header: Header {
                stamp: RosTime {
                    sec: (now.nsec / 1_000_000_000) as i32,
                    nanosec: (now.nsec % 1_000_000_000) as u32,
                },
                frame_id: "map".to_string(),
            },
            pose: Pose {
                position: Point {
                    x: target.x,
                    y: target.y,
                    z: 0.0,
                },
                orientation: Quaternion {
                    x: quat.coords.x,
                    y: quat.coords.y,
                    z: quat.coords.z,
                    w: quat.coords.w,
                },
            },
        };

        info!(
            "[{:?}] Requesting new pending goal to pose [{}, {}] (action={:?})",
            target.agent.index(),
            goal_pose.pose.position.x,
            goal_pose.pose.position.y,
            target.action,
        );

        let request = InnerNavigationRequest {
            agent: target.agent,
            safe_zone_id: target.safe_zone_id.clone(),
            target_pose: goal_pose,
            action: target.action.clone(),
        };

        *inner_client.goal_mut() = Some(ActiveInnerGoal::new(
            target.safe_zone_id.clone(),
            target.action.clone(),
        ));
        inner_client.last_action = Some(target.action.clone());
        let _ = inner_client
            .command_tx
            .unbounded_send(AgentNavCommand::Dispatch(request));
    }
}

fn process_inner_navigation_outcomes(
    mut commands: Commands,
    mut inner_nav_clients: Query<(Entity, &mut InnerNavigationClient)>,
    mut cancelling_inner: Query<&mut CancellingInnerNavigation>,
    plan_error_publishers: Query<&PlanErrorPublisher>,
    mut progress_query: Query<(
        &Nav2Agent,
        &mut ProgressTracker,
        &ProgressPublisher,
        &mut LatestProgress,
        Option<&CurrentSafeZone>,
        Option<&AmclPose>,
        Option<&AgentPose>,
        Option<&LatestInnerFeedbackPose>,
    )>,
) {
    for (_entity, mut inner_client) in inner_nav_clients.iter_mut() {
        let outcomes: Vec<AgentNavOutcome> = inner_client.outcome_rx.try_iter().collect();
        for outcome in outcomes {
            match outcome {
                AgentNavOutcome::Feedback {
                    agent,
                    safe_zone_id,
                    feedback,
                } => {
                    if let Some(active_goal) = inner_client.goal() {
                        if active_goal.id() != &safe_zone_id {
                            continue;
                        }
                    }
                    commands
                        .entity(agent)
                        .insert(LatestInnerFeedbackPose(Some(feedback.current_pose.clone())));
                    commands.trigger(InnerNavigationFeedback::new(agent, safe_zone_id, feedback));
                }
                AgentNavOutcome::Succeeded { request } => {
                    let is_current = inner_client.goal().as_ref().is_some_and(|goal| {
                        goal.id() == &request.safe_zone_id && !goal.is_superseded()
                    });
                    let mut should_retry = false;

                    if is_current {
                        if let Ok((
                            agent,
                            mut tracker,
                            progress_pub,
                            mut latest_progress,
                            current_safe_zone,
                            amcl_pose,
                            agent_pose,
                            latest_fb_pose,
                        )) = progress_query.get_mut(request.agent)
                        {
                            let matches_current_safe_zone = current_safe_zone
                                .and_then(|csz| csz.0.as_ref())
                                .is_none_or(|sz| sz.id == request.safe_zone_id);
                            if matches_current_safe_zone
                                && tracker.plan_id.as_ref() == Some(&request.safe_zone_id.plan_id)
                            {
                                if matches!(
                                    request.action,
                                    InnerNavigationAction::DockRobot { .. }
                                        | InnerNavigationAction::UndockRobot { .. }
                                ) {
                                    tracker.mark_action_completed();
                                }
                                let fb_pose = latest_fb_pose.and_then(|p| p.0.as_ref());
                                let current_pose = amcl_pose
                                    .filter(|_| agent.localized)
                                    .map(|p| p.xy_yaw())
                                    .or_else(|| fb_pose.map(|p| AgentPose(p.clone()).xy_yaw()))
                                    .or_else(|| agent_pose.map(|p| p.xy_yaw()));
                                if !tracker.is_target_pose_satisfied(current_pose) {
                                    warn!(
                                        "[{:?}] Nav2 reported goal succeeded before target pose was satisfied (current={:?}); retrying goal {:?}.",
                                        request.agent.index(),
                                        current_pose,
                                        request.safe_zone_id,
                                    );
                                    should_retry = true;
                                } else if let Some(reached_prog) =
                                    tracker.mark_target_reached(latest_progress.0.as_ref())
                                {
                                    let pose = amcl_pose
                                        .filter(|_| agent.localized)
                                        .map(|p| PoseStamped {
                                            header: p.0.header.clone(),
                                            pose: p.0.pose.pose.clone(),
                                        })
                                        .or_else(|| fb_pose.cloned())
                                        .or_else(|| agent_pose.map(|p| p.0.clone()))
                                        .unwrap_or_default();
                                    let sz_id = current_safe_zone
                                        .and_then(|csz| csz.0.as_ref().map(|sz| &sz.id))
                                        .or(Some(&request.safe_zone_id));
                                    let mode = if tracker.is_holding_at_pre_waypoint() {
                                        Progress::MODE_ACTION
                                    } else {
                                        Progress::MODE_NAVIGATING
                                    };
                                    let progress_msg = build_progress_status(
                                        pose,
                                        mode,
                                        Some(&reached_prog),
                                        sz_id,
                                    );
                                    if latest_progress.0.as_ref() != Some(&progress_msg) {
                                        let _ =
                                            progress_pub.publisher.publish(progress_msg.clone());
                                        latest_progress.0 = Some(progress_msg);
                                    }
                                }
                            }
                        }
                    }

                    if should_retry {
                        let _ = inner_client
                            .command_tx
                            .unbounded_send(AgentNavCommand::Dispatch(request));
                    } else if inner_client
                        .goal()
                        .as_ref()
                        .is_some_and(|g| g.id() == &request.safe_zone_id)
                    {
                        inner_client.reset_goal();
                    }
                }
                AgentNavOutcome::Aborted {
                    request,
                    superseded,
                } => {
                    let active_goal_info = inner_client
                        .goal()
                        .as_ref()
                        .map(|goal| (goal.id(), goal.is_superseded() || superseded));
                    let matches_current_safe_zone = progress_query
                        .get(request.agent)
                        .ok()
                        .and_then(|(_, _, _, _, current_safe_zone, _, _, _)| current_safe_zone)
                        .and_then(|csz| csz.0.as_ref())
                        .is_none_or(|sz| sz.id == request.safe_zone_id);

                    if !matches_current_safe_zone
                        || !should_publish_path_blocked(&request.safe_zone_id, active_goal_info)
                    {
                        info!(
                            "[{:?}] Ignoring stale Nav2 abort {:?}",
                            request.agent.index(),
                            request.safe_zone_id,
                        );
                    } else if let Ok(publisher) = plan_error_publishers.get(request.agent) {
                        let plan_error = ros_env::rmf_prototype_msgs::msg::PlanError {
                            error: ros_env::rmf_prototype_msgs::msg::Error {
                                code: ros_env::rmf_prototype_msgs::msg::PlanError::CODE_PATH_BLOCKED,
                                message: format!(
                                    "Nav2 goal aborted for agent {:?}: path blocked within safe zone",
                                    request.agent.index()
                                ),
                                parameters: String::new(),
                            },
                            plan_id: request.safe_zone_id.plan_id.clone(),
                        };
                        if let Err(e) = publisher.publisher.publish(plan_error) {
                            error!(
                                "Failed to publish PlanError for agent {:?}: {:?}",
                                request.agent.index(),
                                e
                            );
                        } else {
                            warn!(
                                "[{:?}] Goal aborted by Nav2! Published PlanError CODE_PATH_BLOCKED to ~/plan/error.",
                                request.agent.index()
                            );
                        }
                    }

                    if inner_client
                        .goal()
                        .as_ref()
                        .is_some_and(|g| g.id() == &request.safe_zone_id)
                    {
                        inner_client.reset_goal();
                    }
                }
                AgentNavOutcome::Cancelled {
                    agent,
                    request,
                    external,
                } => {
                    if external {
                        if let Ok(mut cancelling) = cancelling_inner.get_mut(agent) {
                            info!(
                                "[{:?}] Goal cancelled. Marking cancellation as success.",
                                agent.index()
                            );
                            cancelling.success = true;
                        }
                    }
                    if let Some(req) = request {
                        if inner_client
                            .goal()
                            .as_ref()
                            .is_some_and(|g| g.id() == &req.safe_zone_id)
                        {
                            inner_client.reset_goal();
                        }
                    } else if external {
                        inner_client.reset_goal();
                    }
                }
            }
        }
    }
}

fn should_publish_path_blocked(
    completed_goal_id: &SafeZoneId,
    active_goal: Option<(&SafeZoneId, bool)>,
) -> bool {
    matches!(
        active_goal,
        Some((active_goal_id, false)) if active_goal_id == completed_goal_id
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    fn safe_zone_id(session: u8, plan_version: u64, safe_zone_version: u64) -> SafeZoneId {
        let mut id = SafeZoneId::default();
        id.plan_id.destination_session.uuid[0] = session;
        id.plan_id.plan_version = plan_version;
        id.safe_zone_version = safe_zone_version;
        id
    }

    #[test]
    fn current_abort_publishes_path_blocked() {
        let completed = safe_zone_id(1, 3, 7);

        assert!(should_publish_path_blocked(
            &completed,
            Some((&completed, false)),
        ));
    }

    #[test]
    fn intentionally_superseded_abort_is_ignored() {
        let completed = safe_zone_id(1, 3, 7);

        assert!(!should_publish_path_blocked(
            &completed,
            Some((&completed, true)),
        ));
    }

    #[test]
    fn abort_from_older_plan_is_ignored() {
        let completed = safe_zone_id(1, 3, 7);
        let active = safe_zone_id(1, 4, 1);

        assert!(!should_publish_path_blocked(
            &completed,
            Some((&active, false)),
        ));
    }

    #[test]
    fn abort_from_older_safe_zone_is_ignored() {
        let completed = safe_zone_id(1, 3, 7);
        let active = safe_zone_id(1, 3, 8);

        assert!(!should_publish_path_blocked(
            &completed,
            Some((&active, false)),
        ));
    }

    #[test]
    fn abort_without_an_active_goal_is_ignored() {
        let completed = safe_zone_id(1, 3, 7);

        assert!(!should_publish_path_blocked(&completed, None));
    }

    #[test]
    fn safe_zone_version_and_session_ordering() {
        let base = safe_zone_id(1, 0, 1);
        assert!(!is_safe_zone_newer(&base, &base));
        assert!(is_safe_zone_newer(&safe_zone_id(1, 0, 2), &base));
        assert!(!is_safe_zone_newer(&base, &safe_zone_id(1, 0, 2)));
        assert!(is_safe_zone_newer(&safe_zone_id(1, 1, 0), &base));
        assert!(is_safe_zone_newer(&safe_zone_id(2, 0, 0), &base));
    }

    #[test]
    fn actor_command_coalescing_keeps_latest_valid_command() {
        let req1 = InnerNavigationRequest {
            agent: Entity::PLACEHOLDER,
            safe_zone_id: safe_zone_id(1, 0, 1),
            target_pose: PoseStamped::default(),
            action: InnerNavigationAction::NavigateToPose,
        };
        let req2 = InnerNavigationRequest {
            agent: Entity::PLACEHOLDER,
            safe_zone_id: safe_zone_id(1, 0, 2),
            target_pose: PoseStamped::default(),
            action: InnerNavigationAction::NavigateToPose,
        };

        // Newer dispatch supersedes older dispatch
        assert_eq!(
            fold_agent_commands(
                AgentNavCommand::Dispatch(req1.clone()),
                AgentNavCommand::Dispatch(req2.clone())
            ),
            AgentNavCommand::Dispatch(req2.clone())
        );

        // Outdated dispatch is ignored when folded after a newer dispatch
        assert_eq!(
            fold_agent_commands(
                AgentNavCommand::Dispatch(req2.clone()),
                AgentNavCommand::Dispatch(req1.clone())
            ),
            AgentNavCommand::Dispatch(req2.clone())
        );

        // Cancel overrides pending dispatch
        assert_eq!(
            fold_agent_commands(
                AgentNavCommand::Dispatch(req2.clone()),
                AgentNavCommand::Cancel
            ),
            AgentNavCommand::Cancel
        );

        // Same SafeZoneId upgraded to DockRobot when Plan arrives late is accepted
        let req2_dock = InnerNavigationRequest {
            agent: Entity::PLACEHOLDER,
            safe_zone_id: safe_zone_id(1, 0, 2),
            target_pose: PoseStamped::default(),
            action: InnerNavigationAction::DockRobot {
                use_dock_id: true,
                dock_id: "dock_north_1".to_string(),
                dock_type: "simple_charging_dock".to_string(),
                navigate_to_staging_pose: false,
            },
        };
        assert!(should_preempt_goal(
            &req2_dock.safe_zone_id,
            &req2_dock.action,
            &req2.safe_zone_id,
            &req2.action,
        ));
    }
}
