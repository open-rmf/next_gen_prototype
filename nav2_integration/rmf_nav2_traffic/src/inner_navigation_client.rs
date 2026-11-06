use crate::{
    agent::AmclPose,
    navigation_server::{AgentPose, CancelledNavigationSessions, CurrentNavigationRequest},
    safe_zone::{
        build_progress_status, CurrentSafeZone, LatestProgress, PlanErrorPublisher,
        ProgressPublisher, ProgressTracker,
    },
    Nav2Agent,
};
use bevy::prelude::*;
use bevy_ros2::{RclrsExecutorCommands, RclrsNode, RosActionClient};
use crossflow::{prelude::*, service::Service};
use futures::{FutureExt, StreamExt};
use nalgebra::UnitQuaternion;
use rclrs::*;
use ros_env::{
    builtin_interfaces::msg::Time as RosTime,
    geometry_msgs::msg::{Point, Pose, PoseStamped, Quaternion},
    nav2_msgs::action::{
        DockRobot, DockRobot_Goal, NavigateToPose, NavigateToPose_Feedback, NavigateToPose_Goal,
        UndockRobot, UndockRobot_Goal,
    },
    rmf_prototype_msgs::msg::{Error as RmfError, PlanError, SafeZoneId},
    std_msgs::msg::Header,
};
use std::{
    future::Future,
    pin::Pin,
    sync::{Arc, Mutex},
};

/// Action-agnostic cancellation strategy for any active ROS 2 inner goal handle.
///
/// Implementing this trait for `GoalClient<A>` decouples `ActiveInnerGoal` and
/// the core traffic manager from concrete action types (such as `NavigateToPose`,
/// `DockRobot`, `UndockRobot`, etc.).
pub trait NavActionStrategy: Send + Sync {
    fn is_cancellation_accepted<'a>(&'a self) -> Pin<Box<dyn Future<Output = bool> + Send + 'a>>;
}

impl<A: ActionIDL> NavActionStrategy for GoalClient<A> {
    fn is_cancellation_accepted<'a>(&'a self) -> Pin<Box<dyn Future<Output = bool> + Send + 'a>> {
        Box::pin(async move {
            let accepted = self.cancellation.cancel().await.is_accepted();
            if accepted {
                let _ = self.result.clone().await;
            }
            accepted
        })
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Default)]
pub enum InnerNavigationAction {
    #[default]
    NavigateToPose,
    DockRobot {
        navigate_to_staging_pose: bool,
    },
    UndockRobot,
}

impl InnerNavigationAction {
    pub fn holds_pre_waypoint_progress(&self) -> bool {
        matches!(self, Self::DockRobot { .. } | Self::UndockRobot)
    }
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
pub struct ActiveInnerGoal {
    pub seq: u64,
    pub safe_zone_id: SafeZoneId,
    pub action: InnerNavigationAction,
    pub superseded: bool,
    pub goal_client: Option<Arc<dyn NavActionStrategy>>,
}

impl ActiveInnerGoal {
    pub fn id(&self) -> &SafeZoneId {
        &self.safe_zone_id
    }

    pub fn is_superseded(&self) -> bool {
        self.superseded
    }

    pub fn mark_superseded(&mut self) {
        self.superseded = true;
    }
}

#[derive(Default)]
pub struct AgentGoalState {
    pub next_seq: u64,
    pub cancelled_up_to_seq: u64,
    pub last_action: Option<InnerNavigationAction>,
    pub active_goal: Option<ActiveInnerGoal>,
}

#[derive(Component, Clone)]
pub struct InnerNavigationClient {
    pub action_client: Arc<RosActionClient<NavigateToPose>>,
    pub dock_client: Arc<RosActionClient<DockRobot>>,
    pub undock_client: Arc<RosActionClient<UndockRobot>>,
    pub state: Arc<Mutex<AgentGoalState>>,
}

impl InnerNavigationClient {
    pub fn new(
        action_client: Arc<RosActionClient<NavigateToPose>>,
        dock_client: Arc<RosActionClient<DockRobot>>,
        undock_client: Arc<RosActionClient<UndockRobot>>,
    ) -> Self {
        Self {
            action_client,
            dock_client,
            undock_client,
            state: Arc::new(Mutex::new(AgentGoalState::default())),
        }
    }

    pub fn has_active_goal(&self) -> bool {
        self.state
            .lock()
            .map(|s| s.active_goal.is_some())
            .unwrap_or(false)
    }

    pub fn active_safe_zone_id(&self) -> Option<SafeZoneId> {
        self.state
            .lock()
            .ok()
            .and_then(|s| s.active_goal.as_ref().map(|g| g.safe_zone_id.clone()))
    }

    pub fn matches_active_goal_id(&self, id: &SafeZoneId) -> bool {
        self.state
            .lock()
            .ok()
            .and_then(|s| s.active_goal.as_ref().map(|g| &g.safe_zone_id == id))
            .unwrap_or(true)
    }

    pub fn mark_active_goal_superseded(&self) {
        if let Ok(mut state) = self.state.lock() {
            if let Some(active) = state.active_goal.as_mut() {
                active.superseded = true;
            }
        }
    }

    pub fn reset_goal(&self) {
        if let Ok(mut state) = self.state.lock() {
            state.active_goal = None;
        }
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

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum GoalOutcomeStatus {
    Succeeded,
    Cancelled,
    Aborted,
    DockingFailed { error_code: u16, msg: String },
    UndockingFailed { error_code: u16, msg: String },
}

#[derive(Clone)]
pub struct GoalExecutionOutcome {
    pub seq: u64,
    pub target: InnerNavigationTarget,
    pub status: GoalOutcomeStatus,
}

#[derive(Resource, Clone)]
pub struct InnerNavigationServices {
    pub dispatch_target: Service<InnerNavigationTarget, ()>,
    pub cancel_agent: Service<Entity, ()>,
}

impl InnerNavigationServices {
    pub fn from_app(app: &mut App) -> Self {
        let preempt_and_prepare_service = app.spawn_service(preempt_and_prepare_goal);
        let execute_dock_service = app.spawn_service(execute_dock_robot);
        let execute_undock_service = app.spawn_service(execute_undock_robot);
        let execute_nav_service = app.spawn_service(execute_navigate_to_pose);
        let reconcile_outcome_service = app.spawn_service(reconcile_goal_outcome);

        // One-shot branching Crossflow workflow:
        //   preempt_and_prepare_goal
        //     ├── [Dock]     -> execute_dock_robot     ──┐
        //     ├── [Undock]   -> execute_undock_robot   ──┼──> reconcile_goal_outcome
        //     └── [Navigate] -> execute_navigate_to_pose ┘
        let dispatch_target = app.world_mut().spawn_io_workflow(|scope, builder| {
            let reconcile = builder.create_node(reconcile_outcome_service);

            builder
                .chain(scope.start)
                .then(preempt_and_prepare_service)
                .dispose_on_none()
                .fork_result(
                    |dock_chain| {
                        dock_chain
                            .then(execute_dock_service)
                            .connect(reconcile.input);
                    },
                    |rest_chain| {
                        rest_chain.fork_result(
                            |undock_chain| {
                                undock_chain
                                    .then(execute_undock_service)
                                    .connect(reconcile.input);
                            },
                            |nav_chain| {
                                nav_chain.then(execute_nav_service).connect(reconcile.input);
                            },
                        );
                    },
                );

            builder.chain(reconcile.output).connect(scope.terminate);
        });

        let prepare_cancel_service = app.spawn_service(prepare_external_cancellation);
        let execute_cancel_service = app.spawn_service(execute_external_cancellation);
        let finish_cancel_service = app.spawn_service(finish_external_cancellation);

        let cancel_agent = app.world_mut().spawn_io_workflow(|scope, builder| {
            builder
                .chain(scope.start)
                .then(prepare_cancel_service)
                .dispose_on_none()
                .then(execute_cancel_service)
                .then(finish_cancel_service)
                .connect(scope.terminate);
        });

        Self {
            dispatch_target,
            cancel_agent,
        }
    }
}

#[derive(Component, Clone, Default)]
pub struct LatestInnerFeedbackPose(pub Option<PoseStamped>);

fn record_inner_navigation_feedback(
    trigger: Trigger<InnerNavigationFeedback>,
    mut commands: Commands,
    inner_nav_clients: Query<&InnerNavigationClient>,
) {
    let event = trigger.event();
    if let Ok(inner_nav_client) = inner_nav_clients.get(event.agent) {
        if !inner_nav_client.matches_active_goal_id(&event.safe_zone_id) {
            return;
        }
    }
    commands
        .entity(event.agent)
        .insert(LatestInnerFeedbackPose(Some(
            event.feedback.current_pose.clone(),
        )));
}

#[derive(Default)]
pub struct InnerNavigationClientPlugin {}

impl Plugin for InnerNavigationClientPlugin {
    fn build(&self, app: &mut App) {
        app.add_event::<InnerNavigationTarget>()
            .add_event::<InnerNavigationFeedback>()
            .add_event::<CancelInnerForAgent>()
            .add_systems(Update, dispatch_external_cancellations)
            .add_observer(create_inner_navigation_client)
            .add_observer(record_inner_navigation_feedback);

        let navigation_services = InnerNavigationServices::from_app(app);
        app.insert_resource(navigation_services);
    }
}

fn create_inner_navigation_client(
    trigger: Trigger<OnAdd, Nav2Agent>,
    mut commands: Commands,
    agents: Query<&Nav2Agent>,
    node: Res<RclrsNode>,
) {
    let e = trigger.target();
    let Ok(agent_name) = agents.get(e).map(|agent| agent.name.clone()) else {
        return;
    };
    let nav_action = format!("{}/inner/navigate_to_pose", agent_name);
    let dock_action = format!("{}/inner/dock_robot", agent_name);
    let undock_action = format!("{}/inner/undock_robot", agent_name);

    commands.entity(e).insert(InnerNavigationClient::new(
        Arc::new(RosActionClient::<NavigateToPose>::new(&node, nav_action)),
        Arc::new(RosActionClient::<DockRobot>::new(&node, dock_action)),
        Arc::new(RosActionClient::<UndockRobot>::new(&node, undock_action)),
    ));
}

fn dispatch_external_cancellations(
    mut commands: Commands,
    mut cancel_requests: EventReader<CancelInnerForAgent>,
    nav_services: Res<InnerNavigationServices>,
) {
    for request in cancel_requests.read() {
        let _ = commands
            .request(request.agent, nav_services.cancel_agent.clone())
            .detach();
    }
}

#[derive(Clone)]
struct PreparedCancelContext {
    agent: Entity,
    prior_to_cancel: Option<Arc<dyn NavActionStrategy>>,
    cancel_all_nav: Option<Arc<RosActionClient<NavigateToPose>>>,
}

fn prepare_external_cancellation(
    Blocking { request: agent, .. }: Blocking<Entity>,
    mut commands: Commands,
    clients: Query<&InnerNavigationClient>,
) -> Option<PreparedCancelContext> {
    let Ok(client) = clients.get(agent) else {
        return None;
    };
    let mut state = client.state.lock().ok()?;
    state.cancelled_up_to_seq = state.next_seq;

    let (prior_to_cancel, cancel_all_nav) = if let Some(active) = state.active_goal.as_mut() {
        active.superseded = true;
        let prior = active.goal_client.take();
        let cancel_nav =
            if prior.is_none() && matches!(active.action, InnerNavigationAction::NavigateToPose) {
                Some(Arc::clone(&client.action_client))
            } else {
                None
            };
        (prior, cancel_nav)
    } else if matches!(
        state.last_action,
        Some(InnerNavigationAction::NavigateToPose)
    ) {
        (None, Some(Arc::clone(&client.action_client)))
    } else {
        return None;
    };

    commands
        .entity(agent)
        .insert(CancellingInnerNavigation { success: false });

    Some(PreparedCancelContext {
        agent,
        prior_to_cancel,
        cancel_all_nav,
    })
}

fn execute_external_cancellation(
    Async {
        request: prepared, ..
    }: Async<PreparedCancelContext>,
    executor_commands: Res<RclrsExecutorCommands>,
) -> impl Future<Output = Entity> {
    let agent = prepared.agent;
    executor_commands
        .run(async move {
            if let Some(prior) = prepared.prior_to_cancel {
                let _ = prior.is_cancellation_accepted().await;
            } else if let Some(nav_client) = prepared.cancel_all_nav {
                let mut cancel_req = nav_client.cancel_all_goals();
                let _ = (&mut *cancel_req).await;
            }
            agent
        })
        .map(move |res| res.unwrap_or(agent))
}

fn finish_external_cancellation(
    Blocking { request: agent, .. }: Blocking<Entity>,
    clients: Query<&InnerNavigationClient>,
    mut cancelling_inner: Query<&mut CancellingInnerNavigation>,
) {
    if let Ok(client) = clients.get(agent) {
        if let Ok(mut state) = client.state.lock() {
            if state
                .active_goal
                .as_ref()
                .is_some_and(|g| g.seq <= state.cancelled_up_to_seq)
            {
                state.active_goal = None;
            }
        }
    }
    if let Ok(mut cancelling) = cancelling_inner.get_mut(agent) {
        cancelling.success = true;
    }
}

#[derive(Clone)]
struct PreparedGoalContext {
    seq: u64,
    target: InnerNavigationTarget,
    state: Arc<Mutex<AgentGoalState>>,
    prior_to_cancel: Option<Arc<dyn NavActionStrategy>>,
    cancel_all_nav: Option<Arc<RosActionClient<NavigateToPose>>>,
}

#[derive(Clone)]
struct PreparedDockGoal {
    ctx: PreparedGoalContext,
    client: Arc<RosActionClient<DockRobot>>,
    goal: DockRobot_Goal,
}

#[derive(Clone)]
struct PreparedUndockGoal {
    ctx: PreparedGoalContext,
    client: Arc<RosActionClient<UndockRobot>>,
    goal: UndockRobot_Goal,
}

#[derive(Clone)]
struct PreparedNavGoal {
    ctx: PreparedGoalContext,
    client: Arc<RosActionClient<NavigateToPose>>,
    goal: NavigateToPose_Goal,
}

type PreparedBranch = Result<PreparedDockGoal, Result<PreparedUndockGoal, PreparedNavGoal>>;

fn target_to_pose_stamped(x: f64, y: f64, yaw: f64) -> PoseStamped {
    let quat = UnitQuaternion::from_euler_angles(0.0, 0.0, yaw);
    let [qx, qy, qz, qw] = quat.coords.into();
    PoseStamped {
        header: Header {
            stamp: RosTime { sec: 0, nanosec: 0 },
            frame_id: "map".to_string(),
        },
        pose: Pose {
            position: Point { x, y, z: 0.0 },
            orientation: Quaternion {
                x: qx,
                y: qy,
                z: qz,
                w: qw,
            },
        },
    }
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

/// Step 1 (Blocking): Preempts any older active goal and routes the target into
/// the Dock, Undock, or NavigateToPose branch.
fn preempt_and_prepare_goal(
    Blocking {
        request: target, ..
    }: Blocking<InnerNavigationTarget>,
    clients: Query<&InnerNavigationClient>,
    cancelled_sessions: Query<&CancelledNavigationSessions>,
) -> Option<PreparedBranch> {
    if cancelled_sessions
        .get(target.agent)
        .is_ok_and(|cancelled| cancelled.contains(&target.safe_zone_id.plan_id.destination_session))
    {
        return None;
    }

    let Ok(client) = clients.get(target.agent) else {
        return None;
    };
    let mut state = client.state.lock().ok()?;

    let mut prior_to_cancel: Option<Arc<dyn NavActionStrategy>> = None;
    let mut cancel_all_nav: Option<Arc<RosActionClient<NavigateToPose>>> = None;
    let current_seq = state.next_seq;
    let mut mark_cancelled_up_to = false;

    if let Some(active) = state.active_goal.as_mut() {
        if !should_preempt_goal(
            &target.safe_zone_id,
            &target.action,
            &active.safe_zone_id,
            &active.action,
        ) {
            return None;
        }
        active.superseded = true;
        let existing_is_nav = matches!(active.action, InnerNavigationAction::NavigateToPose);
        let next_is_nav = matches!(target.action, InnerNavigationAction::NavigateToPose);

        // Consecutive NavigateToPose goals preempt seamlessly inside Nav2's bt_navigator;
        // switching between Dock/Undock/Navigate requires cancelling the previous action first.
        if !(existing_is_nav && next_is_nav) {
            mark_cancelled_up_to = true;
            prior_to_cancel = active.goal_client.take();
            if prior_to_cancel.is_none() && existing_is_nav {
                cancel_all_nav = Some(Arc::clone(&client.action_client));
            }
        }
    } else if state
        .last_action
        .as_ref()
        .is_some_and(|last| last != &target.action)
    {
        mark_cancelled_up_to = true;
    }

    if mark_cancelled_up_to {
        state.cancelled_up_to_seq = current_seq;
    }

    state.next_seq = current_seq + 1;
    let seq = state.next_seq;
    state.last_action = Some(target.action.clone());
    state.active_goal = Some(ActiveInnerGoal {
        seq,
        safe_zone_id: target.safe_zone_id.clone(),
        action: target.action.clone(),
        superseded: false,
        goal_client: None,
    });

    let pose = target_to_pose_stamped(target.x, target.y, target.yaw);
    let ctx = PreparedGoalContext {
        seq,
        target: target.clone(),
        state: Arc::clone(&client.state),
        prior_to_cancel,
        cancel_all_nav,
    };

    match &target.action {
        InnerNavigationAction::DockRobot {
            navigate_to_staging_pose,
        } => {
            let goal = DockRobot_Goal {
                use_dock_id: false,
                dock_id: String::new(),
                dock_pose: pose,
                dock_type: String::new(),
                max_staging_time: 1000.0,
                navigate_to_staging_pose: *navigate_to_staging_pose,
            };
            Some(Ok(PreparedDockGoal {
                ctx,
                client: Arc::clone(&client.dock_client),
                goal,
            }))
        }
        InnerNavigationAction::UndockRobot => {
            let goal = UndockRobot_Goal {
                dock_type: String::new(),
                max_undocking_time: 30.0,
            };
            Some(Err(Ok(PreparedUndockGoal {
                ctx,
                client: Arc::clone(&client.undock_client),
                goal,
            })))
        }
        InnerNavigationAction::NavigateToPose => {
            let goal = NavigateToPose_Goal {
                pose,
                behavior_tree: String::new(),
            };
            Some(Err(Err(PreparedNavGoal {
                ctx,
                client: Arc::clone(&client.action_client),
                goal,
            })))
        }
    }
}

async fn cancel_prior_if_needed(ctx: &PreparedGoalContext) {
    if let Some(prior) = &ctx.prior_to_cancel {
        let _ = prior.is_cancellation_accepted().await;
    } else if let Some(nav_client) = &ctx.cancel_all_nav {
        let mut cancel_req = nav_client.cancel_all_goals();
        let _ = (&mut *cancel_req).await;
    }
}

fn register_or_cancel_handle(
    state_arc: &Arc<Mutex<AgentGoalState>>,
    seq: u64,
    handle: Arc<dyn NavActionStrategy>,
) -> Option<Arc<dyn NavActionStrategy>> {
    let Ok(mut state) = state_arc.lock() else {
        return Some(handle);
    };
    if seq <= state.cancelled_up_to_seq {
        return Some(handle);
    }
    if let Some(active) = state.active_goal.as_mut() {
        if active.seq == seq {
            active.goal_client = Some(handle);
            return None;
        }
    }
    None
}

/// Branch A (Async): Executes `DockRobot` on `/<robot>/inner/dock_robot`.
fn execute_dock_robot(
    Async {
        request: prepared, ..
    }: Async<PreparedDockGoal>,
    executor_commands: Res<RclrsExecutorCommands>,
) -> impl Future<Output = GoalExecutionOutcome> {
    let fallback_seq = prepared.ctx.seq;
    let fallback_target = prepared.ctx.target.clone();
    executor_commands
        .run(async move {
            let PreparedDockGoal { ctx, client, goal } = prepared;
            cancel_prior_if_needed(&ctx).await;

            let Some(goal_client) = client.request_goal(goal).await else {
                return GoalExecutionOutcome {
                    seq: ctx.seq,
                    target: ctx.target,
                    status: GoalOutcomeStatus::DockingFailed {
                        error_code: 0,
                        msg: "Failed to request DockRobot goal".to_string(),
                    },
                };
            };

            let strategy_handle: Arc<dyn NavActionStrategy> = Arc::new(goal_client.clone());
            if let Some(to_cancel) = register_or_cancel_handle(&ctx.state, ctx.seq, strategy_handle)
            {
                let _ = to_cancel.is_cancellation_accepted().await;
                return GoalExecutionOutcome {
                    seq: ctx.seq,
                    target: ctx.target,
                    status: GoalOutcomeStatus::Cancelled,
                };
            }

            let (status, response) = goal_client.result.await;
            let outcome_status = match status {
                GoalStatusCode::Succeeded if response.success => GoalOutcomeStatus::Succeeded,
                GoalStatusCode::Cancelled => GoalOutcomeStatus::Cancelled,
                _ => GoalOutcomeStatus::DockingFailed {
                    error_code: response.error_code,
                    msg: response.error_msg,
                },
            };

            GoalExecutionOutcome {
                seq: ctx.seq,
                target: ctx.target,
                status: outcome_status,
            }
        })
        .map(move |res| {
            res.unwrap_or(GoalExecutionOutcome {
                seq: fallback_seq,
                target: fallback_target,
                status: GoalOutcomeStatus::DockingFailed {
                    error_code: 0,
                    msg: "Executor cancelled DockRobot future".to_string(),
                },
            })
        })
}

/// Branch B (Async): Executes `UndockRobot` on `/<robot>/inner/undock_robot`.
fn execute_undock_robot(
    Async {
        request: prepared, ..
    }: Async<PreparedUndockGoal>,
    executor_commands: Res<RclrsExecutorCommands>,
) -> impl Future<Output = GoalExecutionOutcome> {
    let fallback_seq = prepared.ctx.seq;
    let fallback_target = prepared.ctx.target.clone();
    executor_commands
        .run(async move {
            let PreparedUndockGoal { ctx, client, goal } = prepared;
            cancel_prior_if_needed(&ctx).await;

            let Some(goal_client) = client.request_goal(goal).await else {
                return GoalExecutionOutcome {
                    seq: ctx.seq,
                    target: ctx.target,
                    status: GoalOutcomeStatus::UndockingFailed {
                        error_code: 0,
                        msg: "Failed to request UndockRobot goal".to_string(),
                    },
                };
            };

            let strategy_handle: Arc<dyn NavActionStrategy> = Arc::new(goal_client.clone());
            if let Some(to_cancel) = register_or_cancel_handle(&ctx.state, ctx.seq, strategy_handle)
            {
                let _ = to_cancel.is_cancellation_accepted().await;
                return GoalExecutionOutcome {
                    seq: ctx.seq,
                    target: ctx.target,
                    status: GoalOutcomeStatus::Cancelled,
                };
            }

            let (status, response) = goal_client.result.await;
            let outcome_status = match status {
                GoalStatusCode::Succeeded if response.success => GoalOutcomeStatus::Succeeded,
                GoalStatusCode::Cancelled => GoalOutcomeStatus::Cancelled,
                _ => GoalOutcomeStatus::UndockingFailed {
                    error_code: response.error_code,
                    msg: response.error_msg,
                },
            };

            GoalExecutionOutcome {
                seq: ctx.seq,
                target: ctx.target,
                status: outcome_status,
            }
        })
        .map(move |res| {
            res.unwrap_or(GoalExecutionOutcome {
                seq: fallback_seq,
                target: fallback_target,
                status: GoalOutcomeStatus::UndockingFailed {
                    error_code: 0,
                    msg: "Executor cancelled UndockRobot future".to_string(),
                },
            })
        })
}

/// Branch C (Async): Executes `NavigateToPose` on `/<robot>/inner/navigate_to_pose`.
fn execute_navigate_to_pose(
    Async {
        request: prepared,
        channel,
        ..
    }: Async<PreparedNavGoal>,
    executor_commands: Res<RclrsExecutorCommands>,
) -> impl Future<Output = GoalExecutionOutcome> {
    let fallback_seq = prepared.ctx.seq;
    let fallback_target = prepared.ctx.target.clone();
    executor_commands
        .run(async move {
            let PreparedNavGoal { ctx, client, goal } = prepared;
            cancel_prior_if_needed(&ctx).await;

            let Some(goal_client) = client.request_goal(goal).await else {
                return GoalExecutionOutcome {
                    seq: ctx.seq,
                    target: ctx.target,
                    status: GoalOutcomeStatus::Aborted,
                };
            };

            let strategy_handle: Arc<dyn NavActionStrategy> = Arc::new(goal_client.clone());
            if let Some(to_cancel) = register_or_cancel_handle(&ctx.state, ctx.seq, strategy_handle)
            {
                let _ = to_cancel.is_cancellation_accepted().await;
                return GoalExecutionOutcome {
                    seq: ctx.seq,
                    target: ctx.target,
                    status: GoalOutcomeStatus::Cancelled,
                };
            }

            let agent = ctx.target.agent;
            let safe_zone_id = ctx.target.safe_zone_id.clone();
            let mut stream = goal_client.stream();
            let mut final_status = GoalOutcomeStatus::Aborted;

            while let Some(event) = stream.next().await {
                match event {
                    GoalEvent::Feedback(fb) => {
                        let sz_id = safe_zone_id.clone();
                        channel.commands(move |commands| {
                            commands.trigger(InnerNavigationFeedback::new(agent, sz_id, fb));
                        });
                    }
                    GoalEvent::Status(_) => {}
                    GoalEvent::Result((status, _)) => {
                        final_status = match status {
                            GoalStatusCode::Succeeded => GoalOutcomeStatus::Succeeded,
                            GoalStatusCode::Cancelled => GoalOutcomeStatus::Cancelled,
                            _ => GoalOutcomeStatus::Aborted,
                        };
                        break;
                    }
                }
            }

            GoalExecutionOutcome {
                seq: ctx.seq,
                target: ctx.target,
                status: final_status,
            }
        })
        .map(move |res| {
            res.unwrap_or(GoalExecutionOutcome {
                seq: fallback_seq,
                target: fallback_target,
                status: GoalOutcomeStatus::Aborted,
            })
        })
}

/// Step 3 (Blocking): Reconciles the finished goal outcome with `ProgressTracker`,
/// re-dispatching if orientation is not yet satisfied or releasing holds and
/// publishing `PlanError` on abort / docking failure.
fn reconcile_goal_outcome(
    Blocking {
        request: outcome, ..
    }: Blocking<GoalExecutionOutcome>,
    mut commands: Commands,
    nav_services: Res<InnerNavigationServices>,
    clients: Query<&InnerNavigationClient>,
    mut cancelling_inner: Query<&mut CancellingInnerNavigation>,
    current_nav_requests: Query<&CurrentNavigationRequest>,
    mut cancelled_sessions: Query<&mut CancelledNavigationSessions>,
    error_publishers: Query<&PlanErrorPublisher>,
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
    let agent = outcome.target.agent;
    let mut was_superseded = true;

    if let Ok(client) = clients.get(agent) {
        if let Ok(mut state) = client.state.lock() {
            if let Some(active) = state.active_goal.as_ref() {
                if active.seq == outcome.seq {
                    was_superseded = active.superseded;
                    state.active_goal = None;
                }
            }
        }
    }

    if outcome.status == GoalOutcomeStatus::Cancelled {
        if let Ok(mut cancelling) = cancelling_inner.get_mut(agent) {
            info!(
                "[{:?}] Goal cancelled. Marking cancellation as success.",
                agent.index()
            );
            cancelling.success = true;
        }
        return;
    }

    if was_superseded {
        return;
    }

    match outcome.status {
        GoalOutcomeStatus::Succeeded => {
            if let Ok((
                agent_comp,
                mut tracker,
                progress_pub,
                mut latest_progress,
                current_safe_zone,
                amcl_pose,
                agent_pose,
                latest_fb_pose,
            )) = progress_query.get_mut(agent)
            {
                let matches_sz = current_safe_zone
                    .and_then(|csz| csz.0.as_ref())
                    .map(|sz| sz.id == outcome.target.safe_zone_id)
                    .unwrap_or(true);
                if matches_sz
                    && tracker.plan_id.as_ref() == Some(&outcome.target.safe_zone_id.plan_id)
                {
                    if outcome.target.action.holds_pre_waypoint_progress() {
                        tracker.mark_action_completed();
                    }
                    let fb_pose = latest_fb_pose.and_then(|p| p.0.as_ref());
                    let current_pose = amcl_pose
                        .filter(|_| agent_comp.localized)
                        .map(|p| p.xy_yaw())
                        .or_else(|| fb_pose.map(|p| AgentPose(p.clone()).xy_yaw()))
                        .or_else(|| agent_pose.map(|p| p.xy_yaw()));
                    if !tracker.is_target_pose_satisfied(current_pose) {
                        warn!(
                            "[{:?}] Nav2 reported goal succeeded before target pose was satisfied (current={:?}); retrying goal {:?}.",
                            agent.index(),
                            current_pose,
                            outcome.target.safe_zone_id,
                        );
                        let _ = commands
                            .request(outcome.target, nav_services.dispatch_target.clone())
                            .detach();
                        return;
                    }
                    if let Some(reached_prog) =
                        tracker.mark_target_reached(latest_progress.0.as_ref())
                    {
                        let pose = amcl_pose
                            .filter(|_| agent_comp.localized)
                            .map(|p| PoseStamped {
                                header: p.0.header.clone(),
                                pose: p.0.pose.pose.clone(),
                            })
                            .or_else(|| fb_pose.cloned())
                            .or_else(|| agent_pose.map(|p| p.0.clone()))
                            .unwrap_or_default();
                        let sz_id = current_safe_zone
                            .and_then(|csz| csz.0.as_ref().map(|sz| &sz.id))
                            .or(Some(&outcome.target.safe_zone_id));
                        let mode = tracker.execution_mode(true);
                        let progress_msg =
                            build_progress_status(pose, mode, Some(&reached_prog), sz_id);
                        if latest_progress.0.as_ref() != Some(&progress_msg) {
                            let _ = progress_pub.publisher.publish(progress_msg.clone());
                            latest_progress.0 = Some(progress_msg);
                        }
                    }
                }
            }
        }
        GoalOutcomeStatus::DockingFailed { error_code, msg }
        | GoalOutcomeStatus::UndockingFailed { error_code, msg } => {
            if let Ok((
                agent_comp,
                mut tracker,
                progress_pub,
                mut latest_progress,
                current_safe_zone,
                amcl_pose,
                agent_pose,
                latest_fb_pose,
            )) = progress_query.get_mut(agent)
            {
                tracker.force_release_holds();
                if tracker.plan_id.is_some() {
                    let fb_pose = latest_fb_pose.and_then(|p| p.0.as_ref());
                    let current_pose = amcl_pose
                        .filter(|_| agent_comp.localized)
                        .map(|p| p.xy_yaw())
                        .or_else(|| fb_pose.map(|p| AgentPose(p.clone()).xy_yaw()))
                        .or_else(|| agent_pose.map(|p| p.xy_yaw()));
                    let updated_prog =
                        tracker.compute_progress(current_pose, latest_progress.0.as_ref());
                    let pose = amcl_pose
                        .filter(|_| agent_comp.localized)
                        .map(|p| PoseStamped {
                            header: p.0.header.clone(),
                            pose: p.0.pose.pose.clone(),
                        })
                        .or_else(|| fb_pose.cloned())
                        .or_else(|| agent_pose.map(|p| p.0.clone()))
                        .unwrap_or_default();
                    let sz_id = current_safe_zone
                        .and_then(|csz| csz.0.as_ref().map(|sz| &sz.id))
                        .or(Some(&outcome.target.safe_zone_id));
                    let mode = tracker.execution_mode(false);
                    let progress_msg =
                        build_progress_status(pose, mode, Some(&updated_prog), sz_id);
                    let _ = progress_pub.publisher.publish(progress_msg.clone());
                    latest_progress.0 = Some(progress_msg);
                }
            }

            if let Ok(current_nav_req) = current_nav_requests.get(agent) {
                if current_nav_req.session()
                    == &outcome.target.safe_zone_id.plan_id.destination_session
                {
                    if let Ok(mut cancelled) = cancelled_sessions.get_mut(agent) {
                        for session in current_nav_req.superseded_sessions() {
                            cancelled.insert(session.clone());
                        }
                        cancelled.insert(current_nav_req.session().clone());
                    }
                    current_nav_req.cancel();
                    commands.entity(agent).remove::<CurrentNavigationRequest>();
                }
            }

            if let Ok(publisher) = error_publishers.get(agent) {
                let plan_error = PlanError {
                    error: RmfError {
                        code: RmfError::CODE_UNSPECIFIED_ERROR,
                        message: format!(
                            "Docking/undocking action failed for agent {:?} (code={}): {}",
                            agent.index(),
                            error_code,
                            msg
                        ),
                        parameters: String::new(),
                    },
                    plan_id: outcome.target.safe_zone_id.plan_id.clone(),
                };
                if let Err(e) = publisher.publisher.publish(plan_error) {
                    error!(
                        "Failed to publish PlanError for agent {:?}: {:?}",
                        agent.index(),
                        e
                    );
                } else {
                    warn!(
                        "[{:?}] Docking action failed (code={}, msg='{}'); gave up and published PlanError (CODE_UNSPECIFIED_ERROR).",
                        agent.index(),
                        error_code,
                        msg
                    );
                }
            }
        }
        GoalOutcomeStatus::Aborted => {
            if let Ok(publisher) = error_publishers.get(agent) {
                let plan_error = PlanError {
                    error: RmfError {
                        code: PlanError::CODE_PATH_BLOCKED,
                        message: format!(
                            "Nav2 goal aborted for agent {:?}: path blocked within safe zone",
                            agent.index()
                        ),
                        parameters: String::new(),
                    },
                    plan_id: outcome.target.safe_zone_id.plan_id.clone(),
                };
                if let Err(e) = publisher.publisher.publish(plan_error) {
                    error!(
                        "Failed to publish PlanError for agent {:?}: {:?}",
                        agent.index(),
                        e
                    );
                } else {
                    warn!(
                        "[{:?}] Goal aborted by Nav2! Published PlanError CODE_PATH_BLOCKED to ~/plan/error.",
                        agent.index()
                    );
                }
            }
        }
        GoalOutcomeStatus::Cancelled => {}
    }
}

pub fn should_publish_path_blocked(
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
    fn newer_safe_zone_detection_orders_sessions_plans_and_versions() {
        let sz_v1 = safe_zone_id(1, 2, 3);
        let sz_v2 = safe_zone_id(1, 2, 4);
        let sz_next_plan = safe_zone_id(1, 3, 0);
        let sz_new_session = safe_zone_id(2, 0, 0);

        assert!(is_safe_zone_newer(&sz_v2, &sz_v1));
        assert!(!is_safe_zone_newer(&sz_v1, &sz_v2));
        assert!(is_safe_zone_newer(&sz_next_plan, &sz_v2));
        assert!(is_safe_zone_newer(&sz_new_session, &sz_next_plan));
    }

    #[test]
    fn preemption_triggers_on_action_upgrade_for_same_safe_zone() {
        let sz = safe_zone_id(1, 2, 3);
        let nav = InnerNavigationAction::NavigateToPose;
        let dock = InnerNavigationAction::DockRobot {
            navigate_to_staging_pose: false,
        };

        assert!(!should_preempt_goal(&sz, &nav, &sz, &nav));
        assert!(should_preempt_goal(&sz, &dock, &sz, &nav));
    }
}
