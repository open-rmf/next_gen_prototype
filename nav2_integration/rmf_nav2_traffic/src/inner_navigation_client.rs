use crate::{
    agent::{AgentPose, AmclPose},
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
    rmf_prototype_msgs::msg::{PlanError, Progress, SafeZoneId},
    std_msgs::msg::Header,
};
use std::{
    future::Future,
    sync::{Arc, Mutex},
};

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

#[derive(Clone, Debug)]
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
    pub async fn cancel_and_wait(&self) {
        match self {
            Self::NavigateToPose(c) => {
                if c.cancellation.cancel().await.is_accepted() {
                    let _ = c.result.clone().await;
                }
            }
            Self::DockRobot(c) => {
                if c.cancellation.cancel().await.is_accepted() {
                    let _ = c.result.clone().await;
                }
            }
            Self::UndockRobot(c) => {
                if c.cancellation.cancel().await.is_accepted() {
                    let _ = c.result.clone().await;
                }
            }
        }
    }
}

#[derive(Clone)]
pub struct ActiveInnerGoal {
    pub seq: u64,
    pub safe_zone_id: SafeZoneId,
    pub action: InnerNavigationAction,
    pub superseded: bool,
    pub goal_client: Option<InnerGoalClient>,
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
    pub nav_client: Arc<RosActionClient<NavigateToPose>>,
    pub dock_client: Arc<RosActionClient<DockRobot>>,
    pub undock_client: Arc<RosActionClient<UndockRobot>>,
    pub state: Arc<Mutex<AgentGoalState>>,
}

impl InnerNavigationClient {
    pub fn new(
        nav_client: Arc<RosActionClient<NavigateToPose>>,
        dock_client: Arc<RosActionClient<DockRobot>>,
        undock_client: Arc<RosActionClient<UndockRobot>>,
    ) -> Self {
        Self {
            nav_client,
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

    pub fn mark_active_goal_superseded(&self) {
        if let Ok(mut state) = self.state.lock() {
            if let Some(active) = state.active_goal.as_mut() {
                active.superseded = true;
            }
        }
    }
}

#[derive(Event)]
pub struct InnerNavigationFeedback {
    pub agent: Entity,
    pub feedback: NavigateToPose_Feedback,
}

#[derive(Resource, Clone)]
pub struct InnerNavigationServices {
    pub dispatch_target: Service<InnerNavigationTarget, ()>,
}

impl InnerNavigationServices {
    pub fn from_app(app: &mut App) -> Self {
        let preempt_and_prepare_service = app.spawn_service(preempt_and_prepare_goal);
        let execute_dock_service = app.spawn_service(execute_dock_robot);
        let execute_undock_service = app.spawn_service(execute_undock_robot);
        let execute_nav_service = app.spawn_service(execute_navigate_to_pose);
        let reconcile_outcome_service = app.spawn_service(reconcile_goal_outcome);

        // One-shot branching workflow matching the BT structure:
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

        Self { dispatch_target }
    }
}

#[derive(Default)]
pub struct InnerNavigationClientPlugin {}

impl Plugin for InnerNavigationClientPlugin {
    fn build(&self, app: &mut App) {
        app.add_event::<InnerNavigationFeedback>()
            .add_observer(create_inner_navigation_client)
            .add_observer(record_inner_navigation_feedback);

        let services = InnerNavigationServices::from_app(app);
        app.insert_resource(services);
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
    let nav_action = format!("/{}/navigate_to_pose", agent_name);
    let dock_action = format!("/{}/dock_robot", agent_name);
    let undock_action = format!("/{}/undock_robot", agent_name);

    commands.entity(e).insert(InnerNavigationClient::new(
        Arc::new(RosActionClient::<NavigateToPose>::new(&node, nav_action)),
        Arc::new(RosActionClient::<DockRobot>::new(&node, dock_action)),
        Arc::new(RosActionClient::<UndockRobot>::new(&node, undock_action)),
    ));
}

fn record_inner_navigation_feedback(
    trigger: Trigger<InnerNavigationFeedback>,
    mut commands: Commands,
    agents: Query<&Nav2Agent>,
) {
    let feedback = trigger.event();
    if agents
        .get(feedback.agent)
        .is_ok_and(|agent| !agent.localized)
    {
        commands
            .entity(feedback.agent)
            .insert(AgentPose(feedback.feedback.current_pose.clone()));
    }
}

#[derive(Clone)]
struct PreparedGoalContext {
    seq: u64,
    target: InnerNavigationTarget,
    state: Arc<Mutex<AgentGoalState>>,
    prior_to_cancel: Option<InnerGoalClient>,
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

#[derive(Clone)]
struct GoalExecutionOutcome {
    seq: u64,
    target: InnerNavigationTarget,
    status: GoalStatusCode,
}

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
) -> Option<PreparedBranch> {
    let Ok(client) = clients.get(target.agent) else {
        return None;
    };
    let mut state = client.state.lock().ok()?;

    let mut prior_to_cancel: Option<InnerGoalClient> = None;
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
                cancel_all_nav = Some(Arc::clone(&client.nav_client));
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
            use_dock_id,
            dock_id,
            dock_type,
            navigate_to_staging_pose,
        } => {
            let goal = DockRobot_Goal {
                use_dock_id: *use_dock_id,
                dock_id: dock_id.clone(),
                dock_pose: pose,
                dock_type: dock_type.clone(),
                max_staging_time: 1000.0,
                navigate_to_staging_pose: *navigate_to_staging_pose,
            };
            Some(Ok(PreparedDockGoal {
                ctx,
                client: Arc::clone(&client.dock_client),
                goal,
            }))
        }
        InnerNavigationAction::UndockRobot { dock_type } => {
            let goal = UndockRobot_Goal {
                dock_type: dock_type.clone(),
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
                client: Arc::clone(&client.nav_client),
                goal,
            })))
        }
    }
}

async fn cancel_prior_if_needed(ctx: &PreparedGoalContext) {
    if let Some(prior) = &ctx.prior_to_cancel {
        prior.cancel_and_wait().await;
    } else if let Some(nav_client) = &ctx.cancel_all_nav {
        let mut cancel_req = nav_client.cancel_all_goals();
        let _ = (&mut *cancel_req).await;
    }
}

fn register_or_cancel_handle(
    state_arc: &Arc<Mutex<AgentGoalState>>,
    seq: u64,
    handle: InnerGoalClient,
) -> Option<InnerGoalClient> {
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

/// Branch A (Async): Executes `DockRobot` directly on `/<robot>/dock_robot`.
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
                    status: GoalStatusCode::Aborted,
                };
            };

            if let Some(to_cancel) = register_or_cancel_handle(
                &ctx.state,
                ctx.seq,
                InnerGoalClient::DockRobot(goal_client.clone()),
            ) {
                to_cancel.cancel_and_wait().await;
                return GoalExecutionOutcome {
                    seq: ctx.seq,
                    target: ctx.target,
                    status: GoalStatusCode::Cancelled,
                };
            }

            let (status, response) = goal_client.result.await;
            let status = if status == GoalStatusCode::Succeeded && !response.success {
                GoalStatusCode::Aborted
            } else {
                status
            };

            GoalExecutionOutcome {
                seq: ctx.seq,
                target: ctx.target,
                status,
            }
        })
        .map(move |res| {
            res.unwrap_or(GoalExecutionOutcome {
                seq: fallback_seq,
                target: fallback_target,
                status: GoalStatusCode::Aborted,
            })
        })
}

/// Branch B (Async): Executes `UndockRobot` directly on `/<robot>/undock_robot`.
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
                    status: GoalStatusCode::Aborted,
                };
            };

            if let Some(to_cancel) = register_or_cancel_handle(
                &ctx.state,
                ctx.seq,
                InnerGoalClient::UndockRobot(goal_client.clone()),
            ) {
                to_cancel.cancel_and_wait().await;
                return GoalExecutionOutcome {
                    seq: ctx.seq,
                    target: ctx.target,
                    status: GoalStatusCode::Cancelled,
                };
            }

            let (status, response) = goal_client.result.await;
            let status = if status == GoalStatusCode::Succeeded && !response.success {
                GoalStatusCode::Aborted
            } else {
                status
            };

            GoalExecutionOutcome {
                seq: ctx.seq,
                target: ctx.target,
                status,
            }
        })
        .map(move |res| {
            res.unwrap_or(GoalExecutionOutcome {
                seq: fallback_seq,
                target: fallback_target,
                status: GoalStatusCode::Aborted,
            })
        })
}

/// Branch C (Async): Executes `NavigateToPose` directly on `/<robot>/navigate_to_pose`.
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
                    status: GoalStatusCode::Aborted,
                };
            };

            if let Some(to_cancel) = register_or_cancel_handle(
                &ctx.state,
                ctx.seq,
                InnerGoalClient::NavigateToPose(goal_client.clone()),
            ) {
                to_cancel.cancel_and_wait().await;
                return GoalExecutionOutcome {
                    seq: ctx.seq,
                    target: ctx.target,
                    status: GoalStatusCode::Cancelled,
                };
            }

            let agent = ctx.target.agent;
            let mut stream = goal_client.stream();
            let mut final_status = GoalStatusCode::Aborted;

            while let Some(event) = stream.next().await {
                match event {
                    GoalEvent::Feedback(fb) => {
                        channel.commands(move |commands| {
                            commands.trigger(InnerNavigationFeedback {
                                agent,
                                feedback: fb,
                            });
                        });
                    }
                    GoalEvent::Status(_) => {}
                    GoalEvent::Result((status, _)) => {
                        final_status = status;
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
                status: GoalStatusCode::Aborted,
            })
        })
}

/// Step 3 (Blocking): Reconciles the finished goal outcome with `ProgressTracker`,
/// re-dispatching if orientation is not yet satisfied or publishing `PlanError` on abort.
fn reconcile_goal_outcome(
    Blocking {
        request: outcome, ..
    }: Blocking<GoalExecutionOutcome>,
    mut commands: Commands,
    nav_services: Res<InnerNavigationServices>,
    clients: Query<&InnerNavigationClient>,
    error_publishers: Query<&PlanErrorPublisher>,
    mut progress_query: Query<(
        &Nav2Agent,
        Option<&AmclPose>,
        &CurrentSafeZone,
        &ProgressPublisher,
        &mut LatestProgress,
        &mut ProgressTracker,
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

    if was_superseded {
        return;
    }

    match outcome.status {
        GoalStatusCode::Succeeded => {
            if let Ok((
                nav_agent,
                amcl_pose,
                current_sz,
                progress_pub,
                mut latest_progress,
                mut tracker,
            )) = progress_query.get_mut(agent)
            {
                let matches_sz = current_sz
                    .0
                    .as_ref()
                    .map(|sz| sz.id == outcome.target.safe_zone_id)
                    .unwrap_or(true);
                if matches_sz
                    && tracker.plan_id.as_ref() == Some(&outcome.target.safe_zone_id.plan_id)
                {
                    if matches!(
                        outcome.target.action,
                        InnerNavigationAction::DockRobot { .. }
                            | InnerNavigationAction::UndockRobot { .. }
                    ) {
                        tracker.mark_action_completed();
                    }
                    let current_pose = amcl_pose
                        .filter(|_| nav_agent.localized)
                        .map(|p| p.xy_yaw());
                    if !tracker.is_target_pose_satisfied(current_pose) {
                        warn!(
                            "[{}] Nav2 reported Succeeded before target orientation satisfied; re-dispatching ({:.2}, {:.2}, {:.2})",
                            nav_agent.name, outcome.target.x, outcome.target.y, outcome.target.yaw
                        );
                        let _ = commands
                            .request(outcome.target, nav_services.dispatch_target.clone())
                            .detach();
                    } else if let Some(updated_prog) =
                        tracker.mark_target_reached(latest_progress.0.as_ref())
                    {
                        let pose = amcl_pose
                            .filter(|_| nav_agent.localized)
                            .map(|p| PoseStamped {
                                header: p.0.header.clone(),
                                pose: p.0.pose.pose.clone(),
                            })
                            .unwrap_or_default();
                        let mode = if tracker.is_holding_at_pre_waypoint() {
                            Progress::MODE_ACTION
                        } else {
                            Progress::MODE_IDLE
                        };
                        let sz_id = current_sz.0.as_ref().map(|sz| &sz.id);
                        let progress_msg =
                            build_progress_status(pose, mode, Some(&updated_prog), sz_id);
                        let _ = progress_pub.publisher.publish(progress_msg.clone());
                        latest_progress.0 = Some(progress_msg);
                    }
                }
            }
        }
        GoalStatusCode::Aborted => {
            let matches_current_sz = progress_query
                .get_mut(agent)
                .map(|(_, _, current_sz, _, _, _)| {
                    current_sz
                        .0
                        .as_ref()
                        .map(|sz| sz.id == outcome.target.safe_zone_id)
                        .unwrap_or(true)
                })
                .unwrap_or(true);

            if matches_current_sz {
                if let Ok(err_pub) = error_publishers.get(agent) {
                    let mut plan_err = PlanError::default();
                    plan_err.plan_id = outcome.target.safe_zone_id.plan_id;
                    plan_err.error.code = PlanError::CODE_PATH_BLOCKED;
                    plan_err.error.message =
                        "Nav2 goal aborted: path blocked within safe zone".to_string();
                    let _ = err_pub.publisher.publish(plan_err);
                }
            }
        }
        _ => {}
    }
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
    fn safe_zone_version_and_session_ordering() {
        let base = safe_zone_id(1, 0, 1);
        assert!(!is_safe_zone_newer(&base, &base));
        assert!(is_safe_zone_newer(&safe_zone_id(1, 0, 2), &base));
        assert!(!is_safe_zone_newer(&base, &safe_zone_id(1, 0, 2)));
        assert!(is_safe_zone_newer(&safe_zone_id(1, 1, 0), &base));
        assert!(is_safe_zone_newer(&safe_zone_id(2, 0, 0), &base));
    }

    #[test]
    fn preempts_when_action_upgraded_on_same_safe_zone() {
        let id = safe_zone_id(1, 0, 1);
        let nav = InnerNavigationAction::NavigateToPose;
        let dock = InnerNavigationAction::DockRobot {
            use_dock_id: true,
            dock_id: "dock_north_1".to_string(),
            dock_type: "simple_charging_dock".to_string(),
            navigate_to_staging_pose: false,
        };
        assert!(!should_preempt_goal(&id, &nav, &id, &nav));
        assert!(should_preempt_goal(&id, &dock, &id, &nav));
    }
}
