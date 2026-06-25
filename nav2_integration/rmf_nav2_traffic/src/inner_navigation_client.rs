use crate::{Nav2Agent, NavigateToPoseServer, RclrsExecutorCommands, RclrsNode, RosActionClient};
use bevy::prelude::*;
use crossflow::{prelude::*, service::Service};
use futures::StreamExt;
use mapf_post::na::DualQuaternion;
use nalgebra::UnitQuaternion;
use rclrs::*;
use ros_env::{
    builtin_interfaces::msg::Time as RosTime,
    geometry_msgs::msg::{Point, Pose, PoseStamped, Quaternion},
    nav2_msgs::action::{NavigateToPose, NavigateToPose_Feedback, NavigateToPose_Goal},
    rmf_prototype_msgs::msg::SafeZoneId,
    std_msgs::msg::Header,
};
use std::{future::Future, sync::Arc};
use thiserror::Error;

#[derive(Clone, Debug, Event)]
pub struct InnerNavigationTarget {
    agent: Entity,
    safe_zone_id: SafeZoneId,
    x: f64,
    y: f64,
    yaw: f64,
}

impl InnerNavigationTarget {
    pub fn new(agent: Entity, safe_zone_id: SafeZoneId, x: f64, y: f64, yaw: f64) -> Self {
        Self {
            agent,
            safe_zone_id,
            x,
            y,
            yaw,
        }
    }
}

#[derive(Clone)]
pub struct ActiveInnerGoal {
    pub goal_client: GoalClient<NavigateToPose>,
    pub safe_zone_id: SafeZoneId,
}

impl ActiveInnerGoal {
    pub fn new(goal_client: GoalClient<NavigateToPose>, safe_zone_id: SafeZoneId) -> Self {
        Self {
            goal_client,
            safe_zone_id,
        }
    }

    pub fn client(&self) -> &GoalClient<NavigateToPose> {
        &self.goal_client
    }

    pub fn client_mut(&mut self) -> &mut GoalClient<NavigateToPose> {
        &mut self.goal_client
    }

    pub fn id(&self) -> &SafeZoneId {
        &self.safe_zone_id
    }
}

#[derive(Component, Clone)]
pub struct InnerNavigationClient {
    // Stores the action client object
    pub action_client: Arc<RosActionClient<NavigateToPose>>,
    // The ongoing action goal (if any)
    pub active_goal: Option<ActiveInnerGoal>,
}

impl InnerNavigationClient {
    pub fn new(action_client: Arc<RosActionClient<NavigateToPose>>) -> Self {
        Self {
            action_client,
            active_goal: None,
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

#[derive(Default)]
pub struct InnerNavigationClientPlugin {}

impl Plugin for InnerNavigationClientPlugin {
    fn build(&self, app: &mut App) {
        app.add_event::<InnerNavigationTarget>()
            .add_event::<InnerNavigationFeedback>()
            .add_event::<CancelInnerForAgent>()
            .add_observer(create_inner_navigation_client);

        // Initialize Navigation services
        let navigation_services = InnerNavigationServices::from_app(app);
        app.insert_resource(navigation_services);
        let await_and_send_goal = app
            .world()
            .resource::<InnerNavigationServices>()
            .await_and_send_goal
            .clone();

        app.world_mut().command(|commands| {
            let _ = commands.request((), await_and_send_goal).detach();
        });
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
    let action_name = agent_name + "/inner/navigate_to_pose";
    // Set up client for the inner Nav2 action
    let inner_action_client = RosActionClient::<NavigateToPose>::new(&node, action_name);

    commands
        .entity(e)
        .insert(InnerNavigationClient::new(Arc::new(inner_action_client)));
}

#[derive(Clone)]
struct InnerNavigationRequest {
    agent: Entity,
    safe_zone_id: SafeZoneId,
    target_pose: PoseStamped,
}

#[derive(Clone)]
struct CurrentInnerNavigationGoal {
    request: InnerNavigationRequest,
    goal_client: GoalClient<NavigateToPose>,
}

#[derive(Clone)]
struct CancelInnerNavigation {
    agent: Entity,
    new_request: Option<InnerNavigationRequest>,
    cancel_client: GoalClient<NavigateToPose>,
}

#[derive(Clone)]
struct InnerNavigationSuccess {
    pub handle: CurrentInnerNavigationGoal,
}

#[derive(Clone)]
struct InnerNavigationError {
    pub handle: Option<CurrentInnerNavigationGoal>,
    pub kind: InnerNavigationErrorKind,
}

#[derive(Clone, Debug, Error)]
enum InnerNavigationErrorKind {
    #[error("Failed to cancel current goal!")]
    CancelGoalError,
    #[error("Failed to request new goal!")]
    RequestGoalError,
    #[error("Goal was aborted!")]
    GoalAbortedError,
    #[error("Goal was cancelled!")]
    GoalCancelledError,
    #[error("Incoming request has an older plan or safe zone version!")]
    DuplicateOrOutdatedRequestError,
    #[error("Unknown error!")]
    UnknownError,
}

#[derive(Clone, Debug, Event)]
pub struct CancelInnerForAgent {
    pub agent: Entity,
}

type InnerNavigationResult = Result<InnerNavigationSuccess, InnerNavigationError>;

#[derive(Resource)]
pub struct InnerNavigationServices {
    await_and_send_goal: Service<(), (), ()>,
}

impl InnerNavigationServices {
    pub fn from_app(app: &mut App) -> Self {
        let await_new_requests_service = app.spawn_continuous_service(Update, await_new_requests);
        let await_external_cancellation_service =
            app.spawn_continuous_service(Update, await_external_cancellation);
        let check_existing_goal_service = app.spawn_service(check_existing_goal);
        let handle_existing_goal_result_service = app.spawn_service(handle_existing_goal_result);
        let async_cancel_goal_service = app.spawn_service(async_cancel_goal);
        let async_request_new_goal_service = app.spawn_service(async_request_new_goal);
        let update_goal_client_service = app.spawn_service(update_goal_client);
        let async_monitor_ongoing_navigation_service =
            app.spawn_service(async_monitor_ongoing_navigation);
        let process_navigation_result_service = app.spawn_service(process_navigation_result);
        let cleanup_goal_client_service = app.spawn_service(cleanup_goal_client);
        let log_error_service = app.spawn_service(log_inner_navigation_error);

        let await_and_send_goal = app.world_mut().spawn_workflow(|scope, builder| {
            let fork_input = scope.start.fork_clone(builder);
            let await_requests = fork_input
                .clone_chain(builder)
                .then_node(await_new_requests_service);
            let await_external_cancellation = fork_input
                .clone_chain(builder)
                .then_node(await_external_cancellation_service);

            // Create nodes
            let check_existing_goal = builder.create_node(check_existing_goal_service);
            let handle_existing_goal_result =
                builder.create_node(handle_existing_goal_result_service);
            let async_cancel_goal = builder.create_node(async_cancel_goal_service);
            let async_request_new_goal = builder.create_node(async_request_new_goal_service);
            let update_goal_client = builder.create_node(update_goal_client_service);
            let async_monitor_new_navigation_request =
                builder.create_node(async_monitor_ongoing_navigation_service);
            let post_nav_processing = builder.create_node(process_navigation_result_service);
            let cleanup_goal_client = builder.create_node(cleanup_goal_client_service);
            let log_error = builder.create_node(log_error_service);

            // Stream out new requests to downstream nodes
            builder.connect(await_requests.streams, check_existing_goal.input);

            // Stream out external cancellation requests to downstream nodes
            builder.connect(await_external_cancellation.streams, async_cancel_goal.input);

            // Check if there is an existing goal client, and compare whether the new
            // request is duplicate or outdated. If so, discard (err branch).
            let (check_existing_fork_result_input, check_existing_fork_result) =
                builder.create_fork_result();
            builder.connect(check_existing_goal.output, check_existing_fork_result_input);
            builder.connect(
                check_existing_fork_result.ok,
                handle_existing_goal_result.input,
            );

            // After discarding invalid new requests, check if there is an existing
            // goal client; if so, connect to cancellation node, else request new goal
            let (handle_existing_fork_result_input, handle_existing_fork_result) =
                builder.create_fork_result();
            builder.connect(
                handle_existing_goal_result.output,
                handle_existing_fork_result_input,
            );
            builder.connect(handle_existing_fork_result.ok, async_cancel_goal.input);
            builder.connect(
                handle_existing_fork_result.err,
                async_request_new_goal.input,
            );

            // Handles incoming inner/navigate_to_pose requests - cancels ongoing
            // action goals and sends new action goal.
            let (cancel_goal_fork_result_input, cancel_goal_fork_result) =
                builder.create_fork_result();
            builder.connect(async_cancel_goal.output, cancel_goal_fork_result_input);

            builder.connect(cancel_goal_fork_result.ok, async_request_new_goal.input);
            let (new_goal_fork_result_input, new_goal_fork_result) = builder.create_fork_result();
            builder.connect(async_request_new_goal.output, new_goal_fork_result_input);

            // Handles and monitors ongoing inner/navigate_to_pose request
            builder.connect(new_goal_fork_result.ok, update_goal_client.input);
            builder.connect(
                update_goal_client.output,
                async_monitor_new_navigation_request.input,
            );

            // On completed navigation request, check goal status code and process accordingly
            // Aborted -> retry, Cancelled -> publish cancellation to outer workflow, Succeeded -> do nothing
            // Then connect to cleanup existing goal client data
            builder.connect(
                async_monitor_new_navigation_request.output,
                post_nav_processing.input,
            );
            let (post_nav_fork_result_input, post_nav_fork_result) = builder.create_fork_result();
            builder.connect(post_nav_processing.output, post_nav_fork_result_input);
            // If goal was aborted, retry navigation by requesting new goal
            // Goal client will be updated in a downstream node
            builder.connect(post_nav_fork_result.ok, async_request_new_goal.input);
            // Otherwise, cleanup goal client
            builder.connect(post_nav_fork_result.err, cleanup_goal_client.input);

            // Connect errors to logging node
            builder.connect(cancel_goal_fork_result.err, log_error.input);
            builder.connect(new_goal_fork_result.err, log_error.input);

            // Connect only await_requests output to terminate
            builder.connect(await_requests.output, scope.terminate);
        });

        Self {
            await_and_send_goal,
        }
    }
}

fn await_new_requests(
    srv: ContinuousService<(), (), StreamOf<InnerNavigationRequest>>,
    mut orders: ContinuousQuery<(), (), StreamOf<InnerNavigationRequest>>,
    mut nav_target: EventReader<InnerNavigationTarget>,
    node: Res<RclrsNode>,
) {
    let Some(mut orders) = orders.get_mut(&srv.key) else {
        return;
    };
    if orders.is_empty() {
        return;
    }
    if nav_target.is_empty() {
        return;
    }

    for target in nav_target.read() {
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
        let pending_request = InnerNavigationRequest {
            agent: target.agent,
            safe_zone_id: target.safe_zone_id.clone(),
            target_pose: goal_pose.clone(),
        };
        info!(
            "[{:?}] Requesting new pending goal to pose [{}, {}]",
            target.agent.index(),
            goal_pose.pose.position.x,
            goal_pose.pose.position.y
        );

        orders.for_each(|order| order.streams().send(pending_request.clone()));
    }
}

fn await_external_cancellation(
    srv: ContinuousService<(), (), StreamOf<CancelInnerNavigation>>,
    mut orders: ContinuousQuery<(), (), StreamOf<CancelInnerNavigation>>,
    mut cancel_requests: EventReader<CancelInnerForAgent>,
    inner_nav_clients: Query<&InnerNavigationClient>,
) {
    let Some(mut orders) = orders.get_mut(&srv.key) else {
        return;
    };
    if orders.is_empty() {
        return;
    }

    for request in cancel_requests.read() {
        info!(
            "Received external cancellation request for agent {:?}",
            request.agent.index()
        );
        let Some(existing_goal) = inner_nav_clients
            .get(request.agent)
            .ok()
            .and_then(|inner_client| inner_client.goal().as_ref())
        else {
            continue;
        };
        let client = existing_goal.client();
        orders.for_each(|order| {
            order.streams().send(CancelInnerNavigation {
                agent: request.agent,
                new_request: None,
                cancel_client: client.clone(),
            })
        });
    }
}

enum CheckExistingGoalResult {
    ReplanAndCancel(CancelInnerNavigation),
    NoExistingGoal(InnerNavigationRequest),
}

/// Checks if there is an existing goal for the requested agent.
/// If yes, check if the existing goal is outdated, i.e. it has a lower plan or
/// safe zone version than the new request. If so, cancel the existing goal and
/// request a new goal. If the new request is duplicate or outdated, return
/// Err(None) to indicate that no new goal should be requested.
/// If there isn't an existing goal, return Err(Some(request)) so that the
/// workflow can proceed to request a new goal without cancellation.
fn check_existing_goal(
    Blocking { request, .. }: Blocking<InnerNavigationRequest>,
    inner_nav_clients: Query<&InnerNavigationClient>,
) -> Result<CheckExistingGoalResult, InnerNavigationError> {
    // TODO(@xiyuoh) Create a replan mechanism instead of cancelling goal on every
    // new request
    let mut replan_and_cancel = false;
    let Some(existing_goal) = inner_nav_clients
        .get(request.agent)
        .ok()
        .and_then(|inner_client| inner_client.goal().as_ref())
    else {
        // No existing goal, proceed to request for new goal
        return Ok(CheckExistingGoalResult::NoExistingGoal(request));
    };

    // If plan or safe zone version is later, then replan/cancel
    let curr_safe_zone_id = existing_goal.id();
    let next_safe_zone_id = &request.safe_zone_id;

    if next_safe_zone_id.plan_id.plan_version > curr_safe_zone_id.plan_id.plan_version {
        replan_and_cancel = true;
    } else if next_safe_zone_id.plan_id.plan_version == curr_safe_zone_id.plan_id.plan_version
        && next_safe_zone_id.safe_zone_version > curr_safe_zone_id.safe_zone_version
    {
        replan_and_cancel = true;
    }

    let client = existing_goal.client();

    if replan_and_cancel {
        return Ok(CheckExistingGoalResult::ReplanAndCancel(
            CancelInnerNavigation {
                agent: request.agent,
                new_request: Some(request),
                cancel_client: client.clone(),
            },
        ));
    }

    // There is an existing goal but the new request is duplicate or outdated,
    // so do not request a new goal
    return Err(InnerNavigationError {
        handle: None,
        kind: InnerNavigationErrorKind::DuplicateOrOutdatedRequestError,
    });
}

fn handle_existing_goal_result(
    Blocking {
        request: result, ..
    }: Blocking<CheckExistingGoalResult>,
) -> Result<CancelInnerNavigation, InnerNavigationRequest> {
    match result {
        CheckExistingGoalResult::ReplanAndCancel(cancel_request) => Ok(cancel_request),
        CheckExistingGoalResult::NoExistingGoal(new_request) => Err(new_request),
    }
}

#[derive(Clone, Debug, Component)]
pub struct CancellingInnerNavigation {
    pub success: bool,
}

fn async_cancel_goal(
    Async { request, .. }: Async<CancelInnerNavigation>,
    mut commands: Commands,
    executor_commands: Res<RclrsExecutorCommands>,
) -> impl Future<Output = Result<InnerNavigationRequest, InnerNavigationError>> {
    if request.new_request.is_none() {
        commands
            .entity(request.agent)
            .insert(CancellingInnerNavigation { success: false });
    }
    executor_commands
        .run(async move {
            let mut cancellation = request.cancel_client.cancellation.cancel().await;
            // If this is an external cancellation attempt, persist until it is accepted
            while !cancellation.is_accepted() {
                info!(
                    "[{}] Cancellation request rejected for inner navigation, retrying...",
                    request.agent.index()
                );
                cancellation = request.cancel_client.cancellation.cancel().await;
            }
            info!(
                "[{}] Cancellation request accepted for inner navigation, requesting new goal",
                request.agent.index()
            );
            if let Some(new_request) = request.new_request {
                // If this a replan attempt with a new navigation request,
                // regardless of whether cancellation was successful, mark as Ok()
                return Ok(new_request);
            } else {
                return Err(InnerNavigationError {
                    handle: None,
                    kind: InnerNavigationErrorKind::CancelGoalError,
                });
            }
        })
        .then(|res| async move {
            res.unwrap_or(Err(InnerNavigationError {
                handle: None,
                kind: InnerNavigationErrorKind::CancelGoalError,
            }))
        })
}

fn async_request_new_goal(
    Async { request, .. }: Async<InnerNavigationRequest>,
    mut inner_nav_clients: Query<&mut InnerNavigationClient>,
    executor_commands: Res<RclrsExecutorCommands>,
) -> impl Future<Output = Result<CurrentInnerNavigationGoal, InnerNavigationError>> {
    let inner_nav_client_result = inner_nav_clients.get_mut(request.agent);
    if inner_nav_client_result.is_err() {
        return std::future::ready(Err(InnerNavigationError {
            handle: None,
            kind: InnerNavigationErrorKind::RequestGoalError,
        }))
        .left_future();
    }
    let mut inner_nav_client = inner_nav_client_result.unwrap();

    // Reset goal client before submitting new request
    inner_nav_client.reset_goal();

    let mut pose = request.target_pose.clone();
    debug!(
        "[{:?}] Requesting new navigation goal to pose [{}, {}]",
        request.agent.index(),
        pose.pose.position.x,
        pose.pose.position.y
    );
    pose.header.frame_id = "map".to_string();
    let nav_request = inner_nav_client
        .action_client
        .request_goal(NavigateToPose_Goal { pose, ..default() });

    executor_commands
        .run(async move {
            match nav_request.await {
                Some(handle) => Ok(CurrentInnerNavigationGoal {
                    request,
                    goal_client: handle,
                }),
                None => Err(InnerNavigationError {
                    handle: None,
                    kind: InnerNavigationErrorKind::RequestGoalError,
                }),
            }
        })
        .then(|res| async move {
            res.unwrap_or(Err(InnerNavigationError {
                handle: None,
                kind: InnerNavigationErrorKind::RequestGoalError,
            }))
        })
        .right_future()
}

fn update_goal_client(
    Blocking {
        request: handle, ..
    }: Blocking<CurrentInnerNavigationGoal>,
    mut inner_nav_clients: Query<&mut InnerNavigationClient>,
) -> CurrentInnerNavigationGoal {
    if let Ok(mut inner_nav_client) = inner_nav_clients.get_mut(handle.request.agent) {
        *inner_nav_client.goal_mut() = Some(ActiveInnerGoal::new(
            handle.goal_client.clone(),
            handle.request.safe_zone_id.clone(),
        ));
    } else {
        warn!(
            "[{:?}] InnerNavigationClient not found!",
            handle.request.agent.index()
        );
    }
    handle
}

fn log_inner_navigation_error(Blocking { request: err, .. }: Blocking<InnerNavigationError>) {
    error!("InnerNavigationError occurred: {}", err.kind);
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

fn async_monitor_ongoing_navigation(
    Async {
        request: handle,
        channel,
        ..
    }: Async<CurrentInnerNavigationGoal>,
    executor_commands: Res<RclrsExecutorCommands>,
) -> impl Future<Output = InnerNavigationResult> {
    let nav_handle = handle.clone();
    executor_commands
        .run(async move {
            let mut goal_client_stream = handle.goal_client.clone().stream();
            let agent = handle.request.agent.clone();
            // TODO(@xiyuoh) this gets stuck sometimes, find out why
            while let Some(event) = goal_client_stream.next().await {
                match event {
                    GoalEvent::Feedback(feedback) => {
                        let safe_zone_id = handle.request.safe_zone_id.clone();
                        // Publish feedback via observer triggers
                        channel.commands(move |cmds| {
                            cmds.trigger(InnerNavigationFeedback::new(
                                agent,
                                safe_zone_id.clone(),
                                feedback.clone(),
                            ));
                        });
                    }
                    GoalEvent::Status(s) => {
                        debug!(
                            "[{:?}] [inner nav2pose] Status: {:?}",
                            handle.request.agent.index(),
                            s.code
                        );
                    }
                    GoalEvent::Result((status, result)) => {
                        debug!(
                            "[{:?}] [inner nav2pose] Result: {:?}",
                            handle.request.agent.index(),
                            result
                        );
                        match status {
                            GoalStatusCode::Succeeded => {
                                return Ok(InnerNavigationSuccess { handle })
                            }
                            GoalStatusCode::Aborted => {
                                return Err(InnerNavigationError {
                                    handle: Some(handle.clone()),
                                    kind: InnerNavigationErrorKind::GoalAbortedError,
                                })
                            }
                            GoalStatusCode::Cancelled => {
                                return Err(InnerNavigationError {
                                    handle: Some(handle.clone()),
                                    kind: InnerNavigationErrorKind::GoalCancelledError,
                                });
                            }
                            _ => {}
                        }
                    }
                }
            }
            Err(InnerNavigationError {
                handle: Some(handle.clone()),
                kind: InnerNavigationErrorKind::UnknownError,
            })
        })
        .then(|res| async move {
            res.unwrap_or(Err(InnerNavigationError {
                handle: Some(nav_handle.clone()),
                kind: InnerNavigationErrorKind::UnknownError,
            }))
        })
}

// If navigation was aborted, retry
// If navigation was cancelled (not cancelling), publish cancellation to outer workflow
fn process_navigation_result(
    Blocking {
        request: result, ..
    }: Blocking<InnerNavigationResult>,
    mut commands: Commands,
    mut cancelling_inner: Query<&mut CancellingInnerNavigation>,
    inner_nav_clients: Query<&InnerNavigationClient>,
) -> Result<InnerNavigationRequest, InnerNavigationResult> {
    match result {
        Ok(_) => return Err(result),
        Err(ref err) => match err.kind {
            InnerNavigationErrorKind::GoalAbortedError => {
                let Some(ref handle) = err.handle else {
                    return Err(result);
                };
                let target = &handle.request;
                let target_pose = target.target_pose.clone();

                if let Ok(inner_nav_client) = inner_nav_clients.get(target.agent) {
                    if let Some(active_goal) = inner_nav_client.goal() {
                        if active_goal.id() != &target.safe_zone_id {
                            debug!(
                                "[{:?}] Aborted goal is stale, not retrying",
                                target.agent.index()
                            );
                            return Err(result);
                        }
                    }
                }

                debug!("[{:?}] Goal aborted. Retrying", target.agent.index());
                return Ok(InnerNavigationRequest {
                    agent: target.agent,
                    safe_zone_id: target.safe_zone_id.clone(),
                    target_pose,
                });
            }
            InnerNavigationErrorKind::GoalCancelledError => {
                // Only mark cancellation success for external cancellation
                // and not for replan attempts
                if let Some(agent) = err.handle.as_ref().map(|h| h.request.agent) {
                    if let Ok(mut cancelling) = cancelling_inner.get_mut(agent) {
                        info!(
                            "[{:?}] Goal cancelled. Marking cancellation as success.",
                            agent.index()
                        );
                        cancelling.success = true;
                    }
                }
            }
            _ => {}
        },
    }
    return Err(result);
}

fn cleanup_goal_client(
    Blocking {
        request: result, ..
    }: Blocking<InnerNavigationResult>,
    mut inner_nav_clients: Query<&mut InnerNavigationClient>,
) {
    let handle = match result {
        Ok(res) => res.handle,
        Err(err) => {
            let Some(req) = err.handle else {
                return;
            };
            req
        }
    };

    // Clear inner goal client
    if let Ok(mut inner_nav_client) = inner_nav_clients.get_mut(handle.request.agent) {
        let is_current = if let Some(active_goal) = inner_nav_client.goal() {
            active_goal.id() == &handle.request.safe_zone_id
        } else {
            false
        };
        if is_current {
            inner_nav_client.reset_goal();
        } else {
            warn!(
                "[{:?}] Found an incompatible SafeZoneId {:?} while attempting
                to cleanup goal client!",
                handle.request.agent.index(),
                handle.request.safe_zone_id,
            );
        }
    } else {
        warn!(
            "[{:?}] InnerNavigationClient not found!",
            handle.request.agent.index()
        );
    }
}
