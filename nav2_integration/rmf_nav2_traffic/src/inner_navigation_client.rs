use crate::{
    agent::AmclPose,
    safe_zone::{
        build_robot_status, CurrentSafeZone, LatestProgress, PlanErrorPublisher, ProgressPublisher,
        ProgressTracker, RobotStatusPublisher,
    },
    Nav2Agent,
};
use bevy::prelude::*;
use bevy_ros2::{RclrsExecutorCommands, RclrsNode, RosActionClient};
use crossflow::{prelude::*, service::Service};
use futures::StreamExt;
use nalgebra::UnitQuaternion;
use rclrs::*;
use ros_env::{
    builtin_interfaces::msg::Time as RosTime,
    geometry_msgs::msg::{Point, Pose, PoseStamped, Quaternion},
    nav2_msgs::action::{
        DockRobot, DockRobot_Goal, NavigateToPose, NavigateToPose_Feedback, NavigateToPose_Goal,
        UndockRobot, UndockRobot_Goal,
    },
    rmf_prototype_msgs::msg::{RobotStatus, SafeZoneId},
    std_msgs::msg::Header,
};
use std::{future::Future, sync::Arc};
use thiserror::Error;

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
    agent: Entity,
    safe_zone_id: SafeZoneId,
    x: f64,
    y: f64,
    yaw: f64,
    action: InnerNavigationAction,
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
            Self::NavigateToPose(c) => c.cancellation.cancel().await.is_accepted(),
            Self::DockRobot(c) => c.cancellation.cancel().await.is_accepted(),
            Self::UndockRobot(c) => c.cancellation.cancel().await.is_accepted(),
        }
    }
}

#[derive(Clone)]
pub struct ActiveInnerGoal {
    pub goal_client: InnerGoalClient,
    pub safe_zone_id: SafeZoneId,
    superseded: bool,
}

impl ActiveInnerGoal {
    pub fn new(goal_client: InnerGoalClient, safe_zone_id: SafeZoneId) -> Self {
        Self {
            goal_client,
            safe_zone_id,
            superseded: false,
        }
    }

    pub fn client(&self) -> &InnerGoalClient {
        &self.goal_client
    }

    pub fn client_mut(&mut self) -> &mut InnerGoalClient {
        &mut self.goal_client
    }

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

#[derive(Component, Clone)]
pub struct InnerNavigationClient {
    pub action_client: Arc<RosActionClient<NavigateToPose>>,
    pub dock_action_client: Arc<RosActionClient<DockRobot>>,
    pub undock_action_client: Arc<RosActionClient<UndockRobot>>,
    // The ongoing action goal (if any)
    pub active_goal: Option<ActiveInnerGoal>,
}

impl InnerNavigationClient {
    pub fn new(
        action_client: Arc<RosActionClient<NavigateToPose>>,
        dock_action_client: Arc<RosActionClient<DockRobot>>,
        undock_action_client: Arc<RosActionClient<UndockRobot>>,
    ) -> Self {
        Self {
            action_client,
            dock_action_client,
            undock_action_client,
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
    let action_name = agent_name.clone() + "/inner/navigate_to_pose";
    let dock_action_name = agent_name.clone() + "/inner/dock_robot";
    let undock_action_name = agent_name + "/inner/undock_robot";
    // Set up clients for the inner Nav2 actions
    let inner_action_client = RosActionClient::<NavigateToPose>::new(&node, action_name);
    let inner_dock_client = RosActionClient::<DockRobot>::new(&node, dock_action_name);
    let inner_undock_client = RosActionClient::<UndockRobot>::new(&node, undock_action_name);

    commands.entity(e).insert(InnerNavigationClient::new(
        Arc::new(inner_action_client),
        Arc::new(inner_dock_client),
        Arc::new(inner_undock_client),
    ));
}

#[derive(Clone)]
struct InnerNavigationRequest {
    agent: Entity,
    safe_zone_id: SafeZoneId,
    target_pose: PoseStamped,
    action: InnerNavigationAction,
}

#[derive(Clone)]
struct CurrentInnerNavigationGoal {
    request: InnerNavigationRequest,
    goal_client: InnerGoalClient,
}

#[derive(Clone)]
struct CancelInnerNavigation {
    agent: Entity,
    new_request: Option<InnerNavigationRequest>,
    cancel_client: InnerGoalClient,
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

/// This continuous service listens for incoming InnerNavigationTarget via
/// events, converts them into PoseStamped, and streams them out as navigation
/// requests to downstream nodes.
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
            action: target.action.clone(),
        };
        info!(
            "[{:?}] Requesting new pending goal to pose [{}, {}] (action={:?})",
            target.agent.index(),
            goal_pose.pose.position.x,
            goal_pose.pose.position.y,
            target.action,
        );

        orders.for_each(|order| order.streams().send(pending_request.clone()));
    }
}

/// This continuous services listens for external navigation cancellation requests
/// and creates corresponding requests to cancel the relevant inner navigation
/// action.
fn await_external_cancellation(
    srv: ContinuousService<(), (), StreamOf<CancelInnerNavigation>>,
    mut orders: ContinuousQuery<(), (), StreamOf<CancelInnerNavigation>>,
    mut cancel_requests: EventReader<CancelInnerForAgent>,
    mut inner_nav_clients: Query<&mut InnerNavigationClient>,
) {
    let Some(mut orders) = orders.get_mut(&srv.key) else {
        return;
    };
    if orders.is_empty() {
        return;
    };

    for request in cancel_requests.read() {
        info!(
            "Received external cancellation request for agent {:?}",
            request.agent.index()
        );
        let Ok(mut inner_client) = inner_nav_clients.get_mut(request.agent) else {
            continue;
        };
        let Some(existing_goal) = inner_client.goal_mut().as_mut() else {
            continue;
        };
        existing_goal.mark_superseded();
        let client = existing_goal.client().clone();
        orders.for_each(|order| {
            order.streams().send(CancelInnerNavigation {
                agent: request.agent,
                new_request: None,
                cancel_client: client.clone(),
            })
        });
    }
}

#[allow(dead_code)]
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
    mut inner_nav_clients: Query<&mut InnerNavigationClient>,
) -> Result<CheckExistingGoalResult, InnerNavigationError> {
    // TODO(@xiyuoh) Create a replan mechanism instead of cancelling goal on every
    // new request
    let mut replan_and_cancel = false;
    let Ok(mut inner_client) = inner_nav_clients.get_mut(request.agent) else {
        return Ok(CheckExistingGoalResult::NoExistingGoal(request));
    };
    let Some(existing_goal) = inner_client.goal_mut().as_mut() else {
        // No existing goal, proceed to request for new goal
        return Ok(CheckExistingGoalResult::NoExistingGoal(request));
    };

    // If plan or safe zone version is later, then replan/cancel
    let curr_safe_zone_id = existing_goal.id();
    let next_safe_zone_id = &request.safe_zone_id;

    if next_safe_zone_id.plan_id.destination_session.uuid
        != curr_safe_zone_id.plan_id.destination_session.uuid
    {
        replan_and_cancel = true;
    } else if next_safe_zone_id.plan_id.plan_version > curr_safe_zone_id.plan_id.plan_version {
        replan_and_cancel = true;
    } else if next_safe_zone_id.plan_id.plan_version == curr_safe_zone_id.plan_id.plan_version
        && next_safe_zone_id.safe_zone_version > curr_safe_zone_id.safe_zone_version
    {
        replan_and_cancel = true;
    }

    if replan_and_cancel {
        existing_goal.mark_superseded();
        let existing_is_nav = matches!(existing_goal.client(), InnerGoalClient::NavigateToPose(_));
        let next_is_nav = matches!(request.action, InnerNavigationAction::NavigateToPose);
        if existing_is_nav && next_is_nav {
            // Do not treat preemption of this goal as a new blockage.
            // Send the new goal directly so Nav2's SimpleActionServer smoothly
            // preempts the active goal in accept_pending_goal() without calling
            // terminate_all() (which would abort a pending new goal if cancel() is
            // still in flight).
            return Ok(CheckExistingGoalResult::NoExistingGoal(request));
        }
        // When switching between different action servers (e.g. NavigateToPose <-> DockRobot),
        // cancel the existing action server's goal before dispatching to the other server.
        let cancel_client = existing_goal.client().clone();
        return Ok(CheckExistingGoalResult::ReplanAndCancel(
            CancelInnerNavigation {
                agent: request.agent,
                new_request: Some(request),
                cancel_client,
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

/// Routes CheckExistingGoalResult members into Result to facilitate workflow
/// branching
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

/// Executes the cancellation of an ongoing inner navigation goal asynchronously
/// Persists until the cancellation is accepted. If there is a pending new request,
/// output the new request to downstream nodes via Ok, otherwise discard via Err.
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
            let accepted = request.cancel_client.is_cancellation_accepted().await;
            if !accepted {
                warn!(
                    "[{}] Cancellation request rejected for inner navigation, it may have already completed.",
                    request.agent.index()
                );
            }
            if let Some(new_request) = request.new_request {
                info!(
                    "[{}] Cancellation logic finished for inner navigation, requesting new goal",
                    request.agent.index()
                );
                // If this a replan attempt with a new navigation request,
                // regardless of whether cancellation was successful, mark as Ok()
                return Ok(new_request);
            } else {
                info!(
                    "[{}] Cancellation request finished for inner navigation",
                    request.agent.index()
                );
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

/// Submits a new NavigateToPose, DockRobot, or UndockRobot goal asynchronously
/// to the corresponding Nav2 action server and returns a handle containing the goal client.
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
    pose.header.frame_id = "map".to_string();

    enum PendingActionGoal {
        NavigateToPose(RequestedGoalClient<NavigateToPose>),
        DockRobot(RequestedGoalClient<DockRobot>),
        UndockRobot(RequestedGoalClient<UndockRobot>),
    }

    let pending_goal = match &request.action {
        InnerNavigationAction::NavigateToPose => {
            debug!(
                "[{:?}] Requesting new NavigateToPose goal to pose [{}, {}]",
                request.agent.index(),
                pose.pose.position.x,
                pose.pose.position.y
            );
            PendingActionGoal::NavigateToPose(
                inner_nav_client
                    .action_client
                    .request_goal(NavigateToPose_Goal { pose, ..default() }),
            )
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
                dock_pose: pose,
                dock_type: dock_type.clone(),
                max_staging_time: 1000.0,
                navigate_to_staging_pose: *navigate_to_staging_pose,
            };
            PendingActionGoal::DockRobot(
                inner_nav_client.dock_action_client.request_goal(dock_goal),
            )
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
            PendingActionGoal::UndockRobot(
                inner_nav_client
                    .undock_action_client
                    .request_goal(undock_goal),
            )
        }
    };

    executor_commands
        .run(async move {
            let goal_client = match pending_goal {
                PendingActionGoal::NavigateToPose(req) => {
                    req.await.map(InnerGoalClient::NavigateToPose)
                }
                PendingActionGoal::DockRobot(req) => req.await.map(InnerGoalClient::DockRobot),
                PendingActionGoal::UndockRobot(req) => req.await.map(InnerGoalClient::UndockRobot),
            };
            match goal_client {
                Some(goal_client) => Ok(CurrentInnerNavigationGoal {
                    request,
                    goal_client,
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

/// Updates the stored goal client with the newly accepted navigation goal.
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

/// Logs any errors that occur during the inner navigation workflow.
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

/// Monitors the feedback of an ongoing navigation goal, and publish it back
/// to the external navigation request. When the goal is completed (success,
/// aborted or cancelled), output the navigation result to downstream nodes for
/// processing.
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
            let agent = handle.request.agent;
            match handle.goal_client.clone() {
                InnerGoalClient::NavigateToPose(client) => {
                    let mut goal_client_stream = client.stream();
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
                }
                InnerGoalClient::DockRobot(client) => {
                    let mut goal_client_stream = client.stream();
                    while let Some(event) = goal_client_stream.next().await {
                        match event {
                            GoalEvent::Feedback(feedback) => {
                                debug!(
                                    "[{:?}] [inner dock_robot] state={}, retries={}",
                                    handle.request.agent.index(),
                                    feedback.state,
                                    feedback.num_retries
                                );
                            }
                            GoalEvent::Status(s) => {
                                debug!(
                                    "[{:?}] [inner dock_robot] Status: {:?}",
                                    handle.request.agent.index(),
                                    s.code
                                );
                            }
                            GoalEvent::Result((status, result)) => {
                                info!(
                                    "[{:?}] [inner dock_robot] Result: status={:?}, success={}, error_code={}, msg='{}'",
                                    handle.request.agent.index(),
                                    status,
                                    result.success,
                                    result.error_code,
                                    result.error_msg
                                );
                                match status {
                                    GoalStatusCode::Succeeded if result.success => {
                                        return Ok(InnerNavigationSuccess { handle })
                                    }
                                    GoalStatusCode::Succeeded | GoalStatusCode::Aborted => {
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
                }
                InnerGoalClient::UndockRobot(client) => {
                    let mut goal_client_stream = client.stream();
                    while let Some(event) = goal_client_stream.next().await {
                        match event {
                            GoalEvent::Feedback(_feedback) => {}
                            GoalEvent::Status(s) => {
                                debug!(
                                    "[{:?}] [inner undock_robot] Status: {:?}",
                                    handle.request.agent.index(),
                                    s.code
                                );
                            }
                            GoalEvent::Result((status, result)) => {
                                info!(
                                    "[{:?}] [inner undock_robot] Result: status={:?}, success={}, error_code={}, msg='{}'",
                                    handle.request.agent.index(),
                                    status,
                                    result.success,
                                    result.error_code,
                                    result.error_msg
                                );
                                match status {
                                    GoalStatusCode::Succeeded if result.success => {
                                        return Ok(InnerNavigationSuccess { handle })
                                    }
                                    GoalStatusCode::Succeeded | GoalStatusCode::Aborted => {
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

/// Evaluates the outcome of a completed or failed navigation goal.
/// If a goal is aborted (and is not stale), it retries the goal.
/// If a goal is cancelled via external request, it marks the cancellation as
/// successful and publish to external navigation workflow.
fn process_navigation_result(
    Blocking {
        request: result, ..
    }: Blocking<InnerNavigationResult>,
    mut cancelling_inner: Query<&mut CancellingInnerNavigation>,
    inner_nav_clients: Query<&InnerNavigationClient>,
    plan_error_publishers: Query<&PlanErrorPublisher>,
    mut progress_query: Query<(
        &mut ProgressTracker,
        &ProgressPublisher,
        &RobotStatusPublisher,
        &mut LatestProgress,
        Option<&CurrentSafeZone>,
        Option<&AmclPose>,
    )>,
) -> Result<InnerNavigationRequest, InnerNavigationResult> {
    match result {
        Ok(ref success) => {
            let target = &success.handle.request;
            let is_current = inner_nav_clients
                .get(target.agent)
                .ok()
                .and_then(|c| c.goal().as_ref())
                .is_some_and(|goal| goal.id() == &target.safe_zone_id && !goal.is_superseded());
            if is_current {
                if let Ok((
                    mut tracker,
                    progress_pub,
                    status_pub,
                    mut latest_progress,
                    current_safe_zone,
                    amcl_pose,
                )) = progress_query.get_mut(target.agent)
                {
                    if tracker.plan_id.as_ref() == Some(&target.safe_zone_id.plan_id) {
                        if matches!(
                            target.action,
                            InnerNavigationAction::DockRobot { .. }
                                | InnerNavigationAction::UndockRobot { .. }
                        ) {
                            tracker.mark_action_completed();
                        }
                        let current_pose = amcl_pose.map(|p| p.xy_yaw());
                        if !tracker.is_target_pose_satisfied(current_pose) {
                            warn!(
                                "[{:?}] Nav2 reported goal succeeded before target pose was satisfied (current={:?}); retrying goal {:?}.",
                                target.agent.index(),
                                current_pose,
                                target.safe_zone_id,
                            );
                            return Ok(target.clone());
                        }
                        if let Some(reached_prog) =
                            tracker.mark_target_reached(latest_progress.0.as_ref())
                        {
                            if latest_progress.0.as_ref() != Some(&reached_prog) {
                                let _ = progress_pub.publisher.publish(reached_prog.clone());
                                latest_progress.0 = Some(reached_prog.clone());
                            }
                            let pose = amcl_pose
                                .map(|p| PoseStamped {
                                    header: p.0.header.clone(),
                                    pose: p.0.pose.pose.clone(),
                                })
                                .unwrap_or_default();
                            let sz_id = current_safe_zone
                                .and_then(|csz| csz.0.as_ref().map(|sz| &sz.id))
                                .or(Some(&target.safe_zone_id));
                            let mode = if tracker.is_holding_at_pre_waypoint() {
                                RobotStatus::MODE_ACTION
                            } else {
                                RobotStatus::MODE_NAVIGATING
                            };
                            let status_msg =
                                build_robot_status(pose, mode, Some(&reached_prog), sz_id);
                            let _ = status_pub.publisher.publish(status_msg);
                        }
                    }
                }
            }
            return Err(result);
        }
        Err(ref err) => match err.kind {
            InnerNavigationErrorKind::GoalAbortedError => {
                let Some(ref handle) = err.handle else {
                    return Err(result);
                };
                let target = &handle.request;
                let active_goal = inner_nav_clients
                    .get(target.agent)
                    .ok()
                    .and_then(|inner_client| inner_client.goal().as_ref());

                if !should_publish_path_blocked(
                    &target.safe_zone_id,
                    active_goal.map(|goal| (goal.id(), goal.is_superseded())),
                ) {
                    info!(
                        "[{:?}] Ignoring stale Nav2 abort {:?}",
                        target.agent.index(),
                        target.safe_zone_id,
                    );
                    return Err(result);
                }

                if let Ok(publisher) = plan_error_publishers.get(target.agent) {
                    let plan_error = ros_env::rmf_prototype_msgs::msg::PlanError {
                        error: ros_env::rmf_prototype_msgs::msg::Error {
                            code: ros_env::rmf_prototype_msgs::msg::PlanError::CODE_PATH_BLOCKED,
                            message: format!(
                                "Nav2 goal aborted for agent {:?}: path blocked within safe zone",
                                target.agent.index()
                            ),
                            parameters: String::new(),
                        },
                        plan_id: handle.request.safe_zone_id.plan_id.clone(),
                    };
                    if let Err(e) = publisher.publisher.publish(plan_error) {
                        error!(
                            "Failed to publish PlanError for agent {:?}: {:?}",
                            target.agent.index(),
                            e
                        );
                    } else {
                        warn!(
                            "[{:?}] Goal aborted by Nav2! Published PlanError CODE_PATH_BLOCKED to ~/plan/error.",
                            target.agent.index()
                        );
                    }
                }

                return Err(result);
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

fn should_publish_path_blocked(
    completed_goal_id: &SafeZoneId,
    active_goal: Option<(&SafeZoneId, bool)>,
) -> bool {
    matches!(
        active_goal,
        Some((active_goal_id, false)) if active_goal_id == completed_goal_id
    )
}

/// Clears existing goal clients for this agent after verifying that the
/// completed goal matches the currently tracked active goal, to avoid
/// accidentally clearing a newly requested goal.
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
            debug!(
                "[{:?}] Ignoring stale Nav2 goal cleanup {:?}",
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
}
