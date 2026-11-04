use crate::{
    execution_node::{AgentPose, SharedSessionCoordinator},
    ros_helpers::{RosPublisher, RosSubscription},
    safe_zone::{is_valid, next_target, CurrentSafeZone},
};
use crossbeam::channel::{unbounded, Receiver, Sender};
use log::{debug, info};
use rclrs::{
    ActionServerState, BeginAcceptedGoal, Executor, NodeState, RequestedGoal, TerminatedGoal,
};
use ros_env::{
    geometry_msgs::msg::PoseStamped,
    nav2_msgs::action::{NavigateToPose, NavigateToPose_Feedback, NavigateToPose_Result},
    rmf_prototype_msgs::msg::{
        DestinationConstraints, DestinationGoal, PlanId, Region, SafeZone, TargetOrientation,
        TargetRegion,
    },
    unique_identifier_msgs::msg::UUID as RosUuid,
};
use std::sync::Arc;
use tokio::sync::mpsc::{error::SendError, unbounded_channel, UnboundedReceiver, UnboundedSender};
use uuid::Uuid;

#[derive(Clone, Debug)]
pub enum NavigationActionUpdate {
    Feedback(NavigateToPose_Feedback),
    Completed,
    Cancelled,
}

#[derive(Clone)]
pub struct NavigationRequest {
    pub agent: usize,
    pub plan_id: PlanId,
    pub target: PoseStamped,
    pub threshold: f64,
}

impl NavigationRequest {
    pub fn new(agent: usize, target: PoseStamped) -> Self {
        Self {
            agent,
            plan_id: PlanId {
                destination_session: new_uuid(),
                plan_version: 0,
            },
            target,
            threshold: 0.5,
        }
    }

    pub fn threshold(&mut self, threshold: f64) -> &mut Self {
        self.threshold = threshold;
        self
    }

    pub fn plan_id(&mut self, plan_id: PlanId) -> &mut Self {
        self.plan_id = plan_id;
        self
    }

    pub fn destination_reached(&self, pose: &AgentPose) -> bool {
        let current_pos = &pose.pose().pose.position;
        let target_pos = &self.target.pose.position;
        dist((current_pos.x, current_pos.y), (target_pos.x, target_pos.y)) < self.threshold
    }
}

pub struct CurrentNavigationRequest {
    pub request: NavigationRequest,
    pub superseded_sessions: Vec<RosUuid>,
    pub feedback_sender: UnboundedSender<NavigationActionUpdate>,
    pub cancel_receiver: UnboundedReceiver<usize>,
}

impl CurrentNavigationRequest {
    pub fn plan_id(&self) -> &PlanId {
        &self.request.plan_id
    }

    pub fn session(&self) -> &RosUuid {
        &self.request.plan_id.destination_session
    }

    pub fn superseded_sessions(&self) -> &[RosUuid] {
        &self.superseded_sessions
    }

    pub fn add_superseded_session(&mut self, session: RosUuid) {
        if &session != self.session() && !self.superseded_sessions.contains(&session) {
            self.superseded_sessions.push(session);
        }
    }

    pub fn is_superseded_session(&self, session: &RosUuid) -> bool {
        self.superseded_sessions.contains(session)
    }

    pub fn clear_superseded_sessions(&mut self) {
        self.superseded_sessions.clear();
    }

    pub fn send_feedback(&self, feedback: NavigateToPose_Feedback) {
        let _ = self
            .feedback_sender
            .send(NavigationActionUpdate::Feedback(feedback));
    }

    pub fn complete(&self) {
        let _ = self.feedback_sender.send(NavigationActionUpdate::Completed);
    }

    pub fn cancel(&self) {
        let _ = self.feedback_sender.send(NavigationActionUpdate::Cancelled);
    }
}

pub struct AgentMockServerContext {
    pub index: usize,
    pub name: String,
    pub _action_server: Arc<ActionServerState<NavigateToPose>>,
    pub request_receiver: Receiver<CurrentNavigationRequest>,
    pub _request_sender: Sender<CurrentNavigationRequest>,
    pub destination_pub: Arc<RosPublisher<DestinationGoal>>,
    pub cancel_session_pub: Arc<RosPublisher<RosUuid>>,
    pub current_nav_request: Option<CurrentNavigationRequest>,
    pub standalone_safe_zone_sub: Option<RosSubscription<SafeZone>>,
    pub standalone_feedback_pose_sub: Option<RosSubscription<PoseStamped>>,
}

impl AgentMockServerContext {
    pub fn new(index: usize, name: String, node: &Arc<NodeState>, standalone: bool) -> Self {
        let destination_pub = Arc::new(RosPublisher::new_transient_local(
            node,
            format!("{name}/destination/goal"),
        ));
        let cancel_session_pub = Arc::new(RosPublisher::new_transient_local(
            node,
            format!("{name}/destination/cancel_session"),
        ));

        let (tx, rx) = unbounded::<CurrentNavigationRequest>();
        let tx_for_closure = tx.clone();
        let publisher_for_closure = Arc::clone(&destination_pub);
        let action_name = format!("{name}/navigate_to_pose");

        let action_server = node
            .create_action_server::<NavigateToPose, _>(&action_name, move |handle| {
                let (feedback_sender, feedback_receiver) =
                    unbounded_channel::<NavigationActionUpdate>();
                let (cancel_sender, cancel_receiver) = unbounded_channel::<usize>();
                let pose = &handle.goal().pose;
                info!("Received a new NavigationRequest to {:?}!", pose);

                let nav_request = NavigationRequest::new(index, pose.clone());
                let session = nav_request.plan_id.destination_session.clone();
                let _ = tx_for_closure.send(CurrentNavigationRequest {
                    request: nav_request,
                    superseded_sessions: Vec::new(),
                    feedback_sender,
                    cancel_receiver,
                });
                let send_cancel = move || cancel_sender.send(index);

                nav_to_pose_action(
                    handle,
                    session,
                    feedback_receiver,
                    send_cancel,
                    publisher_for_closure.clone(),
                )
            })
            .expect("failed to create outer NavigateToPose action server");

        let (standalone_safe_zone_sub, standalone_feedback_pose_sub) = if standalone {
            (
                Some(RosSubscription::new(node, format!("{name}/plan/safe_zone"))),
                Some(RosSubscription::new(
                    node,
                    format!("{name}/inner/feedback_pose"),
                )),
            )
        } else {
            (None, None)
        };

        Self {
            index,
            name,
            _action_server: action_server,
            request_receiver: rx,
            _request_sender: tx,
            destination_pub,
            cancel_session_pub,
            current_nav_request: None,
            standalone_safe_zone_sub,
            standalone_feedback_pose_sub,
        }
    }
}

pub struct Nav2MockServerNode {
    pub node: Arc<NodeState>,
    pub agents: Vec<AgentMockServerContext>,
    pub coordinator: SharedSessionCoordinator,
}

impl Nav2MockServerNode {
    pub fn new(
        executor: &Executor,
        agent_names: Vec<String>,
        coordinator: SharedSessionCoordinator,
    ) -> Self {
        Self::with_mode(executor, agent_names, coordinator, false)
    }

    pub fn new_standalone(
        executor: &Executor,
        agent_names: Vec<String>,
        coordinator: SharedSessionCoordinator,
    ) -> Self {
        Self::with_mode(executor, agent_names, coordinator, true)
    }

    fn with_mode(
        executor: &Executor,
        agent_names: Vec<String>,
        coordinator: SharedSessionCoordinator,
        standalone: bool,
    ) -> Self {
        let executor_commands = executor.commands();
        let node = executor_commands
            .create_node("nav_mock_server_node")
            .expect("failed to create nav_mock_server_node");

        let agents = agent_names
            .into_iter()
            .enumerate()
            .map(|(idx, name)| AgentMockServerContext::new(idx, name, &node, standalone))
            .collect();

        Self {
            node,
            agents,
            coordinator,
        }
    }

    pub fn handle_cancellations(&mut self) {
        for ctx in self.agents.iter_mut() {
            if let Some(ref mut current_nav_request) = ctx.current_nav_request {
                if current_nav_request.cancel_receiver.try_recv().is_ok() {
                    info!(
                        "[{}] Client cancelled NavigationRequest {:?}, cancelling inner navigation",
                        ctx.index,
                        current_nav_request.plan_id()
                    );
                    let session = current_nav_request.session().clone();
                    let superseded = self.coordinator.with_agent(ctx.index, |shared| {
                        let sup: Vec<RosUuid> = shared.superseded_sessions.drain(..).collect();
                        for s in &sup {
                            shared.cancelled_sessions.insert(s.clone());
                        }
                        for s in current_nav_request.superseded_sessions() {
                            shared.cancelled_sessions.insert(s.clone());
                        }
                        shared.cancelled_sessions.insert(session.clone());
                        shared.active_outer_plan_id = None;
                        shared.pending_inner_cancel = true;
                        sup
                    });
                    for s in superseded {
                        let _ = ctx.cancel_session_pub.publish(s);
                    }
                    for s in current_nav_request.superseded_sessions() {
                        let _ = ctx.cancel_session_pub.publish(s.clone());
                    }
                    let _ = ctx.cancel_session_pub.publish(session);
                    ctx.current_nav_request = None;
                }
            }
        }
    }

    pub fn accept_requests(&mut self) {
        for ctx in self.agents.iter_mut() {
            let external_preempted = self.coordinator.with_agent(ctx.index, |shared| {
                if shared.superseded_sessions.is_empty() {
                    if let Some(ref mut req) = ctx.current_nav_request {
                        req.clear_superseded_sessions();
                    }
                }
                std::mem::take(&mut shared.external_preempt_outer)
            });
            if external_preempted {
                if let Some(existing) = ctx.current_nav_request.take() {
                    existing.cancel();
                }
            }

            let mut latest_request: Option<CurrentNavigationRequest> = None;
            let mut queued_superseded_sessions: Vec<RosUuid> = Vec::new();
            while let Ok(request) = ctx.request_receiver.try_recv() {
                if let Some(prev) = latest_request.replace(request) {
                    info!(
                        "[{}] Cancelling queued NavigationRequest {:?} superseded by newer request",
                        prev.request.agent,
                        prev.plan_id()
                    );
                    queued_superseded_sessions.push(prev.session().clone());
                    prev.cancel();
                }
            }

            let Some(mut request) = latest_request else {
                continue;
            };
            for session in queued_superseded_sessions {
                request.add_superseded_session(session);
            }

            let mut should_cancel_inner = false;
            if let Some(existing) = ctx.current_nav_request.take() {
                info!(
                    "[{}] Cancelling active NavigationRequest {:?} superseded by new NavigationRequest {:?}",
                    ctx.index,
                    existing.plan_id(),
                    request.plan_id()
                );
                for session in existing.superseded_sessions() {
                    request.add_superseded_session(session.clone());
                }
                request.add_superseded_session(existing.session().clone());
                existing.cancel();
                should_cancel_inner = true;
            }

            let current_sz_session = self.coordinator.with_agent(ctx.index, |shared| {
                shared
                    .current_safe_zone
                    .0
                    .as_ref()
                    .map(|sz| sz.id.plan_id.destination_session.clone())
            });
            if let Some(sz_session) = current_sz_session {
                if &sz_session != request.session() {
                    request.add_superseded_session(sz_session);
                    should_cancel_inner = true;
                }
            }

            for session in request.superseded_sessions() {
                let _ = ctx.cancel_session_pub.publish(session.clone());
            }
            self.coordinator.with_agent(ctx.index, |shared| {
                for session in request.superseded_sessions() {
                    shared.cancelled_sessions.insert(session.clone());
                }
            });

            if request.cancel_receiver.try_recv().is_ok() {
                let session = request.session().clone();
                let _ = ctx.cancel_session_pub.publish(session.clone());
                self.coordinator.with_agent(ctx.index, |shared| {
                    shared.cancelled_sessions.insert(session);
                    shared.active_outer_plan_id = None;
                    shared.superseded_sessions.clear();
                    shared.pending_inner_cancel = true;
                });
                continue;
            }

            let plan_id = request.plan_id().clone();
            let superseded = request.superseded_sessions().to_vec();
            self.coordinator.with_agent(ctx.index, |shared| {
                shared.active_outer_plan_id = Some(plan_id);
                shared.superseded_sessions = superseded;
                if should_cancel_inner {
                    shared.pending_inner_cancel = true;
                }
            });

            ctx.current_nav_request = Some(request);
        }
    }

    pub fn update_and_cleanup(&mut self) {
        for ctx in self.agents.iter_mut() {
            let feedbacks: Vec<NavigateToPose_Feedback> =
                self.coordinator.with_agent(ctx.index, |shared| {
                    shared.outer_feedback_queue.drain(..).collect()
                });
            if let Some(ref current_nav_req) = ctx.current_nav_request {
                for fb in feedbacks {
                    current_nav_req.send_feedback(fb);
                }
            }

            let mut completed_plan_id: Option<PlanId> = None;
            let mut cancelled_by_client = false;

            if let Some(ref mut current_nav_req) = ctx.current_nav_request {
                let (inner_active, maybe_pose) = self.coordinator.with_agent(ctx.index, |shared| {
                    (shared.inner_active_goal_present, shared.agent_pose.clone())
                });

                if !inner_active {
                    if let Some(ref pose) = maybe_pose {
                        if current_nav_req.request.destination_reached(pose) {
                            info!(
                                "[{}] Destination reached, marking NavigationRequest as completed",
                                ctx.index
                            );
                            completed_plan_id = Some(current_nav_req.plan_id().clone());
                        }
                    }
                }

                if completed_plan_id.is_none() && current_nav_req.cancel_receiver.try_recv().is_ok()
                {
                    cancelled_by_client = true;
                }
            }

            if let Some(plan_id) = completed_plan_id {
                if let Some(req) = ctx.current_nav_request.take() {
                    req.complete();
                }
                self.coordinator.with_agent(ctx.index, |shared| {
                    shared.active_outer_plan_id = None;
                    shared.superseded_sessions.clear();
                    shared.pending_idle_completion = Some(plan_id);
                });
            } else if cancelled_by_client {
                if let Some(req) = ctx.current_nav_request.take() {
                    info!(
                        "[{}] Client cancelled NavigationRequest, cancelling inner navigation",
                        ctx.index
                    );
                    let session = req.session().clone();
                    let superseded = self.coordinator.with_agent(ctx.index, |shared| {
                        let sup: Vec<RosUuid> = shared.superseded_sessions.drain(..).collect();
                        for s in &sup {
                            shared.cancelled_sessions.insert(s.clone());
                        }
                        for s in req.superseded_sessions() {
                            shared.cancelled_sessions.insert(s.clone());
                        }
                        shared.cancelled_sessions.insert(session.clone());
                        shared.active_outer_plan_id = None;
                        shared.pending_inner_cancel = true;
                        sup
                    });
                    for s in superseded {
                        let _ = ctx.cancel_session_pub.publish(s);
                    }
                    for s in req.superseded_sessions() {
                        let _ = ctx.cancel_session_pub.publish(s.clone());
                    }
                    let _ = ctx.cancel_session_pub.publish(session);
                }
            }
        }
    }

    pub fn tick_standalone(&mut self) {
        for ctx in self.agents.iter_mut() {
            if let Some(ref fb_sub) = ctx.standalone_feedback_pose_sub {
                while let Some(pose) = fb_sub.take_data() {
                    self.coordinator.with_agent(ctx.index, |shared| {
                        shared.agent_pose = Some(AgentPose(pose.clone()));
                        shared
                            .outer_feedback_queue
                            .push_back(NavigateToPose_Feedback {
                                current_pose: pose,
                                ..Default::default()
                            });
                    });
                }
            }
            if let Some(ref sz_sub) = ctx.standalone_safe_zone_sub {
                let mut latest_sz = None;
                while let Some(sz) = sz_sub.take_data() {
                    latest_sz = Some(sz);
                }
                if let Some(safe_zone) = latest_sz {
                    if is_valid(&safe_zone) && next_target(&safe_zone).is_some() {
                        let incoming_session = safe_zone.id.plan_id.destination_session.clone();
                        self.coordinator.with_agent(ctx.index, |shared| {
                            if shared.cancelled_sessions.contains(&incoming_session) {
                                return;
                            }
                            if let Some(ref outer_plan_id) = shared.active_outer_plan_id.clone() {
                                if outer_plan_id.destination_session == incoming_session {
                                    shared.superseded_sessions.clear();
                                } else if !shared.superseded_sessions.contains(&incoming_session) {
                                    for s in shared.superseded_sessions.drain(..) {
                                        shared.cancelled_sessions.insert(s);
                                    }
                                    shared
                                        .cancelled_sessions
                                        .insert(outer_plan_id.destination_session.clone());
                                    shared.active_outer_plan_id = None;
                                    shared.external_preempt_outer = true;
                                }
                            }
                            shared.current_safe_zone = CurrentSafeZone(Some(safe_zone));
                        });
                    }
                }
            }
        }
        self.handle_cancellations();
        self.accept_requests();
        self.update_and_cleanup();
    }
}

async fn nav_to_pose_action(
    handle: RequestedGoal<NavigateToPose>,
    session: RosUuid,
    mut receiver: UnboundedReceiver<NavigationActionUpdate>,
    send_cancel: impl Fn() -> Result<(), SendError<usize>>,
    destination_publisher: Arc<RosPublisher<DestinationGoal>>,
) -> TerminatedGoal {
    let goal_order = &handle.goal().pose;
    let destination_goal = pose_stamped_to_destination_goal(goal_order, session);
    let Ok(_) = destination_publisher.publish(destination_goal) else {
        return handle.reject();
    };

    let result = NavigateToPose_Result::default();

    let executing = match handle.accept().begin() {
        BeginAcceptedGoal::Execute(executing) => executing,
        BeginAcceptedGoal::Cancel(cancelling) => {
            let _ = send_cancel();
            return cancelling.cancelled_with(result);
        }
    };

    let cancelling = loop {
        match executing.unless_cancel_requested(receiver.recv()).await {
            Ok(Some(NavigationActionUpdate::Feedback(next))) => {
                executing.publish_feedback(next);
            }
            Ok(Some(NavigationActionUpdate::Completed)) => {
                debug!("NavigationRequest completed!");
                return executing.succeeded_with(result);
            }
            Ok(Some(NavigationActionUpdate::Cancelled)) | Ok(None) => {
                debug!("NavigationRequest preempted/cancelled!");
                return executing.begin_cancelling().cancelled_with(result);
            }
            Err(_) => {
                debug!("NavigationRequest cancelling!");
                let cancelling = executing.begin_cancelling();
                break cancelling;
            }
        }
    };

    debug!("NavigationRequest cancelled!");
    let _ = send_cancel();
    cancelling.cancelled_with(result)
}

pub fn pose_stamped_to_destination_goal(pose: &PoseStamped, session: RosUuid) -> DestinationGoal {
    let x = pose.pose.position.x as f32;
    let y = pose.pose.position.y as f32;
    let q = &pose.pose.orientation;
    let siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    let cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    let yaw = siny_cosp.atan2(cosy_cosp) as f32;

    let region = Region {
        points: vec![x, y],
        hint: Region::HINT_POINT,
    };
    let target_orientation = TargetOrientation {
        orientation_radians: yaw,
        spread_radians: 0.0,
        tolerance_radians: 0.0,
    };
    let target_region = TargetRegion {
        tolerance: 0.5,
        region,
        orientations: vec![target_orientation],
    };

    let constraints = DestinationConstraints {
        regions: vec![target_region],
        nodes: vec![],
    };

    DestinationGoal {
        one_of: vec![constraints],
        cost_bias: vec![],
        session,
    }
}

fn new_uuid() -> RosUuid {
    let random_uuid = Uuid::new_v4();
    let bytes: [u8; 16] = *random_uuid.as_bytes();
    RosUuid { uuid: bytes }
}

fn dist(a: (f64, f64), b: (f64, f64)) -> f64 {
    ((a.0 - b.0).powi(2) + (a.1 - b.1).powi(2)).sqrt()
}
