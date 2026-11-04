use crossbeam::channel::{unbounded as unbounded_crossbeam, Receiver, Sender};
use futures::{
    channel::mpsc::{unbounded as unbounded_mpsc, UnboundedReceiver, UnboundedSender},
    future::{select, Either},
    StreamExt,
};
use log::{debug, error, info, warn};
use rclrs::*;
use ros_env::{
    geometry_msgs::msg::PoseStamped,
    nav2_msgs::action::{
        DockRobot, DockRobot_Goal, NavigateToPose, NavigateToPose_Feedback, NavigateToPose_Goal,
        UndockRobot, UndockRobot_Goal,
    },
    rmf_prototype_msgs::msg::SafeZoneId,
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

#[derive(Clone, Debug)]
pub struct InnerNavigationTarget {
    pub agent: usize,
    pub safe_zone_id: SafeZoneId,
    pub x: f64,
    pub y: f64,
    pub yaw: f64,
    pub action: InnerNavigationAction,
}

impl InnerNavigationTarget {
    pub fn new(agent: usize, safe_zone_id: SafeZoneId, x: f64, y: f64, yaw: f64) -> Self {
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
        agent: usize,
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
    pub agent: usize,
    pub safe_zone_id: SafeZoneId,
    pub target_pose: PoseStamped,
    pub action: InnerNavigationAction,
}

#[derive(Clone, Debug, PartialEq)]
pub enum AgentNavCommand {
    Dispatch(InnerNavigationRequest),
    Cancel,
}

#[derive(Clone, Debug)]
pub enum AgentNavOutcome {
    Feedback {
        agent: usize,
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
        agent: usize,
        request: Option<InnerNavigationRequest>,
        external: bool,
    },
}

#[derive(Clone)]
pub struct InnerNavigationClient {
    pub action_client: Arc<ActionClientState<NavigateToPose>>,
    pub dock_action_client: Arc<ActionClientState<DockRobot>>,
    pub undock_action_client: Arc<ActionClientState<UndockRobot>>,
    pub active_goal: Option<ActiveInnerGoal>,
    pub last_action: Option<InnerNavigationAction>,
    pub command_tx: UnboundedSender<AgentNavCommand>,
    pub outcome_rx: Receiver<AgentNavOutcome>,
}

impl InnerNavigationClient {
    pub fn create_for_agent(
        node: &Arc<NodeState>,
        executor_commands: &Arc<ExecutorCommands>,
        agent_index: usize,
        agent_name: &str,
    ) -> Self {
        let nav_name = format!("{agent_name}/inner/navigate_to_pose");
        let dock_name = format!("{agent_name}/inner/dock_robot");
        let undock_name = format!("{agent_name}/inner/undock_robot");

        let action_client = node
            .create_action_client::<NavigateToPose>(&nav_name)
            .unwrap();
        let dock_action_client = node.create_action_client::<DockRobot>(&dock_name).unwrap();
        let undock_action_client = node
            .create_action_client::<UndockRobot>(&undock_name)
            .unwrap();

        let (command_tx, command_rx) = unbounded_mpsc::<AgentNavCommand>();
        let (outcome_tx, outcome_rx) = unbounded_crossbeam::<AgentNavOutcome>();

        let _ = executor_commands.run(run_agent_navigation_actor(
            agent_index,
            Arc::clone(&action_client),
            Arc::clone(&dock_action_client),
            Arc::clone(&undock_action_client),
            command_rx,
            outcome_tx,
        ));

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

pub fn fold_agent_commands(current: AgentNavCommand, next: AgentNavCommand) -> AgentNavCommand {
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
    nav_action_client: &Arc<ActionClientState<NavigateToPose>>,
    dock_action_client: &Arc<ActionClientState<DockRobot>>,
    undock_action_client: &Arc<ActionClientState<UndockRobot>>,
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
    nav_action_client: &Arc<ActionClientState<NavigateToPose>>,
    dock_action_client: &Arc<ActionClientState<DockRobot>>,
    undock_action_client: &Arc<ActionClientState<UndockRobot>>,
    request: &InnerNavigationRequest,
) -> Option<InnerGoalClient> {
    let mut pose = request.target_pose.clone();
    pose.header.frame_id = "map".to_string();

    match &request.action {
        InnerNavigationAction::NavigateToPose => {
            debug!(
                "[{}] Requesting new NavigateToPose goal to pose [{}, {}]",
                request.agent, pose.pose.position.x, pose.pose.position.y
            );
            let req = nav_action_client.request_goal(NavigateToPose_Goal {
                pose,
                ..Default::default()
            });
            req.await.map(InnerGoalClient::NavigateToPose)
        }
        InnerNavigationAction::DockRobot {
            use_dock_id,
            dock_id,
            dock_type,
            navigate_to_staging_pose,
        } => {
            info!(
                "[{}] Requesting new DockRobot goal (use_dock_id={}, dock_id='{}', dock_type='{}', navigate_to_staging_pose={}) to pose [{}, {}]",
                request.agent,
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
                "[{}] Requesting new UndockRobot goal (dock_type='{}')",
                request.agent, dock_type
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
    agent: usize,
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
                            debug!("[{}] [inner nav2pose] Status: {:?}", agent, s.code);
                        }
                        GoalEvent::Result((status, result)) => {
                            debug!("[{}] [inner nav2pose] Result: {:?}", agent, result);
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
                                "[{}] [inner dock_robot] state={}, retries={}",
                                agent, feedback.state, feedback.num_retries
                            );
                        }
                        GoalEvent::Status(s) => {
                            debug!("[{}] [inner dock_robot] Status: {:?}", agent, s.code);
                        }
                        GoalEvent::Result((status, result)) => {
                            info!(
                                "[{}] [inner dock_robot] Result: status={:?}, success={}, error_code={}, msg='{}'",
                                agent,
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
                            debug!("[{}] [inner undock_robot] Status: {:?}", agent, s.code);
                        }
                        GoalEvent::Result((status, result)) => {
                            info!(
                                "[{}] [inner undock_robot] Result: status={:?}, success={}, error_code={}, msg='{}'",
                                agent,
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

pub async fn run_agent_navigation_actor(
    agent: usize,
    nav_action_client: Arc<ActionClientState<NavigateToPose>>,
    dock_action_client: Arc<ActionClientState<DockRobot>>,
    undock_action_client: Arc<ActionClientState<UndockRobot>>,
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
                            agent
                        );
                    } else {
                        info!(
                            "[{}] Cancellation request finished for inner navigation",
                            agent
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
                                "[{}] Ignoring duplicate or outdated inner navigation request {:?}",
                                agent, request.safe_zone_id
                            );
                        } else {
                            let existing_is_nav =
                                matches!(curr_req.action, InnerNavigationAction::NavigateToPose);
                            let next_is_nav =
                                matches!(request.action, InnerNavigationAction::NavigateToPose);
                            if existing_is_nav && next_is_nav {
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
                                    "[{}] Failed to request new inner navigation goal {:?}",
                                    agent, request.safe_zone_id
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
            agent: 0,
            safe_zone_id: safe_zone_id(1, 0, 1),
            target_pose: PoseStamped::default(),
            action: InnerNavigationAction::NavigateToPose,
        };
        let req2 = InnerNavigationRequest {
            agent: 0,
            safe_zone_id: safe_zone_id(1, 0, 2),
            target_pose: PoseStamped::default(),
            action: InnerNavigationAction::NavigateToPose,
        };

        assert_eq!(
            fold_agent_commands(
                AgentNavCommand::Dispatch(req1.clone()),
                AgentNavCommand::Dispatch(req2.clone())
            ),
            AgentNavCommand::Dispatch(req2.clone())
        );

        assert_eq!(
            fold_agent_commands(
                AgentNavCommand::Dispatch(req2.clone()),
                AgentNavCommand::Dispatch(req1.clone())
            ),
            AgentNavCommand::Dispatch(req2.clone())
        );

        assert_eq!(
            fold_agent_commands(
                AgentNavCommand::Dispatch(req2.clone()),
                AgentNavCommand::Cancel
            ),
            AgentNavCommand::Cancel
        );

        let req2_dock = InnerNavigationRequest {
            agent: 0,
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
