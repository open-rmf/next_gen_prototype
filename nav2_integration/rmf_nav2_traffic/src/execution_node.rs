use crate::{
    inner_navigation_client::{
        should_preempt_goal, should_publish_path_blocked, ActiveInnerGoal, AgentNavCommand,
        AgentNavOutcome, InnerNavigationAction, InnerNavigationClient, InnerNavigationRequest,
        InnerNavigationTarget,
    },
    ros_helpers::{RosPublisher, RosSubscription},
    safe_zone::{
        build_progress_status, costmap_content_eq, is_valid, next_target, seq_to_stamp,
        stamp_to_seq, CostmapHandshakeState, CurrentPlan, CurrentSafeZone, LatestProgress,
        PendingNavigationTarget, ProgressTracker,
    },
};
use log::{debug, error, info, warn};
use nalgebra::UnitQuaternion;
use rclrs::{Executor, ExecutorCommands, NodeState};
use ros_env::{
    builtin_interfaces::msg::Time as RosTime,
    geometry_msgs::msg::{
        Point, Pose, PoseStamped, PoseWithCovarianceStamped, Quaternion, TwistWithCovariance,
    },
    nav2_msgs::{action::NavigateToPose_Feedback, msg::Costmap},
    nav_msgs::msg::Odometry,
    rmf_prototype_msgs::msg::{
        Error as RmfError, Participant, ParticipantList, Plan, PlanError, PlanId, Progress,
        SafeZone, SafeZoneId,
    },
    std_msgs::msg::{Empty as RosEmpty, Header},
    unique_identifier_msgs::msg::UUID as RosUuid,
};
use std::{
    collections::VecDeque,
    sync::{Arc, Mutex},
    time::{Duration, Instant},
};

#[derive(Clone, Debug)]
pub struct Nav2Agent {
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
            id,
            name,
            localized: false,
            last_safe_zone_id: None,
        }
    }
}

#[derive(Clone, Debug, Default)]
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

#[derive(Clone, Debug, Default)]
pub struct AgentPose(pub PoseStamped);

impl AgentPose {
    pub fn pose(&self) -> &PoseStamped {
        &self.0
    }

    pub fn pose_mut(&mut self) -> &mut PoseStamped {
        &mut self.0
    }

    pub fn xy_yaw(&self) -> (f32, f32, f32) {
        let pos = &self.0.pose.position;
        let q = &self.0.pose.orientation;
        let siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
        let cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
        let yaw = siny_cosp.atan2(cosy_cosp) as f32;
        (pos.x as f32, pos.y as f32, yaw)
    }
}

#[derive(Clone, Debug, Default)]
pub struct CancelledNavigationSessions {
    sessions: Vec<RosUuid>,
}

impl CancelledNavigationSessions {
    pub fn insert(&mut self, session: RosUuid) {
        if !self.sessions.contains(&session) {
            self.sessions.push(session);
        }
    }

    pub fn contains(&self, session: &RosUuid) -> bool {
        self.sessions.contains(session)
    }
}

#[derive(Debug, Default)]
pub struct SharedAgentState {
    pub cancelled_sessions: CancelledNavigationSessions,
    pub active_outer_plan_id: Option<PlanId>,
    pub superseded_sessions: Vec<RosUuid>,
    pub external_preempt_outer: bool,
    pub pending_inner_cancel: bool,
    pub inner_active_goal_present: bool,
    pub current_safe_zone: CurrentSafeZone,
    pub latest_safe_zone_received: Option<SafeZone>,
    pub agent_pose: Option<AgentPose>,
    pub outer_feedback_queue: VecDeque<NavigateToPose_Feedback>,
    pub pending_idle_completion: Option<PlanId>,
}

#[derive(Clone, Default)]
pub struct SharedSessionCoordinator {
    agents: Arc<Mutex<Vec<SharedAgentState>>>,
}

impl SharedSessionCoordinator {
    pub fn new(num_agents: usize) -> Self {
        let mut agents = Vec::with_capacity(num_agents);
        for _ in 0..num_agents {
            agents.push(SharedAgentState::default());
        }
        Self {
            agents: Arc::new(Mutex::new(agents)),
        }
    }

    pub fn with_agent<R>(&self, index: usize, f: impl FnOnce(&mut SharedAgentState) -> R) -> R {
        let mut guard = self.agents.lock().unwrap();
        f(&mut guard[index])
    }
}

pub struct AgentExecutionContext {
    pub index: usize,
    pub agent: Nav2Agent,
    pub amcl_pose: AmclPose,
    pub amcl_pose_sub: RosSubscription<PoseWithCovarianceStamped>,
    pub odom_pub: RosPublisher<Odometry>,
    pub safe_zone_sub: RosSubscription<SafeZone>,
    pub plan_sub: RosSubscription<Plan>,
    pub gate_release_sub: RosSubscription<RosEmpty>,
    pub cancel_session_sub: RosSubscription<RosUuid>,
    pub costmap_pub: RosPublisher<Costmap>,
    pub costmap_updated_sub: RosSubscription<Header>,
    pub handshake: CostmapHandshakeState,
    pub progress_pub: RosPublisher<Progress>,
    pub latest_progress: LatestProgress,
    pub progress_tracker: ProgressTracker,
    pub plan_error_pub: RosPublisher<PlanError>,
    pub feedback_pose_pub: RosPublisher<PoseStamped>,
    pub current_safe_zone: CurrentSafeZone,
    pub current_plan: CurrentPlan,
    pub inner_client: InnerNavigationClient,
    pub latest_fb_pose: Option<PoseStamped>,
    pub cancelling_inner: bool,
    pub pending_targets: Vec<InnerNavigationTarget>,
    pub pending_cancel: bool,
}

impl AgentExecutionContext {
    pub fn new(
        index: usize,
        name: String,
        node: &Arc<NodeState>,
        executor_commands: &Arc<ExecutorCommands>,
    ) -> Self {
        let amcl_pose_sub = RosSubscription::new(node, format!("{name}/inner/amcl_pose"));
        let odom_pub = RosPublisher::new(node, format!("{name}/odom"));
        let safe_zone_sub = RosSubscription::new(node, format!("{name}/plan/safe_zone"));
        let plan_sub = RosSubscription::new(node, format!("{name}/plan"));
        let gate_release_sub = RosSubscription::new(node, format!("{name}/plan/gate_release"));
        let cancel_session_sub =
            RosSubscription::new(node, format!("{name}/destination/cancel_session"));
        let costmap_pub = RosPublisher::new_transient_local(
            node,
            format!("{name}/inner/global_costmap/plan/costmap"),
        );
        let costmap_updated_sub = RosSubscription::new(
            node,
            format!("{name}/inner/global_costmap/plan/costmap_updated"),
        );
        let progress_pub = RosPublisher::new_transient_local(node, format!("{name}/plan/progress"));
        let plan_error_pub = RosPublisher::new_transient_local(node, format!("{name}/plan/error"));
        let feedback_pose_pub = RosPublisher::new(node, format!("{name}/inner/feedback_pose"));

        let inner_client =
            InnerNavigationClient::create_for_agent(node, executor_commands, index, &name);

        Self {
            index,
            agent: Nav2Agent::new(name),
            amcl_pose: AmclPose::default(),
            amcl_pose_sub,
            odom_pub,
            safe_zone_sub,
            plan_sub,
            gate_release_sub,
            cancel_session_sub,
            costmap_pub,
            costmap_updated_sub,
            handshake: CostmapHandshakeState::default(),
            progress_pub,
            latest_progress: LatestProgress::default(),
            progress_tracker: ProgressTracker::default(),
            plan_error_pub,
            feedback_pose_pub,
            current_safe_zone: CurrentSafeZone::default(),
            current_plan: CurrentPlan::default(),
            inner_client,
            latest_fb_pose: None,
            cancelling_inner: false,
            pending_targets: Vec::new(),
            pending_cancel: false,
        }
    }
}

pub struct Nav2ExecutionNode {
    pub node: Arc<NodeState>,
    pub discovery_pub: RosPublisher<ParticipantList>,
    pub discovery_published: bool,
    pub agents: Vec<AgentExecutionContext>,
    pub coordinator: SharedSessionCoordinator,
}

impl Nav2ExecutionNode {
    pub fn new(
        executor: &Executor,
        agent_names: Vec<String>,
        coordinator: SharedSessionCoordinator,
    ) -> Self {
        let executor_commands = Arc::clone(executor.commands());
        let node = executor_commands
            .create_node("nav_execution_node")
            .expect("failed to create nav_execution_node");
        let discovery_pub =
            RosPublisher::new_transient_local(&node, "/destination/discovery".to_string());

        let agents = agent_names
            .into_iter()
            .enumerate()
            .map(|(idx, name)| AgentExecutionContext::new(idx, name, &node, &executor_commands))
            .collect();

        Self {
            node,
            discovery_pub,
            discovery_published: false,
            agents,
            coordinator,
        }
    }

    pub fn step_pre_requests(&mut self) {
        self.publish_discovery();
        self.update_amcl_poses();
        self.update_incremental_targets();
    }

    pub fn step_dispatch_and_outcomes(&mut self) {
        for ctx in self.agents.iter_mut() {
            let should_cancel = self.coordinator.with_agent(ctx.index, |shared| {
                std::mem::take(&mut shared.pending_inner_cancel)
            });
            if should_cancel {
                ctx.pending_cancel = true;
            }
        }
        self.dispatch_inner_navigation_commands();
        self.process_inner_navigation_outcomes();
    }

    pub fn tick(&mut self) {
        self.step_pre_requests();
        self.step_dispatch_and_outcomes();
        self.flush_post_tick();
    }

    pub fn flush_post_tick(&mut self) {
        for ctx in self.agents.iter_mut() {
            let (should_cancel, idle_completion) =
                self.coordinator.with_agent(ctx.index, |shared| {
                    let cancel = std::mem::take(&mut shared.pending_inner_cancel);
                    let completed = shared.pending_idle_completion.take();
                    (cancel, completed)
                });
            if should_cancel {
                ctx.pending_cancel = true;
            }
            if let Some(_plan_id) = idle_completion {
                if let Some(safe_zone) = ctx
                    .safe_zone_sub
                    .data_callback()
                    .or_else(|| ctx.current_safe_zone.0.clone())
                {
                    let target_wp = safe_zone.target_waypoint.first().copied().unwrap_or(0);
                    let agent_pose = self
                        .coordinator
                        .with_agent(ctx.index, |shared| shared.agent_pose.clone());
                    let pose = if ctx.agent.localized {
                        PoseStamped {
                            header: ctx.amcl_pose.0.header.clone(),
                            pose: ctx.amcl_pose.0.pose.pose.clone(),
                        }
                    } else {
                        agent_pose.map(|p| p.0).unwrap_or_default()
                    };
                    let base_progress = Progress {
                        progress: safe_zone.target_progress,
                        reached_waypoint: target_wp,
                        target_waypoint: target_wp,
                        reached_keys: vec![],
                        plan_id: safe_zone.id.plan_id.clone(),
                        ..Default::default()
                    };
                    let final_progress = build_progress_status(
                        pose,
                        Progress::MODE_IDLE,
                        Some(&base_progress),
                        Some(&safe_zone.id),
                    );
                    let _ = ctx.progress_pub.publish(final_progress.clone());
                    ctx.latest_progress.0 = Some(final_progress);
                }
                ctx.cancelling_inner = false;
            }
        }
        self.dispatch_inner_navigation_commands();
    }

    fn publish_discovery(&mut self) {
        if self.discovery_published {
            return;
        }
        let mut msg = ParticipantList::default();
        for ctx in &self.agents {
            msg.participants.push(Participant {
                name: ctx.agent.name.clone(),
                components: vec![],
            });
        }
        let _ = self.discovery_pub.publish(msg);
        self.discovery_published = true;
    }

    fn update_amcl_poses(&mut self) {
        for ctx in self.agents.iter_mut() {
            while let Some(amcl_pose_msg) = ctx.amcl_pose_sub.take_data() {
                ctx.amcl_pose.0 = amcl_pose_msg;
                ctx.agent.localized = true;
            }
            if !ctx.agent.localized {
                continue;
            }

            let pose = PoseStamped {
                header: ctx.amcl_pose.0.header.clone(),
                pose: ctx.amcl_pose.0.pose.pose.clone(),
            };
            let outer_active = self.coordinator.with_agent(ctx.index, |shared| {
                shared.agent_pose = Some(AgentPose(pose.clone()));
                shared.active_outer_plan_id.is_some()
            });

            let mut odom = Odometry::default();
            odom.header = ctx.amcl_pose.0.header.clone();
            odom.child_frame_id = "odom".to_string();
            odom.pose = ctx.amcl_pose.0.pose.clone();
            odom.twist = TwistWithCovariance::default();
            let _ = ctx.odom_pub.publish(odom);

            if ctx.progress_tracker.plan_id.is_some() {
                let current_pose = Some(ctx.amcl_pose.xy_yaw());
                let updated_prog = ctx
                    .progress_tracker
                    .compute_progress(current_pose, ctx.latest_progress.0.as_ref());
                ctx.latest_progress.0 = Some(updated_prog);
            }

            let mode = if ctx.progress_tracker.is_holding_at_pre_waypoint() {
                Progress::MODE_ACTION
            } else if outer_active || ctx.inner_client.goal().is_some() {
                Progress::MODE_NAVIGATING
            } else {
                Progress::MODE_IDLE
            };
            let prog = ctx.latest_progress.0.as_ref();
            let sz_id = ctx.current_safe_zone.0.as_ref().map(|sz| &sz.id);
            let progress_msg = build_progress_status(pose, mode, prog, sz_id);
            ctx.latest_progress.0 = Some(progress_msg.clone());
            let _ = ctx.progress_pub.publish(progress_msg);
        }
    }

    fn update_incremental_targets(&mut self) {
        for ctx in self.agents.iter_mut() {
            // Drain any session cancellations received over ROS topic (standalone mode)
            while let Some(cancelled_uuid) = ctx.cancel_session_sub.take_data() {
                let is_newly_cancelled = self.coordinator.with_agent(ctx.index, |shared| {
                    let already = shared.cancelled_sessions.contains(&cancelled_uuid);
                    shared.cancelled_sessions.insert(cancelled_uuid.clone());
                    !already
                });
                if is_newly_cancelled {
                    ctx.pending_cancel = true;
                }
            }

            // Drain any inner cancel requested via in-process coordinator
            let should_cancel = self.coordinator.with_agent(ctx.index, |shared| {
                std::mem::take(&mut shared.pending_inner_cancel)
            });
            if should_cancel {
                ctx.pending_cancel = true;
            }

            while let Some(plan) = ctx.plan_sub.take_data() {
                let prev_action = ctx.progress_tracker.active_action.clone();
                ctx.progress_tracker
                    .set_plan(&plan, ctx.current_safe_zone.0.as_ref());
                let new_action = ctx.progress_tracker.active_action.clone();
                if prev_action != new_action {
                    if let Some(sz) = ctx
                        .current_safe_zone
                        .0
                        .as_ref()
                        .filter(|sz| sz.id.plan_id == plan.plan_id)
                    {
                        if let Some(pending) = ctx.handshake.pending_target.as_mut() {
                            if pending.target.safe_zone_id.plan_id == plan.plan_id {
                                pending.target.action = new_action;
                            }
                        } else if let Some((tx, ty, tyaw)) = next_target(sz) {
                            ctx.pending_targets.push(InnerNavigationTarget::with_action(
                                ctx.index,
                                sz.id.clone(),
                                tx as f64,
                                ty as f64,
                                tyaw as f64,
                                new_action,
                            ));
                        }
                    }
                }
                ctx.current_plan.insert(plan);
            }

            let mut gate_released = false;
            while ctx.gate_release_sub.take_data().is_some() {
                gate_released = true;
            }
            if gate_released {
                ctx.progress_tracker.release_gate();
                if ctx.progress_tracker.plan_id.is_some() {
                    let current_pose = ctx.agent.localized.then(|| ctx.amcl_pose.xy_yaw());
                    let updated_prog = ctx
                        .progress_tracker
                        .compute_progress(current_pose, ctx.latest_progress.0.as_ref());
                    let pose = if ctx.agent.localized {
                        PoseStamped {
                            header: ctx.amcl_pose.0.header.clone(),
                            pose: ctx.amcl_pose.0.pose.pose.clone(),
                        }
                    } else {
                        PoseStamped::default()
                    };
                    let sz_id = ctx.current_safe_zone.0.as_ref().map(|sz| &sz.id);
                    let progress_msg = build_progress_status(
                        pose,
                        Progress::MODE_NAVIGATING,
                        Some(&updated_prog),
                        sz_id,
                    );
                    if ctx.latest_progress.0.as_ref() != Some(&progress_msg) {
                        let _ = ctx.progress_pub.publish(progress_msg.clone());
                        ctx.latest_progress.0 = Some(progress_msg);
                    }
                }
            }

            while let Some(ack_header) = ctx.costmap_updated_sub.take_data() {
                let seq = stamp_to_seq(&ack_header.stamp);
                if seq > ctx.handshake.acked_seq {
                    ctx.handshake.acked_seq = seq;
                }
            }

            let mut latest_safe_zone = None;
            while let Some(safe_zone) = ctx.safe_zone_sub.take_data() {
                latest_safe_zone = Some(safe_zone);
            }

            if let Some(safe_zone) = latest_safe_zone {
                self.coordinator.with_agent(ctx.index, |shared| {
                    shared.latest_safe_zone_received = Some(safe_zone.clone());
                });

                if !ctx.current_safe_zone.matches(&safe_zone) && is_valid(&safe_zone) {
                    if let Some((target_x, target_y, target_yaw)) = next_target(&safe_zone) {
                        let incoming_plan_id = &safe_zone.id.plan_id;
                        let incoming_session = &incoming_plan_id.destination_session;

                        let skip_safe_zone = self.coordinator.with_agent(ctx.index, |shared| {
                            if shared.cancelled_sessions.contains(incoming_session) {
                                debug!(
                                    "[{:?}] Ignoring SafeZone for cancelled destination_session {:?}",
                                    ctx.agent.name, incoming_plan_id
                                );
                                return true;
                            }
                            if let Some(ref outer_plan_id) = shared.active_outer_plan_id.clone() {
                                if &outer_plan_id.destination_session == incoming_session {
                                    shared.superseded_sessions.clear();
                                } else if shared.superseded_sessions.contains(incoming_session) {
                                    return true;
                                } else {
                                    info!(
                                        "[{:?}] Cancelling active NavigationRequest {:?} due to SafeZone plan_id {:?}",
                                        ctx.agent.name, outer_plan_id, incoming_plan_id
                                    );
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
                            false
                        });

                        if skip_safe_zone {
                            continue;
                        }

                        let should_update = ctx.current_safe_zone.should_update_target(&safe_zone);
                        let costmap_changed = ctx
                            .handshake
                            .last_published_costmap
                            .as_ref()
                            .is_none_or(|prev| !costmap_content_eq(prev, &safe_zone.costmap));

                        let mut published_costmap = None;
                        if should_update || costmap_changed {
                            ctx.handshake.next_seq += 1;
                            let seq = ctx.handshake.next_seq;
                            let mut costmap = safe_zone.costmap.clone();
                            costmap.header.stamp = seq_to_stamp(seq);

                            if ctx.costmap_pub.publish(costmap.clone()).is_err() {
                                error!("Failed to publish costmap for agent [{}]", ctx.agent.name);
                                continue;
                            }
                            ctx.handshake.last_published_costmap = Some(costmap.clone());
                            published_costmap = Some((seq, costmap));
                        }

                        let current_pose = ctx.agent.localized.then(|| ctx.amcl_pose.xy_yaw());
                        let matching_plan = ctx.current_plan.get(&safe_zone.id.plan_id);
                        let base_progress = ctx.progress_tracker.on_safe_zone_with_plan(
                            &safe_zone,
                            matching_plan,
                            current_pose,
                            ctx.latest_progress.0.as_ref(),
                        );
                        let pose = if ctx.agent.localized {
                            PoseStamped {
                                header: ctx.amcl_pose.0.header.clone(),
                                pose: ctx.amcl_pose.0.pose.pose.clone(),
                            }
                        } else {
                            PoseStamped::default()
                        };
                        let mode = if ctx.progress_tracker.is_holding_at_pre_waypoint() {
                            Progress::MODE_ACTION
                        } else {
                            Progress::MODE_NAVIGATING
                        };
                        let progress_msg = build_progress_status(
                            pose,
                            mode,
                            Some(&base_progress),
                            Some(&safe_zone.id),
                        );
                        let meaningful_progress_changed = match ctx.latest_progress.0.as_ref() {
                            None => true,
                            Some(prev) => {
                                prev.plan_id != progress_msg.plan_id
                                    || prev.mode != progress_msg.mode
                                    || prev.reached_waypoint != progress_msg.reached_waypoint
                                    || prev.target_waypoint != progress_msg.target_waypoint
                                    || (prev.progress - progress_msg.progress).abs() > 1e-4
                            }
                        };

                        if !should_update {
                            if let (Some(pending), Some((_, costmap))) =
                                (ctx.handshake.pending_target.as_mut(), published_costmap)
                            {
                                pending.costmap = costmap;
                            }
                            if meaningful_progress_changed {
                                let Ok(_) = ctx.progress_pub.publish(progress_msg.clone()) else {
                                    error!(
                                        "Failed to publish progress for agent [{}]",
                                        ctx.agent.name
                                    );
                                    continue;
                                };
                                ctx.latest_progress.0 = Some(progress_msg);
                            }
                        } else {
                            let (seq, costmap) = published_costmap.unwrap_or_else(|| {
                                let seq = ctx.handshake.next_seq;
                                let mut cm = safe_zone.costmap.clone();
                                cm.header.stamp = seq_to_stamp(seq);
                                (seq, cm)
                            });
                            let action = ctx.progress_tracker.active_action.clone();
                            info!(
                                "[{:?}] Queuing SafeZone target (waiting for costmap seq={}, action={:?}): ({:.2}, {:.2}, {:.2})",
                                ctx.agent.name, seq, action, target_x, target_y, target_yaw
                            );

                            let Ok(_) = ctx.progress_pub.publish(progress_msg.clone()) else {
                                error!("Failed to publish progress for agent [{}]", ctx.agent.name);
                                continue;
                            };
                            ctx.latest_progress.0 = Some(progress_msg);

                            if let Some(existing_goal) = ctx.inner_client.goal_mut().as_mut() {
                                existing_goal.mark_superseded();
                            }

                            ctx.current_safe_zone = CurrentSafeZone(Some(safe_zone.clone()));
                            self.coordinator.with_agent(ctx.index, |shared| {
                                shared.current_safe_zone = CurrentSafeZone(Some(safe_zone.clone()));
                            });

                            ctx.handshake.pending_target = Some(PendingNavigationTarget {
                                required_seq: seq,
                                target: InnerNavigationTarget::with_action(
                                    ctx.index,
                                    safe_zone.id,
                                    target_x as f64,
                                    target_y as f64,
                                    target_yaw as f64,
                                    action,
                                ),
                                target_xy_yaw: (target_x, target_y, target_yaw),
                                costmap,
                                last_published: Instant::now(),
                            });
                        }
                    }
                }
            }

            let acked_seq = ctx.handshake.acked_seq;
            if ctx
                .handshake
                .pending_target
                .as_ref()
                .is_some_and(|pending| acked_seq >= pending.required_seq)
            {
                let is_cancelled = ctx
                    .handshake
                    .pending_target
                    .as_ref()
                    .is_some_and(|pending| {
                        self.coordinator.with_agent(ctx.index, |shared| {
                            shared
                                .cancelled_sessions
                                .contains(&pending.target.safe_zone_id.plan_id.destination_session)
                        })
                    });
                if is_cancelled {
                    ctx.handshake.pending_target = None;
                } else {
                    let pending = ctx.handshake.pending_target.take().unwrap();
                    let (tx, ty, tyaw) = pending.target_xy_yaw;
                    info!(
                        "[{:?}] Costmap (seq={}) applied, dispatching SafeZone target: ({:.2}, {:.2}, {:.2})",
                        ctx.agent.name, pending.required_seq, tx, ty, tyaw
                    );
                    ctx.pending_targets.push(pending.target);
                }
            } else if let Some(pending) = ctx.handshake.pending_target.as_mut() {
                let is_cancelled = self.coordinator.with_agent(ctx.index, |shared| {
                    shared
                        .cancelled_sessions
                        .contains(&pending.target.safe_zone_id.plan_id.destination_session)
                });
                if is_cancelled {
                    ctx.handshake.pending_target = None;
                } else if pending.last_published.elapsed() >= Duration::from_millis(500) {
                    let _ = ctx.costmap_pub.publish(pending.costmap.clone());
                    pending.last_published = Instant::now();
                }
            }
        }
    }

    fn dispatch_inner_navigation_commands(&mut self) {
        let now = self.node.get_clock().now();
        for ctx in self.agents.iter_mut() {
            if std::mem::take(&mut ctx.pending_cancel) {
                info!(
                    "Received external cancellation request for agent {}",
                    ctx.index
                );
                if ctx.inner_client.goal().is_some() || ctx.inner_client.last_action.is_some() {
                    if let Some(existing_goal) = ctx.inner_client.goal_mut().as_mut() {
                        existing_goal.mark_superseded();
                    }
                    ctx.inner_client.reset_goal();
                    ctx.cancelling_inner = true;
                    let _ = ctx
                        .inner_client
                        .command_tx
                        .unbounded_send(AgentNavCommand::Cancel);
                }
            }

            let targets: Vec<InnerNavigationTarget> = ctx.pending_targets.drain(..).collect();
            for target in targets {
                let is_cancelled = self.coordinator.with_agent(ctx.index, |shared| {
                    shared
                        .cancelled_sessions
                        .contains(&target.safe_zone_id.plan_id.destination_session)
                });
                if is_cancelled {
                    continue;
                }

                if let Some(existing_goal) = ctx.inner_client.goal() {
                    if !should_preempt_goal(
                        &target.safe_zone_id,
                        &target.action,
                        existing_goal.id(),
                        existing_goal.action(),
                    ) {
                        continue;
                    }
                }

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
                    "[{}] Requesting new pending goal to pose [{}, {}] (action={:?})",
                    ctx.index, goal_pose.pose.position.x, goal_pose.pose.position.y, target.action,
                );

                let request = InnerNavigationRequest {
                    agent: ctx.index,
                    safe_zone_id: target.safe_zone_id.clone(),
                    target_pose: goal_pose,
                    action: target.action.clone(),
                };

                *ctx.inner_client.goal_mut() = Some(ActiveInnerGoal::new(
                    target.safe_zone_id.clone(),
                    target.action.clone(),
                ));
                ctx.inner_client.last_action = Some(target.action.clone());
                let _ = ctx
                    .inner_client
                    .command_tx
                    .unbounded_send(AgentNavCommand::Dispatch(request));
            }

            let has_active_goal = ctx.inner_client.goal().is_some();
            self.coordinator.with_agent(ctx.index, |shared| {
                shared.inner_active_goal_present = has_active_goal;
            });
        }
    }

    fn process_inner_navigation_outcomes(&mut self) {
        for ctx in self.agents.iter_mut() {
            let outcomes: Vec<AgentNavOutcome> = ctx.inner_client.outcome_rx.try_iter().collect();
            for outcome in outcomes {
                match outcome {
                    AgentNavOutcome::Feedback {
                        agent: _,
                        safe_zone_id,
                        feedback,
                    } => {
                        if let Some(active_goal) = ctx.inner_client.goal() {
                            if active_goal.id() != &safe_zone_id {
                                continue;
                            }
                        }
                        let fb_pose = feedback.current_pose.clone();
                        ctx.latest_fb_pose = Some(fb_pose.clone());
                        let _ = ctx.feedback_pose_pub.publish(fb_pose.clone());
                        self.coordinator.with_agent(ctx.index, |shared| {
                            shared.agent_pose = Some(AgentPose(fb_pose));
                            shared.outer_feedback_queue.push_back(feedback);
                        });
                    }
                    AgentNavOutcome::Succeeded { request } => {
                        let is_current = ctx.inner_client.goal().as_ref().is_some_and(|goal| {
                            goal.id() == &request.safe_zone_id && !goal.is_superseded()
                        });
                        let mut should_retry = false;

                        if is_current {
                            let matches_current_safe_zone = ctx
                                .current_safe_zone
                                .0
                                .as_ref()
                                .is_none_or(|sz| sz.id == request.safe_zone_id);
                            if matches_current_safe_zone
                                && ctx.progress_tracker.plan_id.as_ref()
                                    == Some(&request.safe_zone_id.plan_id)
                            {
                                if matches!(
                                    request.action,
                                    InnerNavigationAction::DockRobot { .. }
                                        | InnerNavigationAction::UndockRobot { .. }
                                ) {
                                    ctx.progress_tracker.mark_action_completed();
                                }
                                let agent_pose = self
                                    .coordinator
                                    .with_agent(ctx.index, |shared| shared.agent_pose.clone());
                                let current_pose = if ctx.agent.localized {
                                    Some(ctx.amcl_pose.xy_yaw())
                                } else {
                                    ctx.latest_fb_pose
                                        .as_ref()
                                        .map(|p| AgentPose(p.clone()).xy_yaw())
                                        .or_else(|| agent_pose.as_ref().map(|p| p.xy_yaw()))
                                };
                                if !ctx.progress_tracker.is_target_pose_satisfied(current_pose) {
                                    warn!(
                                        "[{}] Nav2 reported goal succeeded before target pose was satisfied (current={:?}); retrying goal {:?}.",
                                        ctx.index, current_pose, request.safe_zone_id,
                                    );
                                    should_retry = true;
                                } else if let Some(reached_prog) = ctx
                                    .progress_tracker
                                    .mark_target_reached(ctx.latest_progress.0.as_ref())
                                {
                                    let pose = if ctx.agent.localized {
                                        PoseStamped {
                                            header: ctx.amcl_pose.0.header.clone(),
                                            pose: ctx.amcl_pose.0.pose.pose.clone(),
                                        }
                                    } else {
                                        ctx.latest_fb_pose
                                            .clone()
                                            .or_else(|| agent_pose.map(|p| p.0))
                                            .unwrap_or_default()
                                    };
                                    let sz_id = ctx
                                        .current_safe_zone
                                        .0
                                        .as_ref()
                                        .map(|sz| &sz.id)
                                        .or(Some(&request.safe_zone_id));
                                    let mode = if ctx.progress_tracker.is_holding_at_pre_waypoint()
                                    {
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
                                    if ctx.latest_progress.0.as_ref() != Some(&progress_msg) {
                                        let _ = ctx.progress_pub.publish(progress_msg.clone());
                                        ctx.latest_progress.0 = Some(progress_msg);
                                    }
                                }
                            }
                        }

                        if should_retry {
                            let _ = ctx
                                .inner_client
                                .command_tx
                                .unbounded_send(AgentNavCommand::Dispatch(request));
                        } else if ctx
                            .inner_client
                            .goal()
                            .as_ref()
                            .is_some_and(|g| g.id() == &request.safe_zone_id)
                        {
                            ctx.inner_client.reset_goal();
                        }
                    }
                    AgentNavOutcome::Aborted {
                        request,
                        superseded,
                    } => {
                        let active_goal_info = ctx
                            .inner_client
                            .goal()
                            .as_ref()
                            .map(|goal| (goal.id(), goal.is_superseded() || superseded));
                        let matches_current_safe_zone = ctx
                            .current_safe_zone
                            .0
                            .as_ref()
                            .is_none_or(|sz| sz.id == request.safe_zone_id);

                        if !matches_current_safe_zone
                            || !should_publish_path_blocked(&request.safe_zone_id, active_goal_info)
                        {
                            info!(
                                "[{}] Ignoring stale Nav2 abort {:?}",
                                ctx.index, request.safe_zone_id,
                            );
                        } else {
                            let plan_error = PlanError {
                                error: RmfError {
                                    code: PlanError::CODE_PATH_BLOCKED,
                                    message: format!(
                                        "Nav2 goal aborted for agent {}: path blocked within safe zone",
                                        ctx.index
                                    ),
                                    parameters: String::new(),
                                },
                                plan_id: request.safe_zone_id.plan_id.clone(),
                            };
                            if let Err(e) = ctx.plan_error_pub.publish(plan_error) {
                                error!(
                                    "Failed to publish PlanError for agent {}: {:?}",
                                    ctx.index, e
                                );
                            } else {
                                warn!(
                                    "[{}] Goal aborted by Nav2! Published PlanError CODE_PATH_BLOCKED to ~/plan/error.",
                                    ctx.index
                                );
                            }
                        }

                        if ctx
                            .inner_client
                            .goal()
                            .as_ref()
                            .is_some_and(|g| g.id() == &request.safe_zone_id)
                        {
                            ctx.inner_client.reset_goal();
                        }
                    }
                    AgentNavOutcome::Cancelled {
                        agent: _,
                        request,
                        external,
                    } => {
                        if external {
                            ctx.cancelling_inner = false;
                        }
                        if let Some(req) = request {
                            if ctx
                                .inner_client
                                .goal()
                                .as_ref()
                                .is_some_and(|g| g.id() == &req.safe_zone_id)
                            {
                                ctx.inner_client.reset_goal();
                            }
                        } else if external {
                            ctx.inner_client.reset_goal();
                        }
                    }
                }
            }

            let has_active_goal = ctx.inner_client.goal().is_some();
            self.coordinator.with_agent(ctx.index, |shared| {
                shared.inner_active_goal_present = has_active_goal;
            });
        }
    }
}
