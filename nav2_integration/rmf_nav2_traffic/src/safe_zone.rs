use crate::{
    agent::AmclPose, inner_navigation_client::InnerNavigationTarget,
    navigation_server::CurrentNavigationRequest, Nav2Agent,
};
use bevy::prelude::*;
use bevy_ros2::{RclrsNode, RosPublisher, RosSubscription};
use ros_env::{
    geometry_msgs::msg::PoseStamped,
    nav2_msgs::msg::Costmap,
    rmf_prototype_msgs::msg::{Plan, PlanError, PlanId, Progress, Region, SafeZone, SafeZoneId},
};
use std::{collections::VecDeque, sync::Arc};

#[derive(Component)]
pub struct SafeZoneSubscription {
    pub subscriber: Arc<RosSubscription<SafeZone>>,
}

#[derive(Component)]
pub struct PlanSubscription {
    pub subscriber: Arc<RosSubscription<Plan>>,
}

#[derive(Component, Debug, Clone, Default)]
pub struct LocalPlanRegistry {
    entries: VecDeque<Plan>,
}

impl LocalPlanRegistry {
    const MAX_CACHED_PLANS: usize = 16;

    pub fn insert(&mut self, plan: Plan) {
        if let Some(existing) = self.entries.iter_mut().find(|p| p.plan_id == plan.plan_id) {
            *existing = plan;
            return;
        }
        if self.entries.len() >= Self::MAX_CACHED_PLANS {
            self.entries.pop_front();
        }
        self.entries.push_back(plan);
    }

    pub fn get(&self, plan_id: &PlanId) -> Option<&Plan> {
        self.entries.iter().find(|p| &p.plan_id == plan_id)
    }
}

#[derive(Component)]
pub struct CostmapPublisher {
    pub publisher: Arc<RosPublisher<Costmap>>,
}

#[derive(Component)]
pub struct ProgressPublisher {
    pub publisher: Arc<RosPublisher<Progress>>,
}

#[derive(Component, Debug, Clone, Default, Deref)]
pub struct LatestProgress(pub Option<Progress>);

pub fn shortest_angular_distance(a: f32, b: f32) -> f32 {
    let diff = (a - b).abs().rem_euclid(std::f32::consts::TAU);
    diff.min(std::f32::consts::TAU - diff)
}

#[derive(Component, Debug, Clone, Default)]
pub struct ProgressTracker {
    pub plan_id: Option<PlanId>,
    pub plan_waypoints: Vec<([f32; 2], f32)>,
    pub start_waypoint: u64,
    pub start_progress: f32,
    pub target_waypoint: u64,
    pub target_progress: f32,
    pub start_xy: (f32, f32),
    pub target_xy: (f32, f32),
    pub target_tolerance: f32,
    pub target_yaw: Option<f32>,
    pub target_yaw_tolerance: f32,
}

impl ProgressTracker {
    fn extract_plan_waypoints(plan: &Plan) -> Vec<([f32; 2], f32)> {
        plan.waypoints
            .iter()
            .enumerate()
            .map(|(i, wp)| {
                let prog = if i > 0 && wp.progress <= 0.0 {
                    i as f32
                } else {
                    wp.progress
                };
                (wp.position, prog)
            })
            .collect()
    }

    fn waypoint_progress(&self, wp_idx: u64) -> Option<f32> {
        self.plan_waypoints.get(wp_idx as usize).map(|&(_, p)| p)
    }

    pub fn set_plan(&mut self, plan: &Plan) {
        if self.plan_id.as_ref() == Some(&plan.plan_id) {
            self.plan_waypoints = Self::extract_plan_waypoints(plan);
            if let Some(wp_prog) = self.waypoint_progress(self.start_waypoint) {
                self.start_progress = wp_prog.min(self.target_progress);
            }
        }
    }

    fn extract_orientation_constraint(safe_zone: &SafeZone) -> (Option<f32>, f32) {
        let ori = safe_zone
            .incremental_target
            .regions
            .first()
            .and_then(|r| r.orientations.first())
            .or_else(|| {
                safe_zone
                    .incremental_target
                    .nodes
                    .first()
                    .and_then(|n| n.orientations.first())
            });
        let Some(ori) = ori else {
            return (None, std::f32::consts::PI);
        };
        let effective_tol = if ori.tolerance_radians <= 0.0 && ori.spread_radians <= 0.0 {
            0.25
        } else {
            0.5 * ori.spread_radians.max(0.0) + ori.tolerance_radians.max(0.0)
        };
        if effective_tol < std::f32::consts::PI - 1e-3 {
            (Some(ori.orientation_radians), effective_tol)
        } else {
            (None, std::f32::consts::PI)
        }
    }

    fn is_orientation_satisfied(&self, current_yaw: f32) -> bool {
        match self.target_yaw {
            Some(target_yaw) if self.target_yaw_tolerance < std::f32::consts::PI - 1e-3 => {
                shortest_angular_distance(current_yaw, target_yaw) <= self.target_yaw_tolerance
            }
            _ => true,
        }
    }

    pub fn is_target_pose_satisfied(&self, current_pose: Option<(f32, f32, f32)>) -> bool {
        let Some((rx, ry, ryaw)) = current_pose else {
            return true;
        };
        let dist_to_target = (rx - self.target_xy.0).hypot(ry - self.target_xy.1);
        dist_to_target <= self.target_tolerance && self.is_orientation_satisfied(ryaw)
    }

    pub fn on_safe_zone(
        &mut self,
        safe_zone: &SafeZone,
        current_pose: Option<(f32, f32, f32)>,
        previous: Option<&Progress>,
    ) -> Progress {
        self.on_safe_zone_with_plan(safe_zone, None, current_pose, previous)
    }

    pub fn on_safe_zone_with_plan(
        &mut self,
        safe_zone: &SafeZone,
        plan: Option<&Plan>,
        current_pose: Option<(f32, f32, f32)>,
        previous: Option<&Progress>,
    ) -> Progress {
        let plan_id = safe_zone.id.plan_id.clone();
        let target_wp = safe_zone.target_waypoint.first().copied().unwrap_or(0);
        let target_xy = CurrentSafeZone::get_point(safe_zone)
            .map(|(x, y)| (x as f32, y as f32))
            .unwrap_or((0.0, 0.0));
        let target_tolerance = safe_zone
            .incremental_target
            .regions
            .first()
            .map(|r| r.tolerance)
            .filter(|&t| t > 0.0)
            .unwrap_or(0.25);
        let (target_yaw, target_yaw_tolerance) = Self::extract_orientation_constraint(safe_zone);

        let same_plan = self.plan_id.as_ref() == Some(&plan_id);
        let current_xy = current_pose.map(|(x, y, _)| (x, y));
        let fallback_start = current_xy.unwrap_or(target_xy);

        if !same_plan {
            self.plan_waypoints = plan
                .filter(|p| p.plan_id == plan_id)
                .map(Self::extract_plan_waypoints)
                .unwrap_or_default();
            self.plan_id = Some(plan_id);
            self.start_waypoint = safe_zone.last_waypoint;
            self.start_progress = self
                .waypoint_progress(safe_zone.last_waypoint)
                .unwrap_or(safe_zone.last_waypoint as f32)
                .min(safe_zone.target_progress);
            self.target_waypoint = target_wp;
            self.target_progress = safe_zone.target_progress;
            self.start_xy = fallback_start;
            self.target_xy = target_xy;
            self.target_tolerance = target_tolerance;
            self.target_yaw = target_yaw;
            self.target_yaw_tolerance = target_yaw_tolerance;
        } else {
            if self.plan_waypoints.is_empty() {
                if let Some(p) = plan.filter(|p| p.plan_id == plan_id) {
                    self.plan_waypoints = Self::extract_plan_waypoints(p);
                    if let Some(wp_prog) = self.waypoint_progress(self.start_waypoint) {
                        self.start_progress = wp_prog.min(safe_zone.target_progress);
                    }
                }
            }
            let yaw_changed = match (self.target_yaw, target_yaw) {
                (Some(a), Some(b)) => shortest_angular_distance(a, b) > 0.05,
                (None, None) => false,
                _ => true,
            };
            let target_changed = target_wp != self.target_waypoint
                || (target_xy.0 - self.target_xy.0).hypot(target_xy.1 - self.target_xy.1) > 1e-3
                || yaw_changed;
            if target_changed {
                let prev_same_plan = previous.filter(|p| p.plan_id == plan_id);
                self.start_xy = current_xy.unwrap_or(self.target_xy);
                self.start_waypoint = self.start_waypoint.max(safe_zone.last_waypoint);
                self.start_progress = self
                    .waypoint_progress(self.start_waypoint)
                    .unwrap_or_else(|| {
                        prev_same_plan
                            .map(|p| p.progress)
                            .unwrap_or(self.start_waypoint as f32)
                    })
                    .min(safe_zone.target_progress);
            }
            self.target_waypoint = target_wp;
            self.target_progress = safe_zone.target_progress;
            self.target_xy = target_xy;
            self.target_tolerance = target_tolerance;
            self.target_yaw = target_yaw;
            self.target_yaw_tolerance = target_yaw_tolerance;
        }

        self.compute_progress(current_pose, previous)
    }

    pub fn compute_progress(
        &self,
        current_pose: Option<(f32, f32, f32)>,
        previous: Option<&Progress>,
    ) -> Progress {
        let Some(plan_id) = self.plan_id.clone() else {
            return Progress::default();
        };

        let prev_same_plan = previous.filter(|p| p.plan_id == plan_id);
        let prev_reached = prev_same_plan.map(|p| p.reached_waypoint).unwrap_or(0);
        let prev_progress = prev_same_plan.map(|p| p.progress).unwrap_or(0.0);

        let start_prog = self
            .waypoint_progress(self.start_waypoint)
            .unwrap_or(self.start_progress);
        let mut reached_waypoint = self.start_waypoint.max(prev_reached);
        let mut progress = start_prog.max(prev_progress);

        if let Some((rx, ry, ryaw)) = current_pose {
            let dist_to_target = (rx - self.target_xy.0).hypot(ry - self.target_xy.1);
            let pos_reached = dist_to_target <= self.target_tolerance;
            let ori_reached = self.is_orientation_satisfied(ryaw);

            if pos_reached && ori_reached {
                reached_waypoint = reached_waypoint.max(self.target_waypoint);
                progress = progress.max(self.target_progress);
            } else if reached_waypoint < self.target_waypoint {
                let (sx, sy) = self.start_xy;
                let (tx, ty) = self.target_xy;
                let dx = tx - sx;
                let dy = ty - sy;
                let seg_len_sq = dx * dx + dy * dy;
                let prog_span = (self.target_progress - start_prog).max(0.0);
                let wp_span = (self.target_waypoint - self.start_waypoint) as f32;
                let pre_final_cap = if self.target_yaw.is_some() {
                    (self.target_progress - 1e-3).max(start_prog)
                } else {
                    self.target_progress
                };

                if pos_reached && !ori_reached {
                    let pre_rot_wp = self
                        .target_waypoint
                        .saturating_sub(1)
                        .max(self.start_waypoint);
                    reached_waypoint = reached_waypoint.max(pre_rot_wp);
                    let pre_rot_prog = self.waypoint_progress(pre_rot_wp).unwrap_or_else(|| {
                        if wp_span > 0.0 {
                            let frac = (pre_rot_wp - self.start_waypoint) as f32 / wp_span;
                            start_prog + frac * prog_span
                        } else {
                            start_prog
                        }
                    });
                    let interp = if seg_len_sq > 1e-6 {
                        pre_final_cap.max(pre_rot_prog).min(pre_final_cap)
                    } else {
                        pre_rot_prog.min(pre_final_cap)
                    };
                    progress = progress.max(interp).min(self.target_progress);
                } else {
                    let target_idx = (self.target_waypoint as usize)
                        .min(self.plan_waypoints.len().saturating_sub(1));
                    let search_start =
                        (reached_waypoint as usize).max(self.start_waypoint as usize);
                    let mut best_seg: Option<(f32, usize, f32, f32)> = None;

                    if !self.plan_waypoints.is_empty() && search_start < target_idx {
                        for i in search_start..target_idx {
                            let (p0, prog0) = self.plan_waypoints[i];
                            let (p1, prog1) = self.plan_waypoints[i + 1];
                            let sdx = p1[0] - p0[0];
                            let sdy = p1[1] - p0[1];
                            let slen_sq = sdx * sdx + sdy * sdy;
                            if slen_sq <= 1e-6 {
                                continue;
                            }
                            let t = (((rx - p0[0]) * sdx + (ry - p0[1]) * sdy) / slen_sq)
                                .clamp(0.0, 1.0);
                            let proj_x = p0[0] + t * sdx;
                            let proj_y = p0[1] + t * sdy;
                            let dist_sq = (rx - proj_x).powi(2) + (ry - proj_y).powi(2);
                            let seg_prog = prog0 + t * (prog1 - prog0);
                            match best_seg {
                                None => best_seg = Some((dist_sq, i, t, seg_prog)),
                                Some((best_dist_sq, _, _, _)) if dist_sq + 1e-3 < best_dist_sq => {
                                    best_seg = Some((dist_sq, i, t, seg_prog));
                                }
                                _ => {}
                            }
                        }
                    }

                    if let Some((_, seg_idx, t, seg_prog)) = best_seg {
                        let interp = seg_prog.clamp(start_prog, pre_final_cap);
                        progress = progress.max(interp).min(self.target_progress);
                        if self.target_waypoint > self.start_waypoint + 1 {
                            let seg_wp = if t >= 1.0 - 1e-4 {
                                (seg_idx + 1) as u64
                            } else {
                                seg_idx as u64
                            };
                            let intermediate_wp =
                                seg_wp.min(self.target_waypoint.saturating_sub(1));
                            reached_waypoint = reached_waypoint.max(intermediate_wp);
                        }
                    } else if seg_len_sq > 1e-6 {
                        let t = (((rx - sx) * dx + (ry - sy) * dy) / seg_len_sq).clamp(0.0, 1.0);
                        let interp = (start_prog + t * prog_span).min(pre_final_cap);
                        progress = progress.max(interp).min(self.target_progress);
                        if self.target_waypoint > self.start_waypoint + 1 && wp_span > 0.0 {
                            let intermediate_wp = (self.start_waypoint
                                + (t * wp_span).floor() as u64)
                                .min(self.target_waypoint.saturating_sub(1));
                            reached_waypoint = reached_waypoint.max(intermediate_wp);
                        }
                    }
                }
            }
        }

        Progress {
            progress,
            reached_waypoint,
            target_waypoint: self.target_waypoint,
            reached_keys: vec![],
            plan_id,
            ..Default::default()
        }
    }

    pub fn mark_target_reached(&self, previous: Option<&Progress>) -> Option<Progress> {
        let plan_id = self.plan_id.clone()?;
        let prev_same_plan = previous.filter(|p| p.plan_id == plan_id);
        let prev_reached = prev_same_plan.map(|p| p.reached_waypoint).unwrap_or(0);
        let prev_progress = prev_same_plan.map(|p| p.progress).unwrap_or(0.0);

        Some(Progress {
            progress: prev_progress.max(self.target_progress),
            reached_waypoint: prev_reached.max(self.target_waypoint),
            target_waypoint: self.target_waypoint,
            reached_keys: vec![],
            plan_id,
            ..Default::default()
        })
    }
}

pub fn build_progress_status(
    pose: PoseStamped,
    mode: u8,
    progress: Option<&Progress>,
    safe_zone_id: Option<&SafeZoneId>,
) -> Progress {
    let mut msg = progress.cloned().unwrap_or_default();
    msg.pose = pose;
    msg.mode = mode;
    msg.safe_zone_id = match safe_zone_id {
        Some(sz_id) => {
            let rmw_sz = <SafeZoneId as rosidl_runtime_rs::Message>::into_rmw_message(
                std::borrow::Cow::Borrowed(sz_id),
            )
            .into_owned();
            vec![rmw_sz].try_into().unwrap()
        }
        None => vec![].try_into().unwrap(),
    };
    msg
}

#[derive(Component)]
pub struct PlanErrorPublisher {
    pub publisher: Arc<RosPublisher<PlanError>>,
}

#[derive(Component, Debug, Clone, Default, Deref)]
pub struct CurrentSafeZone(pub Option<SafeZone>);

impl CurrentSafeZone {
    pub fn update(&mut self, value: SafeZone) {
        self.0 = Some(value);
    }

    pub fn clear(&mut self) {
        self.0 = None;
    }

    pub fn matches(&self, other: &SafeZone) -> bool {
        self.as_ref().is_some_and(|sz| sz.id == other.id)
    }

    pub fn should_update_target(&self, other: &SafeZone) -> bool {
        let Some(current) = self.as_ref() else {
            return true;
        };

        // Resend an unchanged target when it belongs to a new plan.
        if current.id.plan_id != other.id.plan_id {
            return true;
        }

        // Resend when targeting a new waypoint in the plan (e.g. short step or final approach).
        if !other.target_waypoint.is_empty() && current.target_waypoint != other.target_waypoint {
            return true;
        }

        // Resend if target orientation has significantly changed.
        if let (Some(cur_yaw), Some(other_yaw)) =
            (Self::get_orientation(current), Self::get_orientation(other))
        {
            let diff = (cur_yaw - other_yaw)
                .abs()
                .rem_euclid(std::f32::consts::TAU);
            let norm_diff = diff.min(std::f32::consts::TAU - diff);
            if norm_diff > 0.05 {
                return true;
            }
        }

        self.distancesq_to_target(other) >= 0.5
    }

    fn get_orientation(safe_zone: &SafeZone) -> Option<f32> {
        safe_zone
            .incremental_target
            .regions
            .first()
            .and_then(|r| r.orientations.first().map(|o| o.orientation_radians))
            .or_else(|| {
                safe_zone
                    .incremental_target
                    .nodes
                    .first()
                    .and_then(|n| n.orientations.first().map(|o| o.orientation_radians))
            })
    }

    pub fn distancesq_to_target(&self, other: &SafeZone) -> f64 {
        let Some(safe_zone) = self.as_ref() else {
            return f64::INFINITY;
        };

        let Some((sx, sy)) = Self::get_point(safe_zone) else {
            return f64::INFINITY;
        };

        let Some((dx, dy)) = Self::get_point(other) else {
            return f64::INFINITY;
        };

        (sx - dx).powi(2) + (sy - dy).powi(2)
    }

    fn get_point(safe_zone: &SafeZone) -> Option<(f64, f64)> {
        let Some(region) = safe_zone.incremental_target.regions.first() else {
            return None;
        };
        match region.region.hint {
            Region::HINT_POINT => {
                if region.region.points.len() != 2 {
                    None
                } else {
                    Some((
                        region.region.points[0].into(),
                        region.region.points[1].into(),
                    ))
                }
            }
            _ => None,
        }
    }
}

#[derive(Default)]
pub struct SafeZoneSubscriptionPlugin {}

impl Plugin for SafeZoneSubscriptionPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(
            PreUpdate,
            (update_local_plans, update_incremental_target).chain(),
        )
        .add_observer(create_safe_zone_subscriber)
        .add_observer(create_costmap_publisher)
        .add_observer(create_progress_publisher)
        .add_observer(create_plan_error_publisher);
    }
}

fn create_safe_zone_subscriber(
    trigger: Trigger<OnAdd, Nav2Agent>,
    mut commands: Commands,
    agents: Query<&Nav2Agent>,
    node: Res<RclrsNode>,
) {
    let e = trigger.target();
    let Ok(agent_name) = agents.get(e).map(|agent| agent.name.clone()) else {
        return;
    };
    let topic = agent_name.clone() + "/plan/safe_zone";
    let subscription = Arc::new(RosSubscription::<SafeZone>::new(&node, topic));
    let plan_topic = agent_name + "/plan";
    let plan_subscription = Arc::new(RosSubscription::<Plan>::new(&node, plan_topic));
    commands.entity(e).insert((
        SafeZoneSubscription {
            subscriber: Arc::clone(&subscription),
        },
        PlanSubscription {
            subscriber: Arc::clone(&plan_subscription),
        },
        LocalPlanRegistry::default(),
        CurrentSafeZone::default(),
    ));
}

fn create_costmap_publisher(
    trigger: Trigger<OnAdd, Nav2Agent>,
    mut commands: Commands,
    agents: Query<&Nav2Agent>,
    node: Res<RclrsNode>,
) {
    let e = trigger.target();
    let Ok(agent_name) = agents.get(e).map(|agent| agent.name.clone()) else {
        return;
    };
    // TODO(@xiyuoh) review this topic name
    let topic = agent_name + "/inner/global_costmap/plan/costmap";
    let publisher = Arc::new(RosPublisher::<Costmap>::new(&node, topic));
    commands.entity(e).insert(CostmapPublisher {
        publisher: Arc::clone(&publisher),
    });
}

fn create_progress_publisher(
    trigger: Trigger<OnAdd, Nav2Agent>,
    mut commands: Commands,
    agents: Query<&Nav2Agent>,
    node: Res<RclrsNode>,
) {
    let e = trigger.target();
    let Ok(agent_name) = agents.get(e).map(|agent| agent.name.clone()) else {
        return;
    };
    let topic = agent_name + "/plan/progress";
    let publisher = Arc::new(RosPublisher::<Progress>::new_transient_local(&node, topic));
    commands.entity(e).insert((
        ProgressPublisher {
            publisher: Arc::clone(&publisher),
        },
        LatestProgress::default(),
        ProgressTracker::default(),
    ));
}

fn create_plan_error_publisher(
    trigger: Trigger<OnAdd, Nav2Agent>,
    mut commands: Commands,
    agents: Query<&Nav2Agent>,
    node: Res<RclrsNode>,
) {
    let e = trigger.target();
    let Ok(agent_name) = agents.get(e).map(|agent| agent.name.clone()) else {
        return;
    };
    let topic = agent_name + "/plan/error";
    let publisher = Arc::new(RosPublisher::<PlanError>::new_transient_local(&node, topic));
    commands.entity(e).insert(PlanErrorPublisher {
        publisher: Arc::clone(&publisher),
    });
}

fn update_local_plans(
    mut agents: Query<(
        &PlanSubscription,
        &mut LocalPlanRegistry,
        &mut ProgressTracker,
    )>,
) {
    for (plan_sub, mut local_plans, mut progress_tracker) in agents.iter_mut() {
        if let Some(plan) = plan_sub.subscriber.data_callback() {
            if local_plans.get(&plan.plan_id).is_none() {
                local_plans.insert(plan.clone());
                progress_tracker.set_plan(&plan);
            }
        }
    }
}

fn update_incremental_target(
    mut nav_target: EventWriter<InnerNavigationTarget>,
    mut subscriptions: Query<
        (
            Entity,
            &SafeZoneSubscription,
            &LocalPlanRegistry,
            &CostmapPublisher,
            &ProgressPublisher,
            &mut LatestProgress,
            &mut ProgressTracker,
            &mut CurrentSafeZone,
            Option<&AmclPose>,
            &Nav2Agent,
        ),
        // Only respond to a SafeZone message if there is an active NavigationRequest
        // for this agent
        With<CurrentNavigationRequest>,
    >,
) {
    for (
        e,
        safe_zone_sub,
        local_plans,
        costmap_pub,
        progress_pub,
        mut latest_progress,
        mut progress_tracker,
        mut current_safe_zone,
        amcl_pose,
        agent,
    ) in subscriptions.iter_mut()
    {
        let Some(safe_zone) = safe_zone_sub.subscriber.data_callback() else {
            continue;
        };
        //if current_safe_zone.matches(&safe_zone) {
        //    continue;
        //}

        // Call updateCosts() before setting new inner nav target
        let Ok(_) = costmap_pub.publisher.publish(safe_zone.costmap.clone()) else {
            error!("Failed to publish costmap for agent [{}]", agent.name);
            continue;
        };

        // Validate safe zone msg
        if !is_valid(&safe_zone) {
            continue;
        }

        let Some((target_x, target_y, target_yaw)) = next_target(&safe_zone) else {
            continue;
        };

        let current_pose = amcl_pose.map(|p| p.xy_yaw());

        // Publish progress
        let matching_plan = local_plans.get(&safe_zone.id.plan_id);
        let base_progress = progress_tracker.on_safe_zone_with_plan(
            &safe_zone,
            matching_plan,
            current_pose,
            latest_progress.0.as_ref(),
        );
        let pose = amcl_pose
            .map(|p| PoseStamped {
                header: p.0.header.clone(),
                pose: p.0.pose.pose.clone(),
            })
            .unwrap_or_default();
        let progress_msg = build_progress_status(
            pose,
            Progress::MODE_NAVIGATING,
            Some(&base_progress),
            Some(&safe_zone.id),
        );
        let Ok(_) = progress_pub.publisher.publish(progress_msg.clone()) else {
            error!("Failed to publish progress for agent [{}]", agent.name);
            continue;
        };
        latest_progress.0 = Some(progress_msg);

        if !current_safe_zone.should_update_target(&safe_zone) {
            continue;
        }
        debug!(
            "[{:?}] Updating SafeZone to target: ({:.2}, {:.2}, {:.2})",
            agent.name, target_x, target_y, target_yaw
        );

        *current_safe_zone = CurrentSafeZone(Some(safe_zone.clone()));

        nav_target.write(InnerNavigationTarget::new(
            e,
            safe_zone.id,
            target_x as f64,
            target_y as f64,
            target_yaw as f64,
        ));
    }
}

fn is_valid(safe_zone: &SafeZone) -> bool {
    if safe_zone.target_waypoint.is_empty() {
        error!("Received a SafeZone message with empty target_waypoint");
        return false;
    }
    if safe_zone.incremental_target.regions.is_empty()
        && safe_zone.incremental_target.nodes.is_empty()
    {
        error!("Received a SafeZone message with empty incremental_target");
        return false;
    }

    true
}

/// Extract the `(x, y, yaw)` target pose from a [`SafeZone`].
///
/// Heading precedence:
/// 1. Orientations from `incremental_target.regions` are evaluated in order.
/// 2. Orientations from `incremental_target.nodes` are evaluated in order and override
///    any region heading, with the last node that specifies an orientation winning.
fn next_target(safe_zone: &SafeZone) -> Option<(f32, f32, f32)> {
    // TODO(@xiyuoh) more sophisticated point selection taking into account
    // all factors (region hints, orientations, etc.)

    let constraints = &safe_zone.incremental_target;
    let mut xy: Option<(f32, f32)> = None;
    let mut yaw: Option<f32> = None;

    // Assume either regions or nodes will be populated, not both.
    for target_region in constraints.regions.iter() {
        let region = &target_region.region;
        let points = &region.points;

        match region.hint {
            Region::HINT_POINT => {
                // There should only be exactly 2 elements forming (x, y)
                if points.len() != 2 {
                    continue;
                }
                // TODO(@xiyuoh)
                xy = Some((points[0], points[1]));
            }
            Region::HINT_AXIS_ALIGNED_RECTANGLE => {
                //
            }
            Region::HINT_RECTANGLE => {
                //
            }
            Region::HINT_CONVEX_POLYGON => {
                //
            }
            Region::HINT_POLYGON | Region::HINT_UNSPECIFIED => {
                //
            }
            _ => {
                //
            }
        }

        for target_ori in target_region.orientations.iter() {
            yaw = Some(target_ori.orientation_radians);
            // TODO(@xiyuoh) some processing using spread and tolerance
        }
    }
    for target_node in constraints.nodes.iter() {
        for target_ori in target_node.orientations.iter() {
            yaw = Some(target_ori.orientation_radians);
        }
    }

    xy.zip(yaw).map(|((x, y), yaw)| (x, y, yaw))
}

#[cfg(test)]
mod tests {
    use super::*;
    use ros_env::rmf_prototype_msgs::msg::{TargetOrientation, TargetRegion};

    fn safe_zone(
        session: u8,
        plan_version: u64,
        safe_zone_version: u64,
        x: f32,
        y: f32,
    ) -> SafeZone {
        let mut safe_zone = SafeZone::default();
        safe_zone.id.plan_id.destination_session.uuid[0] = session;
        safe_zone.id.plan_id.plan_version = plan_version;
        safe_zone.id.safe_zone_version = safe_zone_version;

        let mut target = TargetRegion::default();
        target.region.hint = Region::HINT_POINT;
        target.region.points = vec![x, y];
        safe_zone.incremental_target.regions.push(target);
        safe_zone
    }

    #[test]
    fn first_target_is_sent() {
        let current = CurrentSafeZone::default();
        let next = safe_zone(1, 0, 0, 4.0, 4.0);

        assert!(current.should_update_target(&next));
    }

    #[test]
    fn nearby_target_from_same_plan_is_suppressed() {
        let current = CurrentSafeZone(Some(safe_zone(1, 3, 7, 4.0, 4.0)));
        let next = safe_zone(1, 3, 8, 4.25, 4.0);

        assert!(!current.should_update_target(&next));
    }

    #[test]
    fn moved_target_from_same_plan_is_sent() {
        let current = CurrentSafeZone(Some(safe_zone(1, 3, 7, 4.0, 4.0)));
        let next = safe_zone(1, 3, 8, 5.0, 4.0);

        assert!(current.should_update_target(&next));
    }

    #[test]
    fn same_target_from_new_plan_is_sent() {
        let current = CurrentSafeZone(Some(safe_zone(1, 3, 7, 4.0, 4.0)));
        let next = safe_zone(1, 4, 0, 4.0, 4.0);

        assert!(current.should_update_target(&next));
    }

    #[test]
    fn same_target_from_new_destination_session_is_sent() {
        let current = CurrentSafeZone(Some(safe_zone(1, 3, 7, 4.0, 4.0)));
        let next = safe_zone(2, 3, 0, 4.0, 4.0);

        assert!(current.should_update_target(&next));
    }

    #[test]
    fn target_update_sent_when_waypoint_advances_even_if_nearby() {
        let mut current_sz = safe_zone(1, 0, 0, 0.0, 2.0);
        current_sz.target_waypoint = vec![0].try_into().unwrap();
        let current = CurrentSafeZone(Some(current_sz));

        // Next waypoint is only 0.5m away (distancesq = 0.25 < 0.5)
        let mut next_sz = safe_zone(1, 0, 1, 0.0, 1.5);
        next_sz.target_waypoint = vec![1].try_into().unwrap();

        assert!(current.should_update_target(&next_sz));
    }

    #[test]
    fn target_update_sent_when_orientation_changes() {
        use ros_env::rmf_prototype_msgs::msg::TargetOrientation;

        let mut current_sz = safe_zone(1, 0, 0, 0.0, 1.5);
        let mut ori1 = TargetOrientation::default();
        ori1.orientation_radians = 0.0;
        current_sz.incremental_target.regions[0]
            .orientations
            .push(ori1);
        let current = CurrentSafeZone(Some(current_sz));

        // Same position, but orientation changes to -pi/2
        let mut next_sz = safe_zone(1, 0, 1, 0.0, 1.5);
        let mut ori2 = TargetOrientation::default();
        ori2.orientation_radians = -std::f32::consts::FRAC_PI_2;
        next_sz.incremental_target.regions[0]
            .orientations
            .push(ori2);

        assert!(current.should_update_target(&next_sz));
    }

    #[test]
    fn build_progress_status_populates_optional_fields() {
        let mut pose = PoseStamped::default();
        pose.pose.position.x = 2.5;
        pose.pose.position.y = -1.5;

        let idle_status = build_progress_status(pose.clone(), Progress::MODE_IDLE, None, None);
        assert_eq!(idle_status.mode, Progress::MODE_IDLE);
        assert_eq!(idle_status.progress, 0.0);
        assert!(idle_status.safe_zone_id.is_empty());

        let sz = safe_zone(7, 2, 5, 3.0, 4.0);
        let prog = Progress {
            progress: 1.5,
            reached_waypoint: 1,
            target_waypoint: 2,
            reached_keys: vec![],
            plan_id: sz.id.plan_id.clone(),
            ..Default::default()
        };
        let nav_status =
            build_progress_status(pose, Progress::MODE_NAVIGATING, Some(&prog), Some(&sz.id));
        assert_eq!(nav_status.mode, Progress::MODE_NAVIGATING);
        assert_eq!(nav_status.reached_waypoint, 1);
        assert_eq!(nav_status.target_waypoint, 2);
        assert_eq!(nav_status.safe_zone_id.len(), 1);
        assert_eq!(nav_status.safe_zone_id[0].safe_zone_version, 5);
    }

    #[test]
    fn progress_tracker_interpolates_and_reaches_target() {
        let mut tracker = ProgressTracker::default();
        let mut sz = safe_zone(1, 0, 1, 4.0, 0.0);
        sz.last_waypoint = 1;
        sz.target_waypoint = vec![4].try_into().unwrap();
        sz.target_progress = 4.0;
        sz.incremental_target.regions[0].tolerance = 0.25;

        // Initial SafeZone at robot position (1.0, 0.0, 0.0)
        let p0 = tracker.on_safe_zone(&sz, Some((1.0, 0.0, 0.0)), None);
        assert_eq!(p0.reached_waypoint, 1);
        assert_eq!(p0.target_waypoint, 4);
        assert!((p0.progress - 1.0).abs() < 1e-4);

        // Midway at (2.5, 0.0, 0.0) -> progress = 2.5, intermediate reached_waypoint = 2
        let p_mid = tracker.compute_progress(Some((2.5, 0.0, 0.0)), Some(&p0));
        assert_eq!(p_mid.reached_waypoint, 2);
        assert!((p_mid.progress - 2.5).abs() < 1e-4);

        // Within tolerance at (3.85, 0.0, 0.0) -> target reached (wp 4, progress 4.0)
        let p_done = tracker.compute_progress(Some((3.85, 0.0, 0.0)), Some(&p_mid));
        assert_eq!(p_done.reached_waypoint, 4);
        assert!((p_done.progress - 4.0).abs() < 1e-4);

        // Subsequent SafeZone for same plan with older last_waypoint must not regress progress
        let p_next_sz = tracker.on_safe_zone(&sz, Some((3.85, 0.0, 0.0)), Some(&p_done));
        assert_eq!(p_next_sz.reached_waypoint, 4);
        assert!((p_next_sz.progress - 4.0).abs() < 1e-4);
    }

    #[test]
    fn progress_tracker_does_not_overreport_when_target_has_duplicate_hold_waypoints() {
        use ros_env::rmf_prototype_msgs::msg::Waypoint;

        let mut tracker = ProgressTracker::default();
        let mut sz = safe_zone(1, 0, 1, 5.0, 0.0);
        sz.last_waypoint = 0;
        sz.target_waypoint = vec![6].try_into().unwrap();
        sz.target_progress = 6.0;
        sz.incremental_target.regions[0].tolerance = 0.25;

        // Plan has 1m steps 0..=5 from (0,0) to (5,0), plus a duplicate hold waypoint 6 at (5,0).
        let mut plan = Plan::default();
        plan.plan_id = sz.id.plan_id.clone();
        for (i, x) in [0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 5.0].into_iter().enumerate() {
            plan.waypoints.push(Waypoint {
                position: [x, 0.0],
                progress: i as f32,
                ..Default::default()
            });
        }

        let p0 = tracker.on_safe_zone_with_plan(&sz, Some(&plan), Some((0.0, 0.0, 0.0)), None);
        assert_eq!(p0.reached_waypoint, 0);

        let p_at_1_98 = tracker.compute_progress(Some((1.98, 0.0, 0.0)), Some(&p0));
        assert_eq!(p_at_1_98.reached_waypoint, 1);
        assert!((p_at_1_98.progress - 1.98).abs() < 1e-4);

        // When PlanExecutor echoes reached_waypoint=1 back in safe_zone.last_waypoint
        // while target_waypoint=6 is unchanged, on_safe_zone must not double-count
        // start_waypoint + segment progress from the original start_xy=(0,0).
        sz.id.safe_zone_version = 2;
        sz.last_waypoint = 1;
        let p_at_1_10 = tracker.on_safe_zone_with_plan(
            &sz,
            Some(&plan),
            Some((1.10, 0.0, 0.0)),
            Some(&p_at_1_98),
        );
        assert_eq!(p_at_1_10.reached_waypoint, 1);
        assert!((p_at_1_10.progress - 1.98).abs() < 1e-4);
    }

    #[test]
    fn progress_tracker_handles_non_unit_waypoint_spacing() {
        use ros_env::rmf_prototype_msgs::msg::Waypoint;

        // 1. SafeZone-only fallback with waypoints spaced 2m apart at X = 0, 2, 4, 6
        // (start_waypoint = 0, target_waypoint = 3, target_progress = 3.0).
        // A robot at X = 2.0 (or 2.1) has only reached waypoint 1, NOT waypoint 2.
        let mut fallback_tracker = ProgressTracker::default();
        let mut sz = safe_zone(1, 0, 1, 6.0, 0.0);
        sz.last_waypoint = 0;
        sz.target_waypoint = vec![3].try_into().unwrap();
        sz.target_progress = 3.0;
        sz.incremental_target.regions[0].tolerance = 0.25;

        let p0 = fallback_tracker.on_safe_zone(&sz, Some((0.0, 0.0, 0.0)), None);
        assert_eq!(p0.reached_waypoint, 0);

        let p_at_2 = fallback_tracker.compute_progress(Some((2.0, 0.0, 0.0)), Some(&p0));
        assert_eq!(p_at_2.reached_waypoint, 1);
        assert!((p_at_2.progress - 1.0).abs() < 1e-4);

        let p_at_3_9 = fallback_tracker.compute_progress(Some((3.9, 0.0, 0.0)), Some(&p_at_2));
        assert_eq!(p_at_3_9.reached_waypoint, 1);
        assert!((p_at_3_9.progress - 1.95).abs() < 1e-4);

        let p_at_4 = fallback_tracker.compute_progress(Some((4.0, 0.0, 0.0)), Some(&p_at_3_9));
        assert_eq!(p_at_4.reached_waypoint, 2);
        assert!((p_at_4.progress - 2.0).abs() < 1e-4);

        // 2. Plan-backed tracking with non-unit spacing (2m) AND non-unit timestep (2.0s/wp:
        // progress = [0.0, 2.0, 4.0, 6.0] for indices [0, 1, 2, 3]).
        // At X = 2.0, progress is 2.0 while reached_waypoint index is still 1 (not 2!).
        let mut plan_tracker = ProgressTracker::default();
        let mut sz2 = safe_zone(2, 0, 1, 6.0, 0.0);
        sz2.last_waypoint = 0;
        sz2.target_waypoint = vec![3].try_into().unwrap();
        sz2.target_progress = 6.0;
        sz2.incremental_target.regions[0].tolerance = 0.25;

        let mut plan = Plan::default();
        plan.plan_id = sz2.id.plan_id.clone();
        for (i, x) in [0.0, 2.0, 4.0, 6.0].into_iter().enumerate() {
            plan.waypoints.push(Waypoint {
                position: [x, 0.0],
                progress: (i * 2) as f32,
                ..Default::default()
            });
        }

        let pp0 =
            plan_tracker.on_safe_zone_with_plan(&sz2, Some(&plan), Some((0.0, 0.0, 0.0)), None);
        let pp_at_2 = plan_tracker.compute_progress(Some((2.0, 0.0, 0.0)), Some(&pp0));
        assert_eq!(pp_at_2.reached_waypoint, 1);
        assert!((pp_at_2.progress - 2.0).abs() < 1e-4);

        let pp_at_3 = plan_tracker.compute_progress(Some((3.0, 0.0, 0.0)), Some(&pp_at_2));
        assert_eq!(pp_at_3.reached_waypoint, 1);
        assert!((pp_at_3.progress - 3.0).abs() < 1e-4);
    }

    #[test]
    fn progress_tracker_gates_final_progress_on_target_orientation() {
        let mut tracker = ProgressTracker::default();
        let mut sz = safe_zone(1, 0, 1, 3.0, 0.0);
        sz.last_waypoint = 0;
        sz.target_waypoint = vec![4].try_into().unwrap();
        sz.target_progress = 4.0;
        sz.incremental_target.regions[0].tolerance = 0.25;
        sz.incremental_target.regions[0].orientations = vec![TargetOrientation {
            orientation_radians: std::f32::consts::FRAC_PI_2,
            spread_radians: 0.0,
            tolerance_radians: 0.25,
        }];

        let p0 = tracker.on_safe_zone(&sz, Some((0.0, 0.0, 0.0)), None);
        assert_eq!(p0.reached_waypoint, 0);

        // Robot arrives at (3.0, 0.0) with yaw = 0.0 (still needs to rotate to PI/2)
        let p_pos_only = tracker.compute_progress(Some((3.0, 0.0, 0.0)), Some(&p0));
        assert_eq!(p_pos_only.reached_waypoint, 3);
        assert!(
            p_pos_only.progress < 4.0,
            "Final progress must not reach 4.0 before target orientation is achieved, got {}",
            p_pos_only.progress
        );

        // Robot finishes rotating within tolerance of PI/2 (e.g. 1.45 rad)
        let p_oriented = tracker.compute_progress(Some((3.0, 0.0, 1.45)), Some(&p_pos_only));
        assert_eq!(p_oriented.reached_waypoint, 4);
        assert!((p_oriented.progress - 4.0).abs() < 1e-4);
    }

    #[test]
    fn progress_tracker_in_place_rotation_does_not_complete_immediately() {
        let mut tracker = ProgressTracker::default();
        let mut sz = safe_zone(1, 0, 1, 0.0, 0.0);
        sz.last_waypoint = 0;
        sz.target_waypoint = vec![1].try_into().unwrap();
        sz.target_progress = 1.0;
        sz.incremental_target.regions[0].tolerance = 0.25;
        sz.incremental_target.regions[0].orientations = vec![TargetOrientation {
            orientation_radians: std::f32::consts::PI,
            spread_radians: 0.0,
            tolerance_radians: 0.25,
        }];

        // Already at (0, 0) but yaw = 0.0 != PI
        let p0 = tracker.on_safe_zone(&sz, Some((0.0, 0.0, 0.0)), None);
        assert_eq!(p0.reached_waypoint, 0);
        assert!((p0.progress - 0.0).abs() < 1e-4);

        // Halfway through rotation (yaw = PI/2) -> still not complete
        let p_half =
            tracker.compute_progress(Some((0.0, 0.0, std::f32::consts::FRAC_PI_2)), Some(&p0));
        assert_eq!(p_half.reached_waypoint, 0);
        assert!(p_half.progress < 1.0);

        // Reaches yaw = 3.0 (within 0.25 rad of PI) -> now complete!
        let p_done = tracker.compute_progress(Some((0.0, 0.0, 3.0)), Some(&p_half));
        assert_eq!(p_done.reached_waypoint, 1);
        assert!((p_done.progress - 1.0).abs() < 1e-4);
    }

    #[test]
    fn progress_tracker_unconstrained_orientation_completes_on_position() {
        let mut tracker = ProgressTracker::default();
        let mut sz = safe_zone(1, 0, 1, 1.0, 0.0);
        sz.last_waypoint = 0;
        sz.target_waypoint = vec![1].try_into().unwrap();
        sz.target_progress = 1.0;
        sz.incremental_target.regions[0].tolerance = 0.25;
        sz.incremental_target.regions[0].orientations = vec![TargetOrientation {
            orientation_radians: 0.0,
            spread_radians: 0.0,
            tolerance_radians: std::f32::consts::PI,
        }];

        let p0 = tracker.on_safe_zone(&sz, Some((0.0, 0.0, 0.0)), None);
        // Reaching (1.0, 0.0) even with opposite yaw = PI completes immediately when unconstrained
        let p_done = tracker.compute_progress(Some((1.0, 0.0, std::f32::consts::PI)), Some(&p0));
        assert_eq!(p_done.reached_waypoint, 1);
        assert!((p_done.progress - 1.0).abs() < 1e-4);
    }
}
