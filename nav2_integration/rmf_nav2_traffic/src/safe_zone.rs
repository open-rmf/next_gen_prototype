use crate::{
    agent::AmclPose,
    inner_navigation_client::{
        CancellingInnerNavigation, InnerNavigationAction, InnerNavigationTarget,
    },
    Nav2Agent,
};
use bevy::prelude::*;
use bevy_ros2::{RclrsNode, RosPublisher, RosSubscription};
use ros_env::{
    builtin_interfaces::msg::Time as RosTime,
    geometry_msgs::msg::PoseStamped,
    nav2_msgs::msg::Costmap,
    rmf_prototype_msgs::msg::{
        Plan, PlanError, Progress, Region, RobotStatus, SafeZone, SafeZoneId,
    },
    std_msgs::msg::{Empty as RosEmpty, Header},
};
use std::{
    sync::Arc,
    time::{Duration, Instant},
};

#[derive(Component)]
pub struct SafeZoneSubscription {
    pub subscriber: Arc<RosSubscription<SafeZone>>,
}

#[derive(Component)]
pub struct PlanSubscription {
    pub subscriber: Arc<RosSubscription<Plan>>,
}

#[derive(Component, Debug, Clone, Default, Deref)]
pub struct CurrentPlan(pub Option<Plan>);

#[derive(Component)]
pub struct GateReleaseSubscription {
    pub subscriber: Arc<RosSubscription<RosEmpty>>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct KnownDock {
    pub id: String,
    pub dock_type: String,
    pub x: f32,
    pub y: f32,
    pub yaw: f32,
    pub staging_x: f32,
    pub staging_y: f32,
}

impl KnownDock {
    pub fn corridor_docks() -> Vec<Self> {
        use std::f32::consts::FRAC_PI_2;
        vec![
            Self {
                id: "dock_north_1".to_string(),
                dock_type: "simple_charging_dock".to_string(),
                x: -1.0,
                y: 2.0,
                yaw: FRAC_PI_2,
                staging_x: -1.0,
                staging_y: 1.0,
            },
            Self {
                id: "dock_north_2".to_string(),
                dock_type: "simple_charging_dock".to_string(),
                x: 1.0,
                y: 2.0,
                yaw: FRAC_PI_2,
                staging_x: 1.0,
                staging_y: 1.0,
            },
            Self {
                id: "dock_north_3".to_string(),
                dock_type: "simple_charging_dock".to_string(),
                x: 3.0,
                y: 2.0,
                yaw: FRAC_PI_2,
                staging_x: 3.0,
                staging_y: 1.0,
            },
            Self {
                id: "dock_south_1".to_string(),
                dock_type: "simple_charging_dock".to_string(),
                x: -1.0,
                y: -2.0,
                yaw: -FRAC_PI_2,
                staging_x: -1.0,
                staging_y: -1.0,
            },
            Self {
                id: "dock_south_2".to_string(),
                dock_type: "simple_charging_dock".to_string(),
                x: 1.0,
                y: -2.0,
                yaw: -FRAC_PI_2,
                staging_x: 1.0,
                staging_y: -1.0,
            },
            Self {
                id: "dock_south_3".to_string(),
                dock_type: "simple_charging_dock".to_string(),
                x: 3.0,
                y: -2.0,
                yaw: -FRAC_PI_2,
                staging_x: 3.0,
                staging_y: -1.0,
            },
        ]
    }

    pub fn match_by_id(id: &str) -> Option<Self> {
        Self::corridor_docks()
            .into_iter()
            .find(|d| d.id.eq_ignore_ascii_case(id))
    }

    pub fn match_by_pose(x: f32, y: f32, tolerance: f32) -> Option<Self> {
        Self::corridor_docks()
            .into_iter()
            .find(|d| (d.x - x).hypot(d.y - y) <= tolerance)
    }
}

#[derive(Component)]
pub struct CostmapPublisher {
    pub publisher: Arc<RosPublisher<Costmap>>,
}

#[derive(Component)]
pub struct CostmapUpdatedSubscription {
    pub subscriber: Arc<RosSubscription<Header>>,
}

#[derive(Clone, Debug)]
pub struct PendingNavigationTarget {
    pub required_seq: u64,
    pub target: InnerNavigationTarget,
    pub target_xy_yaw: (f32, f32, f32),
    pub costmap: Costmap,
    pub last_published: Instant,
}

#[derive(Component, Debug, Clone, Default)]
pub struct CostmapHandshakeState {
    pub next_seq: u64,
    pub acked_seq: u64,
    pub pending_target: Option<PendingNavigationTarget>,
}

pub fn seq_to_stamp(seq: u64) -> RosTime {
    RosTime {
        sec: (seq >> 32) as i32,
        nanosec: (seq & 0xFFFF_FFFF) as u32,
    }
}

pub fn stamp_to_seq(stamp: &RosTime) -> u64 {
    ((stamp.sec as u32 as u64) << 32) | (stamp.nanosec as u64)
}

#[derive(Component)]
pub struct ProgressPublisher {
    pub publisher: Arc<RosPublisher<Progress>>,
}

#[derive(Component)]
pub struct RobotStatusPublisher {
    pub publisher: Arc<RosPublisher<RobotStatus>>,
}

#[derive(Component, Debug, Clone, Default, Deref)]
pub struct LatestProgress(pub Option<Progress>);

pub fn shortest_angular_distance(a: f32, b: f32) -> f32 {
    let diff = (a - b).abs().rem_euclid(std::f32::consts::TAU);
    diff.min(std::f32::consts::TAU - diff)
}

#[derive(Component, Debug, Clone, Default)]
pub struct ProgressTracker {
    pub plan_id: Option<ros_env::rmf_prototype_msgs::msg::PlanId>,
    pub start_waypoint: u64,
    pub target_waypoint: u64,
    pub target_progress: f32,
    pub start_xy: (f32, f32),
    pub target_xy: (f32, f32),
    pub target_tolerance: f32,
    pub target_yaw: Option<f32>,
    pub target_yaw_tolerance: f32,
    pub active_action: InnerNavigationAction,
    pub action_completed: bool,
    pub gate_active: bool,
}

impl ProgressTracker {
    fn parse_action_string(
        action_str: &str,
        target_xy: (f32, f32),
    ) -> Option<(InnerNavigationAction, bool)> {
        let trimmed = action_str.trim();
        if trimmed.is_empty() {
            return None;
        }
        let lower = trimmed.to_ascii_lowercase();
        let has_gate = lower.contains("gate")
            || lower.contains("load")
            || lower.contains("offload")
            || lower.contains("off-load");

        if lower.starts_with("undock") {
            let dock_type = trimmed
                .split_once(':')
                .and_then(|(_, rest)| KnownDock::match_by_id(rest.trim()))
                .map(|d| d.dock_type)
                .unwrap_or_else(|| "simple_charging_dock".to_string());
            return Some((InnerNavigationAction::UndockRobot { dock_type }, has_gate));
        }

        if lower.starts_with("dock") {
            let explicit_id = trimmed.split_once(':').map(|(_, rest)| rest.trim());
            if let Some(dock_id) = explicit_id.filter(|s| !s.is_empty()) {
                let dock_type = KnownDock::match_by_id(dock_id)
                    .map(|d| d.dock_type)
                    .unwrap_or_else(|| "simple_charging_dock".to_string());
                return Some((
                    InnerNavigationAction::DockRobot {
                        use_dock_id: KnownDock::match_by_id(dock_id).is_some(),
                        dock_id: dock_id.to_string(),
                        dock_type,
                        navigate_to_staging_pose: false,
                    },
                    has_gate,
                ));
            }
            if let Some(dock) = KnownDock::match_by_pose(target_xy.0, target_xy.1, 0.45) {
                return Some((
                    InnerNavigationAction::DockRobot {
                        use_dock_id: true,
                        dock_id: dock.id,
                        dock_type: dock.dock_type,
                        navigate_to_staging_pose: false,
                    },
                    has_gate,
                ));
            }
            return Some((
                InnerNavigationAction::DockRobot {
                    use_dock_id: false,
                    dock_id: String::new(),
                    dock_type: "simple_charging_dock".to_string(),
                    navigate_to_staging_pose: false,
                },
                has_gate,
            ));
        }

        if has_gate {
            return Some((InnerNavigationAction::NavigateToPose, true));
        }

        None
    }

    pub fn resolve_target_action(
        safe_zone: &SafeZone,
        plan: Option<&Plan>,
        start_xy: (f32, f32),
        target_xy: (f32, f32),
    ) -> (InnerNavigationAction, bool) {
        let target_wp_idx = safe_zone.target_waypoint.first().copied().unwrap_or(0) as usize;
        let last_wp_idx = safe_zone.last_waypoint as usize;

        if let Some(plan) = plan.filter(|p| p.plan_id == safe_zone.id.plan_id) {
            if let Some(wp) = plan.waypoints.get(target_wp_idx) {
                if let Some(res) = Self::parse_action_string(&wp.arrival_action, target_xy) {
                    return res;
                }
                for node in &wp.arrival_constraints.nodes {
                    if let Some(name_seq) = node.key.name.first() {
                        let name_str = name_seq.to_string();
                        if let Some(dock) = KnownDock::match_by_id(&name_str) {
                            return (
                                InnerNavigationAction::DockRobot {
                                    use_dock_id: true,
                                    dock_id: dock.id,
                                    dock_type: dock.dock_type,
                                    navigate_to_staging_pose: false,
                                },
                                false,
                            );
                        }
                    }
                }
            }
            if let Some(prev_wp) = plan.waypoints.get(last_wp_idx) {
                if let Some(res) = Self::parse_action_string(&prev_wp.departure_action, target_xy) {
                    return res;
                }
            }
            if target_wp_idx > 0 {
                if let Some(prev_wp) = plan.waypoints.get(target_wp_idx - 1) {
                    if let Some(res) =
                        Self::parse_action_string(&prev_wp.departure_action, target_xy)
                    {
                        return res;
                    }
                }
            }
        }

        for node in &safe_zone.incremental_target.nodes {
            if let Some(name_seq) = node.key.name.first() {
                let name_str = name_seq.to_string();
                if let Some(dock) = KnownDock::match_by_id(&name_str) {
                    return (
                        InnerNavigationAction::DockRobot {
                            use_dock_id: true,
                            dock_id: dock.id,
                            dock_type: dock.dock_type,
                            navigate_to_staging_pose: false,
                        },
                        false,
                    );
                }
            }
        }

        if let Some(dock) = KnownDock::match_by_pose(target_xy.0, target_xy.1, 0.35) {
            let dist_to_dock = (start_xy.0 - dock.x).hypot(start_xy.1 - dock.y);
            if dist_to_dock > 0.35 {
                let dist_to_staging =
                    (start_xy.0 - dock.staging_x).hypot(start_xy.1 - dock.staging_y);
                let navigate_to_staging_pose = dist_to_staging > 0.45;
                return (
                    InnerNavigationAction::DockRobot {
                        use_dock_id: true,
                        dock_id: dock.id,
                        dock_type: dock.dock_type,
                        navigate_to_staging_pose,
                    },
                    false,
                );
            }
        }

        (InnerNavigationAction::NavigateToPose, false)
    }

    pub fn is_holding_at_pre_waypoint(&self) -> bool {
        let waiting_for_dock_action = matches!(
            self.active_action,
            InnerNavigationAction::DockRobot { .. } | InnerNavigationAction::UndockRobot { .. }
        ) && !self.action_completed;
        waiting_for_dock_action || self.gate_active
    }

    pub fn set_gate(&mut self, gated: bool) {
        self.gate_active = gated;
    }

    pub fn release_gate(&mut self) {
        self.gate_active = false;
    }

    pub fn mark_action_completed(&mut self) {
        self.action_completed = true;
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
        if matches!(
            self.active_action,
            InnerNavigationAction::DockRobot { .. } | InnerNavigationAction::UndockRobot { .. }
        ) {
            return self.action_completed;
        }
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
            let (action, has_gate) =
                Self::resolve_target_action(safe_zone, plan, fallback_start, target_xy);
            self.plan_id = Some(plan_id);
            self.start_waypoint = safe_zone.last_waypoint;
            self.target_waypoint = target_wp;
            self.target_progress = safe_zone.target_progress;
            self.start_xy = fallback_start;
            self.target_xy = target_xy;
            self.target_tolerance = target_tolerance;
            self.target_yaw = target_yaw;
            self.target_yaw_tolerance = target_yaw_tolerance;
            self.active_action = action;
            self.action_completed = false;
            self.gate_active = has_gate;
        } else {
            let yaw_changed = match (self.target_yaw, target_yaw) {
                (Some(a), Some(b)) => shortest_angular_distance(a, b) > 0.05,
                (None, None) => false,
                _ => true,
            };
            let target_changed = target_wp != self.target_waypoint
                || (target_xy.0 - self.target_xy.0).hypot(target_xy.1 - self.target_xy.1) > 1e-3
                || yaw_changed;
            if target_changed {
                let new_start_xy = current_xy.unwrap_or(self.target_xy);
                let (action, has_gate) =
                    Self::resolve_target_action(safe_zone, plan, new_start_xy, target_xy);
                self.start_xy = new_start_xy;
                self.start_waypoint = self.start_waypoint.max(safe_zone.last_waypoint);
                self.active_action = action;
                self.action_completed = false;
                self.gate_active = has_gate;
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

        let mut reached_waypoint = self.start_waypoint.max(prev_reached);
        let mut progress = (reached_waypoint as f32).max(prev_progress);

        // Hold progress at the pre-dock / pre-gate waypoint while a docking action
        // is in progress or while an explicit progress gate is active.
        if self.is_holding_at_pre_waypoint() {
            return Progress {
                progress,
                reached_waypoint,
                target_waypoint: self.target_waypoint,
                reached_keys: vec![],
                plan_id,
            };
        }

        if matches!(
            self.active_action,
            InnerNavigationAction::DockRobot { .. } | InnerNavigationAction::UndockRobot { .. }
        ) && self.action_completed
        {
            return Progress {
                progress: progress.max(self.target_progress),
                reached_waypoint: reached_waypoint.max(self.target_waypoint),
                target_waypoint: self.target_waypoint,
                reached_keys: vec![],
                plan_id,
            };
        }

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
                let start_prog = self.start_waypoint as f32;
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
                    let seg_len = seg_len_sq.sqrt();
                    let interp = (start_prog + seg_len)
                        .max(pre_rot_wp as f32)
                        .min(pre_final_cap);
                    progress = progress.max(interp).min(self.target_progress);
                } else if seg_len_sq > 1e-6 {
                    let seg_len = seg_len_sq.sqrt();
                    let t = (((rx - sx) * dx + (ry - sy) * dy) / seg_len_sq).clamp(0.0, 1.0);
                    let dist_along_seg = t * seg_len;
                    let wp_span = (self.target_progress - start_prog).max(0.0);
                    let interp = (start_prog + dist_along_seg.min(wp_span)).min(pre_final_cap);
                    progress = progress.max(interp).min(self.target_progress);
                    if self.target_waypoint > self.start_waypoint + 1 {
                        let intermediate_wp =
                            (interp.floor() as u64).min(self.target_waypoint.saturating_sub(1));
                        reached_waypoint = reached_waypoint.max(intermediate_wp);
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
        }
    }

    pub fn mark_target_reached(&mut self, previous: Option<&Progress>) -> Option<Progress> {
        self.action_completed = true;
        let plan_id = self.plan_id.clone()?;
        let prev_same_plan = previous.filter(|p| p.plan_id == plan_id);
        let prev_reached = prev_same_plan.map(|p| p.reached_waypoint).unwrap_or(0);
        let prev_progress = prev_same_plan.map(|p| p.progress).unwrap_or(0.0);

        if self.gate_active {
            let reached_waypoint = self.start_waypoint.max(prev_reached);
            let progress = (reached_waypoint as f32).max(prev_progress);
            return Some(Progress {
                progress,
                reached_waypoint,
                target_waypoint: self.target_waypoint,
                reached_keys: vec![],
                plan_id,
            });
        }

        Some(Progress {
            progress: prev_progress.max(self.target_progress),
            reached_waypoint: prev_reached.max(self.target_waypoint),
            target_waypoint: self.target_waypoint,
            reached_keys: vec![],
            plan_id,
        })
    }
}

pub fn build_robot_status(
    pose: PoseStamped,
    mode: u8,
    progress: Option<&Progress>,
    safe_zone_id: Option<&SafeZoneId>,
) -> RobotStatus {
    let progress_seq = match progress {
        Some(prog) => {
            let rmw_prog = <Progress as rosidl_runtime_rs::Message>::into_rmw_message(
                std::borrow::Cow::Borrowed(prog),
            )
            .into_owned();
            vec![rmw_prog].try_into().unwrap()
        }
        None => vec![].try_into().unwrap(),
    };

    let safe_zone_seq = match safe_zone_id {
        Some(sz_id) => {
            let rmw_sz = <SafeZoneId as rosidl_runtime_rs::Message>::into_rmw_message(
                std::borrow::Cow::Borrowed(sz_id),
            )
            .into_owned();
            vec![rmw_sz].try_into().unwrap()
        }
        None => vec![].try_into().unwrap(),
    };

    RobotStatus {
        pose,
        mode,
        progress: progress_seq,
        safe_zone_id: safe_zone_seq,
    }
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
        app.add_systems(PreUpdate, update_incremental_target)
            .add_observer(create_safe_zone_subscriber)
            .add_observer(create_plan_subscriber)
            .add_observer(create_gate_release_subscriber)
            .add_observer(create_costmap_publisher)
            .add_observer(create_progress_publisher)
            .add_observer(create_robot_status_publisher)
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
    let topic = agent_name + "/plan/safe_zone";
    let subscription = Arc::new(RosSubscription::<SafeZone>::new(&node, topic.clone()));
    commands.entity(e).insert((
        SafeZoneSubscription {
            subscriber: Arc::clone(&subscription),
        },
        CurrentSafeZone::default(),
    ));
}

fn create_plan_subscriber(
    trigger: Trigger<OnAdd, Nav2Agent>,
    mut commands: Commands,
    agents: Query<&Nav2Agent>,
    node: Res<RclrsNode>,
) {
    let e = trigger.target();
    let Ok(agent_name) = agents.get(e).map(|agent| agent.name.clone()) else {
        return;
    };
    let topic = agent_name + "/plan";
    let subscription = Arc::new(RosSubscription::<Plan>::new(&node, topic));
    commands.entity(e).insert((
        PlanSubscription {
            subscriber: Arc::clone(&subscription),
        },
        CurrentPlan::default(),
    ));
}

fn create_gate_release_subscriber(
    trigger: Trigger<OnAdd, Nav2Agent>,
    mut commands: Commands,
    agents: Query<&Nav2Agent>,
    node: Res<RclrsNode>,
) {
    let e = trigger.target();
    let Ok(agent_name) = agents.get(e).map(|agent| agent.name.clone()) else {
        return;
    };
    let topic = agent_name + "/plan/gate_release";
    let subscription = Arc::new(RosSubscription::<RosEmpty>::new(&node, topic));
    commands.entity(e).insert(GateReleaseSubscription {
        subscriber: Arc::clone(&subscription),
    });
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
    let topic = agent_name.clone() + "/inner/global_costmap/plan/costmap";
    let publisher = Arc::new(RosPublisher::<Costmap>::new_transient_local(&node, topic));
    let updated_topic = agent_name + "/inner/global_costmap/plan/costmap_updated";
    let updated_subscriber = Arc::new(RosSubscription::<Header>::new(&node, updated_topic));
    commands.entity(e).insert((
        CostmapPublisher {
            publisher: Arc::clone(&publisher),
        },
        CostmapUpdatedSubscription {
            subscriber: Arc::clone(&updated_subscriber),
        },
        CostmapHandshakeState::default(),
    ));
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

fn create_robot_status_publisher(
    trigger: Trigger<OnAdd, Nav2Agent>,
    mut commands: Commands,
    agents: Query<&Nav2Agent>,
    node: Res<RclrsNode>,
) {
    let e = trigger.target();
    let Ok(agent_name) = agents.get(e).map(|agent| agent.name.clone()) else {
        return;
    };
    let topic = agent_name + "/status";
    let publisher = Arc::new(RosPublisher::<RobotStatus>::new_transient_local(
        &node, topic,
    ));
    commands.entity(e).insert(RobotStatusPublisher {
        publisher: Arc::clone(&publisher),
    });
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

fn update_incremental_target(
    mut nav_target: EventWriter<InnerNavigationTarget>,
    mut subscriptions: Query<
        (
            Entity,
            &SafeZoneSubscription,
            &PlanSubscription,
            &GateReleaseSubscription,
            &CostmapPublisher,
            &CostmapUpdatedSubscription,
            &mut CostmapHandshakeState,
            &ProgressPublisher,
            &RobotStatusPublisher,
            &mut LatestProgress,
            &mut ProgressTracker,
            &mut CurrentSafeZone,
            &mut CurrentPlan,
            Option<&AmclPose>,
            &Nav2Agent,
        ),
        Without<CancellingInnerNavigation>,
    >,
) {
    for (
        e,
        safe_zone_sub,
        plan_sub,
        gate_release_sub,
        costmap_pub,
        costmap_updated_sub,
        mut handshake,
        progress_pub,
        status_pub,
        mut latest_progress,
        mut progress_tracker,
        mut current_safe_zone,
        mut current_plan,
        amcl_pose,
        agent,
    ) in subscriptions.iter_mut()
    {
        while let Some(plan) = plan_sub.subscriber.take_data() {
            current_plan.0 = Some(plan);
        }

        let mut gate_released = false;
        while gate_release_sub.subscriber.take_data().is_some() {
            gate_released = true;
        }
        if gate_released {
            progress_tracker.release_gate();
            if progress_tracker.plan_id.is_some() {
                let current_pose = amcl_pose.map(|p| p.xy_yaw());
                let updated_prog =
                    progress_tracker.compute_progress(current_pose, latest_progress.0.as_ref());
                if latest_progress.0.as_ref() != Some(&updated_prog) {
                    let _ = progress_pub.publisher.publish(updated_prog.clone());
                    latest_progress.0 = Some(updated_prog.clone());
                    let pose = amcl_pose
                        .map(|p| PoseStamped {
                            header: p.0.header.clone(),
                            pose: p.0.pose.pose.clone(),
                        })
                        .unwrap_or_default();
                    let sz_id = current_safe_zone.0.as_ref().map(|sz| &sz.id);
                    let status_msg = build_robot_status(
                        pose,
                        RobotStatus::MODE_NAVIGATING,
                        Some(&updated_prog),
                        sz_id,
                    );
                    let _ = status_pub.publisher.publish(status_msg);
                }
            }
        }

        while let Some(ack_header) = costmap_updated_sub.subscriber.take_data() {
            let seq = stamp_to_seq(&ack_header.stamp);
            if seq > handshake.acked_seq {
                handshake.acked_seq = seq;
            }
        }

        let mut latest_safe_zone = None;
        while let Some(safe_zone) = safe_zone_sub.subscriber.take_data() {
            latest_safe_zone = Some(safe_zone);
        }

        if let Some(safe_zone) = latest_safe_zone {
            if !current_safe_zone.matches(&safe_zone) && is_valid(&safe_zone) {
                if let Some((target_x, target_y, target_yaw)) = next_target(&safe_zone) {
                    handshake.next_seq += 1;
                    let seq = handshake.next_seq;
                    let mut costmap = safe_zone.costmap.clone();
                    costmap.header.stamp = seq_to_stamp(seq);

                    // Publish stamped costmap and wait for global_costmap ack before dispatching target
                    if costmap_pub.publisher.publish(costmap.clone()).is_err() {
                        error!("Failed to publish costmap for agent [{}]", agent.name);
                        continue;
                    }

                    let current_pose = amcl_pose.map(|p| p.xy_yaw());

                    // Publish progress when it changes
                    let progress_msg = progress_tracker.on_safe_zone_with_plan(
                        &safe_zone,
                        current_plan.0.as_ref(),
                        current_pose,
                        latest_progress.0.as_ref(),
                    );
                    let progress_changed = latest_progress.0.as_ref() != Some(&progress_msg);
                    if progress_changed {
                        let Ok(_) = progress_pub.publisher.publish(progress_msg.clone()) else {
                            error!("Failed to publish progress for agent [{}]", agent.name);
                            continue;
                        };
                        latest_progress.0 = Some(progress_msg.clone());
                    }

                    let mode = if progress_tracker.is_holding_at_pre_waypoint() {
                        RobotStatus::MODE_ACTION
                    } else {
                        RobotStatus::MODE_NAVIGATING
                    };

                    if !current_safe_zone.should_update_target(&safe_zone) {
                        if let Some(pending) = handshake.pending_target.as_mut() {
                            pending.costmap = costmap;
                        }
                        if progress_changed {
                            let pose = amcl_pose
                                .map(|p| PoseStamped {
                                    header: p.0.header.clone(),
                                    pose: p.0.pose.pose.clone(),
                                })
                                .unwrap_or_default();
                            let status_msg = build_robot_status(
                                pose,
                                mode,
                                Some(&progress_msg),
                                Some(&safe_zone.id),
                            );
                            let _ = status_pub.publisher.publish(status_msg);
                        }
                    } else {
                        let action = progress_tracker.active_action.clone();
                        info!(
                            "[{:?}] Queuing SafeZone target (waiting for costmap seq={}, action={:?}): ({:.2}, {:.2}, {:.2})",
                            agent.name, seq, action, target_x, target_y, target_yaw
                        );

                        let pose = amcl_pose
                            .map(|p| PoseStamped {
                                header: p.0.header.clone(),
                                pose: p.0.pose.pose.clone(),
                            })
                            .unwrap_or_default();
                        let status_msg = build_robot_status(
                            pose,
                            mode,
                            Some(&progress_msg),
                            Some(&safe_zone.id),
                        );
                        let _ = status_pub.publisher.publish(status_msg);

                        *current_safe_zone = CurrentSafeZone(Some(safe_zone.clone()));

                        handshake.pending_target = Some(PendingNavigationTarget {
                            required_seq: seq,
                            target: InnerNavigationTarget::with_action(
                                e,
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

        let acked_seq = handshake.acked_seq;
        if handshake
            .pending_target
            .as_ref()
            .is_some_and(|pending| acked_seq >= pending.required_seq)
        {
            let pending = handshake.pending_target.take().unwrap();
            let (tx, ty, tyaw) = pending.target_xy_yaw;
            info!(
                "[{:?}] Costmap (seq={}) applied, dispatching SafeZone target: ({:.2}, {:.2}, {:.2})",
                agent.name, pending.required_seq, tx, ty, tyaw
            );
            nav_target.write(pending.target);
        } else if let Some(pending) = handshake.pending_target.as_mut() {
            if pending.last_published.elapsed() >= Duration::from_millis(500) {
                let _ = costmap_pub.publisher.publish(pending.costmap.clone());
                pending.last_published = Instant::now();
            }
        }
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

fn next_target(safe_zone: &SafeZone) -> Option<(f32, f32, f32)> {
    // TODO(@xiyuoh) more sophisticated point selection taking into account
    // all factors (region hints, orientations, etc.)

    let constraints = &safe_zone.incremental_target;
    let mut xy: Option<(f32, f32)> = None;
    let mut yaw: Option<f32> = None;

    // Assume either regions or nodes will be populated, not both.
    for target_region in constraints.regions.iter() {
        let _tolerance = target_region.tolerance;
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
    fn build_robot_status_populates_optional_fields() {
        let mut pose = PoseStamped::default();
        pose.pose.position.x = 2.5;
        pose.pose.position.y = -1.5;

        let idle_status = build_robot_status(pose.clone(), RobotStatus::MODE_IDLE, None, None);
        assert_eq!(idle_status.mode, RobotStatus::MODE_IDLE);
        assert!(idle_status.progress.is_empty());
        assert!(idle_status.safe_zone_id.is_empty());

        let sz = safe_zone(7, 2, 5, 3.0, 4.0);
        let prog = Progress {
            progress: 1.5,
            reached_waypoint: 1,
            target_waypoint: 2,
            reached_keys: vec![],
            plan_id: sz.id.plan_id.clone(),
        };
        let nav_status = build_robot_status(
            pose,
            RobotStatus::MODE_NAVIGATING,
            Some(&prog),
            Some(&sz.id),
        );
        assert_eq!(nav_status.mode, RobotStatus::MODE_NAVIGATING);
        assert_eq!(nav_status.progress.len(), 1);
        assert_eq!(nav_status.progress[0].reached_waypoint, 1);
        assert_eq!(nav_status.progress[0].target_waypoint, 2);
        assert_eq!(nav_status.safe_zone_id.len(), 1);
        assert_eq!(nav_status.safe_zone_id[0].safe_zone_version, 5);
    }

    #[test]
    fn costmap_seq_stamp_roundtrip() {
        for seq in [0u64, 1, 42, u32::MAX as u64, (1u64 << 32) + 7, u64::MAX] {
            let stamp = seq_to_stamp(seq);
            assert_eq!(stamp_to_seq(&stamp), seq);
        }
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
        let mut tracker = ProgressTracker::default();
        let mut sz = safe_zone(1, 0, 1, 5.0, 0.0);
        sz.last_waypoint = 0;
        sz.target_waypoint = vec![6].try_into().unwrap();
        sz.target_progress = 6.0;
        sz.incremental_target.regions[0].tolerance = 0.25;

        let p0 = tracker.on_safe_zone(&sz, Some((0.0, 0.0, 0.0)), None);
        assert_eq!(p0.reached_waypoint, 0);

        let p_at_1_98 = tracker.compute_progress(Some((1.98, 0.0, 0.0)), Some(&p0));
        assert_eq!(p_at_1_98.reached_waypoint, 1);
        assert!((p_at_1_98.progress - 1.98).abs() < 1e-4);

        // When PlanExecutor echoes reached_waypoint=1 back in safe_zone.last_waypoint
        // while target_waypoint=6 is unchanged, on_safe_zone must not double-count
        // start_waypoint + dist_along_seg from the original start_xy=(0,0).
        sz.id.safe_zone_version = 2;
        sz.last_waypoint = 1;
        let p_at_1_10 = tracker.on_safe_zone(&sz, Some((1.10, 0.0, 0.0)), Some(&p_at_1_98));
        assert_eq!(p_at_1_10.reached_waypoint, 1);
        assert!((p_at_1_10.progress - 1.98).abs() < 1e-4);
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

    #[test]
    fn progress_tracker_holds_at_pre_dock_waypoint_during_docking() {
        let mut tracker = ProgressTracker::default();
        // SafeZone targets dock_north_1 at (-1.0, 2.0, pi/2) from pre-dock waypoint 1 at (-1.0, 1.0)
        let mut sz = safe_zone(1, 0, 1, -1.0, 2.0);
        sz.last_waypoint = 1;
        sz.target_waypoint = vec![2].try_into().unwrap();
        sz.target_progress = 2.0;
        sz.incremental_target.regions[0].tolerance = 0.25;
        sz.incremental_target.regions[0].orientations = vec![TargetOrientation {
            orientation_radians: std::f32::consts::FRAC_PI_2,
            spread_radians: 0.0,
            tolerance_radians: 0.25,
        }];

        let p0 = tracker.on_safe_zone(&sz, Some((-1.0, 1.0, std::f32::consts::FRAC_PI_2)), None);
        assert_eq!(
            tracker.active_action,
            InnerNavigationAction::DockRobot {
                use_dock_id: true,
                dock_id: "dock_north_1".to_string(),
                dock_type: "simple_charging_dock".to_string(),
                navigate_to_staging_pose: false,
            }
        );
        assert_eq!(p0.reached_waypoint, 1);
        assert!((p0.progress - 1.0).abs() < 1e-4);

        // Even while moving into the dock (-1.0, 1.8) or arriving at (-1.0, 2.0),
        // progress must remain held at pre-dock waypoint 1 (progress = 1.0) until DockRobot succeeds.
        let p_mid =
            tracker.compute_progress(Some((-1.0, 1.8, std::f32::consts::FRAC_PI_2)), Some(&p0));
        assert_eq!(p_mid.reached_waypoint, 1);
        assert!((p_mid.progress - 1.0).abs() < 1e-4);

        let p_at_dock =
            tracker.compute_progress(Some((-1.0, 2.0, std::f32::consts::FRAC_PI_2)), Some(&p_mid));
        assert_eq!(p_at_dock.reached_waypoint, 1);
        assert!((p_at_dock.progress - 1.0).abs() < 1e-4);

        // Once DockRobot action succeeds, progress advances to target_waypoint 2 (progress = 2.0).
        let p_completed = tracker.mark_target_reached(Some(&p_at_dock)).unwrap();
        assert_eq!(p_completed.reached_waypoint, 2);
        assert!((p_completed.progress - 2.0).abs() < 1e-4);
    }

    #[test]
    fn progress_tracker_plan_action_and_gate_hold_until_gate_released() {
        use ros_env::rmf_prototype_msgs::msg::Waypoint;

        let mut tracker = ProgressTracker::default();
        let mut sz = safe_zone(1, 0, 1, 2.0, 0.0);
        sz.last_waypoint = 1;
        sz.target_waypoint = vec![2].try_into().unwrap();
        sz.target_progress = 2.0;
        sz.incremental_target.regions[0].tolerance = 0.25;

        let mut plan = Plan::default();
        plan.plan_id = sz.id.plan_id.clone();
        let mut wp0 = Waypoint::default();
        wp0.position = [0.0, 0.0];
        wp0.progress = 0.0;
        let mut wp1 = Waypoint::default();
        wp1.position = [1.0, 0.0];
        wp1.progress = 1.0;
        let mut wp2 = Waypoint::default();
        wp2.position = [2.0, 0.0];
        wp2.progress = 2.0;
        wp2.arrival_action = "dock:dock_north_1+load".to_string();
        plan.waypoints = vec![wp0, wp1, wp2];

        let p0 = tracker.on_safe_zone_with_plan(&sz, Some(&plan), Some((1.0, 0.0, 0.0)), None);
        assert!(tracker.gate_active);
        assert!(tracker.is_holding_at_pre_waypoint());
        assert_eq!(p0.reached_waypoint, 1);
        assert!((p0.progress - 1.0).abs() < 1e-4);

        // DockRobot completes, but loading gate is still active -> progress stays held at waypoint 1
        let p_after_dock = tracker.mark_target_reached(Some(&p0)).unwrap();
        assert!(tracker.action_completed);
        assert!(tracker.gate_active);
        assert_eq!(p_after_dock.reached_waypoint, 1);
        assert!((p_after_dock.progress - 1.0).abs() < 1e-4);

        // External gate release triggers -> progress now advances to waypoint 2 (2.0)
        tracker.release_gate();
        assert!(!tracker.is_holding_at_pre_waypoint());
        let p_unblocked = tracker.compute_progress(Some((2.0, 0.0, 0.0)), Some(&p_after_dock));
        assert_eq!(p_unblocked.reached_waypoint, 2);
        assert!((p_unblocked.progress - 2.0).abs() < 1e-4);
    }
}
