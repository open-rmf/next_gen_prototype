use crate::{
    agent::AmclPose,
    inner_navigation_client::{
        InnerNavigationAction, InnerNavigationClient, InnerNavigationServices,
        InnerNavigationTarget,
    },
    navigation_server::{CancelledNavigationSessions, CurrentNavigationRequest},
    Nav2Agent,
};
use bevy::prelude::*;
use bevy_ros2::{RclrsNode, RosPublisher, RosSubscription};
use crossflow::prelude::*;
use ros_env::{
    builtin_interfaces::msg::Time as RosTime,
    geometry_msgs::msg::PoseStamped,
    nav2_msgs::msg::Costmap,
    rmf_prototype_msgs::msg::{
        Plan, PlanError, PlanId, Progress, Region, SafeZone, SafeZoneId, TargetNode,
    },
    std_msgs::msg::Header,
};
use std::{
    collections::{HashMap, VecDeque},
    path::Path,
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

#[derive(Component, Debug, Clone, Default)]
pub struct LocalPlanRegistry {
    entries: VecDeque<Plan>,
    latest_plan: Option<Plan>,
}

impl LocalPlanRegistry {
    const MAX_CACHED_PLANS: usize = 16;

    pub fn insert(&mut self, plan: Plan) {
        if let Some(existing) = self.entries.iter_mut().find(|p| p.plan_id == plan.plan_id) {
            *existing = plan.clone();
            self.update_latest(plan);
            return;
        }
        if self.entries.len() >= Self::MAX_CACHED_PLANS {
            self.entries.pop_front();
        }
        self.entries.push_back(plan.clone());
        self.update_latest(plan);
    }

    pub fn get(&self, plan_id: &PlanId) -> Option<&Plan> {
        self.entries.iter().find(|p| &p.plan_id == plan_id)
    }

    fn update_latest(&mut self, plan: Plan) {
        let should_replace = match &self.latest_plan {
            None => true,
            Some(cur) => {
                cur.plan_id.destination_session.uuid != plan.plan_id.destination_session.uuid
                    || plan.plan_id.plan_version >= cur.plan_id.plan_version
            }
        };
        if should_replace {
            self.latest_plan = Some(plan);
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct KnownDock {
    pub id: String,
    pub location_key: Option<i64>,
    pub anchor_key: Option<i64>,
    pub x: f32,
    pub y: f32,
    pub yaw: f32,
    pub staging_x: f32,
    pub staging_y: f32,
}

#[derive(Resource, Clone, Debug, Default, PartialEq)]
pub struct DockRegistry {
    pub docks: Vec<KnownDock>,
}

impl DockRegistry {
    pub fn new(docks: Vec<KnownDock>) -> Self {
        Self { docks }
    }

    pub fn from_site_file<P: AsRef<Path>>(path: P) -> Result<Self, String> {
        let content = std::fs::read_to_string(path.as_ref()).map_err(|e| {
            format!(
                "Failed to read site_file '{}': {e}",
                path.as_ref().display()
            )
        })?;
        Self::from_site_json_str(&content)
    }

    /// Derive dock endpoints, staging points, and approach orientations directly
    /// from an RMF `.site.json` navigation graph (`anchors`, `lanes`, `locations`).
    pub fn from_site_json_str(json_str: &str) -> Result<Self, String> {
        let root: serde_json::Value =
            serde_json::from_str(json_str).map_err(|e| format!("Invalid site JSON: {e}"))?;

        fn extract_anchor_xy(val: &serde_json::Value) -> Option<(f32, f32)> {
            if let Some(arr) = val.as_array() {
                if arr.len() >= 2 {
                    return Some((arr[0].as_f64()? as f32, arr[1].as_f64()? as f32));
                }
            }
            if let Some(t2d) = val.get("Translate2D").and_then(|v| v.as_array()) {
                if t2d.len() >= 2 {
                    return Some((t2d[0].as_f64()? as f32, t2d[1].as_f64()? as f32));
                }
            }
            None
        }

        let mut anchors: HashMap<String, (f32, f32)> = HashMap::new();
        if let Some(top_anchors) = root.get("anchors").and_then(|v| v.as_object()) {
            for (id, pose) in top_anchors {
                if let Some(xy) = extract_anchor_xy(pose) {
                    anchors.insert(id.clone(), xy);
                }
            }
        }
        if let Some(levels) = root.get("levels").and_then(|v| v.as_object()) {
            for level in levels.values() {
                if let Some(level_anchors) = level.get("anchors").and_then(|v| v.as_object()) {
                    for (id, pose) in level_anchors {
                        if let Some(xy) = extract_anchor_xy(pose) {
                            anchors.insert(id.clone(), xy);
                        }
                    }
                }
            }
        }

        let val_to_id = |v: &serde_json::Value| -> Option<String> {
            if let Some(s) = v.as_str() {
                Some(s.to_string())
            } else {
                v.as_u64()
                    .map(|n| n.to_string())
                    .or_else(|| v.as_i64().map(|n| n.to_string()))
            }
        };

        let mut neighbors: HashMap<String, Vec<(f32, f32)>> = HashMap::new();
        let mut docks: Vec<KnownDock> = Vec::new();

        let guided = root.get("navigation").and_then(|n| n.get("guided"));
        if let Some(lanes) = guided
            .and_then(|g| g.get("lanes"))
            .and_then(|l| l.as_object())
        {
            for lane_data in lanes.values() {
                let endpoints =
                    if let Some(arr) = lane_data.get("anchors").and_then(|v| v.as_array()) {
                        if arr.len() >= 2 {
                            val_to_id(&arr[0]).zip(val_to_id(&arr[1]))
                        } else {
                            None
                        }
                    } else {
                        lane_data
                            .get("left")
                            .and_then(val_to_id)
                            .zip(lane_data.get("right").and_then(val_to_id))
                    };

                let Some((left_id, right_id)) = endpoints else {
                    continue;
                };
                let (Some(&left_xy), Some(&right_xy)) =
                    (anchors.get(&left_id), anchors.get(&right_id))
                else {
                    continue;
                };

                neighbors.entry(left_id.clone()).or_default().push(right_xy);
                neighbors.entry(right_id.clone()).or_default().push(left_xy);
            }
        }

        if let Some(locations) = guided
            .and_then(|g| g.get("locations"))
            .and_then(|l| l.as_object())
        {
            for (loc_id, loc_data) in locations {
                let Some(anchor_id) = loc_data.get("anchor").and_then(val_to_id) else {
                    continue;
                };
                let Some(&(x, y)) = anchors.get(&anchor_id) else {
                    continue;
                };
                let default_name = format!("location_{loc_id}");
                let name = loc_data
                    .get("name")
                    .and_then(|v| v.as_str())
                    .unwrap_or(&default_name)
                    .to_string();

                let mut is_parking_spot = false;
                if let Some(tags) = loc_data.get("tags").and_then(|v| v.as_array()) {
                    for tag in tags {
                        if tag.as_str() == Some("ParkingSpot") || tag.get("ParkingSpot").is_some() {
                            is_parking_spot = true;
                        }
                    }
                }
                if is_parking_spot {
                    continue;
                }

                // A non-parking location on a degree-1 spur anchor in the navigation graph
                // represents a terminal dock alcove whose single connected neighbor is its staging point.
                let Some(anchor_neighbors) = neighbors.get(&anchor_id) else {
                    continue;
                };
                if anchor_neighbors.len() != 1 {
                    continue;
                }
                let (staging_x, staging_y) = anchor_neighbors[0];
                let yaw = (y - staging_y).atan2(x - staging_x);

                if docks
                    .iter()
                    .any(|d| d.id.eq_ignore_ascii_case(&name) || (d.x - x).hypot(d.y - y) < 1e-3)
                {
                    continue;
                }

                docks.push(KnownDock {
                    id: name,
                    location_key: loc_id.parse::<i64>().ok(),
                    anchor_key: anchor_id.parse::<i64>().ok(),
                    x,
                    y,
                    yaw,
                    staging_x,
                    staging_y,
                });
            }
        }

        Ok(Self { docks })
    }

    pub fn match_by_id(&self, id: &str) -> Option<KnownDock> {
        self.docks
            .iter()
            .find(|d| d.id.eq_ignore_ascii_case(id))
            .cloned()
    }

    pub fn match_target_node(&self, node: &TargetNode) -> Option<KnownDock> {
        if let Some(name_seq) = node.key.name.first() {
            let name_str = name_seq.to_string();
            if let Some(dock) = self.match_by_id(&name_str) {
                return Some(dock);
            }
        }
        if let Some(&key_id) = node.key.key.first() {
            if let Some(dock) = self
                .docks
                .iter()
                .find(|d| d.location_key == Some(key_id) || d.anchor_key == Some(key_id))
            {
                return Some(dock.clone());
            }
        }
        None
    }

    pub fn match_target_nodes(&self, nodes: &[TargetNode]) -> Option<KnownDock> {
        for node in nodes {
            if let Some(dock) = self.match_target_node(node) {
                return Some(dock);
            }
        }
        None
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
    pub last_published_costmap: Option<Costmap>,
}

pub fn costmap_content_eq(a: &Costmap, b: &Costmap) -> bool {
    a.header.frame_id == b.header.frame_id
        && a.metadata.layer == b.metadata.layer
        && a.metadata.resolution == b.metadata.resolution
        && a.metadata.size_x == b.metadata.size_x
        && a.metadata.size_y == b.metadata.size_y
        && a.metadata.origin == b.metadata.origin
        && a.data == b.data
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
    pub target_waypoint: u64,
    pub target_progress: f32,
    pub start_xy: (f32, f32),
    pub target_xy: (f32, f32),
    pub target_tolerance: f32,
    pub target_yaw: Option<f32>,
    pub target_yaw_tolerance: f32,
    pub active_action: InnerNavigationAction,
    pub action_completed: bool,
    pub dock_registry: DockRegistry,
    pub pending_dock: Option<KnownDock>,
    pub docked_at: Option<KnownDock>,
}

impl ProgressTracker {
    pub fn with_dock_registry(dock_registry: DockRegistry) -> Self {
        Self {
            dock_registry,
            ..Default::default()
        }
    }

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

    pub fn set_plan(&mut self, plan: &Plan, current_safe_zone: Option<&SafeZone>) {
        if self.plan_id.as_ref() == Some(&plan.plan_id) {
            self.plan_waypoints = Self::extract_plan_waypoints(plan);
            if let Some(sz) = current_safe_zone.filter(|sz| sz.id.plan_id == plan.plan_id) {
                let (action, target_dock) = Self::resolve_target_action_with_dock(
                    &self.dock_registry,
                    self.docked_at.as_ref(),
                    sz,
                    Some(plan),
                    self.start_xy,
                    self.target_xy,
                );
                if self.active_action == InnerNavigationAction::NavigateToPose
                    && action != InnerNavigationAction::NavigateToPose
                {
                    self.active_action = action;
                    self.pending_dock = target_dock;
                }
            }
        }
    }

    fn resolve_target_action_with_dock(
        dock_registry: &DockRegistry,
        docked_at: Option<&KnownDock>,
        safe_zone: &SafeZone,
        plan: Option<&Plan>,
        start_xy: (f32, f32),
        target_xy: (f32, f32),
    ) -> (InnerNavigationAction, Option<KnownDock>) {
        let target_wp_idx = safe_zone.target_waypoint.first().copied().unwrap_or(0) as usize;
        let last_wp_idx = safe_zone.last_waypoint as usize;

        let mut target_dock = dock_registry.match_target_nodes(&safe_zone.incremental_target.nodes);
        if target_dock.is_none() {
            if let Some(plan) = plan.filter(|p| p.plan_id == safe_zone.id.plan_id) {
                if let Some(wp) = plan.waypoints.get(target_wp_idx) {
                    target_dock = dock_registry.match_target_nodes(&wp.arrival_constraints.nodes);
                }
            }
        }

        if let Some(dock) = target_dock {
            let dist_to_target = (start_xy.0 - target_xy.0).hypot(start_xy.1 - target_xy.1);
            let navigate_to_staging_pose = dist_to_target > 1.35;
            return (
                InnerNavigationAction::DockRobot {
                    navigate_to_staging_pose,
                },
                Some(dock),
            );
        }

        let prev_wp_dock = plan
            .filter(|p| p.plan_id == safe_zone.id.plan_id)
            .and_then(|p| p.waypoints.get(last_wp_idx))
            .and_then(|wp| dock_registry.match_target_nodes(&wp.arrival_constraints.nodes));

        if let Some(dock) = prev_wp_dock.as_ref().or(docked_at) {
            let dist_to_dock = (start_xy.0 - dock.x).hypot(start_xy.1 - dock.y);
            let dist_to_staging =
                (target_xy.0 - dock.staging_x).hypot(target_xy.1 - dock.staging_y);
            if dist_to_dock <= 0.45 && dist_to_staging <= 0.45 {
                let (target_yaw, _) = Self::extract_orientation_constraint(safe_zone);
                if target_yaw.is_none_or(|yaw| shortest_angular_distance(yaw, dock.yaw) <= 0.5) {
                    return (InnerNavigationAction::UndockRobot, None);
                }
            }
        }

        (InnerNavigationAction::NavigateToPose, None)
    }

    pub fn resolve_target_action(
        dock_registry: &DockRegistry,
        safe_zone: &SafeZone,
        plan: Option<&Plan>,
        start_xy: (f32, f32),
        target_xy: (f32, f32),
    ) -> InnerNavigationAction {
        Self::resolve_target_action_with_dock(
            dock_registry,
            None,
            safe_zone,
            plan,
            start_xy,
            target_xy,
        )
        .0
    }

    pub fn is_holding_at_pre_waypoint(&self) -> bool {
        matches!(
            self.active_action,
            InnerNavigationAction::DockRobot { .. } | InnerNavigationAction::UndockRobot
        ) && !self.action_completed
    }

    /// Determine the agent's current `Progress.mode` without coupling callers to
    /// action-specific hold details.
    pub fn execution_mode(&self, is_actively_navigating: bool) -> u8 {
        if self.is_holding_at_pre_waypoint() {
            Progress::MODE_ACTION
        } else if is_actively_navigating {
            Progress::MODE_NAVIGATING
        } else {
            Progress::MODE_IDLE
        }
    }

    pub fn mark_action_completed(&mut self) {
        self.action_completed = true;
        if matches!(self.active_action, InnerNavigationAction::DockRobot { .. }) {
            if let Some(dock) = self.pending_dock.take() {
                self.docked_at = Some(dock);
            }
        } else {
            self.docked_at = None;
        }
    }

    /// Force-release pre-waypoint holds (e.g. when a docking or undocking action fails)
    /// so the robot gives up on the action and transitions back to idle without
    /// claiming the action completed.
    pub fn force_release_holds(&mut self) {
        self.active_action = InnerNavigationAction::NavigateToPose;
        self.action_completed = false;
        self.pending_dock = None;
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
            InnerNavigationAction::DockRobot { .. } | InnerNavigationAction::UndockRobot
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
            let (action, target_dock) = Self::resolve_target_action_with_dock(
                &self.dock_registry,
                self.docked_at.as_ref(),
                safe_zone,
                plan,
                fallback_start,
                target_xy,
            );
            self.plan_waypoints = plan
                .filter(|p| p.plan_id == plan_id)
                .map(Self::extract_plan_waypoints)
                .unwrap_or_default();
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
            self.pending_dock = target_dock;
            self.action_completed = false;
        } else {
            if self.plan_waypoints.is_empty() {
                if let Some(p) = plan.filter(|p| p.plan_id == plan_id) {
                    self.plan_waypoints = Self::extract_plan_waypoints(p);
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
                let new_start_xy = current_xy.unwrap_or(self.target_xy);
                let (action, target_dock) = Self::resolve_target_action_with_dock(
                    &self.dock_registry,
                    self.docked_at.as_ref(),
                    safe_zone,
                    plan,
                    new_start_xy,
                    target_xy,
                );
                self.start_xy = new_start_xy;
                self.start_waypoint = self.start_waypoint.max(safe_zone.last_waypoint);
                self.active_action = action;
                self.pending_dock = target_dock;
                self.action_completed = false;
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

        if self.is_holding_at_pre_waypoint() {
            return Progress {
                progress,
                reached_waypoint,
                target_waypoint: self.target_waypoint,
                reached_keys: vec![],
                plan_id,
                ..Default::default()
            };
        }

        if matches!(
            self.active_action,
            InnerNavigationAction::DockRobot { .. } | InnerNavigationAction::UndockRobot
        ) && self.action_completed
        {
            return Progress {
                progress: progress.max(self.target_progress),
                reached_waypoint: reached_waypoint.max(self.target_waypoint),
                target_waypoint: self.target_waypoint,
                reached_keys: vec![],
                plan_id,
                ..Default::default()
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
                } else {
                    let target_idx = (self.target_waypoint as usize)
                        .min(self.plan_waypoints.len().saturating_sub(1));
                    let search_start =
                        (reached_waypoint as usize).max(self.start_waypoint as usize);
                    let mut best_seg_prog: Option<(f32, f32)> = None;

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
                            match best_seg_prog {
                                None => best_seg_prog = Some((dist_sq, seg_prog)),
                                Some((best_dist_sq, _)) if dist_sq + 1e-3 < best_dist_sq => {
                                    best_seg_prog = Some((dist_sq, seg_prog));
                                }
                                _ => {}
                            }
                        }
                    }

                    if let Some((_, seg_prog)) = best_seg_prog {
                        let interp = seg_prog.clamp(start_prog, pre_final_cap);
                        progress = progress.max(interp).min(self.target_progress);
                        if self.target_waypoint > self.start_waypoint + 1 {
                            let intermediate_wp =
                                (interp.floor() as u64).min(self.target_waypoint.saturating_sub(1));
                            reached_waypoint = reached_waypoint.max(intermediate_wp);
                        }
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

    pub fn mark_target_reached(&mut self, previous: Option<&Progress>) -> Option<Progress> {
        self.mark_action_completed();
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

        if current.id.plan_id != other.id.plan_id {
            return true;
        }

        if !other.target_waypoint.is_empty() && current.target_waypoint != other.target_waypoint {
            return true;
        }

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
        let dock_registry = app
            .world()
            .get_resource::<RclrsNode>()
            .and_then(|node| {
                let site_file = node
                    .declare_parameter::<Arc<str>>("site_file")
                    .default(Arc::from(""))
                    .mandatory()
                    .ok()
                    .map(|p| p.get().to_string())
                    .unwrap_or_default();
                let path = site_file.trim();
                if path.is_empty() {
                    return None;
                }
                match DockRegistry::from_site_file(path) {
                    Ok(registry) => {
                        info!(
                            "Loaded {} dock(s) from site_file '{}'",
                            registry.docks.len(),
                            path
                        );
                        Some(registry)
                    }
                    Err(err) => {
                        warn!("{err}");
                        None
                    }
                }
            })
            .unwrap_or_default();

        app.insert_resource(dock_registry)
            .add_systems(PreUpdate, update_incremental_target)
            .add_observer(create_safe_zone_subscriber)
            .add_observer(create_plan_subscriber)
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
    let topic = agent_name + "/plan/safe_zone";
    let subscription = Arc::new(RosSubscription::<SafeZone>::new(&node, topic));
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
        LocalPlanRegistry::default(),
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
    dock_registry: Option<Res<DockRegistry>>,
) {
    let e = trigger.target();
    let Ok(agent_name) = agents.get(e).map(|agent| agent.name.clone()) else {
        return;
    };
    let topic = agent_name + "/plan/progress";
    let publisher = Arc::new(RosPublisher::<Progress>::new_transient_local(&node, topic));
    let registry = dock_registry.map(|r| (*r).clone()).unwrap_or_default();
    commands.entity(e).insert((
        ProgressPublisher {
            publisher: Arc::clone(&publisher),
        },
        LatestProgress::default(),
        ProgressTracker::with_dock_registry(registry),
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

pub(crate) fn update_incremental_target(
    mut commands: Commands,
    nav_services: Res<InnerNavigationServices>,
    mut current_nav_requests: Query<&mut CurrentNavigationRequest>,
    mut cancelled_sessions: Query<&mut CancelledNavigationSessions>,
    inner_nav_clients: Query<&InnerNavigationClient>,
    mut subscriptions: Query<(
        Entity,
        &SafeZoneSubscription,
        &PlanSubscription,
        &CostmapPublisher,
        &CostmapUpdatedSubscription,
        &mut CostmapHandshakeState,
        &ProgressPublisher,
        &mut LatestProgress,
        &mut ProgressTracker,
        &mut CurrentSafeZone,
        &mut LocalPlanRegistry,
        Option<&AmclPose>,
        &Nav2Agent,
    )>,
) {
    for (
        e,
        safe_zone_sub,
        plan_sub,
        costmap_pub,
        costmap_updated_sub,
        mut handshake,
        progress_pub,
        mut latest_progress,
        mut progress_tracker,
        mut current_safe_zone,
        mut plan_registry,
        amcl_pose,
        agent,
    ) in subscriptions.iter_mut()
    {
        // 1. Drain incoming plans and upgrade action if SafeZone arrived first
        while let Some(plan) = plan_sub.subscriber.take_data() {
            let prev_action = progress_tracker.active_action.clone();
            progress_tracker.set_plan(&plan, current_safe_zone.0.as_ref());
            let new_action = progress_tracker.active_action.clone();

            if prev_action != new_action {
                if let Some(sz) = current_safe_zone
                    .0
                    .as_ref()
                    .filter(|sz| sz.id.plan_id == plan.plan_id)
                {
                    if let Some(pending) = handshake
                        .pending_target
                        .as_mut()
                        .filter(|p| p.target.safe_zone_id.plan_id == plan.plan_id)
                    {
                        pending.target.action = new_action;
                    } else if let Some((tx, ty, tyaw)) = next_target(sz) {
                        let upgraded_target = InnerNavigationTarget::with_action(
                            e,
                            sz.id.clone(),
                            tx as f64,
                            ty as f64,
                            tyaw as f64,
                            new_action,
                        );
                        let _ = commands
                            .request(upgraded_target, nav_services.dispatch_target.clone())
                            .detach();
                    }

                    let current_pose = amcl_pose.filter(|_| agent.localized).map(|p| p.xy_yaw());
                    let base_progress =
                        progress_tracker.compute_progress(current_pose, latest_progress.0.as_ref());
                    let pose = amcl_pose
                        .filter(|_| agent.localized)
                        .map(|p| PoseStamped {
                            header: p.0.header.clone(),
                            pose: p.0.pose.pose.clone(),
                        })
                        .unwrap_or_default();
                    let mode = progress_tracker.execution_mode(true);
                    let progress_msg =
                        build_progress_status(pose, mode, Some(&base_progress), Some(&sz.id));
                    let _ = progress_pub.publisher.publish(progress_msg.clone());
                    latest_progress.0 = Some(progress_msg);
                }
            }
            plan_registry.insert(plan);
        }

        // 2. Drain costmap update acknowledgements
        while let Some(ack_header) = costmap_updated_sub.subscriber.take_data() {
            let seq = stamp_to_seq(&ack_header.stamp);
            if seq > handshake.acked_seq {
                handshake.acked_seq = seq;
            }
        }

        // 3. Drain incoming SafeZones (only the latest matters)
        let mut latest_safe_zone = None;
        while let Some(safe_zone) = safe_zone_sub.subscriber.take_data() {
            latest_safe_zone = Some(safe_zone);
        }

        if let Some(safe_zone) = latest_safe_zone {
            if !current_safe_zone.matches(&safe_zone) && is_valid(&safe_zone) {
                if let Some((target_x, target_y, target_yaw)) = next_target(&safe_zone) {
                    let incoming_plan_id = &safe_zone.id.plan_id;
                    let incoming_session = &incoming_plan_id.destination_session;

                    if cancelled_sessions
                        .get(e)
                        .is_ok_and(|cancelled| cancelled.contains(incoming_session))
                    {
                        debug!(
                            "[{:?}] Ignoring SafeZone for cancelled destination_session {:?}",
                            agent.name, incoming_plan_id
                        );
                        continue;
                    }

                    if let Ok(mut current_nav_req) = current_nav_requests.get_mut(e) {
                        if current_nav_req.session() == incoming_session {
                            current_nav_req.clear_superseded_sessions();
                        } else if current_nav_req.is_superseded_session(incoming_session) {
                            continue;
                        } else {
                            info!(
                                "[{:?}] Cancelling active NavigationRequest {:?} due to SafeZone plan_id {:?}",
                                agent.name,
                                current_nav_req.plan_id(),
                                incoming_plan_id
                            );
                            if let Ok(mut cancelled) = cancelled_sessions.get_mut(e) {
                                for session in current_nav_req.superseded_sessions() {
                                    cancelled.insert(session.clone());
                                }
                                cancelled.insert(current_nav_req.session().clone());
                            }
                            current_nav_req.cancel();
                            commands.entity(e).remove::<CurrentNavigationRequest>();
                        }
                    }

                    let should_update = current_safe_zone.should_update_target(&safe_zone);
                    let costmap_changed = handshake
                        .last_published_costmap
                        .as_ref()
                        .map(|prev| !costmap_content_eq(prev, &safe_zone.costmap))
                        .unwrap_or(true);

                    let mut published_costmap: Option<(u64, Costmap)> = None;
                    if should_update || costmap_changed {
                        handshake.next_seq += 1;
                        let seq = handshake.next_seq;
                        let mut costmap = safe_zone.costmap.clone();
                        costmap.header.stamp = seq_to_stamp(seq);
                        if costmap_pub.publisher.publish(costmap.clone()).is_err() {
                            error!("Failed to publish costmap for agent [{}]", agent.name);
                            continue;
                        }
                        handshake.last_published_costmap = Some(costmap.clone());
                        published_costmap = Some((seq, costmap));
                    }

                    let current_pose = amcl_pose.filter(|_| agent.localized).map(|p| p.xy_yaw());
                    let matching_plan = plan_registry.get(&safe_zone.id.plan_id);

                    let base_progress = progress_tracker.on_safe_zone_with_plan(
                        &safe_zone,
                        matching_plan,
                        current_pose,
                        latest_progress.0.as_ref(),
                    );
                    let pose = amcl_pose
                        .filter(|_| agent.localized)
                        .map(|p| PoseStamped {
                            header: p.0.header.clone(),
                            pose: p.0.pose.pose.clone(),
                        })
                        .unwrap_or_default();
                    let mode = progress_tracker.execution_mode(true);
                    let progress_msg = build_progress_status(
                        pose,
                        mode,
                        Some(&base_progress),
                        Some(&safe_zone.id),
                    );
                    let progress_changed = latest_progress.0.as_ref() != Some(&progress_msg);

                    if !should_update {
                        if let (Some(pending), Some((_, costmap))) =
                            (handshake.pending_target.as_mut(), published_costmap)
                        {
                            pending.costmap = costmap;
                        }
                        if progress_changed {
                            let _ = progress_pub.publisher.publish(progress_msg.clone());
                            latest_progress.0 = Some(progress_msg);
                        }
                    } else {
                        let (seq, costmap) = published_costmap.unwrap_or_else(|| {
                            let seq = handshake.next_seq;
                            let mut cm = safe_zone.costmap.clone();
                            cm.header.stamp = seq_to_stamp(seq);
                            (seq, cm)
                        });

                        let action = progress_tracker.active_action.clone();
                        info!(
                            "[{:?}] Queuing SafeZone target (waiting for costmap seq={}, action={:?}): ({:.2}, {:.2}, {:.2})",
                            agent.name, seq, action, target_x, target_y, target_yaw
                        );

                        let _ = progress_pub.publisher.publish(progress_msg.clone());
                        latest_progress.0 = Some(progress_msg);

                        if let Ok(client) = inner_nav_clients.get(e) {
                            client.mark_active_goal_superseded();
                        }

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

        // 5. Promote PendingNavigationTarget once costmap is acknowledged
        let acked_seq = handshake.acked_seq;
        if handshake
            .pending_target
            .as_ref()
            .is_some_and(|pending| acked_seq >= pending.required_seq)
        {
            if handshake.pending_target.as_ref().is_some_and(|pending| {
                cancelled_sessions.get(e).is_ok_and(|cancelled| {
                    cancelled.contains(&pending.target.safe_zone_id.plan_id.destination_session)
                })
            }) {
                handshake.pending_target = None;
            } else {
                let pending = handshake.pending_target.take().unwrap();
                let (tx, ty, tyaw) = pending.target_xy_yaw;
                info!(
                    "[{:?}] Costmap (seq={}) applied, dispatching SafeZone target: ({:.2}, {:.2}, {:.2})",
                    agent.name, pending.required_seq, tx, ty, tyaw
                );
                let _ = commands
                    .request(pending.target, nav_services.dispatch_target.clone())
                    .detach();
            }
        } else if let Some(pending) = handshake.pending_target.as_mut() {
            if cancelled_sessions.get(e).is_ok_and(|cancelled| {
                cancelled.contains(&pending.target.safe_zone_id.plan_id.destination_session)
            }) {
                handshake.pending_target = None;
            } else if pending.last_published.elapsed() >= Duration::from_millis(500) {
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

/// Extract the `(x, y, yaw)` target pose from a [`SafeZone`].
///
/// Heading precedence:
/// 1. Orientations from `incremental_target.regions` are evaluated in order.
/// 2. Orientations from `incremental_target.nodes` are evaluated in order and override
///    any region heading, with the last node that specifies an orientation winning.
fn next_target(safe_zone: &SafeZone) -> Option<(f32, f32, f32)> {
    let constraints = &safe_zone.incremental_target;
    let mut xy: Option<(f32, f32)> = None;
    let mut yaw: Option<f32> = None;

    for target_region in constraints.regions.iter() {
        let region = &target_region.region;
        let points = &region.points;

        if region.hint == Region::HINT_POINT && points.len() == 2 {
            xy = Some((points[0], points[1]));
        }

        for target_ori in target_region.orientations.iter() {
            yaw = Some(target_ori.orientation_radians);
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

        let mut next_sz = safe_zone(1, 0, 1, 0.0, 1.5);
        next_sz.target_waypoint = vec![1].try_into().unwrap();

        assert!(current.should_update_target(&next_sz));
    }

    #[test]
    fn target_update_sent_when_orientation_changes() {
        let mut current_sz = safe_zone(1, 0, 0, 0.0, 1.5);
        let mut ori1 = TargetOrientation::default();
        ori1.orientation_radians = 0.0;
        current_sz.incremental_target.regions[0]
            .orientations
            .push(ori1);
        let current = CurrentSafeZone(Some(current_sz));

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

        let p0 = tracker.on_safe_zone(&sz, Some((1.0, 0.0, 0.0)), None);
        assert_eq!(p0.reached_waypoint, 1);
        assert_eq!(p0.target_waypoint, 4);
        assert!((p0.progress - 1.0).abs() < 1e-4);

        let p_mid = tracker.compute_progress(Some((2.5, 0.0, 0.0)), Some(&p0));
        assert_eq!(p_mid.reached_waypoint, 2);
        assert!((p_mid.progress - 2.5).abs() < 1e-4);

        let p_done = tracker.compute_progress(Some((3.85, 0.0, 0.0)), Some(&p_mid));
        assert_eq!(p_done.reached_waypoint, 4);
        assert!((p_done.progress - 4.0).abs() < 1e-4);

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

        let p_pos_only = tracker.compute_progress(Some((3.0, 0.0, 0.0)), Some(&p0));
        assert_eq!(p_pos_only.reached_waypoint, 3);
        assert!(p_pos_only.progress < 4.0);

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

        let p0 = tracker.on_safe_zone(&sz, Some((0.0, 0.0, 0.0)), None);
        assert_eq!(p0.reached_waypoint, 0);
        assert!((p0.progress - 0.0).abs() < 1e-4);

        let p_half =
            tracker.compute_progress(Some((0.0, 0.0, std::f32::consts::FRAC_PI_2)), Some(&p0));
        assert_eq!(p_half.reached_waypoint, 0);
        assert!(p_half.progress < 1.0);

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
        let p_done = tracker.compute_progress(Some((1.0, 0.0, std::f32::consts::PI)), Some(&p0));
        assert_eq!(p_done.reached_waypoint, 1);
        assert!((p_done.progress - 1.0).abs() < 1e-4);
    }

    fn test_dock_registry() -> DockRegistry {
        let site_json =
            include_str!("../../../path_server/rmf_path_server_demo/maps/corridor_docks.site.json");
        DockRegistry::from_site_json_str(site_json).expect("valid corridor_docks.site.json")
    }

    #[test]
    fn dock_registry_derives_docks_and_staging_from_nav_graph() {
        let registry = test_dock_registry();
        assert_eq!(registry.docks.len(), 6);

        let dn1 = registry.match_by_id("dock_north_1").unwrap();
        assert!((dn1.x - (-1.0)).abs() < 1e-4);
        assert!((dn1.y - 2.0).abs() < 1e-4);
        assert!((dn1.staging_x - (-1.0)).abs() < 1e-4);
        assert!((dn1.staging_y - 1.0).abs() < 1e-4);
        assert!((dn1.yaw - std::f32::consts::FRAC_PI_2).abs() < 1e-4);

        let ds1 = registry.match_by_id("dock_south_1").unwrap();
        assert!((ds1.x - (-1.0)).abs() < 1e-4);
        assert!((ds1.y - (-2.0)).abs() < 1e-4);
        assert!((ds1.staging_x - (-1.0)).abs() < 1e-4);
        assert!((ds1.staging_y - (-1.0)).abs() < 1e-4);
        assert!((ds1.yaw - (-std::f32::consts::FRAC_PI_2)).abs() < 1e-4);
    }

    #[test]
    fn progress_tracker_holds_at_pre_dock_waypoint_during_docking() {
        use ros_env::rmf_prototype_msgs::msg::{GraphElementKey, TargetNode};

        let mut tracker = ProgressTracker::with_dock_registry(test_dock_registry());
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
        sz.incremental_target.nodes = vec![TargetNode {
            key: GraphElementKey {
                name: vec!["dock_north_1".try_into().unwrap()].try_into().unwrap(),
                ..Default::default()
            },
            orientations: vec![],
        }];

        let p0 = tracker.on_safe_zone(&sz, Some((-1.0, 1.0, std::f32::consts::FRAC_PI_2)), None);
        assert_eq!(
            tracker.active_action,
            InnerNavigationAction::DockRobot {
                navigate_to_staging_pose: false,
            }
        );
        assert_eq!(tracker.execution_mode(true), Progress::MODE_ACTION);
        assert_eq!(p0.reached_waypoint, 1);
        assert!((p0.progress - 1.0).abs() < 1e-4);

        let p_mid =
            tracker.compute_progress(Some((-1.0, 1.8, std::f32::consts::FRAC_PI_2)), Some(&p0));
        assert_eq!(p_mid.reached_waypoint, 1);
        assert!((p_mid.progress - 1.0).abs() < 1e-4);

        let p_at_dock =
            tracker.compute_progress(Some((-1.0, 2.0, std::f32::consts::FRAC_PI_2)), Some(&p_mid));
        assert_eq!(p_at_dock.reached_waypoint, 1);
        assert!((p_at_dock.progress - 1.0).abs() < 1e-4);

        let p_completed = tracker.mark_target_reached(Some(&p_at_dock)).unwrap();
        assert_eq!(tracker.execution_mode(false), Progress::MODE_IDLE);
        assert_eq!(p_completed.reached_waypoint, 2);
        assert!((p_completed.progress - 2.0).abs() < 1e-4);
    }

    #[test]
    fn progress_tracker_does_not_dock_without_target_node() {
        let mut tracker = ProgressTracker::with_dock_registry(test_dock_registry());
        let mut sz = safe_zone(1, 0, 1, -1.0, 2.0);
        sz.last_waypoint = 1;
        sz.target_waypoint = vec![2].try_into().unwrap();
        sz.target_progress = 2.0;

        let _ = tracker.on_safe_zone(&sz, Some((-1.0, 1.0, std::f32::consts::FRAC_PI_2)), None);
        assert_eq!(tracker.active_action, InnerNavigationAction::NavigateToPose);
        assert!(!tracker.is_holding_at_pre_waypoint());
    }

    #[test]
    fn progress_tracker_force_release_holds_unblocks_progress_and_mode() {
        use ros_env::rmf_prototype_msgs::msg::{GraphElementKey, TargetNode};

        let mut tracker = ProgressTracker::with_dock_registry(test_dock_registry());
        let mut sz = safe_zone(1, 0, 1, -1.0, 2.0);
        sz.last_waypoint = 1;
        sz.target_waypoint = vec![2].try_into().unwrap();
        sz.target_progress = 2.0;
        sz.incremental_target.nodes = vec![TargetNode {
            key: GraphElementKey {
                name: vec!["dock_north_1".try_into().unwrap()].try_into().unwrap(),
                ..Default::default()
            },
            orientations: vec![],
        }];

        let p0 = tracker.on_safe_zone(&sz, Some((-1.0, 1.0, std::f32::consts::FRAC_PI_2)), None);
        assert!(tracker.is_holding_at_pre_waypoint());
        assert_eq!(tracker.execution_mode(false), Progress::MODE_ACTION);

        tracker.force_release_holds();
        assert!(!tracker.is_holding_at_pre_waypoint());
        assert_eq!(tracker.execution_mode(false), Progress::MODE_IDLE);
        let p_after =
            tracker.compute_progress(Some((-1.0, 1.0, std::f32::consts::FRAC_PI_2)), Some(&p0));
        assert_eq!(p_after.reached_waypoint, 1);
        assert!((p_after.progress - 1.0).abs() < 1e-4);
    }

    #[test]
    fn progress_tracker_plan_node_resolves_docking_and_holds_until_completed() {
        use ros_env::rmf_prototype_msgs::msg::{GraphElementKey, TargetNode, Waypoint};

        let mut tracker = ProgressTracker::with_dock_registry(test_dock_registry());
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
        let mut node = TargetNode::default();
        node.key = GraphElementKey {
            name: vec!["dock_north_1".try_into().unwrap()].try_into().unwrap(),
            ..Default::default()
        };
        wp2.arrival_constraints.nodes = vec![node];
        plan.waypoints = vec![wp0, wp1, wp2];

        let p0 = tracker.on_safe_zone_with_plan(&sz, Some(&plan), Some((1.0, 0.0, 0.0)), None);
        assert!(tracker.is_holding_at_pre_waypoint());
        assert_eq!(p0.reached_waypoint, 1);
        assert!((p0.progress - 1.0).abs() < 1e-4);

        let p_after_dock = tracker.mark_target_reached(Some(&p0)).unwrap();
        assert!(tracker.action_completed);
        assert!(!tracker.is_holding_at_pre_waypoint());
        assert_eq!(p_after_dock.reached_waypoint, 2);
        assert!((p_after_dock.progress - 2.0).abs() < 1e-4);
    }

    #[test]
    fn progress_tracker_projects_along_multi_segment_plan_waypoints() {
        use ros_env::rmf_prototype_msgs::msg::Waypoint;

        let mut tracker = ProgressTracker::default();
        let mut sz = safe_zone(1, 0, 1, -2.0, 0.0);
        sz.last_waypoint = 1;
        sz.target_waypoint = vec![11].try_into().unwrap();
        sz.target_progress = 11.0;
        sz.incremental_target.regions[0].tolerance = 0.25;

        let coords = [
            [5.0, 0.0],
            [4.0, 0.0],
            [3.0, 0.0],
            [2.0, 0.0],
            [3.0, 0.0],
            [3.0, 1.0],
            [2.0, 1.0],
            [1.0, 1.0],
            [0.0, 1.0],
            [-1.0, 1.0],
            [-2.0, 1.0],
            [-2.0, 0.0],
        ];
        let mut plan = Plan::default();
        plan.plan_id = sz.id.plan_id.clone();
        plan.waypoints = coords
            .iter()
            .enumerate()
            .map(|(i, &pos)| {
                let mut wp = Waypoint::default();
                wp.position = pos;
                wp.progress = i as f32;
                wp
            })
            .collect();

        let p0 = tracker.on_safe_zone_with_plan(
            &sz,
            Some(&plan),
            Some((3.97, 0.11, std::f32::consts::PI)),
            None,
        );
        assert_eq!(p0.reached_waypoint, 1);

        let p_at_3_0 = tracker.compute_progress(Some((3.0, 0.0, std::f32::consts::PI)), Some(&p0));
        assert_eq!(p_at_3_0.reached_waypoint, 2);
        assert!((p_at_3_0.progress - 2.0).abs() < 1e-3);

        let p_turn = tracker.compute_progress(
            Some((3.0, 0.5, std::f32::consts::FRAC_PI_2)),
            Some(&p_at_3_0),
        );
        assert_eq!(p_turn.reached_waypoint, 4);
        assert!((p_turn.progress - 4.5).abs() < 1e-3);

        let p_upper =
            tracker.compute_progress(Some((3.0, 1.0, std::f32::consts::PI)), Some(&p_turn));
        assert_eq!(p_upper.reached_waypoint, 5);
        assert!((p_upper.progress - 5.0).abs() < 1e-3);
    }
}
