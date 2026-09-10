use crate::{inner_navigation_client::InnerNavigationTarget, Nav2Agent};
use bevy::prelude::*;
use bevy_ros2::{RclrsNode, RosPublisher, RosSubscription};
use ros_env::{
    nav2_msgs::msg::Costmap,
    rmf_prototype_msgs::msg::{Plan, PlanError, Progress, Region, SafeZone},
};
use std::sync::Arc;

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

impl CurrentPlan {
    pub fn update(&mut self, value: Plan) {
        self.0 = Some(value);
    }

    pub fn clear(&mut self) {
        self.0 = None;
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
        self.0.as_ref().is_some_and(|sz| sz.id == other.id)
    }

    pub fn should_update_target(&self, other: &SafeZone) -> bool {
        let Some(current) = self.0.as_ref() else {
            return true;
        };

        // Resend an unchanged target when it belongs to a new plan.
        if current.id.plan_id != other.id.plan_id {
            return true;
        }

        // Resend when targeting a new waypoint in the plan (e.g. dock approach waypoint).
        if !other.target_waypoint.is_empty() && current.target_waypoint != other.target_waypoint {
            return true;
        }

        // Resend if target orientation has significantly changed.
        if let (Some(cur_yaw), Some(other_yaw)) =
            (Self::get_orientation(current), Self::get_orientation(other))
        {
            let diff = (cur_yaw - other_yaw).abs();
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
        let Some(safe_zone) = self.0.as_ref() else {
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
            Region::HINT_AXIS_ALIGNED_RECTANGLE | Region::HINT_RECTANGLE => {
                if region.region.points.len() >= 4 {
                    Some((
                        ((region.region.points[0] + region.region.points[2]) * 0.5) as f64,
                        ((region.region.points[1] + region.region.points[3]) * 0.5) as f64,
                    ))
                } else {
                    None
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
        app.add_systems(PreUpdate, (update_incremental_target, update_plan))
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
    let subscription = Arc::new(RosSubscription::<Plan>::new(&node, topic.clone()));
    commands.entity(e).insert((
        PlanSubscription {
            subscriber: Arc::clone(&subscription),
        },
        CurrentPlan::default(),
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
    let publisher = Arc::new(RosPublisher::<Progress>::new(&node, topic));
    commands.entity(e).insert(ProgressPublisher {
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
    let publisher = Arc::new(RosPublisher::<PlanError>::new(&node, topic));
    commands.entity(e).insert(PlanErrorPublisher {
        publisher: Arc::clone(&publisher),
    });
}

fn update_incremental_target(
    mut nav_target: EventWriter<InnerNavigationTarget>,
    mut subscriptions: Query<(
        Entity,
        &SafeZoneSubscription,
        &CostmapPublisher,
        &ProgressPublisher,
        &mut CurrentSafeZone,
        Option<&CurrentPlan>,
        &Nav2Agent,
    )>,
) {
    for (e, safe_zone_sub, costmap_pub, progress_pub, mut current_safe_zone, maybe_plan, agent) in
        subscriptions.iter_mut()
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

        let plan_ref = maybe_plan.and_then(|p| p.0.as_ref());
        let Some((target_x, target_y, target_yaw, dock_action)) = next_target(&safe_zone, plan_ref)
        else {
            continue;
        };

        // Publish progress
        let target_wp = safe_zone
            .target_waypoint
            .first()
            .copied()
            .unwrap_or(safe_zone.last_waypoint);
        let Ok(_) = progress_pub.publisher.publish(Progress {
            progress: safe_zone.target_progress,
            reached_waypoint: safe_zone.last_waypoint,
            target_waypoint: target_wp,
            reached_keys: vec![],
            plan_id: safe_zone.id.plan_id.clone(),
        }) else {
            error!("Failed to publish progress for agent [{}]", agent.name);
            continue;
        };

        if !current_safe_zone.should_update_target(&safe_zone) {
            continue;
        }
        debug!(
            "[{:?}] Updating SafeZone to target: ({:.2}, {:.2}, {:.2}), dock={:?}",
            agent.name, target_x, target_y, target_yaw, dock_action
        );

        *current_safe_zone = CurrentSafeZone(Some(safe_zone.clone()));

        nav_target.write(InnerNavigationTarget::new(
            e,
            safe_zone.id,
            target_x as f64,
            target_y as f64,
            target_yaw as f64,
            dock_action,
        ));
    }
}

fn update_plan(mut subscriptions: Query<(&PlanSubscription, &mut CurrentPlan, &Nav2Agent)>) {
    for (plan_sub, mut current_plan, agent) in subscriptions.iter_mut() {
        let Some(plan) = plan_sub.subscriber.data_callback() else {
            continue;
        };
        let is_new = match current_plan.0.as_ref() {
            Some(current) => current.plan_id != plan.plan_id,
            None => true,
        };
        if is_new {
            debug!(
                "[{}] Received new plan version {} with {} waypoints",
                agent.name,
                plan.plan_id.plan_version,
                plan.waypoints.len()
            );
            *current_plan = CurrentPlan(Some(plan));
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

fn next_target(
    safe_zone: &SafeZone,
    plan: Option<&Plan>,
) -> Option<(f32, f32, f32, Option<String>)> {
    let constraints = &safe_zone.incremental_target;
    let mut xy: Option<(f32, f32)> = None;
    let mut yaw: Option<f32> = None;
    let mut dock_action: Option<String> = None;

    // 1. Extract position and explicit orientations from regions
    for target_region in constraints.regions.iter() {
        let region = &target_region.region;
        let points = &region.points;

        match region.hint {
            Region::HINT_POINT => {
                if points.len() == 2 {
                    xy = Some((points[0], points[1]));
                }
            }
            Region::HINT_AXIS_ALIGNED_RECTANGLE | Region::HINT_RECTANGLE => {
                if points.len() >= 4 {
                    xy = Some(((points[0] + points[2]) * 0.5, (points[1] + points[3]) * 0.5));
                }
            }
            _ => {
                if points.len() >= 2 {
                    xy = Some((points[0], points[1]));
                }
            }
        }

        for target_ori in target_region.orientations.iter() {
            yaw = Some(target_ori.orientation_radians);
        }
    }

    // 2. Extract explicit orientations and dock action from incremental_target.nodes
    for target_node in constraints.nodes.iter() {
        for target_ori in target_node.orientations.iter() {
            yaw = Some(target_ori.orientation_radians);
        }
        if let Some(name) = target_node.key.name.first() {
            let name_str = name.to_string();
            if name_str.contains("dock") {
                dock_action = Some(name_str);
            }
        }
    }

    // 3. Fallback or override from CurrentPlan for the target waypoint
    let target_wp_idx = safe_zone
        .target_waypoint
        .first()
        .copied()
        .map(|w| w as usize);
    if let (Some(plan), Some(wp_idx)) = (plan, target_wp_idx) {
        if let Some(waypoint) = plan.waypoints.get(wp_idx) {
            if xy.is_none() {
                xy = Some((waypoint.position[0], waypoint.position[1]));
            }

            // Extract dock action or workflow from arrival_action
            if !waypoint.arrival_action.is_empty() {
                dock_action = Some(waypoint.arrival_action.clone());
            }

            // Fallback or override from plan.workflow if this is the target/final waypoint
            if dock_action.is_none() && !plan.workflow.is_empty() {
                let is_last_wp = wp_idx + 1 >= plan.waypoints.len();
                if is_last_wp {
                    dock_action = Some(plan.workflow.clone());
                }
            }

            // Fallback dock action from waypoint arrival_constraints.nodes
            if dock_action.is_none() {
                for node in &waypoint.arrival_constraints.nodes {
                    if let Some(name) = node.key.name.first() {
                        let name_str = name.to_string();
                        if name_str.contains("dock") {
                            dock_action = Some(name_str);
                            break;
                        }
                    }
                }
            }

            // Check waypoint arrival_constraints.nodes for orientation
            if yaw.is_none() {
                for node in &waypoint.arrival_constraints.nodes {
                    if let Some(target_ori) = node.orientations.first() {
                        yaw = Some(target_ori.orientation_radians);
                        break;
                    }
                }
            }

            // Check waypoint arrival_constraints.regions for orientation
            if yaw.is_none() {
                for region in &waypoint.arrival_constraints.regions {
                    if let Some(target_ori) = region.orientations.first() {
                        yaw = Some(target_ori.orientation_radians);
                        break;
                    }
                }
            }

            // Semantic dock orientation conventions (e.g. conveyor docks)
            if yaw.is_none() {
                let action_str = dock_action.as_deref().unwrap_or(&waypoint.arrival_action);
                if action_str.contains("conveyor_r1") || action_str.contains("dock_conveyor_r1") {
                    // Row 1 conveyors (Infeed at Y=0.0): approached from South corridor, face South (-pi/2)
                    yaw = Some(-std::f32::consts::FRAC_PI_2);
                } else if action_str.contains("conveyor_r2")
                    || action_str.contains("dock_conveyor_r2")
                {
                    // Row 2 conveyors (Outfeed at Y=5.0): approached from North corridor, face North (+pi/2)
                    yaw = Some(std::f32::consts::FRAC_PI_2);
                }
            }

            // Trajectory heading fallback: compute angle from previous waypoint
            if yaw.is_none() && wp_idx > 0 {
                let [curr_x, curr_y] = waypoint.position;
                for prev_wp in plan.waypoints[..wp_idx].iter().rev() {
                    let dx = curr_x - prev_wp.position[0];
                    let dy = curr_y - prev_wp.position[1];
                    if dx.hypot(dy) > 1e-3 {
                        yaw = Some(dy.atan2(dx));
                        break;
                    }
                }
            }
        }
    }

    let final_yaw = yaw.unwrap_or(0.0);
    xy.map(|(x, y)| (x, y, final_yaw, dock_action))
}

#[cfg(test)]
mod tests {
    use super::*;
    use ros_env::rmf_prototype_msgs::msg::TargetRegion;

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
    fn next_target_uses_region_orientation() {
        let mut sz = safe_zone(1, 0, 0, 1.0, 2.0);
        let mut target_ori = ros_env::rmf_prototype_msgs::msg::TargetOrientation::default();
        target_ori.orientation_radians = 1.23;
        sz.incremental_target.regions[0]
            .orientations
            .push(target_ori);

        let target = next_target(&sz, None);
        assert_eq!(target, Some((1.0, 2.0, 1.23, None)));
    }

    #[test]
    fn next_target_uses_node_orientation() {
        let mut sz = safe_zone(1, 0, 0, 1.0, 2.0);
        let mut node = ros_env::rmf_prototype_msgs::msg::TargetNode::default();
        let mut target_ori = ros_env::rmf_prototype_msgs::msg::TargetOrientation::default();
        target_ori.orientation_radians = -1.57;
        node.orientations.push(target_ori);
        sz.incremental_target.nodes.push(node);

        let target = next_target(&sz, None);
        assert_eq!(target, Some((1.0, 2.0, -1.57, None)));
    }

    #[test]
    fn next_target_uses_conveyor_r1_dock_orientation() {
        let mut sz = safe_zone(1, 0, 0, 0.0, 1.5);
        sz.target_waypoint = vec![1].try_into().unwrap();

        let mut plan = Plan::default();
        plan.waypoints = vec![
            ros_env::rmf_prototype_msgs::msg::Waypoint {
                position: [0.0, 2.0],
                ..Default::default()
            },
            ros_env::rmf_prototype_msgs::msg::Waypoint {
                position: [0.0, 1.5],
                arrival_action: "dock_conveyor_r1_c1".to_string(),
                ..Default::default()
            },
        ];

        let target = next_target(&sz, Some(&plan));
        assert!(target.is_some());
        let (x, y, yaw, dock) = target.unwrap();
        assert_eq!((x, y), (0.0, 1.5));
        assert!((yaw - (-std::f32::consts::FRAC_PI_2)).abs() < 1e-6);
        assert_eq!(dock, Some("dock_conveyor_r1_c1".to_string()));
    }

    #[test]
    fn next_target_uses_conveyor_r2_dock_orientation() {
        let mut sz = safe_zone(1, 0, 0, 2.5, 3.5);
        sz.target_waypoint = vec![1].try_into().unwrap();

        let mut plan = Plan::default();
        plan.waypoints = vec![
            ros_env::rmf_prototype_msgs::msg::Waypoint {
                position: [2.5, 3.0],
                ..Default::default()
            },
            ros_env::rmf_prototype_msgs::msg::Waypoint {
                position: [2.5, 3.5],
                arrival_action: "dock_conveyor_r2_c2".to_string(),
                ..Default::default()
            },
        ];

        let target = next_target(&sz, Some(&plan));
        assert!(target.is_some());
        let (x, y, yaw, dock) = target.unwrap();
        assert_eq!((x, y), (2.5, 3.5));
        assert!((yaw - std::f32::consts::FRAC_PI_2).abs() < 1e-6);
        assert_eq!(dock, Some("dock_conveyor_r2_c2".to_string()));
    }

    #[test]
    fn next_target_uses_trajectory_heading_when_no_explicit_orientation() {
        let mut sz = safe_zone(1, 0, 0, 5.0, 0.0);
        sz.target_waypoint = vec![1].try_into().unwrap();

        let mut plan = Plan::default();
        plan.waypoints = vec![
            ros_env::rmf_prototype_msgs::msg::Waypoint {
                position: [0.0, 0.0],
                ..Default::default()
            },
            ros_env::rmf_prototype_msgs::msg::Waypoint {
                position: [5.0, 0.0],
                ..Default::default()
            },
        ];

        let target = next_target(&sz, Some(&plan));
        assert_eq!(target, Some((5.0, 0.0, 0.0, None)));
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
        let mut current_sz = safe_zone(1, 0, 0, 0.0, 1.5);
        let mut ori1 = ros_env::rmf_prototype_msgs::msg::TargetOrientation::default();
        ori1.orientation_radians = 0.0;
        current_sz.incremental_target.regions[0]
            .orientations
            .push(ori1);
        let current = CurrentSafeZone(Some(current_sz));

        // Same position, but orientation changes to -pi/2
        let mut next_sz = safe_zone(1, 0, 1, 0.0, 1.5);
        let mut ori2 = ros_env::rmf_prototype_msgs::msg::TargetOrientation::default();
        ori2.orientation_radians = -std::f32::consts::FRAC_PI_2;
        next_sz.incremental_target.regions[0]
            .orientations
            .push(ori2);

        assert!(current.should_update_target(&next_sz));
    }

    #[test]
    fn next_target_extracts_json_workflow_from_arrival_action() {
        let mut sz = safe_zone(1, 0, 0, 0.0, 1.5);
        sz.target_waypoint = vec![1].try_into().unwrap();

        let workflow_json = r#"[
            {"action": "dock", "dock_id": "dock_conveyor_r1_c1"},
            {"action": "wait", "duration_sec": 3.0},
            {"action": "undock"}
        ]"#;

        let mut plan = Plan::default();
        plan.waypoints = vec![
            ros_env::rmf_prototype_msgs::msg::Waypoint {
                position: [0.0, 2.0],
                ..Default::default()
            },
            ros_env::rmf_prototype_msgs::msg::Waypoint {
                position: [0.0, 1.5],
                arrival_action: workflow_json.to_string(),
                ..Default::default()
            },
        ];

        let target = next_target(&sz, Some(&plan));
        assert!(target.is_some());
        let (x, y, yaw, action) = target.unwrap();
        assert_eq!((x, y), (0.0, 1.5));
        assert!((yaw - (-std::f32::consts::FRAC_PI_2)).abs() < 1e-6);
        assert_eq!(action, Some(workflow_json.to_string()));
    }

    #[test]
    fn next_target_uses_plan_workflow_on_final_waypoint() {
        let mut sz = safe_zone(1, 0, 0, 2.5, 3.5);
        sz.target_waypoint = vec![1].try_into().unwrap();

        let mut plan = Plan::default();
        plan.workflow = "dock_conveyor_r2_c2".to_string();
        plan.waypoints = vec![
            ros_env::rmf_prototype_msgs::msg::Waypoint {
                position: [2.5, 3.0],
                ..Default::default()
            },
            ros_env::rmf_prototype_msgs::msg::Waypoint {
                position: [2.5, 3.5],
                arrival_action: String::new(),
                ..Default::default()
            },
        ];

        let target = next_target(&sz, Some(&plan));
        assert!(target.is_some());
        let (x, y, yaw, action) = target.unwrap();
        assert_eq!((x, y), (2.5, 3.5));
        assert!((yaw - std::f32::consts::FRAC_PI_2).abs() < 1e-6);
        assert_eq!(action, Some("dock_conveyor_r2_c2".to_string()));
    }
}
