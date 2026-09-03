use crate::{inner_navigation_client::InnerNavigationTarget, Nav2Agent};
use bevy::prelude::*;
use bevy_ros2::{RclrsNode, RosPublisher, RosSubscription};
use ros_env::{
    nav2_msgs::msg::Costmap,
    rmf_prototype_msgs::msg::{PlanError, Progress, Region, SafeZone},
};
use std::sync::Arc;

#[derive(Component)]
pub struct SafeZoneSubscription {
    pub subscriber: Arc<RosSubscription<SafeZone>>,
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

        self.distancesq_to_target(other) >= 0.5
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
        &Nav2Agent,
    )>,
) {
    for (e, safe_zone_sub, costmap_pub, progress_pub, mut current_safe_zone, agent) in
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

        let Some((target_x, target_y, target_yaw)) = next_target(&safe_zone) else {
            continue;
        };

        // Publish progress
        let Ok(_) = progress_pub.publisher.publish(Progress {
            progress: safe_zone.target_progress,
            reached_waypoint: safe_zone.last_waypoint,
            target_waypoint: safe_zone.target_waypoint[0], // TODO(@xiyuoh) review
            reached_keys: vec![],                          // TODO(@xiyuoh)
            plan_id: safe_zone.id.plan_id.clone(),
        }) else {
            error!("Failed to publish progress for agent [{}]", agent.name);
            continue;
        };

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
    for _target_node in constraints.nodes.iter() {
        // TODO(@xiyuoh)
    }

    xy.zip(yaw).map(|((x, y), yaw)| (x, y, yaw))
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
}
