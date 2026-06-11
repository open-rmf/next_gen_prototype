use geometry_msgs::msg::Pose;
use mapf_post::{
    na::{Isometry2, Vector2},
    spatial_allocation::{CurrentPosition, Grid2D},
    MapfResult, WaypointFollower,
};
use nav2_msgs::msg::Costmap;
use nav_msgs::msg::Odometry;
use rclrs::{IntoPrimitiveOptions, Node};
use rmf_prototype_msgs::msg::{
    DestinationConstraints, Plan, PlanRelease, SafeZone, SafeZoneId, TargetOrientation,
};
use std::{
    collections::{BTreeMap, HashMap},
    sync::Arc,
};

pub struct RobotState {
    pub radius: f32,
    pub latest_odom: Option<Odometry>,
    pub plan: Option<Plan>,
    pub waypoint_follower: Option<WaypointFollower>,
    pub safe_zone_version: u64,
    pub last_incremental_target_wp: Option<usize>,
}

pub struct PlanExecutor {
    pub node: Arc<Node>,
    pub active_robots: BTreeMap<String, RobotState>,
    pub plan_release_publishers: HashMap<String, rclrs::Publisher<PlanRelease>>,
    pub safezone_publishers: HashMap<String, rclrs::Publisher<SafeZone>>,
    pub grid: Arc<Grid2D>,
    pub grid_width: u32,
    pub grid_height: u32,
    pub grid_resolution: f32,
    pub grid_origin: Pose,
}

impl PlanExecutor {
    pub fn new(node: Arc<Node>) -> Self {
        let mut origin = Pose::default();
        origin.orientation.w = 1.0;
        Self {
            node,
            active_robots: BTreeMap::new(),
            plan_release_publishers: HashMap::new(),
            safezone_publishers: HashMap::new(),
            grid: Arc::new(Grid2D::new(vec![vec![0; 20]; 20], 1.0)),
            grid_width: 20,
            grid_height: 20,
            grid_resolution: 1.0,
            grid_origin: origin,
        }
    }

    pub fn handle_robot_added(&mut self, robot_id: &str, radius: f32) {
        if !self.active_robots.contains_key(robot_id) {
            rclrs::log!(
                self.node.logger(),
                "PlanExecutor adding participant: {} with radius {}",
                robot_id,
                radius
            );
            self.active_robots.insert(
                robot_id.to_string(),
                RobotState {
                    radius,
                    latest_odom: None,
                    plan: None,
                    waypoint_follower: None,
                    safe_zone_version: 0,
                    last_incremental_target_wp: None,
                },
            );
            self.reindex_followers();
        }
    }

    pub fn handle_robot_removed(&mut self, robot_id: &str) {
        if self.active_robots.remove(robot_id).is_some() {
            rclrs::log!(
                self.node.logger(),
                "PlanExecutor removing participant: {}",
                robot_id
            );
            self.plan_release_publishers.remove(robot_id);
            self.safezone_publishers.remove(robot_id);
            self.reindex_followers();
        }
    }

    fn get_agent_index(&self, robot_id: &str) -> Option<usize> {
        self.active_robots.keys().position(|k| k == robot_id)
    }

    fn reindex_followers(&mut self) {
        let sorted_names: Vec<String> = self.active_robots.keys().cloned().collect();
        for (agent_idx, name) in sorted_names.iter().enumerate() {
            let robot_state = self.active_robots.get_mut(name).unwrap();
            if let Some(plan) = &robot_state.plan {
                let traj_poses: Vec<Isometry2<f32>> = plan
                    .waypoints
                    .iter()
                    .map(|wp| Isometry2::new(Vector2::new(wp.position[0], wp.position[1]), 0.0))
                    .collect();

                let mut follower = WaypointFollower::from_trajectory(
                    agent_idx,
                    mapf_post::Trajectory { poses: traj_poses },
                );

                if let Some(odom) = &robot_state.latest_odom {
                    let current_x = odom.pose.pose.position.x as f32;
                    let current_y = odom.pose.pose.position.y as f32;
                    let position = Isometry2::new(Vector2::new(current_x, current_y), 0.0);
                    follower.update_position_estimate(&position, 0.5);
                }

                robot_state.waypoint_follower = Some(follower);
            }
        }
    }

    pub fn handle_plan(&mut self, robot_id: &str, msg: Plan) {
        rclrs::log!(
            self.node.logger(),
            "Received plan version {} with {} waypoints for robot {}",
            msg.plan_id.plan_version,
            msg.waypoints.len(),
            robot_id
        );

        let Some(state) = self.active_robots.get_mut(robot_id) else {
            return;
        };

        state.plan = Some(msg);
        state.safe_zone_version = 0;
        state.last_incremental_target_wp = None;

        // Reindex because we updated the plan
        self.reindex_followers();
    }

    pub fn handle_odometry(&mut self, robot_id: &str, msg: Odometry) {
        let current_x = msg.pose.pose.position.x as f32;
        let current_y = msg.pose.pose.position.y as f32;

        let Some(state) = self.active_robots.get_mut(robot_id) else {
            return;
        };

        state.latest_odom = Some(msg.clone());

        if let Some(fw) = &mut state.waypoint_follower {
            let before = fw.get_semantic_waypoint().trajectory_index;
            let position = Isometry2::new(Vector2::new(current_x, current_y), 0.0);
            fw.update_position_estimate(&position, 0.5);
            let after = fw.get_semantic_waypoint().trajectory_index;
            rclrs::log!(
                self.node.logger(),
                "[Executor Debug] Robot {} pos=({}, {}) index before={}, after={}",
                robot_id,
                current_x,
                current_y,
                before,
                after
            );
        }

        if !self.ready_to_execute() {
            return;
        }

        // Cache all semantic waypoints first because get_semantic_waypoint requires &mut self.
        // Doing this first avoids borrowing active_robots as mutable during later immutable reads.
        let mut semantic_waypoints = HashMap::new();
        for (name, r_state) in &mut self.active_robots {
            if let Some(fw) = &mut r_state.waypoint_follower {
                semantic_waypoints.insert(name.clone(), fw.get_semantic_waypoint());
            }
        }

        // 1. Calculate PlanRelease
        let agent_idx = self.get_agent_index(robot_id).unwrap();
        let robot_state = &self.active_robots[robot_id];
        let plan = robot_state.plan.as_ref().unwrap();
        let curr_wp_idx = semantic_waypoints
            .get(robot_id)
            .map(|w| w.trajectory_index)
            .unwrap_or(0);

        rclrs::log!(
            self.node.logger(),
            "[Executor Debug] Robot {} curr_wp_idx={} (semantic waypoint trajectory index)",
            robot_id,
            curr_wp_idx
        );

        let mut released_wp_idx = curr_wp_idx;
        while released_wp_idx < plan.waypoints.len() {
            let wp = &plan.waypoints[released_wp_idx];
            let mut blocked = false;
            for blocker in &wp.departure_blockers {
                if let Some(blocker_sem) = semantic_waypoints.get(&blocker.name) {
                    let blocker_progress = blocker_sem.trajectory_index;
                    let required_progress = blocker.required_progress.round() as usize;
                    rclrs::log!(
                        self.node.logger(),
                        "[Executor Debug] Blocker check for robot {} wp {}: blocker={} blocker_progress={} required={}",
                        robot_id,
                        released_wp_idx,
                        blocker.name,
                        blocker_progress,
                        required_progress
                    );
                    if blocker_progress < required_progress {
                        blocked = true;
                        break;
                    }
                } else {
                    rclrs::log!(
                        self.node.logger(),
                        "[Executor Debug] Blocker check for robot {} wp {}: blocker={} is missing semantic waypoint",
                        robot_id,
                        released_wp_idx,
                        blocker.name
                    );
                    blocked = true;
                    break;
                }
            }
            if blocked {
                break;
            }
            released_wp_idx += 1;
        }

        if released_wp_idx < plan.waypoints.len() {
            let wp_pos = &plan.waypoints[released_wp_idx].position;
            for (r_name, r_state) in &self.active_robots {
                if r_name == robot_id {
                    continue;
                }
                if let Some(odom) = &r_state.latest_odom {
                    let dx = wp_pos[0] - odom.pose.pose.position.x as f32;
                    let dy = wp_pos[1] - odom.pose.pose.position.y as f32;
                    if dx.hypot(dy) < 0.8 {
                        released_wp_idx = released_wp_idx.saturating_sub(1);
                        break;
                    }
                }
            }
        }

        if released_wp_idx >= plan.waypoints.len() {
            released_wp_idx = plan.waypoints.len() - 1;
        }

        let pr = PlanRelease {
            waypoint_id: released_wp_idx as u64,
            plan_id: plan.plan_id.clone(),
        };

        // Publish PlanRelease
        if let Some(plan_release_pub) = self.plan_release_publishers.get_mut(robot_id) {
            let _ = plan_release_pub.publish(pr);
        } else {
            let publisher = self
                .node
                .create_publisher(
                    format!("{}/plan/release", robot_id)
                        .as_str()
                        .transient_local()
                        .reliable(),
                )
                .unwrap();
            let _ = publisher.publish(pr);
            self.plan_release_publishers
                .insert(robot_id.to_string(), publisher);
        }

        // --- Dynamic grid resizing ---
        let mut min_x = f32::MAX;
        let mut min_y = f32::MAX;
        let mut max_x = f32::MIN;
        let mut max_y = f32::MIN;

        for r_state in self.active_robots.values() {
            let r_plan = r_state.plan.as_ref().unwrap();
            let r_odom = r_state.latest_odom.as_ref().unwrap();

            min_x = min_x.min(r_odom.pose.pose.position.x as f32);
            min_y = min_y.min(r_odom.pose.pose.position.y as f32);
            max_x = max_x.max(r_odom.pose.pose.position.x as f32);
            max_y = max_y.max(r_odom.pose.pose.position.y as f32);

            for wp in &r_plan.waypoints {
                min_x = min_x.min(wp.position[0]);
                min_y = min_y.min(wp.position[1]);
                max_x = max_x.max(wp.position[0]);
                max_y = max_y.max(wp.position[1]);
            }
        }

        let current_min_x = self.grid_origin.position.x as f32;
        let current_min_y = self.grid_origin.position.y as f32;
        let current_max_x = current_min_x + (self.grid_width as f32 * self.grid_resolution);
        let current_max_y = current_min_y + (self.grid_height as f32 * self.grid_resolution);

        if min_x < current_min_x || min_y < current_min_y || max_x > current_max_x || max_y > current_max_y {
            let new_min_x = min_x - 20.0;
            let new_min_y = min_y - 20.0;
            let new_max_x = max_x + 20.0;
            let new_max_y = max_y + 20.0;

            self.grid_origin.position.x = new_min_x as f64;
            self.grid_origin.position.y = new_min_y as f64;
            self.grid_origin.orientation.w = 1.0;

            self.grid_width = ((new_max_x - new_min_x) / self.grid_resolution).ceil() as u32;
            self.grid_height = ((new_max_y - new_min_y) / self.grid_resolution).ceil() as u32;

            self.grid = Arc::new(Grid2D::new(
                vec![vec![0; self.grid_height as usize]; self.grid_width as usize],
                self.grid_resolution,
            ));
        }
        // --- End dynamic grid resizing ---

        // 2. Generate and publish SafeZone
        let mut trajectories = Vec::new();
        let mut footprints = Vec::new();
        let mut current_positions = Vec::new();

        for (r_name, r_state) in &self.active_robots {
            let r_plan = r_state.plan.as_ref().unwrap();
            let r_odom = r_state.latest_odom.as_ref().unwrap();

            let traj_poses: Vec<Isometry2<f32>> = r_plan
                .waypoints
                .iter()
                .map(|wp| {
                    Isometry2::new(
                        Vector2::new(
                            wp.position[0] - self.grid_origin.position.x as f32,
                            wp.position[1] - self.grid_origin.position.y as f32,
                        ),
                        0.0,
                    )
                })
                .collect();
            trajectories.push(mapf_post::Trajectory { poses: traj_poses });
            footprints.push(Arc::new(mapf_post::shape::Ball::new(r_state.radius))
                as Arc<dyn mapf_post::shape::Shape>);

            let semantic_position = semantic_waypoints.get(r_name).cloned().unwrap();
            current_positions.push(CurrentPosition {
                semantic_position,
                real_position: (
                    r_odom.pose.pose.position.x as f32 - self.grid_origin.position.x as f32,
                    r_odom.pose.pose.position.y as f32 - self.grid_origin.position.y as f32,
                ),
            });
        }

        let mapf_result = MapfResult {
            trajectories,
            footprints,
            discretization_timestep: 1.0,
        };

        let allocation_field = self
            .grid
            .allocate_trajectory(&mapf_result, &current_positions);
        let positions = allocation_field
            .get_alloc_for_agent(agent_idx)
            .unwrap_or_default();

        let mut max_wp_idx = curr_wp_idx;
        for &(x, y) in &positions {
            if let Some(priority) = allocation_field.get_alloc_priority(x as isize, y as isize) {
                if priority > max_wp_idx {
                    max_wp_idx = priority;
                }
            }
        }
        if max_wp_idx > released_wp_idx {
            max_wp_idx = released_wp_idx;
        }

        let target_x = plan.waypoints[max_wp_idx].position[0];
        let target_y = plan.waypoints[max_wp_idx].position[1];

        let costmap = Self::to_costmap_msg(
            &positions,
            self.grid_width,
            self.grid_height,
            self.grid_resolution,
            self.grid_origin.clone(),
            msg.header.stamp.clone(),
        );

        let plan_id = plan.plan_id.clone();
        
        // Update SafeZone version if the target waypoint changed
        let (safe_zone_version, _is_new_target) = {
            let state = self.active_robots.get_mut(robot_id).unwrap();
            if state.last_incremental_target_wp != Some(max_wp_idx) {
                if state.last_incremental_target_wp.is_some() {
                    state.safe_zone_version += 1;
                }
                state.last_incremental_target_wp = Some(max_wp_idx);
            }
            (state.safe_zone_version, state.last_incremental_target_wp == Some(max_wp_idx))
        };

        let safe_zone = SafeZone {
            incremental_target: DestinationConstraints {
                regions: vec![rmf_prototype_msgs::msg::TargetRegion {
                    tolerance: 0.2,
                    region: rmf_prototype_msgs::msg::Region {
                        points: vec![target_x, target_y],
                        hint: rmf_prototype_msgs::msg::Region::HINT_POINT,
                    },
                    orientations: vec![TargetOrientation {
                        orientation_radians: 0.0, // TODO(@xiyuoh)
                        spread_radians: 0.0,
                        tolerance_radians: 0.0,
                    }], // TODO(@xiyuoh) calculate actual orientation
                }],
                nodes: vec![],
            },
            costmap,
            target_waypoint: vec![max_wp_idx as u64].try_into().unwrap(),
            last_waypoint: released_wp_idx as u64,
            target_progress: 0.0,
            id: SafeZoneId {
                plan_id,
                safe_zone_version,
            },
        };

        if let Some(safe_zone_pub) = self.safezone_publishers.get_mut(robot_id) {
            let _ = safe_zone_pub.publish(safe_zone);
        } else {
            let publisher = self
                .node
                .create_publisher(
                    format!("{}/plan/safe_zone", robot_id)
                        .as_str()
                        .transient_local()
                        .reliable(),
                )
                .unwrap();
            let _ = publisher.publish(safe_zone);
            self.safezone_publishers
                .insert(robot_id.to_string(), publisher);
        }
    }

    fn ready_to_execute(&self) -> bool {
        if self.active_robots.is_empty() {
            return false;
        }
        for state in self.active_robots.values() {
            if state.plan.is_none()
                || state.latest_odom.is_none()
                || state.waypoint_follower.is_none()
            {
                return false;
            }
        }
        true
    }

    pub fn to_costmap_msg(
        positions: &[(usize, usize)],
        width: u32,
        height: u32,
        resolution: f32,
        origin: Pose,
        stamp: builtin_interfaces::msg::Time,
    ) -> Costmap {
        let mut data = vec![254u8; (width * height) as usize];
        for &(x, y) in positions {
            if x < width as usize && y < height as usize {
                // Grid2D from mapf_post returns y with top-left origin (i.e. flipped)
                // We need to invert y back for ROS Costmap which expects bottom-left origin
                let inverted_y = (height - 1) as usize - y;
                let index = inverted_y * (width as usize) + x;
                data[index] = 0;
            }
        }

        Costmap {
            header: std_msgs::msg::Header {
                stamp: stamp.clone(),
                frame_id: "map".to_string(),
            },
            metadata: nav2_msgs::msg::CostmapMetaData {
                map_load_time: stamp.clone(),
                update_time: stamp,
                layer: "safe_zone".to_string(),
                resolution,
                size_x: width,
                size_y: height,
                origin,
            },
            data,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use mapf_post::na::{Isometry2, Vector2};
    use mapf_post::{Trajectory, WaypointFollower};

    #[test]
    fn test_robot_2_follower() {
        let poses = vec![
            Isometry2::new(Vector2::new(3.0, 0.0), 0.0),
            Isometry2::new(Vector2::new(4.0, 0.0), 0.0),
            Isometry2::new(Vector2::new(5.0, 0.0), 0.0),
            Isometry2::new(Vector2::new(6.0, 0.0), 0.0),
            Isometry2::new(Vector2::new(7.0, 0.0), 0.0),
            Isometry2::new(Vector2::new(8.0, 0.0), 0.0),
            Isometry2::new(Vector2::new(9.0, 0.0), 0.0),
            Isometry2::new(Vector2::new(10.0, 0.0), 0.0),
            Isometry2::new(Vector2::new(10.0, 0.0), 0.0),
            Isometry2::new(Vector2::new(10.0, 0.0), 0.0),
        ];
        let mut follower = WaypointFollower::from_trajectory(0, Trajectory { poses });

        // Simulating the robot moving through the path:
        follower.update_position_estimate(&Isometry2::new(Vector2::new(3.0, 0.0), 0.0), 0.5);
        assert_eq!(follower.get_semantic_waypoint().trajectory_index, 0);

        follower.update_position_estimate(&Isometry2::new(Vector2::new(4.0, 0.0), 0.0), 0.5);
        assert_eq!(follower.get_semantic_waypoint().trajectory_index, 1);

        follower.update_position_estimate(&Isometry2::new(Vector2::new(5.0, 0.0), 0.0), 0.5);
        assert_eq!(follower.get_semantic_waypoint().trajectory_index, 2);

        follower.update_position_estimate(&Isometry2::new(Vector2::new(6.0, 0.0), 0.0), 0.5);
        assert_eq!(follower.get_semantic_waypoint().trajectory_index, 3);

        follower.update_position_estimate(&Isometry2::new(Vector2::new(7.0, 0.0), 0.0), 0.5);
        assert_eq!(follower.get_semantic_waypoint().trajectory_index, 4);

        follower.update_position_estimate(&Isometry2::new(Vector2::new(8.0, 0.0), 0.0), 0.5);
        assert_eq!(follower.get_semantic_waypoint().trajectory_index, 5);

        follower.update_position_estimate(&Isometry2::new(Vector2::new(9.0, 0.0), 0.0), 0.5);
        assert_eq!(follower.get_semantic_waypoint().trajectory_index, 6);

        follower.update_position_estimate(&Isometry2::new(Vector2::new(10.0, 0.0), 0.0), 0.5);
        println!(
            "After reaching 10.0: index is {}",
            follower.get_semantic_waypoint().trajectory_index
        );

        follower.update_position_estimate(&Isometry2::new(Vector2::new(10.0, 0.0), 0.0), 0.5);
        println!(
            "After waiting at 10.0 (1st time): index is {}",
            follower.get_semantic_waypoint().trajectory_index
        );

        follower.update_position_estimate(&Isometry2::new(Vector2::new(10.0, 0.0), 0.0), 0.5);
        println!(
            "After waiting at 10.0 (2nd time): index is {}",
            follower.get_semantic_waypoint().trajectory_index
        );
    }
}
