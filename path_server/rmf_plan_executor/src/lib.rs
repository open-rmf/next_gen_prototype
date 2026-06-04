use geometry_msgs::msg::Pose;
use mapf_post::MapfResult;
use mapf_post::WaypointFollower;
use mapf_post::na::{Isometry2, Vector2};
use mapf_post::spatial_allocation::{CurrentPosition, Grid2D};
use nav_msgs::msg::Odometry;
use nav2_msgs::msg::Costmap;
use rclrs::Node;
use rmf_prototype_msgs::msg::{DestinationConstraints, Plan, PlanRelease, SafeZone, SafeZoneId};
use std::collections::{BTreeMap, HashMap};
use std::sync::Arc;

pub struct RobotState {
    pub radius: f32,
    pub latest_odom: Option<Odometry>,
    pub plan: Option<Plan>,
    pub waypoint_follower: Option<WaypointFollower>,
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
        Self {
            node,
            active_robots: BTreeMap::new(),
            plan_release_publishers: HashMap::new(),
            safezone_publishers: HashMap::new(),
            grid: Arc::new(Grid2D::new(vec![vec![0; 20]; 20], 1.0)),
            grid_width: 20,
            grid_height: 20,
            grid_resolution: 1.0,
            grid_origin: Pose::default(),
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
            let position = Isometry2::new(Vector2::new(current_x, current_y), 0.0);
            fw.update_position_estimate(&position, 0.5);
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

        let mut released_wp_idx = curr_wp_idx;
        while released_wp_idx < plan.waypoints.len() {
            let wp = &plan.waypoints[released_wp_idx];
            let mut blocked = false;
            for blocker in &wp.departure_blockers {
                if let Some(blocker_sem) = semantic_waypoints.get(&blocker.name) {
                    // Timestep is 1.0, so progress is equivalent to the trajectory index.
                    let blocker_progress = blocker_sem.trajectory_index as f32;
                    if blocker_progress < blocker.required_progress {
                        blocked = true;
                        break;
                    }
                } else {
                    blocked = true;
                    break;
                }
            }
            if blocked {
                break;
            }
            released_wp_idx += 1;
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
                .create_publisher(&format!("{}/plan/release", robot_id))
                .unwrap();
            let _ = publisher.publish(pr);
            self.plan_release_publishers
                .insert(robot_id.to_string(), publisher);
        }

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
                .map(|wp| Isometry2::new(Vector2::new(wp.position[0], wp.position[1]), 0.0))
                .collect();
            trajectories.push(mapf_post::Trajectory { poses: traj_poses });
            footprints.push(Arc::new(mapf_post::shape::Ball::new(r_state.radius))
                as Arc<dyn mapf_post::shape::Shape>);

            let semantic_position = semantic_waypoints.get(r_name).cloned().unwrap();
            current_positions.push(CurrentPosition {
                semantic_position,
                real_position: (
                    r_odom.pose.pose.position.x as f32,
                    r_odom.pose.pose.position.y as f32,
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

        let costmap = Self::to_costmap_msg(
            &positions,
            self.grid_width,
            self.grid_height,
            self.grid_resolution,
            self.grid_origin.clone(),
            msg.header.stamp.clone(),
        );

        let safe_zone = SafeZone {
            incremental_target: DestinationConstraints {
                regions: vec![],
                nodes: vec![],
            },
            costmap,
            target_waypoint: vec![curr_wp_idx as u64].try_into().unwrap(),
            last_waypoint: released_wp_idx as u64,
            target_progress: 0.0,
            id: SafeZoneId {
                plan_id: plan.plan_id.clone(),
                safe_zone_version: 0,
            },
        };

        if let Some(safe_zone_pub) = self.safezone_publishers.get_mut(robot_id) {
            let _ = safe_zone_pub.publish(safe_zone);
        } else {
            let publisher = self
                .node
                .create_publisher(&format!("{}/safe_zone", robot_id))
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
                let index = y * (width as usize) + x;
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
