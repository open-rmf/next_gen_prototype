use geometry_msgs::msg::Pose;
use mapf_post::MapfResult;
use mapf_post::SemanticPlan;
use mapf_post::WaypointFollower;
use mapf_post::na::{Isometry2, Vector2};
use mapf_post::spatial_allocation::{CurrentPosition, Grid2D};
use nav_msgs::msg::Odometry;
use nav2_msgs::msg::Costmap;
use rclrs::Node;
use rmf_prototype_msgs::msg::DestinationConstraints;
use rmf_prototype_msgs::msg::PlanId;
use rmf_prototype_msgs::msg::PlanRelease;
use rmf_prototype_msgs::msg::SafeZone;
use rmf_prototype_msgs::msg::SafeZoneId;
use std::collections::{BTreeMap, HashMap};
use std::sync::Arc;

pub struct PlanExecutor {
    pub node: Arc<Node>,
    pub waypoint_followers: HashMap<String, WaypointFollower>,
    pub plan_release_publishers: HashMap<String, rclrs::Publisher<PlanRelease>>,
    pub safezone_publishers: HashMap<String, rclrs::Publisher<SafeZone>>,
    pub active_plan: Option<MapfResult>,
    pub active_semantic_plan: Option<SemanticPlan>,
    pub active_plan_ids: HashMap<String, PlanId>,
    pub grid: Arc<Grid2D>,
    pub grid_width: u32,
    pub grid_height: u32,
    pub grid_resolution: f32,
    pub grid_origin: Pose,
    pub latest_positions: HashMap<String, (f32, f32)>,
}

impl PlanExecutor {
    pub fn new(node: Arc<Node>) -> Self {
        Self {
            node,
            waypoint_followers: HashMap::new(),
            plan_release_publishers: HashMap::new(),
            safezone_publishers: HashMap::new(),
            active_plan: None,
            active_semantic_plan: None,
            active_plan_ids: HashMap::new(),
            grid: Arc::new(Grid2D::new(vec![vec![0; 20]; 20], 1.0)),
            grid_width: 20,
            grid_height: 20,
            grid_resolution: 1.0,
            grid_origin: Pose::default(),
            latest_positions: HashMap::new(),
        }
    }

    pub fn handle_odometry(&mut self, robot_id: &str, msg: &Odometry) {
        let current_x = msg.pose.pose.position.x as f32;
        let current_y = msg.pose.pose.position.y as f32;

        self.latest_positions
            .insert(robot_id.to_string(), (current_x, current_y));

        if let Some(waypoint_follower_session) = self.waypoint_followers.get_mut(robot_id) {
            //TODO(arjoc): Make "uncertainty" configurable
            let position = Isometry2::new(Vector2::new(current_x, current_y), 0.0);
            waypoint_follower_session.update_position_estimate(&position, 0.5);
        }

        let mut current_positions: BTreeMap<String, CurrentPosition> = BTreeMap::new();

        // Get semantic robot positions
        for (r_id, waypoint_follower_session) in &mut self.waypoint_followers {
            let real_pos = self
                .latest_positions
                .get(r_id)
                .copied()
                .unwrap_or((current_x, current_y));
            current_positions.insert(
                r_id.clone(),
                CurrentPosition {
                    semantic_position: waypoint_follower_session.get_semantic_waypoint(),
                    real_position: real_pos,
                },
            );
        }

        let robot_name_to_id: HashMap<String, usize> = HashMap::from_iter(
            current_positions
                .iter()
                .enumerate()
                .map(|(id, (robot_name, _))| (robot_name.clone(), id)),
        );
        let semantic_pos_vec: Vec<_> = current_positions
            .iter()
            .map(|(_robot, position)| &position.semantic_position)
            .cloned()
            .collect();

        let full_pos_vec: Vec<_> = current_positions
            .iter()
            .map(|(_robot, position)| position)
            .cloned()
            .collect(); // Calculate claim dict
        if let Some(active_sem_plan) = self.active_semantic_plan.clone() {
            let safe_claims = active_sem_plan.get_claim_dict(&semantic_pos_vec);

            if let Some(agent_id) = robot_name_to_id.get(robot_id) {
                if let Some(p) = safe_claims.get(&agent_id) {
                    let pr = PlanRelease {
                        waypoint_id: p.end_id as u64,
                        plan_id: self.active_plan_ids.get(robot_id).unwrap().clone(),
                    };
                    rclrs::log!(
                        self.node.logger(),
                        "Publishing PlanRelease for {}: waypoint_id = {}, plan_version = {}",
                        robot_id,
                        p.end_id,
                        pr.plan_id.plan_version
                    );
                    if let Some(plan_release_pub) = self.plan_release_publishers.get_mut(robot_id) {
                        let _ = plan_release_pub.publish(pr);
                    } else {
                        let publisher = self
                            .node
                            .create_publisher(&format!("{}/plan/release", robot_id))
                            .unwrap();
                        let _ = publisher.publish(pr);
                        self.plan_release_publishers
                            .insert(robot_id.to_owned().clone(), publisher);
                    }

                    let Some(trajectory) = self.active_plan.clone() else {
                        println!("This should not be possible.");
                        return;
                    };
                    let allocation_field =
                        self.grid.allocate_trajectory(&trajectory, &full_pos_vec);
                    let positions = allocation_field
                        .get_alloc_for_agent(*agent_id)
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
                        target_waypoint: vec![p.start_id as u64].try_into().unwrap(),
                        last_waypoint: p.end_id as u64,
                        target_progress: 0.0, // TODO(arjoc): Is this necessary
                        id: SafeZoneId {
                            plan_id: self.active_plan_ids.get(robot_id).unwrap().clone(),
                            safe_zone_version: 0, // TODO(arjoc)L
                        },
                    };

                    if let Some(costmap_pub) = self.safezone_publishers.get_mut(robot_id) {
                        let _ = costmap_pub.publish(safe_zone);
                    } else {
                        let publisher = self
                            .node
                            .create_publisher(&format!("{}/safe_zone", robot_id))
                            .unwrap();
                        let _ = publisher.publish(safe_zone);
                        self.safezone_publishers
                            .insert(robot_id.to_owned().clone(), publisher);
                    }
                }
            }
        }
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
