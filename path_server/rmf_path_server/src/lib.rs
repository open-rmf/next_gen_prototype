use mapf_post::na::Isometry2;
use mapf_post::SemanticPlan;
use mapf_post::SemanticWaypoint;
use nav_msgs::msg::Odometry;
use rclrs::Node;
use rmf_prototype_msgs::msg::Destination;
use rmf_prototype_msgs::msg::Plan;
use rmf_prototype_msgs::msg::PlanId;
use rmf_prototype_msgs::msg::TrafficDependency;
use rmf_prototype_msgs::msg::Waypoint;
use std::collections::HashMap;
use std::sync::Arc;

pub mod planner;
pub use planner::{MapfPlanner, MockPlanner, PibtPlanner};

pub struct PlanServer<P: MapfPlanner> {
    pub active_plans: HashMap<String, Destination>,
    pub latest_pose_estimate: HashMap<String, Odometry>,
    pub node: Arc<Node>,
    pub replan_queue: Vec<(String, Destination)>,
    pub active_plan_ids: HashMap<String, PlanId>,
    pub planner: P,
    pub plan_publishers: HashMap<String, rclrs::Publisher<Plan>>,
}

impl<P: MapfPlanner> PlanServer<P> {
    pub fn new(node: Arc<Node>, planner: P) -> Self {
        Self {
            active_plans: HashMap::new(),
            latest_pose_estimate: HashMap::new(),
            node,
            replan_queue: Vec::new(),
            active_plan_ids: HashMap::new(),
            planner,
            plan_publishers: HashMap::new(),
        }
    }

    pub fn handle_destination(&mut self, robot_id: &str, msg: Destination) {
        rclrs::log!(
            self.node.logger(),
            "PathServer (DestinationsWorker) received updated destination for {} (session UUID {})",
            robot_id,
            msg.session
                .uuid
                .iter()
                .map(|b| format!("{:02x}", b))
                .collect::<Vec<String>>()
                .join("")
        );
        self.replan_queue.push((robot_id.to_owned(), msg));
        self.replan();
    }

    pub fn handle_odometry(&mut self, robot_id: &str, msg: Odometry) {
        rclrs::log!(
            self.node.logger(),
            "PathServer (DestinationsWorker) received odometry for {}: Position({:.2}, {:.2}, {:.2})",
            robot_id,
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        );
        self.latest_pose_estimate.insert(robot_id.to_string(), msg);
        self.replan();
    }

    pub fn replan(&mut self) {
        // There is no need to trigger a replan
        if self.replan_queue.is_empty() {
            return;
        }

        rclrs::log!(self.node.logger(), "Need to trigger plan");
        // Retrieve goals of all participants
        let mut goals = self.active_plans.clone();

        for (robot_id, dest) in &self.replan_queue {
            goals.insert(robot_id.clone(), dest.clone());
        }

        let mut starts = HashMap::new();
        for robot_id in goals.keys() {
            // Make sure we have the latest odometry for all robots.
            // Give up if odometry for some robots is stale.
            if let Some(odom) = self.latest_pose_estimate.get(robot_id) {
                starts.insert(robot_id.clone(), odom.clone());
            } else {
                rclrs::log!(
                    self.node.logger(),
                    "Odometry for robot {} is missing or stale. Skipping plan.",
                    robot_id
                );
                return;
            }
        }

        // We can now safely drain/clear the replan queue since we are proceeding with the planning.
        self.replan_queue.clear();

        // Perform MAPF plan here.
        rclrs::log!(self.node.logger(), "Triggering new plan");

        let mut robot_ids: Vec<String> = goals.keys().cloned().collect();
        robot_ids.sort();

        let basic_plan = match self.planner.plan(&starts, &goals, &robot_ids) {
            Ok(plan) => plan,
            Err(err) => {
                rclrs::log_error!(
                    self.node.logger(),
                    "Planner failed to generate plan: {:?}",
                    err
                );
                return;
            }
        };

        let trajectories: Vec<mapf_post::Trajectory> = basic_plan
            .iter()
            .map(|poses| mapf_post::Trajectory { poses: poses.clone() })
            .collect();

        let num_agents = trajectories.len();
        let footprints: Vec<Arc<dyn mapf_post::shape::Shape>> = (0..num_agents)
            .map(|_| Arc::new(mapf_post::shape::Ball::new(0.49)) as Arc<dyn mapf_post::shape::Shape>)
            .collect();

        let mapf_result = mapf_post::MapfResult {
            trajectories,
            footprints,
            discretization_timestep: 1.0,
        };

        let traffic_dependencies = mapf_post::mapf_post(&mapf_result);

        // First update the active_plan_ids for each robot with their new PlanId
        for robot_id in &robot_ids {
            let dest = goals.get(robot_id).unwrap();
            let prev_plan_id = self.active_plan_ids.get(robot_id);
            let new_version = match prev_plan_id {
                Some(p_id) if p_id.destination_session.uuid == dest.session.uuid => p_id.plan_version + 1,
                _ => 0,
            };

            let plan_id = PlanId {
                destination_session: dest.session.clone(),
                plan_version: new_version,
            };

            self.active_plan_ids.insert(robot_id.clone(), plan_id);
        }

        // Use traffic_dependencies to populate Plan message for each agent
        let mut plans = HashMap::new();
        for (agent_idx, robot_id) in robot_ids.iter().enumerate() {
            let plan_id = self.active_plan_ids.get(robot_id).unwrap().clone();
            let traj = &basic_plan[agent_idx];
            let plan = Self::to_plan_msg(
                agent_idx,
                traj,
                plan_id,
                &traffic_dependencies,
                &robot_ids,
                &self.active_plan_ids,
                1.0,
            );
            plans.insert(robot_id.clone(), plan);
        }

        for (robot_id, plan) in &plans {
            rclrs::log!(
                self.node.logger(),
                "Generated plan with version {} and {} waypoints for robot {}",
                plan.plan_id.plan_version,
                plan.waypoints.len(),
                robot_id
            );

            // Publish the plans
            if !self.plan_publishers.contains_key(robot_id) {
                let topic = format!("{}/plan", robot_id);
                match self.node.create_publisher::<Plan>(&topic) {
                    Ok(pub_) => {
                        self.plan_publishers.insert(robot_id.clone(), pub_);
                    }
                    Err(err) => {
                        rclrs::log_error!(
                            self.node.logger(),
                            "Failed to create plan publisher for {}: {:?}",
                            robot_id,
                            err
                        );
                        continue;
                    }
                }
            }

            if let Some(publisher) = self.plan_publishers.get(robot_id) {
                if let Err(err) = publisher.publish(plan) {
                    rclrs::log_error!(
                        self.node.logger(),
                        "Failed to publish plan for {}: {:?}",
                        robot_id,
                        err
                    );
                }
            }
        }

        self.active_plans = goals;
    }

    pub fn to_plan_msg(
        agent_idx: usize,
        traj: &[Isometry2<f32>],
        plan_id: PlanId,
        traffic_dependencies: &SemanticPlan,
        robot_ids: &[String],
        active_plan_ids: &HashMap<String, PlanId>,
        timestep: f32,
    ) -> Plan {
        let mut waypoints = Vec::new();
        for (i, pose) in traj.iter().enumerate() {
            let mut departure_blockers = Vec::new();
            if let Some(dep_ids) = traffic_dependencies.comes_before(&SemanticWaypoint {
                agent: agent_idx,
                trajectory_index: i,
            }) {
                for &dep_id in dep_ids {
                    let dep_wp = &traffic_dependencies.waypoints[dep_id];
                    if dep_wp.agent == agent_idx {
                        continue;
                    }
                    if let Some(dep_robot_id) = robot_ids.get(dep_wp.agent) {
                        if let Some(dep_plan_id) = active_plan_ids.get(dep_robot_id) {
                            departure_blockers.push(TrafficDependency {
                                name: dep_robot_id.clone(),
                                plan_id: dep_plan_id.clone(),
                                required_progress: dep_wp.trajectory_index as f32 * timestep,
                            });
                        }
                    }
                }
            }

            waypoints.push(Waypoint {
                position: [pose.translation.x, pose.translation.y],
                arrival_constraints: Default::default(),
                progress: i as f32 * timestep,
                maps: Vec::new(),
                departure_blockers,
                departure_trajectory: Vec::new(),
                departure_action: String::new(),
                arrival_action: String::new(),
            });
        }

        Plan {
            waypoints,
            start_time: builtin_interfaces::msg::Time { sec: 0, nanosec: 0 },
            plan_id,
            workflow: String::new(),
        }
    }
}

pub struct RobotPathConnections<P: MapfPlanner> {
    pub _destination_subscription: rclrs::WorkerSubscription<Destination, PlanServer<P>>,
    pub _odom_subscription: rclrs::WorkerSubscription<Odometry, PlanServer<P>>,
}

pub struct DiscoveryServer<P: MapfPlanner> {
    pub node: Arc<Node>,
    pub active_robots: HashMap<String, RobotPathConnections<P>>,
    pub destinations_worker: Arc<rclrs::Worker<PlanServer<P>>>,
}

impl<P: MapfPlanner> DiscoveryServer<P> {
    pub fn new(node: Arc<Node>, destinations_worker: Arc<rclrs::Worker<PlanServer<P>>>) -> Self {
        Self {
            node,
            active_robots: HashMap::new(),
            destinations_worker,
        }
    }
}
