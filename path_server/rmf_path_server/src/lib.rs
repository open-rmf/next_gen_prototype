use mapf_post::MapfResult;
use mapf_post::SemanticPlan;
use mapf_post::SemanticWaypoint;
use mapf_post::na::Isometry2;
use nav_msgs::msg::Odometry;
use rclrs::{IntoPrimitiveOptions, Node};
use rmf_prototype_msgs::msg::Destination;
use rmf_prototype_msgs::msg::Plan;
use rmf_prototype_msgs::msg::PlanId;
use rmf_prototype_msgs::msg::TrafficDependency;
use rmf_prototype_msgs::msg::Waypoint;
use std::collections::HashMap;
use std::sync::Arc;

pub mod planner;
pub use planner::{MapfPlanner, MockPlanner, PibtPlanner};

pub struct PlanSuccess {
    pub session_id: u64,
    pub basic_plan: Vec<Vec<Isometry2<f32>>>,
    pub traffic_dependencies: SemanticPlan,
    pub goals: HashMap<String, Destination>,
    pub robot_ids: Vec<String>,
    pub active_plan: MapfResult,
}

pub enum PlanResult {
    Success(PlanSuccess),
    Failure { session_id: u64, error: String },
}

pub struct PlanServer<P: MapfPlanner> {
    pub active_plans: HashMap<String, Destination>,
    pub latest_pose_estimate: HashMap<String, Odometry>,
    pub node: Arc<Node>,
    pub replan_queue: Vec<(String, Destination)>,
    pub planner: Arc<P>,
    pub plan_publishers: HashMap<String, rclrs::Publisher<Plan>>,
    pub plan_receiver: std::sync::Mutex<std::sync::mpsc::Receiver<PlanResult>>,
    pub plan_sender: std::sync::Mutex<std::sync::mpsc::Sender<PlanResult>>,
    pub is_planning: bool,
    pub current_cancellation: Option<Arc<std::sync::atomic::AtomicBool>>,
    pub planning_session_id: u64,
    pub current_planning_session: Option<u64>,
    pub footprints: Arc<std::sync::Mutex<HashMap<String, f32>>>,
    pub active_plan_ids: HashMap<String, PlanId>,
}

impl<P: MapfPlanner> PlanServer<P> {
    pub fn new(
        node: Arc<Node>,
        planner: P,
        footprints: Arc<std::sync::Mutex<HashMap<String, f32>>>,
    ) -> Self {
        let (plan_sender, plan_receiver) = std::sync::mpsc::channel();
        Self {
            active_plans: HashMap::new(),
            latest_pose_estimate: HashMap::new(),
            node,
            replan_queue: Vec::new(),
            planner: Arc::new(planner),
            plan_publishers: HashMap::new(),
            plan_sender: std::sync::Mutex::new(plan_sender),
            plan_receiver: std::sync::Mutex::new(plan_receiver),
            is_planning: false,
            current_cancellation: None,
            planning_session_id: 0,
            current_planning_session: None,
            footprints,
            active_plan_ids: HashMap::new(),
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

        let is_new_session = match self.active_plans.get(robot_id) {
            Some(active_dest) => active_dest.session.uuid != msg.session.uuid,
            None => true,
        };

        if is_new_session {
            if let Some(cancellation) = &self.current_cancellation {
                cancellation.store(true, std::sync::atomic::Ordering::Relaxed);
            }
            self.is_planning = false;
            self.current_planning_session = None;
            self.current_cancellation = None;
        }

        self.replan_queue.push((robot_id.to_owned(), msg));
    }

    /// For now both the plan server and executor's logic is embodied in this
    /// function.
    pub fn handle_odometry(&mut self, robot_id: &str, msg: Odometry) {
        rclrs::log!(
            self.node.logger(),
            "PathServer (DestinationsWorker) received odometry for {}: Position({:.2}, {:.2}, {:.2})",
            robot_id,
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        );
        self.latest_pose_estimate
            .insert(robot_id.to_string(), msg.clone());
    }

    pub fn replan(&mut self) {
        // 1. Check if any planning results have arrived from the background thread
        if let Ok(receiver) = self.plan_receiver.lock() {
            while let Ok(result) = receiver.try_recv() {
                match result {
                    PlanResult::Success(success) => {
                        if Some(success.session_id) != self.current_planning_session {
                            // Stale planning task result, ignore
                            continue;
                        }
                        self.is_planning = false;
                        self.current_cancellation = None;
                        self.current_planning_session = None;

                        let PlanSuccess {
                            basic_plan,
                            traffic_dependencies,
                            goals,
                            robot_ids,
                            ..
                        } = success;

                        // First update the active_plan_ids for each robot with their new PlanId
                        for robot_id in &robot_ids {
                            let dest = goals.get(robot_id).unwrap();
                            let prev_plan_id = self.active_plan_ids.get(robot_id);
                            let new_version = match prev_plan_id {
                                Some(p_id)
                                    if p_id.destination_session.uuid == dest.session.uuid =>
                                {
                                    p_id.plan_version + 1
                                }
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
                            let mut wp_strs = Vec::new();
                            for (j, wp) in plan.waypoints.iter().enumerate() {
                                let blockers: Vec<String> = wp.departure_blockers.iter().map(|b| {
                                    format!("{} progress >= {}", b.name, b.required_progress)
                                }).collect();
                                wp_strs.push(format!("  wp {}: pos {:?}, progress {}, blockers: {:?}", j, wp.position, wp.progress, blockers));
                            }
                            rclrs::log!(
                                self.node.logger(),
                                "Generated plan with version {} and {} waypoints for robot {}:\n{}",
                                plan.plan_id.plan_version,
                                plan.waypoints.len(),
                                robot_id,
                                wp_strs.join("\n")
                            );

                            // Publish the plans
                            if !self.plan_publishers.contains_key(robot_id) {
                                let topic = format!("{}/plan", robot_id);
                                match self.node.create_publisher::<Plan>(topic.as_str().transient_local().reliable()) {
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
                    PlanResult::Failure { session_id, error } => {
                        if Some(session_id) != self.current_planning_session {
                            continue;
                        }
                        self.is_planning = false;
                        self.current_cancellation = None;
                        self.current_planning_session = None;

                        rclrs::log_error!(
                            self.node.logger(),
                            "Background planner failed: {}",
                            error
                        );
                    }
                }
            }
        }

        // 2. If a planning task is currently running, do not spawn another one
        if self.is_planning {
            return;
        }

        // 3. If there is nothing new in the replan queue, we don't need to schedule a new plan
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

        // Perform MAPF plan in background thread
        rclrs::log!(self.node.logger(), "Triggering new plan in background");

        let mut robot_ids: Vec<String> = goals.keys().cloned().collect();
        robot_ids.sort();

        self.planning_session_id += 1;
        let session_id = self.planning_session_id;
        self.current_planning_session = Some(session_id);
        self.is_planning = true;

        let cancellation = Arc::new(std::sync::atomic::AtomicBool::new(false));
        self.current_cancellation = Some(Arc::clone(&cancellation));

        let planner_clone = Arc::clone(&self.planner);
        let footprints_clone = Arc::clone(&self.footprints);
        let sender_clone = match self.plan_sender.lock() {
            Ok(sender) => sender.clone(),
            Err(_) => {
                self.is_planning = false;
                return;
            }
        };

        std::thread::spawn(move || {
            if cancellation.load(std::sync::atomic::Ordering::Relaxed) {
                return;
            }

            let footprints_map: HashMap<String, Arc<dyn mapf_post::shape::Shape>> = robot_ids
                .iter()
                .map(|id| {
                    let radius = if let Ok(map) = footprints_clone.lock() {
                        *map.get(id).unwrap_or(&0.49)
                    } else {
                        0.49
                    };
                    (
                        id.clone(),
                        Arc::new(mapf_post::shape::Ball::new(radius + 0.1))
                            as Arc<dyn mapf_post::shape::Shape>,
                    )
                })
                .collect();

            let basic_plan = match planner_clone.plan(&starts, &goals, &footprints_map, &robot_ids)
            {
                Ok(plan) => plan,
                Err(err) => {
                    let _ = sender_clone.send(PlanResult::Failure {
                        session_id,
                        error: format!("{:?}", err),
                    });
                    return;
                }
            };

            if cancellation.load(std::sync::atomic::Ordering::Relaxed) {
                return;
            }

            let trajectories: Vec<mapf_post::Trajectory> = basic_plan
                .iter()
                .map(|poses| mapf_post::Trajectory {
                    poses: poses.clone(),
                })
                .collect();

            let footprints: Vec<Arc<dyn mapf_post::shape::Shape>> = robot_ids
                .iter()
                .map(|id| footprints_map.get(id).unwrap().clone())
                .collect();

            let mapf_result = mapf_post::MapfResult {
                trajectories,
                footprints,
                discretization_timestep: 1.0,
            };

            let traffic_dependencies = mapf_post::mapf_post(&mapf_result);

            if cancellation.load(std::sync::atomic::Ordering::Relaxed) {
                return;
            }

            let _ = sender_clone.send(PlanResult::Success(PlanSuccess {
                session_id,
                basic_plan,
                traffic_dependencies,
                goals,
                robot_ids,
                active_plan: mapf_result,
            }));
        });
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
            waypoints.push(Waypoint {
                position: [pose.translation.x, pose.translation.y],
                arrival_constraints: Default::default(),
                progress: i as f32 * timestep,
                maps: Vec::new(),
                departure_blockers: Vec::new(),
                departure_trajectory: Vec::new(),
                departure_action: String::new(),
                arrival_action: String::new(),
            });
        }

        for i in 0..traj.len() {
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
                            let req_progress = (dep_wp.trajectory_index + 1) as f32 * timestep;
                            let blocker = TrafficDependency {
                                name: dep_robot_id.clone(),
                                plan_id: dep_plan_id.clone(),
                                required_progress: req_progress,
                            };
                            waypoints[i].departure_blockers.push(blocker);
                        }
                    }
                }
            }
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



pub struct PathServerRunning<P: MapfPlanner> {
    pub destinations_worker: Arc<rclrs::Worker<PlanServer<P>>>,
    pub discovery_worker: Arc<rclrs::Worker<DiscoveryServer<P>>>,
    pub replan_timer: Box<dyn std::any::Any + Send + Sync>,
    pub list_subscription: rclrs::WorkerSubscription<rmf_prototype_msgs::msg::ParticipantList, DiscoveryServer<P>>,
    pub discovery_subscription: rclrs::WorkerSubscription<rmf_prototype_msgs::msg::ParticipantList, DiscoveryServer<P>>,
}

pub fn start_path_server<P: MapfPlanner + 'static>(
    node: Arc<rclrs::Node>,
    planner: P,
) -> Result<PathServerRunning<P>, Box<dyn std::error::Error>> {
    let footprints = Arc::new(std::sync::Mutex::new(std::collections::HashMap::new()));
    let footprints_clone = Arc::clone(&footprints);

    // Create the Destinations worker (Data Plane)
    let destinations_worker = Arc::new(node.create_worker(PlanServer::new(
        Arc::clone(&node),
        planner,
        footprints,
    )));

    // Create a periodic timer on the Destinations worker to trigger replans asynchronously.
    let replan_timer = destinations_worker.create_timer_repeating(
        std::time::Duration::from_millis(100),
        move |server: &mut PlanServer<P>| {
            server.replan();
        },
    )?;

    // Create the Discovery worker (Control Plane), passing a reference to the Destinations worker
    let discovery_worker = Arc::new(node.create_worker(DiscoveryServer::new(
        Arc::clone(&node),
        Arc::clone(&destinations_worker),
    )));

    let footprints_clone2 = Arc::clone(&footprints_clone);
    let list_subscription = discovery_worker
        .create_subscription::<rmf_prototype_msgs::msg::ParticipantList, _>(
            "/destination/discovery",
            move |_server: &mut DiscoveryServer<P>,
                  msg: rmf_prototype_msgs::msg::ParticipantList| {
                if let Ok(mut map) = footprints_clone2.lock() {
                    for p in msg.participants {
                        let radius = if p.radius > 0.0 { p.radius } else { 0.49 };
                        map.insert(p.name, radius);
                    }
                }
            },
        )?;

    // Subscribe to discovery on the Discovery worker using the shared generic helper
    let discovery_subscription = rmf_participant_discovery::create_discovery_subscription(
        &discovery_worker,
        "/destination/discovery",
        |server: &mut DiscoveryServer<P>, robot_id: &str| {
            if !server.active_robots.contains_key(robot_id) {
                rclrs::log!(
                    server.node.logger(),
                    "Discovered new participant: {}",
                    robot_id
                );

                let robot_id_clone = robot_id.to_string();
                let destination_topic = robot_id.to_string() + "/destination";
                let odom_topic = robot_id.to_string() + "/odom";

                // Create the subscription on the destinations_worker thread context!
                let destination_sub = match server
                    .destinations_worker
                    .create_subscription::<Destination, _>(
                        destination_topic.as_str().transient_local().reliable(),
                        move |dest_server: &mut PlanServer<P>, dest_msg: Destination| {
                            dest_server.handle_destination(&robot_id_clone, dest_msg);
                        },
                    ) {
                    Ok(sub) => sub,
                    Err(err) => {
                        rclrs::log_error!(
                            server.node.logger(),
                            "Failed to create destination subscription on DestinationsWorker for {}: {:?}",
                            robot_id,
                            err
                        );
                        return;
                    }
                };

                let robot_id_clone2 = robot_id.to_string();
                // Subscribe to robot_name/odom as well.
                let odom_sub = match server
                    .destinations_worker
                    .create_subscription::<Odometry, _>(
                        odom_topic.as_str().transient_local().reliable(),
                        move |dest_server: &mut PlanServer<P>, odom_msg: Odometry| {
                            dest_server.handle_odometry(&robot_id_clone2, odom_msg);
                        },
                    ) {
                    Ok(sub) => sub,
                    Err(err) => {
                        rclrs::log_error!(
                            server.node.logger(),
                            "Failed to create odom subscription on DestinationsWorker for {}: {:?}",
                            robot_id,
                            err
                        );
                        return;
                    }
                };

                server.active_robots.insert(
                    robot_id.to_string(),
                    RobotPathConnections {
                        _destination_subscription: destination_sub,
                        _odom_subscription: odom_sub,
                    },
                );
            }
        },
        |server: &mut DiscoveryServer<P>, robot_id: &str| {
            if server.active_robots.remove(robot_id).is_some() {
                rclrs::log!(server.node.logger(), "Participant left: {}", robot_id);
            }
        },
    )?;

    Ok(PathServerRunning {
        destinations_worker,
        discovery_worker,
        replan_timer: Box::new(replan_timer),
        list_subscription,
        discovery_subscription,
    })
}
