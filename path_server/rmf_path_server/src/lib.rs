// Copyright 2026 Open Source Robotics Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
use mapf_post::{na::Isometry2, MapfResult, SemanticPlan, SemanticWaypoint};
use rclrs::{IntoPrimitiveOptions, Node};
use ros_env::{
    builtin_interfaces,
    nav_msgs::msg::{OccupancyGrid, Odometry},
    rmf_prototype_msgs::{
        self,
        msg::{
            ControlPoint, Curve, Destination, Plan, PlanError, PlanId, Region, TargetRegion,
            TrafficDependency, Trajectory, Waypoint,
        },
    },
};
use std::{
    collections::{hash_map::Entry, HashMap},
    sync::Arc,
};

pub mod nav_graph;
pub use nav_graph::{NavGraphData, NavVertex, VertexAction};

pub mod planner;
pub use planner::{Map, MapfPlanner, MockPlanner, PibtPlanner};

pub struct PlanSuccess {
    pub session_id: u64,
    pub basic_plan: Vec<Vec<Isometry2<f32>>>,
    pub traffic_dependencies: SemanticPlan,
    pub goals: HashMap<String, Destination>,
    pub robot_ids: Vec<String>,
    pub active_plan: MapfResult,
    pub target_actions: HashMap<String, String>,
}

pub enum PlanResult {
    Success(PlanSuccess),
    Failure { session_id: u64, error: String },
}

pub struct PlanServer<P: MapfPlanner> {
    pub active_destinations: HashMap<String, Destination>,
    pub latest_pose_estimate: HashMap<String, Odometry>,
    pub node: Node,
    pub replan_queue: Vec<(String, Destination)>,
    pub planner: Arc<P>,
    pub plan_publishers: HashMap<String, rclrs::Publisher<Plan>>,
    pub plan_receiver: std::sync::Mutex<std::sync::mpsc::Receiver<PlanResult>>,
    pub plan_sender: std::sync::mpsc::Sender<PlanResult>,
    pub is_planning: bool,
    pub current_cancellation: Option<Arc<std::sync::atomic::AtomicBool>>,
    pub planning_session_id: u64,
    pub current_planning_session: Option<u64>,
    pub footprints: Arc<std::sync::Mutex<HashMap<String, f32>>>,
    pub active_plan_ids: HashMap<String, PlanId>,
    pub map: Arc<Map>,
    pub nav_graph: Option<Arc<NavGraphData>>,
    pub target_actions: HashMap<String, String>,
}

impl<P: MapfPlanner> PlanServer<P> {
    pub fn new(
        node: Node,
        planner: P,
        footprints: Arc<std::sync::Mutex<HashMap<String, f32>>>,
    ) -> Self {
        Self::new_with_nav_graph(node, planner, footprints, None)
    }

    pub fn new_with_nav_graph(
        node: Node,
        planner: P,
        footprints: Arc<std::sync::Mutex<HashMap<String, f32>>>,
        nav_graph: Option<Arc<NavGraphData>>,
    ) -> Self {
        let (plan_sender, plan_receiver) = std::sync::mpsc::channel();
        Self {
            active_destinations: HashMap::new(),
            latest_pose_estimate: HashMap::new(),
            node,
            replan_queue: Vec::new(),
            planner: Arc::new(planner),
            plan_publishers: HashMap::new(),
            plan_sender,
            plan_receiver: std::sync::Mutex::new(plan_receiver),
            is_planning: false,
            current_cancellation: None,
            planning_session_id: 0,
            current_planning_session: None,
            footprints,
            active_plan_ids: HashMap::new(),
            map: Arc::new(Map::default()),
            nav_graph,
            target_actions: HashMap::new(),
        }
    }

    pub fn handle_destination(&mut self, robot_id: &str, mut msg: Destination) {
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

        // Check if a graph key target node is specified and matches any vertex in the nav graph
        let mut looked_up_action = None;
        if let Some(nav_graph) = &self.nav_graph {
            for node_target in &msg.constraints.nodes {
                if let Some(vertex) = nav_graph.find_vertex(&node_target.key) {
                    rclrs::log!(
                        self.node.logger(),
                        "Matched destination graph key to vertex {} at ({}, {}) for robot {}",
                        vertex.id,
                        vertex.position[0],
                        vertex.position[1],
                        robot_id
                    );

                    // If region constraints are empty, resolve coordinates from vertex position
                    if msg.constraints.regions.is_empty() {
                        msg.constraints.regions.push(TargetRegion {
                            region: Region {
                                points: vec![vertex.position[0], vertex.position[1]],
                                hint: Region::HINT_POINT,
                            },
                            ..Default::default()
                        });
                    }

                    // Check for special arrival action (e.g. docking)
                    if let Some(action) = &vertex.arrival_action {
                        rclrs::log!(
                            self.node.logger(),
                            "Found special action for robot {} at vertex {}: {:?}",
                            robot_id,
                            vertex.id,
                            action
                        );
                        looked_up_action = Some(action.name.clone());
                    }
                    break;
                }
            }
        }

        if let Some(action) = looked_up_action {
            self.target_actions.insert(robot_id.to_string(), action);
        } else {
            self.target_actions.remove(robot_id);
        }

        let is_new_session = match self.active_destinations.get(robot_id) {
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
            self.replan_queue.push((robot_id.to_owned(), msg));
        } else {
            rclrs::log_error!(self.node.logger(), "Duplicate session id received");
        }
    }

    /// For now both the plan server and executor's logic is embodied in this
    /// function.
    ///
    /// Note: We do not trigger a replan directly from this function when new
    /// odometry arrives. Instead, a 100ms periodic timer (configured in `start_path_server`)
    /// polls `replan()`, which will process any queued replan requests once
    /// odometry for all active robots is available. This prevents planning freeze
    /// if planning was previously skipped due to missing odometry.
    pub fn handle_odometry(&mut self, robot_id: &str, msg: Odometry) {
        self.latest_pose_estimate
            .insert(robot_id.to_string(), msg.clone());
    }

    pub fn handle_plan_error(&mut self, robot_id: &str, msg: PlanError) {
        if msg.error.code == PlanError::CODE_PATH_BLOCKED {
            rclrs::log_warn!(
                self.node.logger(),
                "Received CODE_PATH_BLOCKED for robot {}. Enqueuing replan...",
                robot_id
            );
            if let Some(dest) = self.active_destinations.get(robot_id).cloned() {
                self.replan_queue.push((robot_id.to_string(), dest));
            }
        }
    }

    pub fn replan(&mut self) {
        // 1. Check if any planning results have arrived from the background thread
        if let Ok(receiver) = self.plan_receiver.get_mut() {
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
                            target_actions,
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
                            let Some(traj) = &basic_plan.get(agent_idx) else {
                                rclrs::log_error!(
                                    self.node.logger(),
                                    "Missing plan trajectory for agent {}",
                                    agent_idx
                                );
                                continue;
                            };
                            let target_action = target_actions.get(robot_id).map(|s| s.as_str());
                            let plan = Self::to_plan_msg(
                                agent_idx,
                                traj,
                                plan_id,
                                &traffic_dependencies,
                                &robot_ids,
                                &self.active_plan_ids,
                                target_action,
                                1.0,
                            );
                            plans.insert(robot_id.clone(), plan);
                        }

                        for (robot_id, plan) in &plans {
                            let mut wp_strs = Vec::new();
                            for (j, wp) in plan.waypoints.iter().enumerate() {
                                let blockers: Vec<String> = wp
                                    .departure_blockers
                                    .iter()
                                    .map(|b| {
                                        format!("{} progress >= {}", b.name, b.required_progress)
                                    })
                                    .collect();
                                wp_strs.push(format!(
                                    "  wp {}: pos {:?}, progress {}, action: '{}', blockers: {:?}",
                                    j, wp.position, wp.progress, wp.arrival_action, blockers
                                ));
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
                            let publisher = match self.plan_publishers.entry(robot_id.clone()) {
                                Entry::Occupied(entry) => entry.into_mut(),
                                Entry::Vacant(entry) => {
                                    let topic = format!("{}/plan", robot_id);
                                    match self.node.create_publisher::<Plan>(
                                        topic.as_str().transient_local().reliable(),
                                    ) {
                                        Ok(pub_) => entry.insert(pub_),
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
                            };

                            if let Err(err) = publisher.publish(plan) {
                                rclrs::log_error!(
                                    self.node.logger(),
                                    "Failed to publish plan for {}: {:?}",
                                    robot_id,
                                    err
                                );
                            }
                        }

                        self.active_destinations = goals;
                    }
                    PlanResult::Failure { session_id, error } => {
                        // TODO(arjoc): Publish error message
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
        let mut goals = self.active_destinations.clone();

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
        let sender_clone = self.plan_sender.clone();
        let map_clone = self.map.clone();
        let target_actions_clone = self.target_actions.clone();

        std::thread::spawn(move || {
            if cancellation.load(std::sync::atomic::Ordering::Relaxed) {
                return;
            }

            let footprints_map: HashMap<String, Arc<dyn mapf_post::shape::Shape>> = {
                let guard = footprints_clone.lock();
                robot_ids
                    .iter()
                    .map(|id| {
                        let radius = match guard.as_ref() {
                            Ok(map) => *map.get(id).unwrap_or(&0.49),
                            Err(_) => 0.49,
                        };
                        (
                            id.clone(),
                            Arc::new(mapf_post::shape::Ball::new(radius))
                                as Arc<dyn mapf_post::shape::Shape>,
                        )
                    })
                    .collect()
            };

            let basic_plan = match planner_clone.plan(
                &starts,
                &goals,
                &footprints_map,
                &robot_ids,
                map_clone.as_ref(),
                Arc::clone(&cancellation),
            ) {
                Ok(plan) => plan,
                Err(err) => {
                    // TODO(arjoc): publish an error on the {robot}/destination/errors
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
                target_actions: target_actions_clone,
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
        target_action: Option<&str>,
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

        if let Some(action) = target_action {
            if let Some(last_wp) = waypoints.last_mut() {
                last_wp.arrival_action = action.to_string();
            }
        }

        // For any docking waypoints, populate departure_trajectory
        let num_waypoints = waypoints.len();
        for i in 0..num_waypoints {
            let arrival_act = waypoints[i].arrival_action.clone();
            let is_docking = !arrival_act.is_empty()
                && (arrival_act.starts_with("dock") || arrival_act.contains("dock"));
            if is_docking && waypoints[i].departure_trajectory.is_empty() {
                let [dock_x, dock_y] = waypoints[i].position;
                let depart_pos = if i + 1 < num_waypoints {
                    waypoints[i + 1].position
                } else if i > 0 {
                    waypoints[i - 1].position
                } else {
                    [dock_x, dock_y]
                };
                // TODO(arjoc) parameterize it
                let departure_duration = 1.0f32;
                let departure_curve = Curve {
                    degree: 1,
                    control_points: vec![
                        ControlPoint {
                            position: [dock_x, dock_y],
                            weight: 1.0,
                        },
                        ControlPoint {
                            position: depart_pos,
                            weight: 1.0,
                        },
                    ],
                    knots: vec![0.0, 0.0, departure_duration, departure_duration],
                };

                let departure_traj = Trajectory {
                    curve: departure_curve,
                    initial_progress_level: waypoints[i].progress,
                    final_progress_level: waypoints[i].progress + departure_duration,
                    maps: waypoints[i].maps.clone(),
                    keys: Vec::new(),
                };

                waypoints[i].departure_trajectory = vec![departure_traj];
            }
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
    pub _plan_error_subscription: rclrs::WorkerSubscription<PlanError, PlanServer<P>>,
}

pub struct DiscoveryServer<P: MapfPlanner> {
    pub node: Node,
    pub active_robots: HashMap<String, RobotPathConnections<P>>,
    pub destinations_worker: rclrs::Worker<PlanServer<P>>,
}

impl<P: MapfPlanner> DiscoveryServer<P> {
    pub fn new(node: Node, destinations_worker: rclrs::Worker<PlanServer<P>>) -> Self {
        Self {
            node,
            active_robots: HashMap::new(),
            destinations_worker,
        }
    }
}

pub struct PathServerRunning<P: MapfPlanner> {
    pub destinations_worker: rclrs::Worker<PlanServer<P>>,
    pub discovery_worker: rclrs::Worker<DiscoveryServer<P>>,
    pub replan_timer: Box<dyn std::any::Any + Send + Sync>,
    pub list_subscription:
        rclrs::WorkerSubscription<rmf_prototype_msgs::msg::ParticipantList, DiscoveryServer<P>>,
    pub discovery_subscription:
        rclrs::WorkerSubscription<rmf_prototype_msgs::msg::ParticipantList, DiscoveryServer<P>>,
    pub map_subscription: rclrs::WorkerSubscription<OccupancyGrid, PlanServer<P>>,
}

pub fn start_path_server<P: MapfPlanner + 'static>(
    node: rclrs::Node,
    planner: P,
) -> Result<PathServerRunning<P>, Box<dyn std::error::Error>> {
    let nav_graph = match node
        .declare_parameter("site_file")
        .default(Arc::from(""))
        .mandatory()
    {
        Ok(param) => {
            let path: Arc<str> = param.get();
            if !path.is_empty() {
                match NavGraphData::from_site_file(&path) {
                    Ok(graph) => {
                        rclrs::log!(
                            node.logger(),
                            "Loaded navigation graph from '{}' with {} vertices",
                            path,
                            graph.vertices_by_id.len()
                        );
                        Some(Arc::new(graph))
                    }
                    Err(err) => {
                        rclrs::log_error!(
                            node.logger(),
                            "Failed to load site file '{}': {:?}",
                            path,
                            err
                        );
                        None
                    }
                }
            } else {
                None
            }
        }
        Err(err) => {
            rclrs::log_warn!(
                node.logger(),
                "Could not declare optional 'site_file' parameter: {:?}",
                err
            );
            None
        }
    };

    start_path_server_with_nav_graph(node, planner, nav_graph)
}

pub fn start_path_server_with_nav_graph<P: MapfPlanner + 'static>(
    node: rclrs::Node,
    planner: P,
    nav_graph: Option<Arc<NavGraphData>>,
) -> Result<PathServerRunning<P>, Box<dyn std::error::Error>> {
    let footprints = Arc::new(std::sync::Mutex::new(std::collections::HashMap::new()));
    let footprints_clone = Arc::clone(&footprints);

    // Create the Destinations worker
    let destinations_worker = node.create_worker(PlanServer::new_with_nav_graph(
        node.clone(),
        planner,
        footprints,
        nav_graph,
    ));

    let map_subscription = destinations_worker.create_subscription::<OccupancyGrid, _>(
        "/map".transient_local().reliable(),
        move |server: &mut PlanServer<P>, msg: OccupancyGrid| {
            rclrs::log!(server.node.logger(), "Received map message");
            server.map = Arc::new(Map { grid: msg });
        },
    )?;

    // Create a periodic timer on the Destinations worker to trigger replans asynchronously.
    let replan_timer = destinations_worker.create_timer_repeating(
        std::time::Duration::from_millis(100),
        move |server: &mut PlanServer<P>| {
            server.replan();
        },
    )?;

    // Create the Discovery worker (Control Plane), passing a reference to the Destinations worker
    let discovery_worker = node.create_worker(DiscoveryServer::new(
        node.clone(),
        destinations_worker.clone(),
    ));

    let footprints_clone2 = Arc::clone(&footprints_clone);
    let list_subscription = discovery_worker
        .create_subscription::<rmf_prototype_msgs::msg::ParticipantList, _>(
            "/destination/discovery".transient_local().reliable(),
            move |_server: &mut DiscoveryServer<P>,
                  msg: rmf_prototype_msgs::msg::ParticipantList| {
                if let Ok(mut map) = footprints_clone2.lock() {
                    for p in msg.participants {
                        map.insert(p.name, 0.49);
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
                let odom_sub = match server
                    .destinations_worker
                    .create_subscription::<Odometry, _>(
                        odom_topic.as_str().reliable(),
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

                let robot_id_clone3 = robot_id.to_string();
                let plan_error_topic = robot_id.to_string() + "/plan/error";
                let plan_error_sub = match server
                    .destinations_worker
                    .create_subscription::<PlanError, _>(
                        plan_error_topic.as_str(),
                        move |dest_server: &mut PlanServer<P>, error_msg: PlanError| {
                            dest_server.handle_plan_error(&robot_id_clone3, error_msg);
                        },
                    ) {
                    Ok(sub) => sub,
                    Err(err) => {
                        rclrs::log_error!(
                            server.node.logger(),
                            "Failed to create plan error subscription on DestinationsWorker for {}: {:?}",
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
                        _plan_error_subscription: plan_error_sub,
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
        map_subscription,
    })
}
