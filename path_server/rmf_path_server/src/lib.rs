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
use mapf_post::{na::Isometry2, MapfResult, SemanticPlan};
use rclrs::{IntoPrimitiveOptions, Node};
use ros_env::{
    nav_msgs::msg::OccupancyGrid,
    rmf_prototype_msgs::{
        self,
        msg::{Destination, Plan, PlanError, RobotStatus},
    },
};
use std::{
    collections::{hash_map::Entry, HashMap},
    sync::Arc,
};

pub mod plan_registry;
pub mod planner;
pub use plan_registry::PlanRegistry;
pub use planner::{Map, MapfPlanner, MockPlanner, PibtPlanner};

pub struct PlanSuccess {
    pub session_id: u64,
    pub basic_plan: Vec<Vec<Isometry2<f32>>>,
    pub traffic_dependencies: SemanticPlan,
    pub goals: HashMap<String, Destination>,
    pub robot_ids: Vec<String>,
    pub active_plan: MapfResult,
    pub plans: HashMap<String, Plan>,
}

pub enum PlanResult {
    Success(PlanSuccess),
    Failure { session_id: u64, error: String },
}

pub struct PlanServer<P: MapfPlanner> {
    pub active_destinations: HashMap<String, Destination>,
    pub latest_status: HashMap<String, RobotStatus>,
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
    pub plan_registry: PlanRegistry,
    pub map: Arc<Map>,
}

impl<P: MapfPlanner> PlanServer<P> {
    pub fn new(
        node: Node,
        planner: P,
        footprints: Arc<std::sync::Mutex<HashMap<String, f32>>>,
    ) -> Self {
        let (plan_sender, plan_receiver) = std::sync::mpsc::channel();
        Self {
            active_destinations: HashMap::new(),
            latest_status: HashMap::new(),
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
            plan_registry: PlanRegistry::new(),
            map: Arc::new(Map::default()),
        }
    }

    pub fn is_executing_action(&self, robot_id: &str) -> bool {
        let Some(status) = self.latest_status.get(robot_id) else {
            return false;
        };
        self.plan_registry.is_executing_action(robot_id, status)
    }

    pub fn occupied_path_of(&self, robot_id: &str) -> Vec<[f32; 2]> {
        let Some(status) = self.latest_status.get(robot_id) else {
            return Vec::new();
        };
        self.plan_registry.occupied_path_of(robot_id, status)
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
    /// Note: We do not trigger a replan directly from this function when a new
    /// status arrives. Instead, a 100ms periodic timer (configured in `start_path_server`)
    /// polls `replan()`, which will process any queued replan requests once
    /// status for all active robots is available. This prevents planning freeze
    /// if planning was previously skipped due to missing status.
    pub fn handle_status(&mut self, robot_id: &str, msg: RobotStatus) {
        self.latest_status.insert(robot_id.to_string(), msg);
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
                            session_id,
                            goals,
                            plans,
                            ..
                        } = success;

                        self.plan_registry.record_session_plans(session_id, &plans);

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
                                    "  wp {}: pos {:?}, progress {}, blockers: {:?}",
                                    j, wp.position, wp.progress, blockers
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

                        self.active_destinations.extend(goals);
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

        // 3. If there is no actionable (non-committed) robot in the replan queue,
        // wait until either a non-committed robot requests a replan or a committed
        // robot finishes its action.
        if !self
            .replan_queue
            .iter()
            .any(|(robot_id, _)| !self.is_executing_action(robot_id))
        {
            return;
        }

        rclrs::log!(self.node.logger(), "Need to trigger plan");
        // Retrieve goals of all participants
        let mut all_goals = self.active_destinations.clone();

        for (robot_id, dest) in &self.replan_queue {
            all_goals.insert(robot_id.clone(), dest.clone());
        }

        let mut starts = HashMap::new();
        let mut goals = HashMap::new();
        let mut committed_robots = Vec::new();
        for (robot_id, dest) in &all_goals {
            // Make sure we have the latest status for all robots.
            // Give up if status for some robots is stale.
            if let Some(status) = self.latest_status.get(robot_id) {
                if self.is_executing_action(robot_id) {
                    rclrs::log!(
                        self.node.logger(),
                        "Robot {} is executing an action; excluding it from replanning and marking its occupied path",
                        robot_id
                    );
                    committed_robots.push(robot_id.clone());
                } else {
                    starts.insert(robot_id.clone(), status.clone());
                    goals.insert(robot_id.clone(), dest.clone());
                }
            } else {
                rclrs::log!(
                    self.node.logger(),
                    "Status for robot {} is missing or stale. Skipping plan.",
                    robot_id
                );
                return;
            }
        }

        // Retain any replan requests for robots that were deferred because they are mid-action.
        self.replan_queue
            .retain(|(robot_id, _)| committed_robots.contains(robot_id));

        if goals.is_empty() {
            return;
        }

        // Perform MAPF plan in background thread
        rclrs::log!(self.node.logger(), "Triggering new plan in background");

        let mut robot_ids: Vec<String> = goals.keys().cloned().collect();
        robot_ids.sort();

        let mut map = (*self.map).clone();
        for committed_id in &committed_robots {
            map.mark(&self.occupied_path_of(committed_id));
        }

        self.planning_session_id += 1;
        let session_id = self.planning_session_id;
        self.current_planning_session = Some(session_id);
        self.is_planning = true;

        let cancellation = Arc::new(std::sync::atomic::AtomicBool::new(false));
        self.current_cancellation = Some(Arc::clone(&cancellation));

        let planner_clone = Arc::clone(&self.planner);
        let footprints_clone = Arc::clone(&self.footprints);
        let sender_clone = self.plan_sender.clone();
        let registry_snapshot = self.plan_registry.clone();

        std::thread::spawn(move || {
            if cancellation.load(std::sync::atomic::Ordering::Relaxed) {
                return;
            }

            let footprints_map: HashMap<String, Arc<dyn mapf_post::shape::Shape>> = {
                let guard = footprints_clone.lock();
                let mut map: HashMap<String, Arc<dyn mapf_post::shape::Shape>> =
                    match guard.as_ref() {
                        Ok(radii) => radii
                            .iter()
                            .map(|(id, &radius)| {
                                (
                                    id.clone(),
                                    Arc::new(mapf_post::shape::Ball::new(radius))
                                        as Arc<dyn mapf_post::shape::Shape>,
                                )
                            })
                            .collect(),
                        Err(_) => HashMap::new(),
                    };
                for id in &robot_ids {
                    map.entry(id.clone())
                        .or_insert_with(|| Arc::new(mapf_post::shape::Ball::new(0.49)));
                }
                map
            };

            let basic_plan = match planner_clone.plan(
                &starts,
                &goals,
                &footprints_map,
                &robot_ids,
                &map,
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

            let (plans, mapf_result, traffic_dependencies) = registry_snapshot.build_session_plans(
                &basic_plan,
                &goals,
                &robot_ids,
                &footprints_map,
                1.0,
            );

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
                plans,
            }));
        });
    }
}

pub struct RobotPathConnections<P: MapfPlanner> {
    pub _destination_subscription: rclrs::WorkerSubscription<Destination, PlanServer<P>>,
    pub _status_subscription: rclrs::WorkerSubscription<RobotStatus, PlanServer<P>>,
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
    let footprints = Arc::new(std::sync::Mutex::new(std::collections::HashMap::new()));
    let footprints_clone = Arc::clone(&footprints);

    // Create the Destinations worker
    let destinations_worker =
        node.create_worker(PlanServer::new(node.clone(), planner, footprints));

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
                let status_topic = robot_id.to_string() + "/status";

                // Create the subscription on the destinations_worker thread context!
                let destination_sub = match server
                    .destinations_worker
                    .create_subscription::<Destination, _>(
                        destination_topic.as_str().transient_local().reliable(),
                        move |dest_server: &mut PlanServer<P>, dest_msg: Destination| {
                            //rclrs::log!(server.node.logger(), "Received destination for robot");
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
                let status_sub = match server
                    .destinations_worker
                    .create_subscription::<RobotStatus, _>(
                        status_topic.as_str().transient_local().reliable(),
                        move |dest_server: &mut PlanServer<P>, status_msg: RobotStatus| {
                            dest_server.handle_status(&robot_id_clone2, status_msg);
                        },
                    ) {
                    Ok(sub) => sub,
                    Err(err) => {
                        rclrs::log_error!(
                            server.node.logger(),
                            "Failed to create status subscription on DestinationsWorker for {}: {:?}",
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
                        _status_subscription: status_sub,
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
