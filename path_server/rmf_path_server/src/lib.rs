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
            ControlPoint, Curve, Destination, GraphElementKey, Plan, PlanError, PlanId, Progress,
            Region, TargetNode, TargetOrientation, TargetRegion, TrafficDependency, Trajectory,
            Waypoint,
        },
    },
};
use std::{
    collections::{hash_map::Entry, HashMap},
    sync::Arc,
};

// The navigation graph moved into its own package so that `rmf_plan_executor`
// can resolve dock geometry without depending on the whole traffic planner.
// Re-exported under the original paths so existing callers do not have to care.
pub use rmf_nav_graph as nav_graph;
pub use rmf_nav_graph::{NavGraphData, NavVertex, VertexAction};

pub mod planner;
pub use planner::{Map, MapfPlanner, MockPlanner, PibtPlanner};

pub mod dock_maneuver;
pub use dock_maneuver::{
    pad_to_equal_length, splice_dock_maneuvers, DockAnnotations, DockManeuvers, DockSuffix,
    UndockPrefix,
};

/// Wall-clock seconds between consecutive poses of a MAPF trajectory.
///
/// Everything downstream is measured against this: waypoint progress levels and
/// the sweep window `mapf_post` uses when testing two motions for collision.
pub const DISCRETIZATION_TIMESTEP: f32 = 1.0;

/// A dock that a robot has been asked to end its plan at.
///
/// The plan's goal is the *staging* vertex at the mouth of the dock lane; this
/// records the far end so the maneuver can be appended after planning.
#[derive(Clone, Debug)]
pub struct TargetDock {
    pub action: VertexAction,
    /// Position of the dock vertex itself.
    pub dock_position: [f32; 2],
}

/// Where a given agent's dock and destination ended up in its trajectory.
///
/// Both are needed because splicing and padding move them apart: the dock sits
/// one waypoint past the destination, and padding may sit past the dock.
#[derive(Clone, Copy, Debug, Default)]
pub struct PlanIndices {
    pub dock: Option<usize>,
    pub destination: usize,
}

/// Whether a progress report says its robot is physically committed to an
/// action, and so must be left alone.
///
/// Split out from [`PlanServer::is_committed`] so the rule can be exercised
/// without standing up a ROS node. Three things have to hold, and each of them
/// fails safe towards "not committed", i.e. towards replanning a robot rather
/// than abandoning one:
///
/// * there is a report at all;
/// * it is about the plan we last published, since progress measured against a
///   plan that no longer exists says nothing about the current one;
/// * it reports `EXECUTION_STATE_EXECUTING_ACTION`.
pub fn reports_commitment(progress: Option<&Progress>, active_plan_id: Option<&PlanId>) -> bool {
    let (Some(progress), Some(active_plan_id)) = (progress, active_plan_id) else {
        return false;
    };
    progress.plan_id == *active_plan_id
        && progress.execution_state == Progress::EXECUTION_STATE_EXECUTING_ACTION
}

/// Whether a robot holds a destination that the plan it is currently executing
/// was not built for.
///
/// Split out from [`PlanServer::has_unplanned_destination`] so the rule can be
/// exercised without standing up a ROS node.
///
/// A plan carries the session uuid of the destination it was planned for, so
/// comparing that against the current destination separates "a new task arrived
/// while this robot was busy" from "this robot is already doing the thing it was
/// asked to do". Only the former needs replanning.
///
/// Fails safe towards replanning: a robot that has a destination but has never
/// been given a plan is treated as owed one.
pub fn needs_replanning(
    destination: Option<&Destination>,
    active_plan_id: Option<&PlanId>,
) -> bool {
    let Some(destination) = destination else {
        return false;
    };
    match active_plan_id {
        Some(plan_id) => plan_id.destination_session.uuid != destination.session.uuid,
        None => true,
    }
}

pub struct PlanSuccess {
    pub session_id: u64,
    pub basic_plan: Vec<Vec<Isometry2<f32>>>,
    pub traffic_dependencies: SemanticPlan,
    pub goals: HashMap<String, Destination>,
    pub robot_ids: Vec<String>,
    pub active_plan: MapfResult,
    pub target_actions: HashMap<String, TargetDock>,
    /// Dock/undock maneuvers that were spliced into `basic_plan`. Retained so
    /// that the `Plan` message can attach actions to the correct waypoints.
    pub maneuvers: HashMap<String, DockManeuvers>,
    /// Per-agent waypoint indices, recorded before padding obscured them.
    pub plan_indices: Vec<PlanIndices>,
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
    pub target_actions: HashMap<String, TargetDock>,
    /// The most recent `~/plan/progress` from each participant.
    ///
    /// Held for one question only: is this robot physically committed to an
    /// action right now? See [`PlanServer::is_committed`].
    pub latest_progress: HashMap<String, Progress>,
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
            latest_progress: HashMap::new(),
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

        // Check if destination targets a nav graph vertex or docking station
        let mut looked_up_action = None;
        if let Some(nav_graph) = &self.nav_graph {
            let mut matched_vertex = None;

            // 1. Try matching by target node graph key (vertex ID or location/dock name)
            for node_target in &msg.constraints.nodes {
                if let Some(vertex) = nav_graph.find_vertex(&node_target.key) {
                    matched_vertex = Some(vertex);
                    break;
                }
            }

            // 2. If not matched, try matching by proximity to a docking station
            // (e.g. client sent raw contact pose (0.0, 0.95) for a conveyor dock)
            if matched_vertex.is_none() {
                if let Some(first_reg) = msg.constraints.regions.first() {
                    if first_reg.region.points.len() >= 2 {
                        let x = first_reg.region.points[0];
                        let y = first_reg.region.points[1];
                        if let Some(dock_vertex) =
                            nav_graph.find_dock_vertex_by_proximity(x, y, 0.8)
                        {
                            rclrs::log!(
                                self.node.logger(),
                                "Destination point ({}, {}) resolved to dock vertex {} at ({}, {}) by proximity for robot {}",
                                x,
                                y,
                                dock_vertex.id,
                                dock_vertex.position[0],
                                dock_vertex.position[1],
                                robot_id
                            );
                            matched_vertex = Some(dock_vertex);
                        }
                    }
                }
            }

            // 3. If a vertex is matched, snap the destination to it.
            //
            // For a dock we deliberately stop *short*: the goal becomes the
            // staging vertex at the mouth of the dock lane, not the dock vertex
            // itself. The lane between them is appended afterwards as an
            // explicit maneuver so that `mapf_post` can see the corridor and
            // `dock_entry_gate` can decide when it is safe to enter. Making the
            // dock the goal would bury that motion inside the planner, which is
            // how the corridor came to be invisible in the first place.
            if let Some(vertex) = matched_vertex {
                let goal_position = vertex.undock_position.unwrap_or(vertex.position);
                rclrs::log!(
                    self.node.logger(),
                    "Resolved destination to vertex {} at ({}, {}) for robot {}; planning to ({}, {})",
                    vertex.id,
                    vertex.position[0],
                    vertex.position[1],
                    robot_id,
                    goal_position[0],
                    goal_position[1]
                );

                if msg.constraints.regions.is_empty() {
                    msg.constraints.regions.push(TargetRegion {
                        region: Region {
                            points: vec![goal_position[0], goal_position[1]],
                            hint: Region::HINT_POINT,
                        },
                        ..Default::default()
                    });
                } else if let Some(first_reg) = msg.constraints.regions.first_mut() {
                    first_reg.region.points = vec![goal_position[0], goal_position[1]];
                    first_reg.region.hint = Region::HINT_POINT;
                }

                // If vertex has an approach orientation (e.g. facing dock), enforce target orientation
                if let Some(ori) = vertex.orientation {
                    let target_ori = TargetOrientation {
                        orientation_radians: ori,
                        spread_radians: 0.0,
                        tolerance_radians: 0.05,
                    };
                    if let Some(first_reg) = msg.constraints.regions.first_mut() {
                        first_reg.orientations = vec![target_ori.clone()];
                    }
                    if !msg.constraints.nodes.is_empty() {
                        for node in msg.constraints.nodes.iter_mut() {
                            node.orientations = vec![target_ori.clone()];
                        }
                    } else {
                        let mut key = GraphElementKey::default();
                        if let Ok(seq) = vec![vertex.id as i64].try_into() {
                            key.key = seq;
                        }
                        if let Some(vname) = &vertex.name {
                            if let Ok(seq) = vec![vname.clone().into()].try_into() {
                                key.name = seq;
                            }
                        }
                        msg.constraints.nodes.push(TargetNode {
                            key,
                            orientations: vec![target_ori],
                        });
                    }
                }

                // Check for special arrival action (e.g. docking)
                if let Some(action) = &vertex.arrival_action {
                    rclrs::log!(
                        self.node.logger(),
                        "Found special action for robot {} at dock vertex {}: {:?}",
                        robot_id,
                        vertex.id,
                        action
                    );
                    looked_up_action = Some(TargetDock {
                        action: action.clone(),
                        dock_position: vertex.position,
                    });
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
            self.active_destinations
                .insert(robot_id.to_string(), msg.clone());
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

    pub fn handle_progress(&mut self, robot_id: &str, msg: Progress) {
        let was_committed = self.is_committed(robot_id);
        self.latest_progress.insert(robot_id.to_string(), msg);

        if !was_committed || self.is_committed(robot_id) {
            return;
        }

        // The robot has just been released from an action. Nothing else will ask
        // for it to be routed anywhere, because it was held out of every
        // negotiation for as long as it was committed - but it is only owed a
        // plan if a destination arrived during that time.
        //
        // Re-queuing unconditionally looks harmless and is not. A destination
        // that resolved to a dock is still sitting in `active_destinations`
        // after the dock *succeeds*, so the replan regenerates the very same
        // dock suffix; the new plan_id makes the executor re-issue the maneuver;
        // and the robot undocks and re-docks forever. It is committed for almost
        // all of that cycle, so every genuinely new destination is filtered out
        // by `replan` and the robot looks permanently unassignable.
        if !self.has_unplanned_destination(robot_id) {
            rclrs::log_debug!(
                self.node.logger(),
                "Robot {} has finished its action and its current plan already serves its \
                 destination; nothing to replan",
                robot_id
            );
            return;
        }

        rclrs::log!(
            self.node.logger(),
            "Robot {} has finished its action; queuing the replan that was deferred",
            robot_id
        );
        if let Some(dest) = self.active_destinations.get(robot_id).cloned() {
            self.replan_queue.push((robot_id.to_string(), dest));
        }
    }

    /// Whether `robot_id` holds a destination that the plan it is currently
    /// executing was not built for. See [`needs_replanning`].
    pub fn has_unplanned_destination(&self, robot_id: &str) -> bool {
        needs_replanning(
            self.active_destinations.get(robot_id),
            self.active_plan_ids.get(robot_id),
        )
    }

    /// Whether `robot_id` is physically committed to an action and so must not
    /// be replanned.
    ///
    /// This trusts the reported `execution_state` instead of inferring
    /// commitment from waypoint indices, which is what `Progress.msg` asks of a
    /// traffic planner: only the executor knows whether a docking controller
    /// currently has the robot, and the indices cannot distinguish "parked at
    /// the last waypoint" from "driving into the dock".
    ///
    /// A report about any plan other than the one we last published is ignored.
    /// Progress means nothing except against the plan it was measured on, and a
    /// robot cannot be committed to an action in a plan that no longer exists.
    pub fn is_committed(&self, robot_id: &str) -> bool {
        reports_commitment(
            self.latest_progress.get(robot_id),
            self.active_plan_ids.get(robot_id),
        )
    }

    /// The space a committed robot has to be treated as holding, as an ordered
    /// swept path.
    ///
    /// Where it is, then the dock vertex named by `active_action`. It is
    /// somewhere on the line between the two and we cannot tell where, so the
    /// planner is handed the whole segment and rasterises it; claiming only the
    /// two endpoints would leave the cells it crosses in between open.
    ///
    /// The *staging* vertex is deliberately not claimed even though the robot
    /// passes through it. Staging sits on the travel network, and at the
    /// planner's 1 m resolution claiming it would blank a cell the through
    /// route shares, cutting the aisle in half for as long as the dock takes.
    /// The robot's own odometry already covers staging for as long as it is
    /// actually standing there.
    fn frozen_claims(&self, robot_id: &str) -> Vec<[f32; 2]> {
        let mut claims = Vec::new();

        if let Some(odom) = self.latest_pose_estimate.get(robot_id) {
            claims.push([
                odom.pose.pose.position.x as f32,
                odom.pose.pose.position.y as f32,
            ]);
        }

        let action = self
            .latest_progress
            .get(robot_id)
            .map(|progress| progress.active_action.as_str())
            .unwrap_or_default();
        if let Some(nav_graph) = &self.nav_graph {
            if let Some(vertex) = nav_graph
                .vertices_by_name
                .get(action)
                .and_then(|id| nav_graph.vertices_by_id.get(id))
            {
                claims.push(vertex.position);
            }
        }

        claims
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
                            maneuvers,
                            plan_indices,
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
                            let maneuver = maneuvers.get(robot_id);
                            let indices = plan_indices.get(agent_idx).copied().unwrap_or_default();
                            let annotations = DockAnnotations {
                                departure_action: maneuver
                                    .and_then(|m| m.undock.as_ref())
                                    .map(|_| "undock"),
                                arrival_action: target_actions
                                    .get(robot_id)
                                    .map(|target| target.action.name.as_str()),
                                dock_index: indices.dock,
                                destination_index: indices.destination,
                            };
                            let mut plan = Self::to_plan_msg(
                                agent_idx,
                                traj,
                                plan_id,
                                &traffic_dependencies,
                                &robot_ids,
                                &self.active_plan_ids,
                                &annotations,
                                DISCRETIZATION_TIMESTEP,
                            );
                            if let Some(dest) = goals.get(robot_id) {
                                // The destination is the staging vertex when a
                                // dock is appended, so its constraints belong
                                // there rather than on the dock waypoint. The
                                // approach into the dock itself is nav2's.
                                if let Some(dest_wp) =
                                    plan.waypoints.get_mut(annotations.destination_index)
                                {
                                    dest_wp.arrival_constraints = dest.constraints.clone();
                                }
                            }
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
        let mut maneuvers: HashMap<String, DockManeuvers> = HashMap::new();
        for robot_id in goals.keys() {
            // Make sure we have the latest odometry for all robots.
            // Give up if odometry for some robots is stale.
            if let Some(odom) = self.latest_pose_estimate.get(robot_id) {
                let mut start_odom = odom.clone();
                // If robot is at a docked vertex or corridor, assume plan starts at the undocked vertex
                if let Some(nav_graph) = &self.nav_graph {
                    let rx = start_odom.pose.pose.position.x as f32;
                    let ry = start_odom.pose.pose.position.y as f32;
                    if let Some(dock_vertex) = nav_graph.find_dock_vertex_by_proximity(rx, ry, 0.8)
                    {
                        let undock_pos =
                            dock_vertex.undock_position.unwrap_or(dock_vertex.position);
                        rclrs::log!(
                            self.node.logger(),
                            "Robot {} is at docked position ({:.2}, {:.2}); planning from undocked vertex ({:.2}, {:.2}) and splicing the undock back in",
                            robot_id,
                            rx,
                            ry,
                            undock_pos[0],
                            undock_pos[1]
                        );
                        // The grid planner cannot start inside a dock: the lane
                        // is finer than MIN_PLANNING_RESOLUTION. So we still
                        // hand it the staging vertex, but we record the pose it
                        // is really at so `dock_maneuver` can put the undock
                        // back into the trajectory before conflict analysis.
                        maneuvers.entry(robot_id.clone()).or_default().undock =
                            Some(UndockPrefix {
                                docked_pose: Isometry2::translation(rx, ry),
                                via: Some(Isometry2::translation(
                                    dock_vertex.position[0],
                                    dock_vertex.position[1],
                                )),
                            });
                        start_odom.pose.pose.position.x = undock_pos[0] as f64;
                        start_odom.pose.pose.position.y = undock_pos[1] as f64;
                    }
                }
                starts.insert(robot_id.clone(), start_odom);
            } else {
                rclrs::log!(
                    self.node.logger(),
                    "Odometry for robot {} is missing or stale. Skipping plan.",
                    robot_id
                );
                return;
            }
        }

        // Any robot whose destination resolved to a dock gets the dock lane
        // appended after its plan. There is no duration attached: how long the
        // robot spends in the dock is reported at runtime, not planned.
        for (robot_id, target) in &self.target_actions {
            if !goals.contains_key(robot_id) || target.action.action_type != "dock" {
                continue;
            }
            maneuvers.entry(robot_id.clone()).or_default().dock = Some(DockSuffix {
                dock_pose: Isometry2::translation(target.dock_position[0], target.dock_position[1]),
            });
        }

        // A robot part-way through a dock cannot be replanned and will not move
        // aside, so it is not a participant in this negotiation. It keeps the
        // plan and the plan_id it already has; the planner is told about the
        // space it is holding instead of being asked to route it.
        //
        // Its destination deliberately stays in `goals`, and so survives into
        // `active_destinations` below. Dropping it there would strand the robot
        // at the dock with nothing to return to once the action finishes.
        let frozen: Vec<String> = goals
            .keys()
            .filter(|robot_id| self.is_committed(robot_id))
            .cloned()
            .collect();

        // Everyone we are about to plan for has had their request taken up, so
        // their queue entries can go. A frozen robot's cannot: it is not in this
        // negotiation, so dropping its entry destroys the request outright.
        //
        // Holding it makes recovery level-triggered - whichever tick first finds
        // the robot released picks the request up - instead of depending on
        // catching the exact moment the action ends.
        self.replan_queue
            .retain(|(robot_id, _)| frozen.contains(robot_id));
        // One swept path per frozen robot, not one flat list of points: the
        // planner has to know which claims are joined to which, or it cannot
        // tell a robot's approach from the straight line between two unrelated
        // robots.
        let frozen_claims: Vec<Vec<[f32; 2]>> = frozen
            .iter()
            .map(|robot_id| self.frozen_claims(robot_id))
            .filter(|path| !path.is_empty())
            .collect();
        for robot_id in &frozen {
            rclrs::log!(
                self.node.logger(),
                "Robot {} is executing an action; holding its plan and excluding it from this negotiation",
                robot_id
            );
        }

        let mut robot_ids: Vec<String> = goals
            .keys()
            .filter(|robot_id| !frozen.contains(robot_id))
            .cloned()
            .collect();
        robot_ids.sort();

        if robot_ids.is_empty() {
            // Everyone who wanted a plan is mid-action. Their requests stay in
            // the queue, so a later tick will pick them up once they are
            // released.
            return;
        }

        // Perform MAPF plan in background thread
        rclrs::log!(self.node.logger(), "Triggering new plan in background");

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
        let maneuvers_clone = maneuvers.clone();

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

            let mut basic_plan = match planner_clone.plan(
                &starts,
                &goals,
                &footprints_map,
                &robot_ids,
                map_clone.as_ref(),
                &frozen_claims,
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

            // Level the raw planner output up first. PIBT is expected to return
            // trajectories of equal length already, so this is usually a no-op,
            // but nothing enforces it and an agent that is short by even one
            // step drops out of conflict analysis early.
            //
            // Padding here is safe for the same reason it is safe at the end:
            // the repeats sit after every agent has reached its goal, so they
            // can imply no collision that the final configuration did not
            // already imply. Nor do they cost anything at execution time — the
            // ADG decides how fast the real motion happens, and a robot is
            // released through a run of identical poses as fast as its blockers
            // allow.
            //
            // Note the true end of each robot's own motion before doing it,
            // because the repeats are about to bury it.
            let motion_ends: Vec<usize> = basic_plan
                .iter()
                .map(|traj| traj.len().saturating_sub(1))
                .collect();
            pad_to_equal_length(&mut basic_plan);

            // Splice the dock maneuvers in *before* conflict analysis. This is
            // the whole point: `mapf_post` derives its dependencies purely from
            // the swept volumes between consecutive poses, so the corridor a
            // docking robot occupies only becomes visible to its peers if it is
            // present here.
            let mut plan_indices = vec![PlanIndices::default(); basic_plan.len()];
            for (agent_idx, robot_id) in robot_ids.iter().enumerate() {
                let Some(traj) = basic_plan.get_mut(agent_idx) else {
                    continue;
                };
                let maneuver = maneuvers_clone.get(robot_id);
                // Measured before the splice, which prepends the undock and
                // shifts every planned index along with it.
                let prefix = maneuver.map_or(0, |m| m.prefix_len(traj.first()));
                let dock = maneuver.and_then(|m| splice_dock_maneuvers(traj, m));

                plan_indices[agent_idx] = PlanIndices {
                    dock,
                    // Where the robot first arrives at the planner's goal. That
                    // is neither the dock, which is a commitment beyond the
                    // goal, nor any of the repeats holding it there afterwards.
                    destination: prefix + motion_ends[agent_idx],
                };
            }

            // Splicing makes the lengths uneven again: an undock prefix is one
            // or two poses long and a dock suffix one more. Level them a second
            // time so that every robot, parked or not, is present for the whole
            // horizon of the plan.
            pad_to_equal_length(&mut basic_plan);

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
                discretization_timestep: DISCRETIZATION_TIMESTEP,
            };

            // `mapf_post` owns the entire dependency graph. Nothing downstream
            // adds edges of its own: an earlier version did, to force a docking
            // robot to yield, and closed a cycle that deadlocked both parties.
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
                maneuvers: maneuvers_clone,
                plan_indices,
            }));
        });
    }

    #[allow(clippy::too_many_arguments)]
    pub fn to_plan_msg(
        agent_idx: usize,
        traj: &[Isometry2<f32>],
        plan_id: PlanId,
        traffic_dependencies: &SemanticPlan,
        robot_ids: &[String],
        active_plan_ids: &HashMap<String, PlanId>,
        annotations: &DockAnnotations,
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

        if let Some(action) = annotations.arrival_action {
            // Not `last()`: padding holds the robot on the dock pose for as long
            // as the longest plan runs, so the final waypoint is a repeat. The
            // action has to fire once, when the robot arrives.
            let arrival_idx = annotations
                .dock_index
                .unwrap_or(annotations.destination_index);
            if let Some(arrival_wp) = waypoints.get_mut(arrival_idx) {
                arrival_wp.arrival_action = action.to_string();
            }
        }

        if let Some(action) = annotations.departure_action {
            if let Some(first_wp) = waypoints.first_mut() {
                first_wp.departure_action = action.to_string();
            }
        }

        // The curve the robot will follow back out of the dock, so that whoever
        // consumes the plan knows the exit is not free space.
        //
        // It has to be drawn against the pre-dock waypoint by index. The
        // neighbour *after* the dock is a padded duplicate of the dock pose, so
        // looking sideways for a departure position yields a zero-length curve.
        if let Some(dock_idx) = annotations.dock_index {
            let dock_pos = waypoints.get(dock_idx).map(|wp| wp.position);
            let depart_pos = waypoints
                .get(annotations.destination_index)
                .map(|wp| wp.position);

            // Indices are recorded alongside the trajectory they describe, so a
            // miss means the two have been allowed to drift apart. Skip rather
            // than emit a curve to a pose we cannot identify.
            if let (Some(dock_pos), Some(depart_pos)) = (dock_pos, depart_pos) {
                let progress = waypoints[dock_idx].progress;
                let maps = waypoints[dock_idx].maps.clone();
                // TODO(arjoc) parameterize it
                let departure_duration = 1.0f32;
                let departure_curve = Curve {
                    degree: 1,
                    control_points: vec![
                        ControlPoint {
                            position: dock_pos,
                            weight: 1.0,
                        },
                        ControlPoint {
                            position: depart_pos,
                            weight: 1.0,
                        },
                    ],
                    knots: vec![0.0, 0.0, departure_duration, departure_duration],
                };

                waypoints[dock_idx].departure_trajectory = vec![Trajectory {
                    curve: departure_curve,
                    initial_progress_level: progress,
                    final_progress_level: progress + departure_duration,
                    maps,
                    keys: Vec::new(),
                }];
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
    pub _progress_subscription: rclrs::WorkerSubscription<Progress, PlanServer<P>>,
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

                let robot_id_clone4 = robot_id.to_string();
                let progress_topic = robot_id.to_string() + "/plan/progress";
                // Volatile, to match the publisher in rmf_nav2_traffic. A
                // transient_local subscription would be QoS-incompatible with
                // it and would silently receive nothing at all.
                let progress_sub = match server
                    .destinations_worker
                    .create_subscription::<Progress, _>(
                        progress_topic.as_str().reliable(),
                        move |dest_server: &mut PlanServer<P>, progress_msg: Progress| {
                            dest_server.handle_progress(&robot_id_clone4, progress_msg);
                        },
                    ) {
                    Ok(sub) => sub,
                    Err(err) => {
                        rclrs::log_error!(
                            server.node.logger(),
                            "Failed to create progress subscription on DestinationsWorker for {}: {:?}",
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
                        _progress_subscription: progress_sub,
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

#[cfg(test)]
mod tests {
    use super::*;

    fn plan_id(session_byte: u8, version: u64) -> PlanId {
        let mut id = PlanId::default();
        id.destination_session.uuid[0] = session_byte;
        id.plan_version = version;
        id
    }

    fn progress(plan_id: PlanId, execution_state: u8) -> Progress {
        Progress {
            plan_id,
            execution_state,
            ..Default::default()
        }
    }

    #[test]
    fn a_robot_executing_an_action_on_the_current_plan_is_committed() {
        let active = plan_id(1, 0);
        let report = progress(active.clone(), Progress::EXECUTION_STATE_EXECUTING_ACTION);
        assert!(reports_commitment(Some(&report), Some(&active)));
    }

    #[test]
    fn a_robot_that_has_never_reported_is_not_committed() {
        // Silence is not commitment. Freezing on it would strand any robot
        // whose executor has not come up yet.
        assert!(!reports_commitment(None, Some(&plan_id(1, 0))));
    }

    #[test]
    fn a_robot_with_no_plan_of_ours_is_not_committed() {
        let report = progress(plan_id(1, 0), Progress::EXECUTION_STATE_EXECUTING_ACTION);
        assert!(!reports_commitment(Some(&report), None));
    }

    /// The subtle one. The robot really is mid-action, but against a plan we
    /// have already superseded. Honouring it would hold the robot to a plan
    /// nobody is executing, and the waypoint indices in that report do not
    /// refer to the plan we would be reasoning about.
    #[test]
    fn commitment_reported_against_a_superseded_plan_is_ignored() {
        let report = progress(plan_id(1, 0), Progress::EXECUTION_STATE_EXECUTING_ACTION);

        assert!(!reports_commitment(Some(&report), Some(&plan_id(1, 1))));
        assert!(!reports_commitment(Some(&report), Some(&plan_id(2, 0))));
    }

    /// Everything short of `EXECUTING_ACTION` is replannable, including
    /// `HOLDING_FOR_TRAFFIC`: a robot stopped for a blocker has not committed
    /// itself to anything, and is precisely the robot we most want to re-route.
    #[test]
    fn only_executing_action_counts_as_commitment() {
        let active = plan_id(1, 0);
        for state in [
            Progress::EXECUTION_STATE_UNKNOWN,
            Progress::EXECUTION_STATE_IDLE,
            Progress::EXECUTION_STATE_MOVING,
            Progress::EXECUTION_STATE_HOLDING_FOR_TRAFFIC,
        ] {
            let report = progress(active.clone(), state);
            assert!(
                !reports_commitment(Some(&report), Some(&active)),
                "execution_state {state} should not freeze a robot"
            );
        }
    }

    fn destination(session_byte: u8) -> Destination {
        let mut dest = Destination::default();
        dest.session.uuid[0] = session_byte;
        dest
    }

    /// The regression this rule exists for.
    ///
    /// After a dock succeeds the destination that asked for it is still the
    /// robot's active destination. Replanning it regenerates the same dock
    /// suffix, the fresh plan_id makes the executor re-issue the maneuver, and
    /// the robot undocks and re-docks forever - staying `EXECUTING_ACTION` for
    /// almost all of that cycle, which froze it out of every negotiation and
    /// made it impossible to assign anything new.
    #[test]
    fn a_robot_already_serving_its_destination_is_not_owed_a_plan() {
        assert!(!needs_replanning(
            Some(&destination(1)),
            Some(&plan_id(1, 0))
        ));
    }

    /// A later revision of the same plan is still the same task.
    #[test]
    fn plan_version_does_not_affect_whether_a_robot_is_owed_a_plan() {
        assert!(!needs_replanning(
            Some(&destination(1)),
            Some(&plan_id(1, 7))
        ));
    }

    /// The case the freeze is supposed to defer rather than drop: a task
    /// arrived while the robot was mid-dock, so the plan it is executing was
    /// built for a different session and it must be replanned on release.
    #[test]
    fn a_destination_that_arrived_mid_action_is_owed_a_plan() {
        assert!(needs_replanning(
            Some(&destination(2)),
            Some(&plan_id(1, 3))
        ));
    }

    /// Fail safe in both directions: never having been planned for means owed a
    /// plan, but having no destination at all means there is nothing to plan.
    #[test]
    fn missing_state_fails_safe() {
        assert!(needs_replanning(Some(&destination(1)), None));
        assert!(!needs_replanning(None, Some(&plan_id(1, 0))));
        assert!(!needs_replanning(None, None));
    }
}
