use rclrs::{Context, CreateBasicExecutor, IntoPrimitiveOptions, SpinOptions};
use ros_env::{
    geometry_msgs::msg::Point,
    rmf_prototype_msgs::msg::{Action, ActionProgress, Plan, Progress, SafeZone},
    visualization_msgs::msg::{Marker, MarkerArray},
};
use std::collections::HashMap;

struct RobotConnections {
    _plan_subscription: rclrs::WorkerSubscription<Plan, VisualizerServer>,
    _safe_zone_sub: rclrs::WorkerSubscription<SafeZone, VisualizerServer>,
    _progress_sub: rclrs::WorkerSubscription<Progress, VisualizerServer>,
    _action_sub: rclrs::WorkerSubscription<Action, VisualizerServer>,
    _action_progress_sub: rclrs::WorkerSubscription<ActionProgress, VisualizerServer>,
}

#[derive(Default, Clone)]
struct RobotVisualState {
    plan: Option<Plan>,
    safe_zone: Option<SafeZone>,
    progress: Option<Progress>,
    last_action: Option<Action>,
    last_action_progress: Option<ActionProgress>,
}

struct VisualizerServer {
    node: rclrs::Node,
    publishers: HashMap<String, rclrs::Publisher<MarkerArray>>,
    states: HashMap<String, RobotVisualState>,
}

impl VisualizerServer {
    fn new(node: rclrs::Node) -> Self {
        Self {
            node,
            publishers: HashMap::new(),
            states: HashMap::new(),
        }
    }

    fn get_or_create_publisher(
        &mut self,
        robot_id: &str,
    ) -> Option<&rclrs::Publisher<MarkerArray>> {
        if !self.publishers.contains_key(robot_id) {
            let marker_topic = format!("{}/path_markers", robot_id);
            match self
                .node
                .create_publisher::<MarkerArray>(marker_topic.as_str().transient_local().reliable())
            {
                Ok(publ) => {
                    self.publishers.insert(robot_id.to_string(), publ);
                }
                Err(err) => {
                    rclrs::log_error!(
                        self.node.logger(),
                        "Failed to create publisher for {}: {:?}",
                        robot_id,
                        err
                    );
                    return None;
                }
            }
        }
        self.publishers.get(robot_id)
    }

    fn publish_robot_markers(&mut self, robot_id: &str) {
        let Some(state) = self.states.get(robot_id).cloned() else {
            return;
        };

        let mut markers = Vec::new();

        // 1. Planned path LINE_STRIP + Waypoint Action badges
        if let Some(plan) = &state.plan {
            let mut path_marker = Marker::default();
            path_marker.header.frame_id = "map".to_string();
            path_marker.ns = robot_id.to_string();
            path_marker.id = 0;
            path_marker.type_ = Marker::LINE_STRIP as i32;
            path_marker.action = Marker::ADD as i32;
            path_marker.pose.orientation.w = 1.0;
            path_marker.color.r = 0.0;
            path_marker.color.g = 1.0;
            path_marker.color.b = 0.0;
            path_marker.color.a = 1.0;
            path_marker.scale.x = 0.05;

            for wp in &plan.waypoints {
                path_marker.points.push(Point {
                    x: wp.position[0] as f64,
                    y: wp.position[1] as f64,
                    z: 0.02,
                });
            }
            markers.push(path_marker);

            // Clear any old waypoint action markers first
            let mut clear_wp_actions = Marker::default();
            clear_wp_actions.header.frame_id = "map".to_string();
            clear_wp_actions.ns = format!("{}_waypoint_actions", robot_id);
            clear_wp_actions.id = 0;
            clear_wp_actions.action = Marker::DELETEALL as i32;
            markers.push(clear_wp_actions);

            for (idx, wp) in plan.waypoints.iter().enumerate() {
                if wp.arrival_action.is_empty() && wp.departure_action.is_empty() {
                    continue;
                }

                let mut ring = Marker::default();
                ring.header.frame_id = "map".to_string();
                ring.ns = format!("{}_waypoint_actions", robot_id);
                ring.id = 100 + (idx as i32) * 2;
                ring.type_ = Marker::CYLINDER as i32;
                ring.action = Marker::ADD as i32;
                ring.pose.position.x = wp.position[0] as f64;
                ring.pose.position.y = wp.position[1] as f64;
                ring.pose.position.z = 0.02;
                ring.pose.orientation.w = 1.0;
                ring.scale.x = 0.45;
                ring.scale.y = 0.45;
                ring.scale.z = 0.03;
                ring.color.r = 1.0;
                ring.color.g = 0.85;
                ring.color.b = 0.0;
                ring.color.a = 0.7;
                markers.push(ring);

                let mut label = Marker::default();
                label.header.frame_id = "map".to_string();
                label.ns = format!("{}_waypoint_actions", robot_id);
                label.id = 100 + (idx as i32) * 2 + 1;
                label.type_ = Marker::TEXT_VIEW_FACING as i32;
                label.action = Marker::ADD as i32;
                label.pose.position.x = wp.position[0] as f64;
                label.pose.position.y = (wp.position[1] as f64) - 0.35;
                label.pose.position.z = 0.25;
                label.pose.orientation.w = 1.0;
                label.scale.z = 0.16;
                label.color.r = 1.0;
                label.color.g = 0.92;
                label.color.b = 0.2;
                label.color.a = 1.0;
                label.text = match (
                    !wp.arrival_action.is_empty(),
                    !wp.departure_action.is_empty(),
                ) {
                    (true, true) => format!(
                        "wp {}: arr={} | dep={}",
                        idx, wp.arrival_action, wp.departure_action
                    ),
                    (true, false) => format!("wp {}: arr={}", idx, wp.arrival_action),
                    (false, true) => format!("wp {}: dep={}", idx, wp.departure_action),
                    (false, false) => String::new(),
                };
                markers.push(label);
            }
        }

        // 2. SafeZone incremental target SPHERE
        if let Some(sz) = &state.safe_zone {
            if let Some(first_reg) = sz.incremental_target.regions.first() {
                if first_reg.region.points.len() >= 2 {
                    let mut sz_marker = Marker::default();
                    sz_marker.header.frame_id = "map".to_string();
                    sz_marker.ns = format!("{}_safe_zone", robot_id);
                    sz_marker.id = 1;
                    sz_marker.type_ = Marker::SPHERE as i32;
                    sz_marker.action = Marker::ADD as i32;
                    sz_marker.pose.position.x = first_reg.region.points[0] as f64;
                    sz_marker.pose.position.y = first_reg.region.points[1] as f64;
                    sz_marker.pose.position.z = 0.05;
                    sz_marker.pose.orientation.w = 1.0;
                    sz_marker.scale.x = 0.25;
                    sz_marker.scale.y = 0.25;
                    sz_marker.scale.z = 0.25;
                    sz_marker.color.r = 1.0;
                    sz_marker.color.g = 0.0;
                    sz_marker.color.b = 0.0;
                    sz_marker.color.a = 1.0;
                    markers.push(sz_marker);
                }
            }
        }

        // 3. Live Execution State Widget (floating 3D text + status halo + HUD banner)
        let robot_xy = state
            .progress
            .as_ref()
            .map(|p| (p.pose.pose.position.x, p.pose.pose.position.y))
            .or_else(|| {
                state
                    .plan
                    .as_ref()
                    .and_then(|pl| pl.waypoints.first())
                    .map(|wp| (wp.position[0] as f64, wp.position[1] as f64))
            });

        if let Some((rx, ry)) = robot_xy {
            let mode_str = match state.progress.as_ref().map(|p| p.mode) {
                Some(Progress::MODE_NAVIGATING) => "NAVIGATING",
                Some(Progress::MODE_ACTION) => "FLEET_ACTION",
                Some(Progress::MODE_IDLE) => "IDLE",
                Some(Progress::MODE_ERROR) => "ERROR",
                _ => "READY",
            };

            let wp_info = if let Some(prog) = &state.progress {
                let target_wp = prog.target_waypoint;
                format!(
                    "v{} wp {}->{} (prog {:.1})",
                    prog.plan_id.plan_version, prog.reached_waypoint, target_wp, prog.progress
                )
            } else {
                "waiting".to_string()
            };

            let (action_line, r, g, b, halo_scale) = if let Some(ap) = &state.last_action_progress {
                let phase_str = if ap.phase == Action::PHASE_ARRIVAL {
                    "ARRIVAL"
                } else if ap.phase == Action::PHASE_DEPARTURE {
                    "DEPARTURE"
                } else {
                    "ACTION"
                };
                let act_spec = state
                    .last_action
                    .as_ref()
                    .filter(|a| a.waypoint_id == ap.waypoint_id && a.phase == ap.phase)
                    .map(|a| a.action.as_str())
                    .unwrap_or("action");

                match ap.status {
                    ActionProgress::STATUS_RUNNING => (
                        format!(
                            "ACTION RUNNING [{}]: {} (wp {})",
                            phase_str, act_spec, ap.waypoint_id
                        ),
                        1.0f32,
                        0.88f32,
                        0.0f32,
                        0.95f64,
                    ),
                    ActionProgress::STATUS_SUCCEEDED => (
                        format!(
                            "LAST ACTION DONE [{}]: {} (wp {})",
                            phase_str, act_spec, ap.waypoint_id
                        ),
                        0.1f32,
                        1.0f32,
                        0.45f32,
                        0.65f64,
                    ),
                    ActionProgress::STATUS_FAILED => (
                        format!(
                            "ACTION FAILED [{}]: {} ({})",
                            phase_str, act_spec, ap.error.message
                        ),
                        1.0f32,
                        0.2f32,
                        0.2f32,
                        0.95f64,
                    ),
                    _ => ("ACTION: IDLE".to_string(), 0.2f32, 0.9f32, 1.0f32, 0.65f64),
                }
            } else {
                ("ACTION: NONE".to_string(), 0.2f32, 0.9f32, 1.0f32, 0.65f64)
            };

            // Status halo cylinder beneath robot
            let mut halo = Marker::default();
            halo.header.frame_id = "map".to_string();
            halo.ns = format!("{}_execution_state", robot_id);
            halo.id = 2;
            halo.type_ = Marker::CYLINDER as i32;
            halo.action = Marker::ADD as i32;
            halo.pose.position.x = rx;
            halo.pose.position.y = ry;
            halo.pose.position.z = 0.03;
            halo.pose.orientation.w = 1.0;
            halo.scale.x = halo_scale;
            halo.scale.y = halo_scale;
            halo.scale.z = 0.04;
            halo.color.r = r;
            halo.color.g = g;
            halo.color.b = b;
            halo.color.a = 0.55;
            markers.push(halo);

            // Floating 3D text widget directly above the robot
            let mut robot_text = Marker::default();
            robot_text.header.frame_id = "map".to_string();
            robot_text.ns = format!("{}_execution_state", robot_id);
            robot_text.id = 3;
            robot_text.type_ = Marker::TEXT_VIEW_FACING as i32;
            robot_text.action = Marker::ADD as i32;
            robot_text.pose.position.x = rx;
            robot_text.pose.position.y = ry + 0.55;
            robot_text.pose.position.z = 0.65;
            robot_text.pose.orientation.w = 1.0;
            robot_text.scale.z = 0.22;
            robot_text.color.r = r;
            robot_text.color.g = g;
            robot_text.color.b = b;
            robot_text.color.a = 1.0;
            robot_text.text = format!("[{}] {} | {}\n{}", robot_id, mode_str, wp_info, action_line);
            markers.push(robot_text);

            // Top-of-map HUD status banner so it is always readable at a fixed location
            let hud_y = if robot_id.ends_with('0') || robot_id.ends_with('1') && robot_id == "mir1"
            {
                2.85
            } else {
                2.35
            };
            let mut hud_text = Marker::default();
            hud_text.header.frame_id = "map".to_string();
            hud_text.ns = format!("{}_execution_state", robot_id);
            hud_text.id = 4;
            hud_text.type_ = Marker::TEXT_VIEW_FACING as i32;
            hud_text.action = Marker::ADD as i32;
            hud_text.pose.position.x = 1.5;
            hud_text.pose.position.y = hud_y;
            hud_text.pose.position.z = 0.8;
            hud_text.pose.orientation.w = 1.0;
            hud_text.scale.z = 0.24;
            hud_text.color.r = r;
            hud_text.color.g = g;
            hud_text.color.b = b;
            hud_text.color.a = 1.0;
            hud_text.text = format!("{}: {} ({}) | {}", robot_id, mode_str, wp_info, action_line);
            markers.push(hud_text);
        }

        let marker_array = MarkerArray { markers };
        if let Some(publisher) = self.get_or_create_publisher(robot_id) {
            let _ = publisher.publish(&marker_array);
        }
    }
}

struct VisualizerDiscoveryServer {
    node: rclrs::Node,
    active_robots: HashMap<String, RobotConnections>,
    visualizer_worker: rclrs::Worker<VisualizerServer>,
}

impl VisualizerDiscoveryServer {
    fn new(node: rclrs::Node, visualizer_worker: rclrs::Worker<VisualizerServer>) -> Self {
        Self {
            node,
            active_robots: HashMap::new(),
            visualizer_worker,
        }
    }
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();
    let node = executor.create_node("rmf_path_visualizer")?;

    let visualizer_worker = node.create_worker(VisualizerServer::new(node.clone()));
    let discovery_worker = node.create_worker(VisualizerDiscoveryServer::new(
        node.clone(),
        visualizer_worker.clone(),
    ));

    let _discovery_subscription = rmf_participant_discovery::create_discovery_subscription(
        &discovery_worker,
        "/destination/discovery",
        |server: &mut VisualizerDiscoveryServer, robot_id: &str| {
            if !server.active_robots.contains_key(robot_id) {
                rclrs::log!(
                    server.node.logger(),
                    "Discovered new participant for visualization: {}",
                    robot_id
                );

                let id_for_plan = robot_id.to_string();
                let id_for_sz = robot_id.to_string();
                let id_for_prog = robot_id.to_string();
                let id_for_act = robot_id.to_string();
                let id_for_act_prog = robot_id.to_string();

                let plan_topic = format!("{}/plan", robot_id);
                let safe_zone_topic = format!("{}/plan/safe_zone", robot_id);
                let progress_topic = format!("{}/plan/progress", robot_id);
                let action_topic = format!("{}/plan/action", robot_id);
                let action_progress_topic = format!("{}/plan/action/progress", robot_id);

                let plan_sub = match server.visualizer_worker.create_subscription::<Plan, _>(
                    plan_topic.as_str().transient_local().reliable(),
                    move |vs: &mut VisualizerServer, msg: Plan| {
                        let entry = vs.states.entry(id_for_plan.clone()).or_default();
                        entry.plan = Some(msg);
                        vs.publish_robot_markers(&id_for_plan);
                    },
                ) {
                    Ok(sub) => sub,
                    Err(err) => {
                        rclrs::log_error!(
                            server.node.logger(),
                            "Failed to create plan subscription for {}: {:?}",
                            robot_id,
                            err
                        );
                        return;
                    }
                };

                let safe_zone_sub =
                    match server.visualizer_worker.create_subscription::<SafeZone, _>(
                        safe_zone_topic.as_str().transient_local().reliable(),
                        move |vs: &mut VisualizerServer, msg: SafeZone| {
                            let entry = vs.states.entry(id_for_sz.clone()).or_default();
                            entry.safe_zone = Some(msg);
                            vs.publish_robot_markers(&id_for_sz);
                        },
                    ) {
                        Ok(sub) => sub,
                        Err(err) => {
                            rclrs::log_error!(
                                server.node.logger(),
                                "Failed to create safe_zone subscription for {}: {:?}",
                                robot_id,
                                err
                            );
                            return;
                        }
                    };

                let progress_sub =
                    match server.visualizer_worker.create_subscription::<Progress, _>(
                        progress_topic.as_str().transient_local().reliable(),
                        move |vs: &mut VisualizerServer, msg: Progress| {
                            let entry = vs.states.entry(id_for_prog.clone()).or_default();
                            entry.progress = Some(msg);
                            vs.publish_robot_markers(&id_for_prog);
                        },
                    ) {
                        Ok(sub) => sub,
                        Err(err) => {
                            rclrs::log_error!(
                                server.node.logger(),
                                "Failed to create progress subscription for {}: {:?}",
                                robot_id,
                                err
                            );
                            return;
                        }
                    };

                let action_sub = match server.visualizer_worker.create_subscription::<Action, _>(
                    action_topic.as_str().transient_local().reliable(),
                    move |vs: &mut VisualizerServer, msg: Action| {
                        let entry = vs.states.entry(id_for_act.clone()).or_default();
                        entry.last_action = Some(msg);
                        vs.publish_robot_markers(&id_for_act);
                    },
                ) {
                    Ok(sub) => sub,
                    Err(err) => {
                        rclrs::log_error!(
                            server.node.logger(),
                            "Failed to create action subscription for {}: {:?}",
                            robot_id,
                            err
                        );
                        return;
                    }
                };

                let action_progress_sub = match server
                    .visualizer_worker
                    .create_subscription::<ActionProgress, _>(
                        action_progress_topic.as_str().transient_local().reliable(),
                        move |vs: &mut VisualizerServer, msg: ActionProgress| {
                            let entry = vs.states.entry(id_for_act_prog.clone()).or_default();
                            entry.last_action_progress = Some(msg);
                            vs.publish_robot_markers(&id_for_act_prog);
                        },
                    ) {
                    Ok(sub) => sub,
                    Err(err) => {
                        rclrs::log_error!(
                            server.node.logger(),
                            "Failed to create action_progress subscription for {}: {:?}",
                            robot_id,
                            err
                        );
                        return;
                    }
                };

                server.active_robots.insert(
                    robot_id.to_string(),
                    RobotConnections {
                        _plan_subscription: plan_sub,
                        _safe_zone_sub: safe_zone_sub,
                        _progress_sub: progress_sub,
                        _action_sub: action_sub,
                        _action_progress_sub: action_progress_sub,
                    },
                );
            }
        },
        |server: &mut VisualizerDiscoveryServer, robot_id: &str| {
            if server.active_robots.remove(robot_id).is_some() {
                rclrs::log!(
                    server.node.logger(),
                    "Participant left visualization tracking: {}",
                    robot_id
                );
            }
        },
    )?;

    rclrs::log!(node.logger(), "Starting rmf_path_visualizer with discovery");
    executor.spin(SpinOptions::default());
    Ok(())
}
