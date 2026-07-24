use rclrs::{Context, CreateBasicExecutor, IntoPrimitiveOptions, SpinOptions};
use ros_env::{
    geometry_msgs::msg::Point,
    rmf_prototype_msgs::msg::{Plan, SafeZone},
    visualization_msgs::msg::{Marker, MarkerArray},
};
use std::collections::HashMap;

struct RobotConnections {
    _plan_subscription: rclrs::WorkerSubscription<Plan, VisualizerServer>,
    _safe_zone_sub: rclrs::WorkerSubscription<SafeZone, VisualizerServer>,
}

struct VisualizerServer {}

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

    let visualizer_worker = node.create_worker(VisualizerServer {});
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

                let robot_id_clone = robot_id.to_string();
                let robot_id_clone2 = robot_id.to_string();

                let topic_name = format!("{}/plan", robot_id);
                let safe_zone_topic = format!("{}/plan/safe_zone", robot_id);
                let marker_topic = format!("{}/path_markers", robot_id);

                let publisher = match server.node.create_publisher::<MarkerArray>(
                    marker_topic.as_str().transient_local().reliable(),
                ) {
                    Ok(publ) => publ,
                    Err(err) => {
                        rclrs::log_error!(
                            server.node.logger(),
                            "Failed to create publisher for {}: {:?}",
                            robot_id,
                            err
                        );
                        return;
                    }
                };

                let publisher_plan = publisher.clone();
                let plan_sub = match server.visualizer_worker.create_subscription::<Plan, _>(
                    topic_name.as_str().transient_local().reliable(),
                    move |_vs: &mut VisualizerServer, msg: Plan| {
                        let mut marker_array = MarkerArray { markers: vec![] };
                        let mut marker = Marker::default();
                        marker.header.frame_id = "map".to_string();
                        marker.header.stamp = msg.start_time;
                        marker.ns = robot_id_clone.clone();
                        marker.id = 0;
                        marker.type_ = ros_env::visualization_msgs::msg::Marker::LINE_STRIP as i32;
                        marker.action = ros_env::visualization_msgs::msg::Marker::ADD as i32;

                        marker.color.r = 0.0;
                        marker.color.g = 1.0;
                        marker.color.b = 0.0;
                        marker.color.a = 1.0;

                        marker.scale.x = 0.05;

                        for wp in &msg.waypoints {
                            let mut p = Point::default();
                            p.x = wp.position[0] as f64;
                            p.y = wp.position[1] as f64;
                            p.z = 0.0;
                            marker.points.push(p);
                        }

                        marker_array.markers.push(marker);
                        let _ = publisher_plan.publish(&marker_array);
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

                let publisher_sz = publisher.clone();
                let safe_zone_sub =
                    match server.visualizer_worker.create_subscription::<SafeZone, _>(
                        safe_zone_topic.as_str().transient_local().reliable(),
                        move |_vs: &mut VisualizerServer, msg: SafeZone| {
                            if msg.incremental_target.regions.is_empty() {
                                return;
                            }
                            let region = &msg.incremental_target.regions[0].region;
                            if region.points.len() >= 2 {
                                let x = region.points[0] as f64;
                                let y = region.points[1] as f64;

                                let mut marker_array = MarkerArray { markers: vec![] };
                                let mut marker = Marker::default();
                                marker.header.frame_id = "map".to_string();
                                marker.ns = format!("{}_safe_zone", robot_id_clone2);
                                marker.id = 1;
                                marker.type_ =
                                    ros_env::visualization_msgs::msg::Marker::SPHERE as i32;
                                marker.action =
                                    ros_env::visualization_msgs::msg::Marker::ADD as i32;

                                marker.color.r = 1.0;
                                marker.color.g = 0.0;
                                marker.color.b = 0.0;
                                marker.color.a = 1.0;

                                marker.scale.x = 0.25;
                                marker.scale.y = 0.25;
                                marker.scale.z = 0.25;

                                marker.pose.position.x = x;
                                marker.pose.position.y = y;
                                marker.pose.position.z = 0.0;

                                marker.pose.orientation.w = 1.0; // valid quaternion

                                marker_array.markers.push(marker);
                                let _ = publisher_sz.publish(&marker_array);
                            }
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

                server.active_robots.insert(
                    robot_id.to_string(),
                    RobotConnections {
                        _plan_subscription: plan_sub,
                        _safe_zone_sub: safe_zone_sub,
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
