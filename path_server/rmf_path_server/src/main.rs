use rclrs::{Context, CreateBasicExecutor, SpinOptions};
use rmf_prototype_msgs::msg::Destination;
use nav_msgs::msg::Odometry;
use std::sync::Arc;
use rmf_path_server::{PlanServer, DiscoveryServer, PibtPlanner, RobotPathConnections};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let context = Context::default_from_env().unwrap();
    let mut executor = context.create_basic_executor();
    let node = Arc::new(executor.create_node("path_server")?);

    // Create the Destinations worker (Data Plane)
    let destinations_worker = Arc::new(node.create_worker(PlanServer::new(Arc::clone(&node), PibtPlanner::default())));

    // Create a periodic timer on the Destinations worker to trigger replans asynchronously.
    // This avoids blocking subscriber callbacks and coalesces multiple updates within the tick duration.
    let _replan_timer = destinations_worker.create_timer_repeating(
        std::time::Duration::from_millis(100),
        move |server: &mut PlanServer<PibtPlanner>| {
            server.replan();
        },
    )?;

    // Create the Discovery worker (Control Plane), passing a reference to the Destinations worker
    let discovery_worker = Arc::new(node.create_worker(DiscoveryServer::new(
        Arc::clone(&node),
        Arc::clone(&destinations_worker),
    )));

    // Subscribe to discovery on the Discovery worker using the shared generic helper
    let _discovery_subscription = rmf_participant_discovery::create_discovery_subscription(
        &discovery_worker,
        "/destination/discovery",
        |server: &mut DiscoveryServer<PibtPlanner>, robot_id: &str| {
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
                        &destination_topic,
                        move |dest_server: &mut PlanServer<PibtPlanner>, dest_msg: Destination| {
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
                        &odom_topic,
                        move |dest_server: &mut PlanServer<PibtPlanner>, odom_msg: Odometry| {
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
        |server: &mut DiscoveryServer<PibtPlanner>, robot_id: &str| {
            if server.active_robots.remove(robot_id).is_some() {
                rclrs::log!(server.node.logger(), "Participant left: {}", robot_id);
            }
        },
    )?;

    rclrs::log!(
        node.logger(),
        "Path server started with multi-worker separation (DiscoveryWorker + DestinationsWorker). Spinning..."
    );
    executor.spin(SpinOptions::default());
    Ok(())
}
