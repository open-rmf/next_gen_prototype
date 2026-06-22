use rclrs::{Context, CreateBasicExecutor, IntoPrimitiveOptions, SpinOptions};
use rmf_prototype_msgs::msg::{Plan, SafeZone};
use visualization_msgs::msg::{Marker, MarkerArray};
use geometry_msgs::msg::Point;
use std::sync::Arc;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();
    let node = Arc::new(executor.create_node("rmf_path_visualizer")?);
    
    let args: Vec<String> = std::env::args().collect();
    let mut robot_name = "robot0".to_string();
    
    // Simple arg parsing to get robot_name
    for arg in args.iter().skip(1) {
        if !arg.starts_with("--ros-args") && !arg.starts_with("-") {
            robot_name = arg.clone();
            break;
        }
    }

    let topic_name = format!("{}/plan", robot_name);
    let safe_zone_topic = format!("{}/plan/safe_zone", robot_name);
    let marker_topic = format!("{}/path_markers", robot_name);

    let publisher = node.create_publisher::<MarkerArray>(marker_topic.as_str().transient_local().reliable())?;
    
    let publisher_plan = publisher.clone();
    let robot_name_clone = robot_name.clone();

    let _subscription = node.create_subscription::<Plan, _>(
        topic_name.as_str().transient_local().reliable(),
        move |msg: Plan| {
            let mut marker_array = MarkerArray { markers: vec![] };
            let mut marker = Marker::default();
            marker.header.frame_id = "map".to_string();
            // Try to use the start_time from the plan
            marker.header.stamp = msg.start_time;
            marker.ns = robot_name_clone.clone();
            marker.id = 0;
            marker.type_ = visualization_msgs::msg::Marker::LINE_STRIP as i32;
            marker.action = visualization_msgs::msg::Marker::ADD as i32;
            
            // Set color (e.g., green)
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            marker.color.a = 1.0;
            
            // Set line width
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
    )?;

    let publisher_sz = publisher.clone();
    let robot_name_clone2 = robot_name.clone();

    let _safe_zone_sub = node.create_subscription::<SafeZone, _>(
        safe_zone_topic.as_str().transient_local().reliable(),
        move |msg: SafeZone| {
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
                marker.ns = format!("{}_safe_zone", robot_name_clone2);
                marker.id = 1;
                marker.type_ = visualization_msgs::msg::Marker::SPHERE as i32;
                marker.action = visualization_msgs::msg::Marker::ADD as i32;
                
                // Set color (e.g., red dot)
                marker.color.r = 1.0;
                marker.color.g = 0.0;
                marker.color.b = 0.0;
                marker.color.a = 1.0;
                
                // Set scale (size of the dot)
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
    )?;

    rclrs::log!(node.logger(), "Starting rmf_path_visualizer for robot: {}", robot_name);
    executor.spin(SpinOptions::default());
    Ok(())
}
