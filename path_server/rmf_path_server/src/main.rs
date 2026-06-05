use rclrs::{Context, CreateBasicExecutor, SpinOptions};
use rmf_path_server::{start_path_server, PibtPlanner};
use std::sync::Arc;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let context = Context::default_from_env().unwrap();
    let mut executor = context.create_basic_executor();
    let node = Arc::new(executor.create_node("path_server")?);

    let _path_server_guard = start_path_server(Arc::clone(&node), PibtPlanner::default())?;

    rclrs::log!(
        node.logger(),
        "Path server started with multi-worker separation (DiscoveryWorker + DestinationsWorker). Spinning..."
    );
    executor.spin(SpinOptions::default());
    Ok(())
}
