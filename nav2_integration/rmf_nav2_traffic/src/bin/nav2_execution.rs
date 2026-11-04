use log::error;
use nav2_traffic::{resolve_agent_names, Nav2ExecutionNode, SharedSessionCoordinator};
use rclrs::{Context, CreateBasicExecutor, SpinOptions};
use std::{thread, time::Duration};

fn main() {
    let _ = env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info"))
        .try_init();

    let agent_names = resolve_agent_names();
    let coordinator = SharedSessionCoordinator::new(agent_names.len());
    let mut executor = Context::default_from_env()
        .expect("failed to initialize rclrs Context")
        .create_basic_executor();

    let mut execution_node = Nav2ExecutionNode::new(&executor, agent_names, coordinator);

    thread::spawn(move || {
        for err in executor.spin(SpinOptions::default()) {
            error!("An error occurred in rclrs executor: {err}");
        }
    });

    loop {
        execution_node.tick();
        thread::sleep(Duration::from_millis(20));
    }
}
