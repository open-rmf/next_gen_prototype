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

use bevy::{app::ScheduleRunnerPlugin, log::LogPlugin, prelude::*};
use bevy_ros2::{RclrsExecutorCommands, RclrsNode, RosPublisher};
use crossflow::prelude::*;
use futures::FutureExt;
use rmf_process_executor::{
    JsonMessage, JsonSchema, ProcessActionExecution, ProcessActionRequest, ProcessExecutorAppExt,
    ProcessExecutorPlugin,
};
use ros_env::std_msgs::msg::Float64;
use serde::{Deserialize, Serialize};
use std::{future::Future, sync::Arc, time::Duration};

#[derive(Clone, Debug, Default, Serialize, Deserialize, JsonSchema)]
struct ConveyorNodeConfig {
    #[serde(default)]
    conveyor: Option<String>,
    #[serde(default)]
    speed: Option<f64>,
    #[serde(default)]
    duration_sec: Option<f64>,
}

fn run_conveyor_belts(
    node: &RclrsNode,
    executor_commands: &RclrsExecutorCommands,
    robot_id: String,
    conveyor_name: String,
    speed: f64,
    duration: Duration,
) -> impl Future<Output = ()> {
    let robot_track_topic = format!("/{robot_id}/track_cmd_vel");
    let conveyor_track_topic = format!("/model/{conveyor_name}/link/base_link/track_cmd_vel");
    let robot_pub = Arc::new(RosPublisher::<Float64>::new(node, robot_track_topic));
    let conveyor_pub = Arc::new(RosPublisher::<Float64>::new(node, conveyor_track_topic));

    info!(
        "[{}] Running conveyor action (conveyor='{}', speed={:.2}, duration={:.2}s)",
        robot_id,
        conveyor_name,
        speed,
        duration.as_secs_f64()
    );

    executor_commands
        .run(async move {
            let _ = robot_pub.publish(Float64 { data: speed });
            let _ = conveyor_pub.publish(Float64 { data: speed });
            let (tx, rx) = tokio::sync::oneshot::channel::<()>();
            let r_pub = Arc::clone(&robot_pub);
            let c_pub = Arc::clone(&conveyor_pub);
            std::thread::spawn(move || {
                let steps = (duration.as_millis() / 100).max(1);
                for _ in 0..steps {
                    let _ = r_pub.publish(Float64 { data: speed });
                    let _ = c_pub.publish(Float64 { data: speed });
                    std::thread::sleep(Duration::from_millis(100));
                }
                let _ = r_pub.publish(Float64 { data: 0.0 });
                let _ = c_pub.publish(Float64 { data: 0.0 });
                let _ = tx.send(());
            });
            let _ = rx.await;
        })
        .map(|_| ())
}

fn execute_conveyor_action(
    Async { request, .. }: Async<ProcessActionRequest>,
    node: Res<RclrsNode>,
    executor_commands: Res<RclrsExecutorCommands>,
) -> impl Future<Output = ProcessActionExecution> {
    let conveyor_name = request
        .params
        .get("conveyor")
        .or_else(|| request.params.get("arg"))
        .and_then(|v| v.as_str())
        .filter(|s| !s.is_empty() && s.parse::<f64>().is_err())
        .unwrap_or("conveyor_r1_c1")
        .to_string();
    let speed = request
        .params
        .get("speed")
        .and_then(|v| v.as_f64())
        .unwrap_or(0.4);
    let duration = Duration::from_secs_f64(request.duration_sec(4.0));
    let fut = run_conveyor_belts(
        &node,
        &executor_commands,
        request.robot_id.clone(),
        conveyor_name,
        speed,
        duration,
    );
    async move {
        fut.await;
        request.succeed()
    }
}

fn execute_conveyor_diagram_node(
    Async {
        request: (cfg, mut msg),
        ..
    }: Async<(ConveyorNodeConfig, JsonMessage)>,
    node: Res<RclrsNode>,
    executor_commands: Res<RclrsExecutorCommands>,
) -> impl Future<Output = JsonMessage> {
    let robot_id = msg
        .get("robot_id")
        .and_then(|v| v.as_str())
        .unwrap_or("robot0")
        .to_string();
    let conveyor_name = cfg.conveyor.unwrap_or_else(|| "conveyor_r1_c1".to_string());
    let speed = cfg.speed.unwrap_or(0.4);
    let duration = Duration::from_secs_f64(cfg.duration_sec.unwrap_or(4.0).max(0.0));
    let fut = run_conveyor_belts(
        &node,
        &executor_commands,
        robot_id,
        conveyor_name.clone(),
        speed,
        duration,
    );
    async move {
        fut.await;
        if let Some(obj) = msg.as_object_mut() {
            obj.insert(
                "conveyor_completed".to_string(),
                serde_json::Value::String(conveyor_name),
            );
        }
        msg
    }
}

fn main() {
    let mut app = App::new();
    app.add_plugins((
        MinimalPlugins.set(ScheduleRunnerPlugin::run_loop(Duration::from_millis(20))),
        LogPlugin::default(),
    ));
    app.add_plugins(ProcessExecutorPlugin::default());

    let conveyor_service = app.spawn_service(execute_conveyor_action);
    let conveyor_diagram_node = app.spawn_service(execute_conveyor_diagram_node);

    app.register_process_action("conveyor", conveyor_service.clone())
        .register_process_action("conveyor_transfer", conveyor_service)
        .register_diagram_node("conveyor", conveyor_diagram_node.clone())
        .register_diagram_node("conveyor_transfer", conveyor_diagram_node);

    app.run();
}
