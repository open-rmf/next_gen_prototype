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

use bevy::prelude::*;
use bevy_ros2::RclrsExecutorCommands;
use crossflow::prelude::*;
use futures::FutureExt;
use rclrs::{Context, CreateBasicExecutor, IntoPrimitiveOptions, SpinOptions};
use rmf_process_executor::{
    JsonMessage, JsonSchema, ProcessActionExecution, ProcessActionRequest, ProcessExecutorAppExt,
    ProcessExecutorConfig, ProcessExecutorPlugin,
};
use ros_env::rmf_prototype_msgs::msg::{
    Action, ActionProgress, Participant, ParticipantList, PlanId,
};
use serde::{Deserialize, Serialize};
use std::future::Future;
use std::sync::{
    atomic::{AtomicUsize, Ordering},
    Arc, Mutex,
};
use std::time::{Duration, Instant};

#[derive(Resource, Clone)]
struct CustomActionCounter(Arc<AtomicUsize>);

fn custom_conveyor_action(
    Async { request, .. }: Async<ProcessActionRequest>,
    counter: Res<CustomActionCounter>,
    executor_commands: Res<RclrsExecutorCommands>,
) -> impl Future<Output = ProcessActionExecution> {
    counter.0.fetch_add(1, Ordering::SeqCst);
    let req_clone = request.clone();
    executor_commands
        .run(async move {
            let (tx, rx) = tokio::sync::oneshot::channel::<()>();
            std::thread::spawn(move || {
                std::thread::sleep(Duration::from_millis(120));
                let _ = tx.send(());
            });
            let _ = rx.await;
            req_clone.succeed()
        })
        .map(move |res| res.unwrap_or_else(|_| request.succeed()))
}

#[test]
fn test_custom_and_builtin_crossflow_actions_emit_running_and_succeeded() {
    let custom_counter = Arc::new(AtomicUsize::new(0));
    let mut app = App::new();
    app.add_plugins(MinimalPlugins);
    app.insert_resource(CustomActionCounter(Arc::clone(&custom_counter)));
    app.add_plugins(ProcessExecutorPlugin::with_config(ProcessExecutorConfig {
        default_action_duration_sec: 0.1,
        discovery_topic: "/test_proc_exec_1/discovery".to_string(),
    }));

    let custom_service = app.spawn_service(custom_conveyor_action);
    app.register_process_action("custom_conveyor", custom_service);

    let context = Context::default_from_env().unwrap();
    let mut ros_exec = context.create_basic_executor();
    let test_node = ros_exec.create_node("test_process_executor_node").unwrap();

    let discovery_pub = test_node
        .create_publisher::<ParticipantList>(
            "/test_proc_exec_1/discovery".transient_local().reliable(),
        )
        .unwrap();
    let action_pub = test_node
        .create_publisher::<Action>("robot_proc_1/plan/action".transient_local().reliable())
        .unwrap();

    let observed_statuses = Arc::new(Mutex::new(Vec::<ActionProgress>::new()));
    let observed_clone = Arc::clone(&observed_statuses);
    let _progress_sub = test_node
        .create_subscription::<ActionProgress, _>(
            "robot_proc_1/plan/action/progress"
                .transient_local()
                .reliable(),
            move |msg: ActionProgress| {
                observed_clone.lock().unwrap().push(msg);
            },
        )
        .unwrap();

    let discovery_msg = ParticipantList {
        participants: vec![Participant {
            name: "robot_proc_1".to_string(),
            components: vec![],
        }],
    };
    discovery_pub.publish(&discovery_msg).unwrap();

    let mut plan_id = PlanId::default();
    plan_id.destination_session.uuid = [42; 16];
    plan_id.plan_version = 1;

    // 1. Trigger custom_conveyor arrival action on waypoint 1
    let arrival_action = Action {
        plan_id: plan_id.clone(),
        waypoint_id: 1,
        phase: Action::PHASE_ARRIVAL,
        action: "custom_conveyor:station_A".to_string(),
    };
    action_pub.publish(&arrival_action).unwrap();

    let deadline = Instant::now() + Duration::from_secs(5);
    let mut arrival_succeeded = false;
    while Instant::now() < deadline {
        discovery_pub.publish(&discovery_msg).unwrap();
        app.update();
        ros_exec.spin(SpinOptions::spin_once().timeout(Duration::from_millis(20)));

        let statuses = observed_statuses.lock().unwrap();
        if statuses.iter().any(|s| {
            s.waypoint_id == 1
                && s.phase == Action::PHASE_ARRIVAL
                && s.status == ActionProgress::STATUS_SUCCEEDED
        }) {
            arrival_succeeded = true;
            break;
        }
    }

    assert!(
        arrival_succeeded,
        "Timeout waiting for custom_conveyor arrival_action to succeed"
    );
    assert_eq!(custom_counter.load(Ordering::SeqCst), 1);
    {
        let statuses = observed_statuses.lock().unwrap();
        assert!(
            statuses.iter().any(|s| {
                s.waypoint_id == 1
                    && s.phase == Action::PHASE_ARRIVAL
                    && s.status == ActionProgress::STATUS_RUNNING
            }),
            "Expected STATUS_RUNNING before STATUS_SUCCEEDED"
        );
    }

    // 2. Trigger built-in wait departure action on waypoint 1
    let departure_action = Action {
        plan_id: plan_id.clone(),
        waypoint_id: 1,
        phase: Action::PHASE_DEPARTURE,
        action: r#"{"category": "wait", "duration_sec": 0.1}"#.to_string(),
    };
    action_pub.publish(&departure_action).unwrap();

    let deadline = Instant::now() + Duration::from_secs(5);
    let mut departure_succeeded = false;
    while Instant::now() < deadline {
        app.update();
        ros_exec.spin(SpinOptions::spin_once().timeout(Duration::from_millis(20)));

        let statuses = observed_statuses.lock().unwrap();
        if statuses.iter().any(|s| {
            s.waypoint_id == 1
                && s.phase == Action::PHASE_DEPARTURE
                && s.status == ActionProgress::STATUS_SUCCEEDED
        }) {
            departure_succeeded = true;
            break;
        }
    }

    assert!(
        departure_succeeded,
        "Timeout waiting for wait departure_action to succeed"
    );
}

#[derive(Clone, Debug, Default, Serialize, Deserialize, JsonSchema)]
struct StepNodeConfig {
    step_name: String,
}

fn record_diagram_step(
    Blocking {
        request: (cfg, mut msg),
        ..
    }: Blocking<(StepNodeConfig, JsonMessage)>,
    counter: Res<CustomActionCounter>,
) -> JsonMessage {
    counter.0.fetch_add(1, Ordering::SeqCst);
    if let Some(obj) = msg.as_object_mut() {
        obj.insert(
            "last_step".to_string(),
            serde_json::Value::String(cfg.step_name),
        );
    }
    msg
}

#[test]
fn test_crossflow_json_diagram_over_the_wire_executes_multi_step_workflow() {
    let step_counter = Arc::new(AtomicUsize::new(0));
    let mut app = App::new();
    app.add_plugins(MinimalPlugins);
    app.insert_resource(CustomActionCounter(Arc::clone(&step_counter)));
    app.add_plugins(ProcessExecutorPlugin::with_config(ProcessExecutorConfig {
        default_action_duration_sec: 0.1,
        discovery_topic: "/test_proc_exec_2/discovery".to_string(),
    }));

    let step_service = app.spawn_service(record_diagram_step);
    app.register_diagram_node("record_step", step_service);

    let context = Context::default_from_env().unwrap();
    let mut ros_exec = context.create_basic_executor();
    let test_node = ros_exec
        .create_node("test_process_executor_diagram_node")
        .unwrap();

    let discovery_pub = test_node
        .create_publisher::<ParticipantList>(
            "/test_proc_exec_2/discovery".transient_local().reliable(),
        )
        .unwrap();
    let action_pub = test_node
        .create_publisher::<Action>("robot_diag_1/plan/action".transient_local().reliable())
        .unwrap();

    let observed_statuses = Arc::new(Mutex::new(Vec::<ActionProgress>::new()));
    let observed_clone = Arc::clone(&observed_statuses);
    let _progress_sub = test_node
        .create_subscription::<ActionProgress, _>(
            "robot_diag_1/plan/action/progress"
                .transient_local()
                .reliable(),
            move |msg: ActionProgress| {
                observed_clone.lock().unwrap().push(msg);
            },
        )
        .unwrap();

    let discovery_msg = ParticipantList {
        participants: vec![Participant {
            name: "robot_diag_1".to_string(),
            components: vec![],
        }],
    };
    discovery_pub.publish(&discovery_msg).unwrap();

    let mut plan_id = PlanId::default();
    plan_id.destination_session.uuid = [77; 16];
    plan_id.plan_version = 1;

    // Multi-node Crossflow JSON Diagram sent directly over Action.action:
    // step_a -> wait_step -> step_b -> terminate
    let diagram_json = r#"{
        "version": "0.1.0",
        "start": "step_a",
        "ops": {
            "step_a": {
                "type": "node",
                "builder": "record_step",
                "config": { "step_name": "load_pallet" },
                "next": "wait_step"
            },
            "wait_step": {
                "type": "node",
                "builder": "wait",
                "config": { "duration_sec": 0.08 },
                "next": "step_b"
            },
            "step_b": {
                "type": "node",
                "builder": "record_step",
                "config": { "step_name": "verify_latch" },
                "next": { "builtin": "terminate" }
            }
        }
    }"#;

    action_pub
        .publish(&Action {
            plan_id,
            waypoint_id: 2,
            phase: Action::PHASE_ARRIVAL,
            action: diagram_json.to_string(),
        })
        .unwrap();

    let deadline = Instant::now() + Duration::from_secs(5);
    let mut diagram_succeeded = false;
    while Instant::now() < deadline {
        discovery_pub.publish(&discovery_msg).unwrap();
        app.update();
        ros_exec.spin(SpinOptions::spin_once().timeout(Duration::from_millis(20)));

        let statuses = observed_statuses.lock().unwrap();
        if statuses.iter().any(|s| {
            s.waypoint_id == 2
                && s.phase == Action::PHASE_ARRIVAL
                && s.status == ActionProgress::STATUS_SUCCEEDED
        }) {
            diagram_succeeded = true;
            break;
        }
    }

    assert!(
        diagram_succeeded,
        "Timeout waiting for Crossflow JSON Diagram action to succeed"
    );
    assert_eq!(
        step_counter.load(Ordering::SeqCst),
        2,
        "Expected both step_a and step_b nodes in the JSON diagram to execute"
    );
}
