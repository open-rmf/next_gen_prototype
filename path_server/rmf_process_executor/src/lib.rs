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
use bevy_ros2::{RclrsExecutorCommands, RclrsNode, RclrsPlugin, RosPublisher, RosSubscription};
use crossflow::prelude::*;
pub use crossflow::{
    service::Service, CrossflowPlugin, Diagram, DiagramElementRegistry, JsonMessage,
    NodeBuilderOptions,
};
use futures::FutureExt;
use rmf_participant_discovery::ParticipantTracker;
use ros_env::rmf_prototype_msgs::msg::{
    Action, ActionProgress, Error as RmfError, ParticipantList, PlanId,
};
pub use schemars::JsonSchema;
use serde::{de::DeserializeOwned, Deserialize, Serialize};
use std::{
    collections::HashMap,
    future::Future,
    sync::{Arc, Mutex},
    time::Duration,
};

#[derive(Component, Clone, Debug)]
pub struct ProcessAgent {
    pub name: String,
}

impl ProcessAgent {
    pub fn new(name: impl Into<String>) -> Self {
        Self { name: name.into() }
    }
}

#[derive(Component)]
pub struct ActionSubscription {
    pub subscriber: Arc<RosSubscription<Action>>,
}

#[derive(Component)]
pub struct ActionProgressPublisher {
    pub publisher: Arc<RosPublisher<ActionProgress>>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct ActionKey {
    pub plan_id: PlanId,
    pub waypoint_id: u64,
    pub phase: u8,
}

#[derive(Component, Default, Clone, Debug)]
pub struct ProcessAgentState {
    pub next_seq: u64,
    pub active_seq: Option<u64>,
    pub last_dispatched: Option<ActionKey>,
    pub completed: Vec<ActionKey>,
}

impl ProcessAgentState {
    pub fn is_completed(&self, key: &ActionKey) -> bool {
        self.completed.iter().any(|k| k == key)
    }

    pub fn record_completed(&mut self, key: ActionKey) {
        if !self.is_completed(&key) {
            if self.completed.len() >= 32 {
                self.completed.remove(0);
            }
            self.completed.push(key);
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct ParsedActionSpec {
    pub category: String,
    pub params: serde_json::Value,
}

/// Parse an `Action.action` specification string into `(category, params)`.
///
/// Supports:
/// 1. Crossflow JSON `Diagram`s: `{"version": "0.1.0", "start": "...", "ops": {...}}`
///    or `{"category": "diagram", "diagram": {...}, "input": {...}}`
/// 2. JSON objects: `{"category": "wait", "duration_sec": 1.5}` or
///    `{"category": "workcell", "params": {"station": "A"}}`
/// 3. Colon shorthand: `"wait:1.5"` -> `category = "wait"`, `params = {"arg": "1.5", "duration_sec": 1.5}`
/// 4. Bare action identifiers: `"open_door"` -> `category = "open_door"`, `params = {}`
pub fn parse_action_spec(raw: &str) -> ParsedActionSpec {
    let trimmed = raw.trim();
    if trimmed.starts_with('{') {
        match serde_json::from_str::<serde_json::Value>(trimmed) {
            Ok(serde_json::Value::Object(mut map)) => {
                if !map.contains_key("category")
                    && ((map.contains_key("ops") && map.contains_key("start"))
                        || map.contains_key("diagram"))
                {
                    return ParsedActionSpec {
                        category: "diagram".to_string(),
                        params: serde_json::Value::Object(map),
                    };
                }
                let category = map
                    .remove("category")
                    .and_then(|v| v.as_str().map(|s| s.to_string()))
                    .unwrap_or_else(|| "default".to_string());
                let params = map
                    .remove("params")
                    .unwrap_or_else(|| serde_json::Value::Object(map));
                return ParsedActionSpec { category, params };
            }
            Ok(_) => {
                return ParsedActionSpec {
                    category: "fail".to_string(),
                    params: serde_json::json!({
                        "message": "Action JSON must be a JSON object"
                    }),
                };
            }
            Err(err) => {
                return ParsedActionSpec {
                    category: "fail".to_string(),
                    params: serde_json::json!({
                        "message": format!("Malformed action JSON: {err}")
                    }),
                };
            }
        }
    }

    if let Some((cat, arg)) = trimmed.split_once(':') {
        let category = cat.trim().to_string();
        let arg_trimmed = arg.trim();
        let mut obj = serde_json::Map::new();
        obj.insert(
            "arg".to_string(),
            serde_json::Value::String(arg_trimmed.to_string()),
        );
        if let Ok(num) = arg_trimmed.parse::<f64>() {
            if let Some(n) = serde_json::Number::from_f64(num) {
                obj.insert("duration_sec".to_string(), serde_json::Value::Number(n));
            }
        }
        return ParsedActionSpec {
            category,
            params: serde_json::Value::Object(obj),
        };
    }

    ParsedActionSpec {
        category: trimmed.to_string(),
        params: serde_json::Value::Object(serde_json::Map::new()),
    }
}

#[derive(Clone, Debug)]
pub struct ProcessActionRequest {
    pub agent: Entity,
    pub robot_id: String,
    pub seq: u64,
    pub plan_id: PlanId,
    pub waypoint_id: u64,
    pub phase: u8,
    pub action_raw: String,
    pub category: String,
    pub params: serde_json::Value,
}

impl ProcessActionRequest {
    pub fn duration_sec(&self, default_sec: f64) -> f64 {
        if let Some(ms) = self.params.get("duration_ms").and_then(|v| v.as_f64()) {
            return (ms / 1000.0).max(0.0);
        }
        if let Some(raw_val) = self
            .params
            .get("duration_sec")
            .or_else(|| self.params.get("duration"))
            .and_then(|v| v.as_f64())
        {
            let secs = if raw_val >= 60.0 && self.params.get("arg").is_some() {
                raw_val / 1000.0
            } else {
                raw_val
            };
            return secs.max(0.0);
        }
        default_sec.max(0.0)
    }

    pub fn succeed(self) -> ProcessActionExecution {
        ProcessActionExecution {
            request: self,
            outcome: ProcessActionOutcome::Succeeded,
        }
    }

    pub fn fail(self, message: impl Into<String>) -> ProcessActionExecution {
        ProcessActionExecution {
            request: self,
            outcome: ProcessActionOutcome::Failed {
                message: message.into(),
            },
        }
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum ProcessActionOutcome {
    Succeeded,
    Failed { message: String },
}

#[derive(Clone, Debug)]
pub struct ProcessActionExecution {
    pub request: ProcessActionRequest,
    pub outcome: ProcessActionOutcome,
}

#[derive(Resource, Clone)]
pub struct ProcessExecutorConfig {
    pub default_action_duration_sec: f64,
    pub discovery_topic: String,
}

impl Default for ProcessExecutorConfig {
    fn default() -> Self {
        Self {
            default_action_duration_sec: 0.1,
            discovery_topic: "/destination/discovery".to_string(),
        }
    }
}

#[derive(Resource, Clone)]
pub struct ProcessExecutorServices {
    pub prepare_service: Service<ProcessActionRequest, Option<ProcessActionRequest>>,
    pub reconcile_service: Service<ProcessActionExecution, ()>,
    pub default_workflow: Service<ProcessActionRequest, ()>,
    pub category_workflows: HashMap<String, Service<ProcessActionRequest, ()>>,
}

impl ProcessExecutorServices {
    pub fn workflow_for(&self, category: &str) -> Service<ProcessActionRequest, ()> {
        self.category_workflows
            .get(&category.to_ascii_lowercase())
            .cloned()
            .unwrap_or_else(|| self.default_workflow.clone())
    }
}

#[derive(Clone, Debug, Default, Serialize, Deserialize, JsonSchema)]
pub struct WaitDiagramNodeConfig {
    #[serde(default)]
    pub duration_sec: Option<f64>,
    #[serde(default)]
    pub duration_ms: Option<f64>,
}

#[derive(Clone, Debug, Default, Serialize, Deserialize, JsonSchema)]
pub struct FailDiagramNodeConfig {
    #[serde(default)]
    pub message: Option<String>,
}

pub trait ProcessExecutorAppExt {
    /// Register a custom Crossflow service or workflow to handle `Action` requests matching `category`.
    fn register_process_action(
        &mut self,
        category: impl Into<String>,
        service: Service<ProcessActionRequest, ProcessActionExecution>,
    ) -> &mut Self;

    /// Override the default fallback Crossflow service for unregistered action categories.
    fn set_default_process_action(
        &mut self,
        service: Service<ProcessActionRequest, ProcessActionExecution>,
    ) -> &mut Self;

    /// Access and mutate the Crossflow `DiagramElementRegistry` to register node builders,
    /// section builders, or message types for JSON diagrams sent over the wire.
    fn register_diagram_elements(
        &mut self,
        f: impl FnOnce(&mut DiagramElementRegistry),
    ) -> &mut Self;

    /// Register a Bevy/Crossflow `Service<(Config, JsonMessage), JsonMessage>` as a reusable
    /// node builder (`"builder": "<id>"`) inside Crossflow JSON `Diagram`s.
    fn register_diagram_node<Config>(
        &mut self,
        id: impl Into<Arc<str>>,
        service: Service<(Config, JsonMessage), JsonMessage>,
    ) -> &mut Self
    where
        Config: JsonSchema + Serialize + DeserializeOwned + Clone + Send + Sync + 'static;
}

fn build_wrapped_action_workflow(
    app: &mut App,
    prepare_service: Service<ProcessActionRequest, Option<ProcessActionRequest>>,
    action_service: Service<ProcessActionRequest, ProcessActionExecution>,
    reconcile_service: Service<ProcessActionExecution, ()>,
) -> Service<ProcessActionRequest, ()> {
    app.world_mut().spawn_io_workflow(move |scope, builder| {
        builder
            .chain(scope.start)
            .then(prepare_service)
            .dispose_on_none()
            .then(action_service)
            .then(reconcile_service)
            .connect(scope.terminate);
    })
}

impl ProcessExecutorAppExt for App {
    fn register_process_action(
        &mut self,
        category: impl Into<String>,
        service: Service<ProcessActionRequest, ProcessActionExecution>,
    ) -> &mut Self {
        let cat = category.into().to_ascii_lowercase();
        let (prepare, reconcile) = {
            let services = self.world().resource::<ProcessExecutorServices>();
            (
                services.prepare_service.clone(),
                services.reconcile_service.clone(),
            )
        };
        let workflow = build_wrapped_action_workflow(self, prepare, service, reconcile);
        self.world_mut()
            .resource_mut::<ProcessExecutorServices>()
            .category_workflows
            .insert(cat, workflow);
        self
    }

    fn set_default_process_action(
        &mut self,
        service: Service<ProcessActionRequest, ProcessActionExecution>,
    ) -> &mut Self {
        let (prepare, reconcile) = {
            let services = self.world().resource::<ProcessExecutorServices>();
            (
                services.prepare_service.clone(),
                services.reconcile_service.clone(),
            )
        };
        let workflow = build_wrapped_action_workflow(self, prepare, service, reconcile);
        self.world_mut()
            .resource_mut::<ProcessExecutorServices>()
            .default_workflow = workflow;
        self
    }

    fn register_diagram_elements(
        &mut self,
        f: impl FnOnce(&mut DiagramElementRegistry),
    ) -> &mut Self {
        if !self.world().contains_non_send::<DiagramElementRegistry>() {
            self.insert_non_send_resource(DiagramElementRegistry::new());
        }
        let mut registry = self
            .world_mut()
            .non_send_resource_mut::<DiagramElementRegistry>();
        f(&mut registry);
        self
    }

    fn register_diagram_node<Config>(
        &mut self,
        id: impl Into<Arc<str>>,
        service: Service<(Config, JsonMessage), JsonMessage>,
    ) -> &mut Self
    where
        Config: JsonSchema + Serialize + DeserializeOwned + Clone + Send + Sync + 'static,
    {
        let builder_id: Arc<str> = id.into();
        self.register_diagram_elements(move |registry| {
            registry.register_node_builder(
                NodeBuilderOptions::new(builder_id),
                move |builder, config: Config| {
                    let cfg = config.clone();
                    let pair_input =
                        builder.create_map_block(move |msg: JsonMessage| (cfg.clone(), msg));
                    let exec_node = builder.create_node(service);
                    builder.connect(pair_input.output, exec_node.input);
                    crossflow::Node::<JsonMessage, JsonMessage, ()> {
                        input: pair_input.input,
                        output: exec_node.output,
                        streams: (),
                    }
                },
            );
        })
    }
}

fn prepare_process_action(
    Blocking { request, .. }: Blocking<ProcessActionRequest>,
    states: Query<&ProcessAgentState>,
    publishers: Query<&ActionProgressPublisher>,
) -> Option<ProcessActionRequest> {
    let Ok(state) = states.get(request.agent) else {
        return None;
    };
    if state.active_seq != Some(request.seq) {
        return None;
    }

    if let Ok(pub_comp) = publishers.get(request.agent) {
        let _ = pub_comp.publisher.publish(ActionProgress {
            plan_id: request.plan_id.clone(),
            waypoint_id: request.waypoint_id,
            phase: request.phase,
            status: ActionProgress::STATUS_RUNNING,
            error: RmfError::default(),
        });
    }

    info!(
        "[{}] Executing process action (waypoint={}, phase={}, category='{}', raw='{}')",
        request.robot_id, request.waypoint_id, request.phase, request.category, request.action_raw
    );

    Some(request)
}

fn execute_wait_action(
    Async { request, .. }: Async<ProcessActionRequest>,
    config: Res<ProcessExecutorConfig>,
    executor_commands: Res<RclrsExecutorCommands>,
) -> impl Future<Output = ProcessActionExecution> {
    let duration =
        Duration::from_secs_f64(request.duration_sec(config.default_action_duration_sec));
    let req_clone = request.clone();
    executor_commands
        .run(async move {
            let (tx, rx) = tokio::sync::oneshot::channel::<()>();
            std::thread::spawn(move || {
                std::thread::sleep(duration);
                let _ = tx.send(());
            });
            let _ = rx.await;
            req_clone.succeed()
        })
        .map(move |res| res.unwrap_or_else(|_| request.succeed()))
}

fn execute_wait_diagram_node(
    Async {
        request: (node_cfg, msg),
        ..
    }: Async<(WaitDiagramNodeConfig, JsonMessage)>,
    config: Res<ProcessExecutorConfig>,
    executor_commands: Res<RclrsExecutorCommands>,
) -> impl Future<Output = JsonMessage> {
    let secs = if let Some(ms) = node_cfg.duration_ms {
        (ms / 1000.0).max(0.0)
    } else {
        node_cfg
            .duration_sec
            .unwrap_or(config.default_action_duration_sec)
            .max(0.0)
    };
    let duration = Duration::from_secs_f64(secs);
    let msg_clone = msg.clone();
    executor_commands
        .run(async move {
            let (tx, rx) = tokio::sync::oneshot::channel::<()>();
            std::thread::spawn(move || {
                std::thread::sleep(duration);
                let _ = tx.send(());
            });
            let _ = rx.await;
            msg_clone
        })
        .map(move |res| res.unwrap_or(msg))
}

fn execute_fail_diagram_node(
    Blocking {
        request: (node_cfg, mut msg),
        ..
    }: Blocking<(FailDiagramNodeConfig, JsonMessage)>,
) -> JsonMessage {
    let err_msg = node_cfg
        .message
        .unwrap_or_else(|| "Diagram fail node triggered".to_string());
    if let Some(obj) = msg.as_object_mut() {
        obj.insert("error".to_string(), serde_json::Value::String(err_msg));
        msg
    } else {
        serde_json::json!({ "error": err_msg })
    }
}

fn execute_fail_action(
    Blocking { request, .. }: Blocking<ProcessActionRequest>,
) -> ProcessActionExecution {
    let msg = request
        .params
        .get("arg")
        .or_else(|| request.params.get("message"))
        .and_then(|v| v.as_str())
        .unwrap_or("Process action failed")
        .to_string();
    request.fail(msg)
}

fn reconcile_process_action(
    Blocking {
        request: execution, ..
    }: Blocking<ProcessActionExecution>,
    mut states: Query<&mut ProcessAgentState>,
    publishers: Query<&ActionProgressPublisher>,
) {
    let req = execution.request;
    let Ok(mut state) = states.get_mut(req.agent) else {
        return;
    };
    if state.active_seq != Some(req.seq) {
        // Superseded by a newer action or plan
        return;
    }
    state.active_seq = None;

    let key = ActionKey {
        plan_id: req.plan_id.clone(),
        waypoint_id: req.waypoint_id,
        phase: req.phase,
    };

    let progress_msg = match execution.outcome {
        ProcessActionOutcome::Succeeded => {
            state.record_completed(key);
            info!(
                "[{}] Completed process action (waypoint={}, phase={}, category='{}')",
                req.robot_id, req.waypoint_id, req.phase, req.category
            );
            ActionProgress {
                plan_id: req.plan_id,
                waypoint_id: req.waypoint_id,
                phase: req.phase,
                status: ActionProgress::STATUS_SUCCEEDED,
                error: RmfError::default(),
            }
        }
        ProcessActionOutcome::Failed { message } => {
            warn!(
                "[{}] Failed process action (waypoint={}, phase={}, category='{}'): {}",
                req.robot_id, req.waypoint_id, req.phase, req.category, message
            );
            ActionProgress {
                plan_id: req.plan_id,
                waypoint_id: req.waypoint_id,
                phase: req.phase,
                status: ActionProgress::STATUS_FAILED,
                error: RmfError {
                    code: RmfError::CODE_UNSPECIFIED_ERROR,
                    message,
                    parameters: String::new(),
                },
            }
        }
    };

    if let Ok(pub_comp) = publishers.get(req.agent) {
        let _ = pub_comp.publisher.publish(progress_msg);
    }
}

#[derive(Resource)]
struct DiscoveryState {
    subscriber: Arc<RosSubscription<ParticipantList>>,
    tracker: Mutex<ParticipantTracker>,
}

fn setup_agent_io(
    trigger: Trigger<OnAdd, ProcessAgent>,
    mut commands: Commands,
    agents: Query<&ProcessAgent>,
    node: Res<RclrsNode>,
) {
    let e = trigger.target();
    let Ok(agent) = agents.get(e) else {
        return;
    };
    let action_topic = format!("{}/plan/action", agent.name);
    let progress_topic = format!("{}/plan/action/progress", agent.name);

    let sub = Arc::new(RosSubscription::<Action>::new(&node, action_topic));
    let pub_ = Arc::new(RosPublisher::<ActionProgress>::new_transient_local(
        &node,
        progress_topic,
    ));

    commands.entity(e).insert((
        ActionSubscription { subscriber: sub },
        ActionProgressPublisher { publisher: pub_ },
        ProcessAgentState::default(),
    ));
}

fn poll_discovery(
    mut commands: Commands,
    discovery: Option<Res<DiscoveryState>>,
    existing_agents: Query<(Entity, &ProcessAgent)>,
) {
    let Some(discovery) = discovery else {
        return;
    };
    while let Some(msg) = discovery.subscriber.take_data() {
        let (added, removed) = {
            let Ok(mut tracker) = discovery.tracker.lock() else {
                return;
            };
            tracker.update(&msg)
        };

        for rem_name in removed {
            for (entity, agent) in existing_agents.iter() {
                if agent.name == rem_name {
                    commands.entity(entity).despawn();
                }
            }
        }

        for add_name in added {
            if !existing_agents.iter().any(|(_, a)| a.name == add_name) {
                info!("ProcessExecutor discovered participant: {}", add_name);
                commands.spawn(ProcessAgent::new(add_name));
            }
        }
    }
}

fn poll_incoming_actions(
    mut commands: Commands,
    services: Res<ProcessExecutorServices>,
    diagram_registry: NonSend<DiagramElementRegistry>,
    mut agents: Query<(
        Entity,
        &ProcessAgent,
        &ActionSubscription,
        &ActionProgressPublisher,
        &mut ProcessAgentState,
    )>,
) {
    for (entity, agent, action_sub, progress_pub, mut state) in agents.iter_mut() {
        while let Some(action_msg) = action_sub.subscriber.take_data() {
            if action_msg.action.trim().is_empty() {
                continue;
            }
            let key = ActionKey {
                plan_id: action_msg.plan_id.clone(),
                waypoint_id: action_msg.waypoint_id,
                phase: action_msg.phase,
            };

            if state.is_completed(&key) {
                // Re-publish STATUS_SUCCEEDED in case plan_executor missed a transient message
                let _ = progress_pub.publisher.publish(ActionProgress {
                    plan_id: action_msg.plan_id,
                    waypoint_id: action_msg.waypoint_id,
                    phase: action_msg.phase,
                    status: ActionProgress::STATUS_SUCCEEDED,
                    error: RmfError::default(),
                });
                continue;
            }

            if state.active_seq.is_some() && state.last_dispatched.as_ref() == Some(&key) {
                // Already executing this exact Action
                continue;
            }

            state.next_seq += 1;
            let seq = state.next_seq;
            state.active_seq = Some(seq);
            state.last_dispatched = Some(key);

            // Immediately publish STATUS_RUNNING so rmf_path_server and rmf_plan_executor
            // know the process action is active without waiting for the next schedule tick.
            let _ = progress_pub.publisher.publish(ActionProgress {
                plan_id: action_msg.plan_id.clone(),
                waypoint_id: action_msg.waypoint_id,
                phase: action_msg.phase,
                status: ActionProgress::STATUS_RUNNING,
                error: RmfError::default(),
            });

            let spec = parse_action_spec(&action_msg.action);
            let request = ProcessActionRequest {
                agent: entity,
                robot_id: agent.name.clone(),
                seq,
                plan_id: action_msg.plan_id,
                waypoint_id: action_msg.waypoint_id,
                phase: action_msg.phase,
                action_raw: action_msg.action,
                category: spec.category.clone(),
                params: spec.params,
            };

            if spec.category.eq_ignore_ascii_case("diagram")
                && !services.category_workflows.contains_key("diagram")
            {
                let diagram_val = request
                    .params
                    .get("diagram")
                    .cloned()
                    .unwrap_or_else(|| request.params.clone());

                let spawned = Diagram::from_json(diagram_val)
                    .map_err(|e| format!("Invalid Crossflow Diagram JSON: {e}"))
                    .and_then(|diagram| {
                        diagram
                            .spawn_io_workflow::<JsonMessage, JsonMessage>(
                                &mut commands,
                                &diagram_registry,
                            )
                            .map_err(|e| format!("Failed to build Crossflow Diagram workflow: {e}"))
                    });

                match spawned {
                    Ok(diagram_service) => {
                        let prepare_srv = services.prepare_service.clone();
                        let reconcile_srv = services.reconcile_service.clone();
                        let req_for_map = request.clone();
                        let req_for_out = request.clone();
                        let wrapped = commands.spawn_io_workflow(
                            move |scope: Scope<ProcessActionRequest, ()>, builder: &mut Builder| {
                                builder
                                    .chain(scope.start)
                                    .then(prepare_srv)
                                    .dispose_on_none()
                                    .map_block(move |req: ProcessActionRequest| {
                                        let mut input = req
                                            .params
                                            .get("input")
                                            .cloned()
                                            .unwrap_or_else(|| serde_json::json!({}));
                                        if let Some(obj) = input.as_object_mut() {
                                            obj.entry("robot_id".to_string()).or_insert_with(
                                                || {
                                                    serde_json::Value::String(
                                                        req_for_map.robot_id.clone(),
                                                    )
                                                },
                                            );
                                            obj.entry("waypoint_id".to_string()).or_insert_with(
                                                || serde_json::Value::from(req_for_map.waypoint_id),
                                            );
                                            obj.entry("phase".to_string()).or_insert_with(|| {
                                                serde_json::Value::from(req_for_map.phase)
                                            });
                                        }
                                        input
                                    })
                                    .then(diagram_service)
                                    .map_block(move |output: JsonMessage| {
                                        if let Some(err_msg) = output
                                            .get("error")
                                            .and_then(|v| v.as_str())
                                            .filter(|s| !s.is_empty())
                                        {
                                            req_for_out.clone().fail(err_msg.to_string())
                                        } else {
                                            req_for_out.clone().succeed()
                                        }
                                    })
                                    .then(reconcile_srv)
                                    .connect(scope.terminate);
                            },
                        );
                        let _ = commands.request(request, wrapped).detach();
                    }
                    Err(err_msg) => {
                        state.active_seq = None;
                        warn!(
                            "[{}] Diagram action failed before execution: {}",
                            request.robot_id, err_msg
                        );
                        let _ = progress_pub.publisher.publish(ActionProgress {
                            plan_id: request.plan_id,
                            waypoint_id: request.waypoint_id,
                            phase: request.phase,
                            status: ActionProgress::STATUS_FAILED,
                            error: RmfError {
                                code: RmfError::CODE_UNSPECIFIED_ERROR,
                                message: err_msg,
                                parameters: String::new(),
                            },
                        });
                    }
                }
                continue;
            }

            let workflow = services.workflow_for(&spec.category);
            let _ = commands.request(request, workflow).detach();
        }
    }
}

#[derive(Default)]
pub struct ProcessExecutorPlugin {
    pub config: ProcessExecutorConfig,
}

impl ProcessExecutorPlugin {
    pub fn with_config(config: ProcessExecutorConfig) -> Self {
        Self { config }
    }
}

impl Plugin for ProcessExecutorPlugin {
    fn build(&self, app: &mut App) {
        if !app.is_plugin_added::<CrossflowPlugin>() {
            app.add_plugins(CrossflowPlugin::default());
        }
        if !app.world().contains_resource::<RclrsNode>() {
            app.add_plugins(RclrsPlugin::with_node_name("rmf_process_executor"));
        }
        if !app.world().contains_non_send::<DiagramElementRegistry>() {
            app.insert_non_send_resource(DiagramElementRegistry::new());
        }

        let default_duration = app
            .world()
            .get_resource::<RclrsNode>()
            .and_then(|node| {
                node.declare_parameter("default_action_duration")
                    .default(self.config.default_action_duration_sec)
                    .mandatory()
                    .ok()
                    .map(|p| p.get().max(0.0))
            })
            .unwrap_or(self.config.default_action_duration_sec);

        app.insert_resource(ProcessExecutorConfig {
            default_action_duration_sec: default_duration,
            discovery_topic: self.config.discovery_topic.clone(),
        });

        if let Some(node) = app.world().get_resource::<RclrsNode>() {
            let sub = Arc::new(RosSubscription::<ParticipantList>::new(
                node,
                self.config.discovery_topic.clone(),
            ));
            app.insert_resource(DiscoveryState {
                subscriber: sub,
                tracker: Mutex::new(ParticipantTracker::new()),
            });
        }

        let prepare_service = app.spawn_service(prepare_process_action);
        let wait_service = app.spawn_service(execute_wait_action);
        let fail_service = app.spawn_service(execute_fail_action);
        let reconcile_service = app.spawn_service(reconcile_process_action);

        let wait_diagram_node = app.spawn_service(execute_wait_diagram_node);
        let fail_diagram_node = app.spawn_service(execute_fail_diagram_node);

        let default_workflow = build_wrapped_action_workflow(
            app,
            prepare_service.clone(),
            wait_service.clone(),
            reconcile_service.clone(),
        );

        let services = ProcessExecutorServices {
            prepare_service,
            reconcile_service,
            default_workflow,
            category_workflows: HashMap::new(),
        };
        app.insert_resource(services);

        app.register_process_action("wait", wait_service.clone())
            .register_process_action("timer", wait_service.clone())
            .register_process_action("delay", wait_service)
            .register_process_action("fail", fail_service)
            .register_diagram_node("wait", wait_diagram_node.clone())
            .register_diagram_node("timer", wait_diagram_node.clone())
            .register_diagram_node("delay", wait_diagram_node)
            .register_diagram_node("fail", fail_diagram_node);

        app.add_observer(setup_agent_io)
            .add_systems(PreUpdate, (poll_discovery, poll_incoming_actions).chain());
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_parse_action_spec_formats() {
        let json_spec = parse_action_spec(r#"{"category": "wait", "duration_sec": 2.5}"#);
        assert_eq!(json_spec.category, "wait");
        assert_eq!(
            json_spec
                .params
                .get("duration_sec")
                .and_then(|v| v.as_f64()),
            Some(2.5)
        );

        let shorthand = parse_action_spec("wait:1.25");
        assert_eq!(shorthand.category, "wait");
        assert_eq!(
            shorthand
                .params
                .get("duration_sec")
                .and_then(|v| v.as_f64()),
            Some(1.25)
        );

        let bare = parse_action_spec("open_door");
        assert_eq!(bare.category, "open_door");

        let diagram_raw = r#"{
            "version": "0.1.0",
            "start": "step_1",
            "ops": {
                "step_1": {
                    "type": "node",
                    "builder": "wait",
                    "config": { "duration_sec": 0.5 },
                    "next": { "builtin": "terminate" }
                }
            }
        }"#;
        let diagram_spec = parse_action_spec(diagram_raw);
        assert_eq!(diagram_spec.category, "diagram");
        assert!(diagram_spec.params.get("ops").is_some());

        let malformed = parse_action_spec("{bad json: 123");
        assert_eq!(malformed.category, "fail");
        assert!(malformed
            .params
            .get("message")
            .and_then(|v| v.as_str())
            .is_some_and(|m| m.contains("Malformed action JSON")));
    }
}
