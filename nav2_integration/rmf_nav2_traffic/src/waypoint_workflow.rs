use crate::inner_navigation_client::InnerNavigationAction;
use bevy::prelude::*;
use crossflow::{
    diagram::{Diagram, DiagramElementRegistry, NodeBuilderOptions},
    prelude::*,
};
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use std::collections::HashMap;

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize, JsonSchema, Default)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum WaypointNavActionSpec {
    #[default]
    NavigateToPose,
    DockRobot {
        use_dock_id: bool,
        dock_id: String,
        dock_type: String,
        navigate_to_staging_pose: bool,
    },
    UndockRobot {
        dock_type: String,
    },
}

impl From<WaypointNavActionSpec> for InnerNavigationAction {
    fn from(spec: WaypointNavActionSpec) -> Self {
        match spec {
            WaypointNavActionSpec::NavigateToPose => InnerNavigationAction::NavigateToPose,
            WaypointNavActionSpec::DockRobot {
                use_dock_id,
                dock_id,
                dock_type,
                navigate_to_staging_pose,
            } => InnerNavigationAction::DockRobot {
                use_dock_id,
                dock_id,
                dock_type,
                navigate_to_staging_pose,
            },
            WaypointNavActionSpec::UndockRobot { dock_type } => {
                InnerNavigationAction::UndockRobot { dock_type }
            }
        }
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub enum DoorActionMode {
    Open,
    Close,
}

impl Default for DoorActionMode {
    fn default() -> Self {
        Self::Open
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize, JsonSchema)]
pub struct DoorEventRecord {
    pub door_id: String,
    pub mode: DoorActionMode,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize, JsonSchema)]
pub struct HandoverEventRecord {
    pub station_id: String,
    pub direction: String,
    pub item_type: String,
}

/// Message context passed through a Crossflow `Diagram` when resolving and
/// orchestrating a waypoint's `departure_action` or `arrival_action`.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize, JsonSchema)]
pub struct WaypointActionContext {
    pub robot_name: String,
    pub waypoint_index: u64,
    pub target_x: f64,
    pub target_y: f64,
    pub target_yaw: f64,
    #[serde(default)]
    pub navigation_action: WaypointNavActionSpec,
    #[serde(default)]
    pub hold_progress_gate: bool,
    #[serde(default)]
    pub door_events: Vec<DoorEventRecord>,
    #[serde(default)]
    pub handover_events: Vec<HandoverEventRecord>,
    #[serde(default)]
    pub executed_steps: Vec<String>,
}

impl WaypointActionContext {
    pub fn new(
        robot_name: impl Into<String>,
        waypoint_index: u64,
        target_x: f64,
        target_y: f64,
        target_yaw: f64,
    ) -> Self {
        Self {
            robot_name: robot_name.into(),
            waypoint_index,
            target_x,
            target_y,
            target_yaw,
            navigation_action: WaypointNavActionSpec::NavigateToPose,
            hold_progress_gate: false,
            door_events: Vec::new(),
            handover_events: Vec::new(),
            executed_steps: Vec::new(),
        }
    }

    pub fn into_inner_action_and_gate(self) -> (InnerNavigationAction, bool) {
        (self.navigation_action.into(), self.hold_progress_gate)
    }
}

#[derive(Clone, Debug, Default, Serialize, Deserialize, JsonSchema)]
pub struct NavigateNodeConfig {
    #[serde(default)]
    pub override_x: Option<f64>,
    #[serde(default)]
    pub override_y: Option<f64>,
    #[serde(default)]
    pub override_yaw: Option<f64>,
    #[serde(default)]
    pub hold_gate: bool,
}

fn default_true() -> bool {
    true
}

fn default_dock_type() -> String {
    "simple_charging_dock".to_string()
}

#[derive(Clone, Debug, Serialize, Deserialize, JsonSchema)]
pub struct DockNodeConfig {
    #[serde(default = "default_true")]
    pub use_dock_id: bool,
    #[serde(default)]
    pub dock_id: String,
    #[serde(default = "default_dock_type")]
    pub dock_type: String,
    #[serde(default)]
    pub navigate_to_staging_pose: bool,
    #[serde(default)]
    pub hold_gate_after_dock: bool,
}

impl Default for DockNodeConfig {
    fn default() -> Self {
        Self {
            use_dock_id: true,
            dock_id: String::new(),
            dock_type: default_dock_type(),
            navigate_to_staging_pose: false,
            hold_gate_after_dock: false,
        }
    }
}

#[derive(Clone, Debug, Serialize, Deserialize, JsonSchema)]
pub struct UndockNodeConfig {
    #[serde(default = "default_dock_type")]
    pub dock_type: String,
    #[serde(default)]
    pub hold_gate: bool,
}

impl Default for UndockNodeConfig {
    fn default() -> Self {
        Self {
            dock_type: default_dock_type(),
            hold_gate: false,
        }
    }
}

#[derive(Clone, Debug, Default, Serialize, Deserialize, JsonSchema)]
pub struct DoorNodeConfig {
    pub door_id: String,
    #[serde(default)]
    pub mode: DoorActionMode,
    #[serde(default)]
    pub hold_gate_until_confirmed: bool,
}

fn default_handover_direction() -> String {
    "load".to_string()
}

#[derive(Clone, Debug, Serialize, Deserialize, JsonSchema)]
pub struct HandoverNodeConfig {
    pub station_id: String,
    #[serde(default = "default_handover_direction")]
    pub direction: String,
    #[serde(default)]
    pub item_type: String,
    #[serde(default = "default_true")]
    pub wait_for_gate_release: bool,
}

impl Default for HandoverNodeConfig {
    fn default() -> Self {
        Self {
            station_id: String::new(),
            direction: default_handover_direction(),
            item_type: String::new(),
            wait_for_gate_release: true,
        }
    }
}

/// Build a [`DiagramElementRegistry`] populated with reusable robotics, docking,
/// facility door, and item handover primitives for `rmf_nav2_traffic`.
pub fn build_waypoint_diagram_registry() -> DiagramElementRegistry {
    let mut registry = DiagramElementRegistry::new();

    registry
        .register_node_builder(
            NodeBuilderOptions::new("nav2/navigate_to_pose"),
            |builder, config: NavigateNodeConfig| {
                builder.create_map_block(
                    move |mut ctx: WaypointActionContext| -> Result<WaypointActionContext, String> {
                        if let Some(x) = config.override_x {
                            ctx.target_x = x;
                        }
                        if let Some(y) = config.override_y {
                            ctx.target_y = y;
                        }
                        if let Some(yaw) = config.override_yaw {
                            ctx.target_yaw = yaw;
                        }
                        ctx.navigation_action = WaypointNavActionSpec::NavigateToPose;
                        if config.hold_gate {
                            ctx.hold_progress_gate = true;
                        }
                        ctx.executed_steps.push("nav2/navigate_to_pose".to_string());
                        Ok(ctx)
                    },
                )
            },
        )
        .with_result();

    registry
        .register_node_builder(
            NodeBuilderOptions::new("nav2/dock_robot"),
            |builder, config: DockNodeConfig| {
                builder.create_map_block(
                    move |mut ctx: WaypointActionContext| -> Result<WaypointActionContext, String> {
                        if config.use_dock_id && config.dock_id.trim().is_empty() {
                            return Err(
                                "nav2/dock_robot requires a non-empty dock_id when use_dock_id is true"
                                    .to_string(),
                            );
                        }
                        let dock_id = config.dock_id.trim().to_string();
                        ctx.navigation_action = WaypointNavActionSpec::DockRobot {
                            use_dock_id: config.use_dock_id,
                            dock_id: dock_id.clone(),
                            dock_type: config.dock_type.clone(),
                            navigate_to_staging_pose: config.navigate_to_staging_pose,
                        };
                        if config.hold_gate_after_dock {
                            ctx.hold_progress_gate = true;
                        }
                        ctx.executed_steps
                            .push(format!("nav2/dock_robot:{dock_id}"));
                        Ok(ctx)
                    },
                )
            },
        )
        .with_result();

    registry
        .register_node_builder(
            NodeBuilderOptions::new("nav2/undock_robot"),
            |builder, config: UndockNodeConfig| {
                builder.create_map_block(
                    move |mut ctx: WaypointActionContext| -> Result<WaypointActionContext, String> {
                        ctx.navigation_action = WaypointNavActionSpec::UndockRobot {
                            dock_type: config.dock_type.clone(),
                        };
                        if config.hold_gate {
                            ctx.hold_progress_gate = true;
                        }
                        ctx.executed_steps
                            .push(format!("nav2/undock_robot:{}", config.dock_type));
                        Ok(ctx)
                    },
                )
            },
        )
        .with_result();

    registry
        .register_node_builder(
            NodeBuilderOptions::new("facility/door_request"),
            |builder, config: DoorNodeConfig| {
                builder.create_map_block(
                    move |mut ctx: WaypointActionContext| -> Result<WaypointActionContext, String> {
                        if config.door_id.trim().is_empty() {
                            return Err(
                                "facility/door_request requires a non-empty door_id".to_string()
                            );
                        }
                        let door_id = config.door_id.trim().to_string();
                        let mode_str = match config.mode {
                            DoorActionMode::Open => "open",
                            DoorActionMode::Close => "close",
                        };
                        ctx.door_events.push(DoorEventRecord {
                            door_id: door_id.clone(),
                            mode: config.mode.clone(),
                        });
                        if config.hold_gate_until_confirmed {
                            ctx.hold_progress_gate = true;
                        }
                        ctx.executed_steps
                            .push(format!("facility/door_request:{door_id}:{mode_str}"));
                        Ok(ctx)
                    },
                )
            },
        )
        .with_result();

    registry
        .register_node_builder(
            NodeBuilderOptions::new("workcell/handover"),
            |builder, config: HandoverNodeConfig| {
                builder.create_map_block(
                    move |mut ctx: WaypointActionContext| -> Result<WaypointActionContext, String> {
                        if config.station_id.trim().is_empty() {
                            return Err(
                                "workcell/handover requires a non-empty station_id".to_string()
                            );
                        }
                        let station_id = config.station_id.trim().to_string();
                        let direction = config.direction.trim().to_string();
                        ctx.handover_events.push(HandoverEventRecord {
                            station_id: station_id.clone(),
                            direction: direction.clone(),
                            item_type: config.item_type.clone(),
                        });
                        if config.wait_for_gate_release {
                            ctx.hold_progress_gate = true;
                        }
                        ctx.executed_steps
                            .push(format!("workcell/handover:{station_id}:{direction}"));
                        Ok(ctx)
                    },
                )
            },
        )
        .with_result();

    registry.register_node_builder(
        NodeBuilderOptions::new("traffic/hold_gate"),
        |builder, _config: ()| {
            builder.create_map_block(|mut ctx: WaypointActionContext| {
                ctx.hold_progress_gate = true;
                ctx.executed_steps.push("traffic/hold_gate".to_string());
                ctx
            })
        },
    );

    registry.register_node_builder(
        NodeBuilderOptions::new("traffic/release_gate"),
        |builder, _config: ()| {
            builder.create_map_block(|mut ctx: WaypointActionContext| {
                ctx.hold_progress_gate = false;
                ctx.executed_steps.push("traffic/release_gate".to_string());
                ctx
            })
        },
    );

    registry
}

/// Resource storing custom [`Diagram`] overrides keyed by dock ID or action name
/// (e.g. `"dock_north_1"`), allowing deployments to customize docking, door, or
/// post-dock handover workflows at runtime.
#[derive(Resource, Default, Clone)]
pub struct CustomWaypointDiagrams {
    pub diagrams: HashMap<String, Diagram>,
}

impl CustomWaypointDiagrams {
    pub fn insert_json(
        &mut self,
        key: impl Into<String>,
        diagram_json: &str,
    ) -> Result<(), String> {
        let diagram = Diagram::from_json_str(diagram_json).map_err(|e| e.to_string())?;
        self.diagrams.insert(key.into(), diagram);
        Ok(())
    }

    pub fn get(&self, key: &str) -> Option<&Diagram> {
        self.diagrams.get(key)
    }
}

/// Compile either a full Crossflow [`Diagram`] JSON or a high-level shorthand
/// JSON object into a [`Diagram`].
///
/// Supported JSON inputs:
/// 1. **Full Crossflow Diagram**:
///    `{"version": "0.1.0", "start": "...", "ops": { ... }}`
/// 2. **Shorthand Dock + Optional Door / Handover Workflow**:
///    ```json
///    {
///      "category": "dock",
///      "dock_id": "dock_north_1",
///      "dock_type": "simple_charging_dock",
///      "navigate_to_staging_pose": false,
///      "pre_door": {"door_id": "bay_door_1", "mode": "open"},
///      "handover": {"station_id": "conveyor_north_1", "direction": "load", "wait_for_gate_release": true}
///    }
///    ```
pub fn parse_or_compile_waypoint_diagram(action_str: &str) -> Result<Diagram, String> {
    let trimmed = action_str.trim();
    let value: serde_json::Value = serde_json::from_str(trimmed).map_err(|e| e.to_string())?;
    let Some(obj) = value.as_object() else {
        return Err("Action JSON must be an object".to_string());
    };

    // 1. If this is already a native Crossflow Diagram JSON, parse it directly.
    if obj.contains_key("version") && obj.contains_key("start") && obj.contains_key("ops") {
        return Diagram::from_json(value).map_err(|e| e.to_string());
    }

    // 2. Otherwise compile shorthand JSON into a Crossflow Diagram.
    let category = obj
        .get("category")
        .or_else(|| obj.get("action"))
        .or_else(|| obj.get("type"))
        .and_then(|v| v.as_str())
        .unwrap_or("navigate")
        .to_ascii_lowercase();

    let mut ops = serde_json::Map::new();
    let mut step_names: Vec<String> = Vec::new();

    if let Some(pre_door) = obj.get("pre_door").and_then(|v| v.as_object()) {
        let door_id = pre_door
            .get("door_id")
            .and_then(|v| v.as_str())
            .unwrap_or("default_door");
        let mode = pre_door
            .get("mode")
            .and_then(|v| v.as_str())
            .unwrap_or("open");
        let hold_gate = pre_door
            .get("hold_gate_until_confirmed")
            .and_then(|v| v.as_bool())
            .unwrap_or(false);
        step_names.push("pre_door".to_string());
        ops.insert(
            "pre_door".to_string(),
            serde_json::json!({
                "type": "node",
                "builder": "facility/door_request",
                "config": {
                    "door_id": door_id,
                    "mode": mode,
                    "hold_gate_until_confirmed": hold_gate
                },
                "next": { "builtin": "terminate" }
            }),
        );
        step_names.push("pre_door_ok".to_string());
        ops.insert(
            "pre_door_ok".to_string(),
            serde_json::json!({
                "type": "fork_result",
                "ok": { "builtin": "terminate" },
                "err": { "builtin": "dispose" }
            }),
        );
    }

    if category.contains("undock") {
        let dock_type = obj
            .get("dock_type")
            .and_then(|v| v.as_str())
            .unwrap_or("simple_charging_dock");
        let hold_gate = obj
            .get("gate")
            .or_else(|| obj.get("hold_gate"))
            .and_then(|v| v.as_bool())
            .unwrap_or(false);
        step_names.push("undock".to_string());
        ops.insert(
            "undock".to_string(),
            serde_json::json!({
                "type": "node",
                "builder": "nav2/undock_robot",
                "config": {
                    "dock_type": dock_type,
                    "hold_gate": hold_gate
                },
                "next": { "builtin": "terminate" }
            }),
        );
        step_names.push("undock_ok".to_string());
        ops.insert(
            "undock_ok".to_string(),
            serde_json::json!({
                "type": "fork_result",
                "ok": { "builtin": "terminate" },
                "err": { "builtin": "dispose" }
            }),
        );
    } else if category.contains("dock") {
        let dock_id = obj
            .get("dock_id")
            .or_else(|| obj.get("target"))
            .and_then(|v| v.as_str())
            .unwrap_or("");
        let dock_type = obj
            .get("dock_type")
            .and_then(|v| v.as_str())
            .unwrap_or("simple_charging_dock");
        let navigate_to_staging_pose = obj
            .get("navigate_to_staging_pose")
            .and_then(|v| v.as_bool())
            .unwrap_or(false);
        let hold_gate_after_dock = obj
            .get("gate")
            .or_else(|| obj.get("hold_gate"))
            .or_else(|| obj.get("wait_for_trigger"))
            .and_then(|v| v.as_bool())
            .unwrap_or(false);
        step_names.push("dock".to_string());
        ops.insert(
            "dock".to_string(),
            serde_json::json!({
                "type": "node",
                "builder": "nav2/dock_robot",
                "config": {
                    "use_dock_id": !dock_id.is_empty(),
                    "dock_id": dock_id,
                    "dock_type": dock_type,
                    "navigate_to_staging_pose": navigate_to_staging_pose,
                    "hold_gate_after_dock": hold_gate_after_dock
                },
                "next": { "builtin": "terminate" }
            }),
        );
        step_names.push("dock_ok".to_string());
        ops.insert(
            "dock_ok".to_string(),
            serde_json::json!({
                "type": "fork_result",
                "ok": { "builtin": "terminate" },
                "err": { "builtin": "dispose" }
            }),
        );
    } else if category.contains("door") {
        let door_id = obj
            .get("door_id")
            .and_then(|v| v.as_str())
            .unwrap_or("default_door");
        let mode = obj.get("mode").and_then(|v| v.as_str()).unwrap_or("open");
        let hold_gate = obj
            .get("gate")
            .or_else(|| obj.get("hold_gate_until_confirmed"))
            .and_then(|v| v.as_bool())
            .unwrap_or(true);
        step_names.push("door".to_string());
        ops.insert(
            "door".to_string(),
            serde_json::json!({
                "type": "node",
                "builder": "facility/door_request",
                "config": {
                    "door_id": door_id,
                    "mode": mode,
                    "hold_gate_until_confirmed": hold_gate
                },
                "next": { "builtin": "terminate" }
            }),
        );
        step_names.push("door_ok".to_string());
        ops.insert(
            "door_ok".to_string(),
            serde_json::json!({
                "type": "fork_result",
                "ok": { "builtin": "terminate" },
                "err": { "builtin": "dispose" }
            }),
        );
    } else if category.contains("handover")
        || category.contains("load")
        || category.contains("unload")
    {
        let station_id = obj
            .get("station_id")
            .and_then(|v| v.as_str())
            .unwrap_or("default_station");
        let direction = obj
            .get("direction")
            .and_then(|v| v.as_str())
            .unwrap_or("load");
        let item_type = obj.get("item_type").and_then(|v| v.as_str()).unwrap_or("");
        let wait_for_gate_release = obj
            .get("wait_for_gate_release")
            .or_else(|| obj.get("gate"))
            .and_then(|v| v.as_bool())
            .unwrap_or(true);
        step_names.push("handover".to_string());
        ops.insert(
            "handover".to_string(),
            serde_json::json!({
                "type": "node",
                "builder": "workcell/handover",
                "config": {
                    "station_id": station_id,
                    "direction": direction,
                    "item_type": item_type,
                    "wait_for_gate_release": wait_for_gate_release
                },
                "next": { "builtin": "terminate" }
            }),
        );
        step_names.push("handover_ok".to_string());
        ops.insert(
            "handover_ok".to_string(),
            serde_json::json!({
                "type": "fork_result",
                "ok": { "builtin": "terminate" },
                "err": { "builtin": "dispose" }
            }),
        );
    } else {
        let hold_gate = obj
            .get("gate")
            .or_else(|| obj.get("hold_gate"))
            .or_else(|| obj.get("wait_for_trigger"))
            .and_then(|v| v.as_bool())
            .unwrap_or(false);
        step_names.push("navigate".to_string());
        ops.insert(
            "navigate".to_string(),
            serde_json::json!({
                "type": "node",
                "builder": "nav2/navigate_to_pose",
                "config": {
                    "hold_gate": hold_gate
                },
                "next": { "builtin": "terminate" }
            }),
        );
        step_names.push("navigate_ok".to_string());
        ops.insert(
            "navigate_ok".to_string(),
            serde_json::json!({
                "type": "fork_result",
                "ok": { "builtin": "terminate" },
                "err": { "builtin": "dispose" }
            }),
        );
    }

    if let Some(handover) = obj.get("handover").and_then(|v| v.as_object()) {
        let station_id = handover
            .get("station_id")
            .and_then(|v| v.as_str())
            .unwrap_or("default_station");
        let direction = handover
            .get("direction")
            .and_then(|v| v.as_str())
            .unwrap_or("load");
        let item_type = handover
            .get("item_type")
            .and_then(|v| v.as_str())
            .unwrap_or("");
        let wait_for_gate_release = handover
            .get("wait_for_gate_release")
            .and_then(|v| v.as_bool())
            .unwrap_or(true);
        step_names.push("post_handover".to_string());
        ops.insert(
            "post_handover".to_string(),
            serde_json::json!({
                "type": "node",
                "builder": "workcell/handover",
                "config": {
                    "station_id": station_id,
                    "direction": direction,
                    "item_type": item_type,
                    "wait_for_gate_release": wait_for_gate_release
                },
                "next": { "builtin": "terminate" }
            }),
        );
        step_names.push("post_handover_ok".to_string());
        ops.insert(
            "post_handover_ok".to_string(),
            serde_json::json!({
                "type": "fork_result",
                "ok": { "builtin": "terminate" },
                "err": { "builtin": "dispose" }
            }),
        );
    }

    if let Some(post_door) = obj.get("post_door").and_then(|v| v.as_object()) {
        let door_id = post_door
            .get("door_id")
            .and_then(|v| v.as_str())
            .unwrap_or("default_door");
        let mode = post_door
            .get("mode")
            .and_then(|v| v.as_str())
            .unwrap_or("close");
        step_names.push("post_door".to_string());
        ops.insert(
            "post_door".to_string(),
            serde_json::json!({
                "type": "node",
                "builder": "facility/door_request",
                "config": {
                    "door_id": door_id,
                    "mode": mode,
                    "hold_gate_until_confirmed": false
                },
                "next": { "builtin": "terminate" }
            }),
        );
        step_names.push("post_door_ok".to_string());
        ops.insert(
            "post_door_ok".to_string(),
            serde_json::json!({
                "type": "fork_result",
                "ok": { "builtin": "terminate" },
                "err": { "builtin": "dispose" }
            }),
        );
    }

    // Wire `next` / `ok` pointers in sequence across all compiled steps.
    for idx in 0..step_names.len() {
        let current = &step_names[idx];
        let next_target = if idx + 1 < step_names.len() {
            serde_json::Value::String(step_names[idx + 1].clone())
        } else {
            serde_json::json!({ "builtin": "terminate" })
        };
        if let Some(op_obj) = ops.get_mut(current).and_then(|v| v.as_object_mut()) {
            let op_type = op_obj.get("type").and_then(|v| v.as_str()).unwrap_or("");
            if op_type == "fork_result" {
                op_obj.insert("ok".to_string(), next_target);
            } else {
                op_obj.insert("next".to_string(), next_target);
            }
        }
    }

    let start_op = step_names
        .first()
        .cloned()
        .unwrap_or_else(|| "navigate".to_string());
    let diagram_json = serde_json::json!({
        "version": "0.1.0",
        "start": start_op,
        "ops": ops,
    });

    Diagram::from_json(diagram_json).map_err(|e| e.to_string())
}

/// Spawn and run a Crossflow [`Diagram`] on a [`WaypointActionContext`], returning
/// the resulting [`WaypointActionContext`].
pub fn evaluate_waypoint_diagram(
    diagram: &Diagram,
    initial_ctx: WaypointActionContext,
) -> Result<WaypointActionContext, String> {
    let mut app = App::new();
    app.add_plugins((TaskPoolPlugin::default(), CrossflowPlugin::default()));
    let registry = build_waypoint_diagram_registry();

    let workflow = app
        .world_mut()
        .command(|cmds| {
            diagram
                .spawn_io_workflow::<WaypointActionContext, WaypointActionContext>(cmds, &registry)
        })
        .map_err(|e| e.to_string())?;

    let mut outcome = app
        .world_mut()
        .command(|cmds| cmds.request(initial_ctx, workflow).outcome());

    for _ in 0..64 {
        app.update();
        if let Some(result) = outcome.try_recv() {
            return result.map_err(|e| format!("Crossflow diagram cancelled/disposed: {e:?}"));
        }
    }

    Err("Crossflow waypoint diagram did not complete within 64 ticks".to_string())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn native_crossflow_diagram_docks_and_triggers_handover() {
        let diagram_json = r#"
        {
            "version": "0.1.0",
            "start": "open_bay_door",
            "ops": {
                "open_bay_door": {
                    "type": "node",
                    "builder": "facility/door_request",
                    "config": {
                        "door_id": "bay_door_north_1",
                        "mode": "open",
                        "hold_gate_until_confirmed": false
                    },
                    "next": "check_door"
                },
                "check_door": {
                    "type": "fork_result",
                    "ok": "dock_into_station",
                    "err": { "builtin": "dispose" }
                },
                "dock_into_station": {
                    "type": "node",
                    "builder": "nav2/dock_robot",
                    "config": {
                        "use_dock_id": true,
                        "dock_id": "dock_north_1",
                        "dock_type": "simple_charging_dock",
                        "navigate_to_staging_pose": false
                    },
                    "next": "check_dock"
                },
                "check_dock": {
                    "type": "fork_result",
                    "ok": "conveyor_handover",
                    "err": { "builtin": "dispose" }
                },
                "conveyor_handover": {
                    "type": "node",
                    "builder": "workcell/handover",
                    "config": {
                        "station_id": "conveyor_north_1",
                        "direction": "load",
                        "item_type": "tote_A",
                        "wait_for_gate_release": true
                    },
                    "next": "check_handover"
                },
                "check_handover": {
                    "type": "fork_result",
                    "ok": { "builtin": "terminate" },
                    "err": { "builtin": "dispose" }
                }
            }
        }
        "#;

        let diagram = parse_or_compile_waypoint_diagram(diagram_json).unwrap();
        let initial =
            WaypointActionContext::new("robot0", 1, -1.0, 2.0, std::f64::consts::FRAC_PI_2);
        let result = evaluate_waypoint_diagram(&diagram, initial).unwrap();

        assert_eq!(
            result.navigation_action,
            WaypointNavActionSpec::DockRobot {
                use_dock_id: true,
                dock_id: "dock_north_1".to_string(),
                dock_type: "simple_charging_dock".to_string(),
                navigate_to_staging_pose: false,
            }
        );
        assert!(result.hold_progress_gate);
        assert_eq!(
            result.door_events,
            vec![DoorEventRecord {
                door_id: "bay_door_north_1".to_string(),
                mode: DoorActionMode::Open,
            }]
        );
        assert_eq!(
            result.handover_events,
            vec![HandoverEventRecord {
                station_id: "conveyor_north_1".to_string(),
                direction: "load".to_string(),
                item_type: "tote_A".to_string(),
            }]
        );
        assert_eq!(
            result.executed_steps,
            vec![
                "facility/door_request:bay_door_north_1:open",
                "nav2/dock_robot:dock_north_1",
                "workcell/handover:conveyor_north_1:load",
            ]
        );
    }

    #[test]
    fn shorthand_json_compiles_to_crossflow_diagram_with_pre_door_and_handover() {
        let shorthand = r#"
        {
            "category": "dock",
            "dock_id": "dock_south_2",
            "dock_type": "custom_laser_dock",
            "navigate_to_staging_pose": true,
            "pre_door": {
                "door_id": "south_alcove_door",
                "mode": "open"
            },
            "handover": {
                "station_id": "conveyor_south_2",
                "direction": "unload",
                "item_type": "pallet",
                "wait_for_gate_release": true
            },
            "post_door": {
                "door_id": "south_alcove_door",
                "mode": "close"
            }
        }
        "#;

        let diagram = parse_or_compile_waypoint_diagram(shorthand).unwrap();
        let initial =
            WaypointActionContext::new("robot1", 2, 1.0, -2.0, -std::f64::consts::FRAC_PI_2);
        let result = evaluate_waypoint_diagram(&diagram, initial).unwrap();

        let (inner_action, gate) = result.clone().into_inner_action_and_gate();
        assert_eq!(
            inner_action,
            InnerNavigationAction::DockRobot {
                use_dock_id: true,
                dock_id: "dock_south_2".to_string(),
                dock_type: "custom_laser_dock".to_string(),
                navigate_to_staging_pose: true,
            }
        );
        assert!(gate);
        assert_eq!(result.door_events.len(), 2);
        assert_eq!(result.handover_events.len(), 1);
        assert_eq!(
            result.executed_steps,
            vec![
                "facility/door_request:south_alcove_door:open",
                "nav2/dock_robot:dock_south_2",
                "workcell/handover:conveyor_south_2:unload",
                "facility/door_request:south_alcove_door:close",
            ]
        );
    }
}
