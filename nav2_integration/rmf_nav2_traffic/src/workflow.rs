/*
 * Copyright (C) 2026 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

use bevy::prelude::*;
use ros_env::rmf_prototype_msgs::msg::SafeZoneId;
use serde::{Deserialize, Serialize};
use std::collections::HashSet;

/// A discrete step in a generic post-arrival or waypoint execution workflow.
#[derive(Serialize, Deserialize, Clone, Debug, PartialEq)]
#[serde(tag = "action", rename_all = "snake_case")]
pub enum WorkflowActionStep {
    /// Command the robot to perform a docking procedure with the specified dock target.
    Dock {
        #[serde(default)]
        dock_id: String,
    },
    /// Command the robot to undock and return to navigation staging.
    Undock,
    /// Pause execution for the specified duration (in seconds).
    Wait {
        #[serde(default)]
        duration_sec: f32,
    },
    /// An arbitrary or user-defined action step.
    Custom {
        name: String,
        #[serde(default)]
        payload: serde_json::Value,
    },
}

impl WorkflowActionStep {
    pub fn is_dock(&self) -> bool {
        matches!(self, Self::Dock { .. })
    }

    pub fn dock_id(&self) -> Option<&str> {
        match self {
            Self::Dock { dock_id } => Some(dock_id.as_str()),
            _ => None,
        }
    }

    pub fn is_undock(&self) -> bool {
        matches!(self, Self::Undock)
    }

    pub fn is_wait(&self) -> bool {
        matches!(self, Self::Wait { .. })
    }

    pub fn duration_sec(&self) -> Option<f32> {
        match self {
            Self::Wait { duration_sec } => Some(*duration_sec),
            _ => None,
        }
    }
}

/// A structured container for a sequence of workflow action steps.
#[derive(Serialize, Deserialize, Clone, Debug, Default, PartialEq)]
pub struct Workflow {
    pub steps: Vec<WorkflowActionStep>,
}

impl Workflow {
    pub fn new(steps: Vec<WorkflowActionStep>) -> Self {
        Self { steps }
    }

    pub fn is_empty(&self) -> bool {
        self.steps.is_empty()
    }

    pub fn len(&self) -> usize {
        self.steps.len()
    }

    pub fn parse(s: &str) -> Self {
        Self {
            steps: parse_workflow(s),
        }
    }
}

/// Event broadcast in Bevy when a specific workflow step begins execution.
#[derive(Clone, Debug, Event)]
pub struct WorkflowStepEvent {
    pub agent: Entity,
    pub safe_zone_id: SafeZoneId,
    pub step: WorkflowActionStep,
    pub step_index: usize,
    pub total_steps: usize,
}

/// Event broadcast in Bevy when an entire workflow sequence completes.
#[derive(Clone, Debug, Event)]
pub struct WorkflowCompletedEvent {
    pub agent: Entity,
    pub safe_zone_id: SafeZoneId,
    pub success: bool,
}

/// Parses a string representation of an arrival action, departure action,
/// or plan workflow into a sequence of `WorkflowActionStep`s.
///
/// Supported formats:
/// 1. Simple strings: e.g. `"dock_conveyor_r1_c1"`, `"undock"`
/// 2. Single JSON actions: e.g. `{"action": "dock", "dock_id": "dock_1"}`
/// 3. Open-RMF category actions: e.g. `{"category": "dock", "description": {"dock_name": "dock_1"}}`
/// 4. JSON array of steps: `[{"action": "dock", ...}, {"action": "wait", ...}, {"action": "undock"}]`
/// 5. Wrapped sequence objects: `{"steps": [...]}` or `{"category": "sequence", "description": {"sequence": [...]}}`
/// 6. Native Crossflow diagram JSON: `{ "version": "0.1.0", "start": "...", "ops": { ... } }`
pub fn parse_workflow(input: &str) -> Vec<WorkflowActionStep> {
    let trimmed = input.trim();
    if trimmed.is_empty() {
        return Vec::new();
    }

    if let Ok(value) = serde_json::from_str::<serde_json::Value>(trimmed) {
        return parse_workflow_from_value(&value);
    }

    parse_single_string_step(trimmed)
}

/// Parses a `serde_json::Value` into a sequence of `WorkflowActionStep`s.
pub fn parse_workflow_from_value(value: &serde_json::Value) -> Vec<WorkflowActionStep> {
    match value {
        serde_json::Value::Array(arr) => arr.iter().filter_map(parse_step_from_value).collect(),
        serde_json::Value::Object(map) => {
            // Check if this is a Crossflow diagram
            if map.contains_key("ops") && map.contains_key("start") {
                let diagram_steps = parse_diagram_from_value(value);
                if !diagram_steps.is_empty() {
                    return diagram_steps;
                }
            }

            // Check for explicit sequence / steps array wrappers
            for key in ["steps", "sequence", "actions"] {
                if let Some(steps_val) = map.get(key) {
                    if let Some(steps_arr) = steps_val.as_array() {
                        return steps_arr.iter().filter_map(parse_step_from_value).collect();
                    }
                }
            }

            // Check for Open-RMF category = "sequence"
            if let Some(cat) = map.get("category").and_then(|v| v.as_str()) {
                if cat.eq_ignore_ascii_case("sequence") {
                    if let Some(desc) = map.get("description") {
                        if let Some(desc_arr) = desc.as_array() {
                            return desc_arr.iter().filter_map(parse_step_from_value).collect();
                        }
                        if let Some(desc_obj) = desc.as_object() {
                            for key in ["sequence", "steps", "actions"] {
                                if let Some(arr) = desc_obj.get(key).and_then(|v| v.as_array()) {
                                    return arr.iter().filter_map(parse_step_from_value).collect();
                                }
                            }
                        }
                    }
                }
            }

            // Check for nested "workflow" field
            if let Some(wf) = map.get("workflow") {
                if let Some(s) = wf.as_str() {
                    return parse_workflow(s);
                }
                return parse_workflow_from_value(wf);
            }

            // Single action object
            parse_step_from_value(value).into_iter().collect()
        }
        serde_json::Value::String(s) => parse_workflow(s),
        _ => Vec::new(),
    }
}

/// Parses a single step from a JSON Value.
pub fn parse_step_from_value(value: &serde_json::Value) -> Option<WorkflowActionStep> {
    if let Some(s) = value.as_str() {
        return parse_single_string_step(s).into_iter().next();
    }

    // Try direct serde deserialization matching `WorkflowActionStep`
    if let Ok(step) = serde_json::from_value::<WorkflowActionStep>(value.clone()) {
        return Some(step);
    }

    let obj = value.as_object()?;

    // Extract kind from "action", "category", "type", or "name"
    let kind = obj
        .get("action")
        .or_else(|| obj.get("category"))
        .or_else(|| obj.get("type"))
        .or_else(|| obj.get("name"))
        .and_then(|v| v.as_str())
        .map(|s| s.to_ascii_lowercase())?;

    if kind == "undock" || kind.contains("undock") {
        return Some(WorkflowActionStep::Undock);
    }

    if kind == "dock" || (kind.contains("dock") && !kind.contains("undock")) {
        let dock_id = obj
            .get("dock_id")
            .or_else(|| obj.get("dock_name"))
            .and_then(|v| v.as_str())
            .map(|s| s.to_string())
            .or_else(|| {
                obj.get("description").and_then(|desc| {
                    if let Some(s) = desc.as_str() {
                        Some(s.to_string())
                    } else if let Some(desc_obj) = desc.as_object() {
                        desc_obj
                            .get("dock_id")
                            .or_else(|| desc_obj.get("dock_name"))
                            .and_then(|v| v.as_str())
                            .map(|s| s.to_string())
                    } else {
                        None
                    }
                })
            })
            .unwrap_or_else(|| {
                if kind != "dock" {
                    kind.clone()
                } else {
                    String::new()
                }
            });

        return Some(WorkflowActionStep::Dock { dock_id });
    }

    if kind == "wait" || kind == "sleep" || kind == "delay" {
        let duration = obj
            .get("duration_sec")
            .or_else(|| obj.get("duration"))
            .or_else(|| obj.get("seconds"))
            .and_then(|v| v.as_f64())
            .or_else(|| {
                obj.get("description").and_then(|desc| {
                    if let Some(num) = desc.as_f64() {
                        Some(num)
                    } else if let Some(desc_obj) = desc.as_object() {
                        desc_obj
                            .get("duration")
                            .or_else(|| desc_obj.get("duration_sec"))
                            .or_else(|| desc_obj.get("seconds"))
                            .and_then(|v| v.as_f64())
                    } else {
                        None
                    }
                })
            })
            .unwrap_or(0.0) as f32;

        return Some(WorkflowActionStep::Wait {
            duration_sec: duration,
        });
    }

    // Default to Custom action
    let name = obj
        .get("name")
        .and_then(|v| v.as_str())
        .unwrap_or(&kind)
        .to_string();
    let payload = obj
        .get("payload")
        .or_else(|| obj.get("description"))
        .cloned()
        .unwrap_or_else(|| value.clone());

    Some(WorkflowActionStep::Custom { name, payload })
}

/// Fallback for non-JSON strings.
fn parse_single_string_step(s: &str) -> Vec<WorkflowActionStep> {
    let lower = s.to_ascii_lowercase();
    if lower.contains("undock") {
        vec![WorkflowActionStep::Undock]
    } else if lower.contains("dock") {
        vec![WorkflowActionStep::Dock {
            dock_id: s.to_string(),
        }]
    } else {
        vec![WorkflowActionStep::Custom {
            name: s.to_string(),
            payload: serde_json::Value::Null,
        }]
    }
}

/// Parses sequential steps from a Crossflow diagram JSON structure.
fn parse_diagram_from_value(value: &serde_json::Value) -> Vec<WorkflowActionStep> {
    let mut steps = Vec::new();
    let Some(ops) = value.get("ops").and_then(|v| v.as_object()) else {
        return steps;
    };
    let Some(start_id) = value.get("start").and_then(|v| v.as_str()) else {
        return steps;
    };

    let mut current_id = start_id.to_string();
    let mut visited = HashSet::new();

    while !current_id.is_empty() && visited.insert(current_id.clone()) {
        let Some(op) = ops.get(&current_id) else {
            break;
        };

        let builder = op
            .get("builder")
            .and_then(|v| v.as_str())
            .unwrap_or("")
            .to_ascii_lowercase();

        let config = op.get("config").cloned().unwrap_or(serde_json::Value::Null);

        match builder.as_str() {
            "dock" => {
                let dock_id = config
                    .get("dock_id")
                    .or_else(|| config.get("dock_name"))
                    .and_then(|v| v.as_str())
                    .unwrap_or("")
                    .to_string();
                steps.push(WorkflowActionStep::Dock { dock_id });
            }
            "undock" => {
                steps.push(WorkflowActionStep::Undock);
            }
            "wait" | "sleep" | "delay" => {
                let duration = config
                    .get("duration_sec")
                    .or_else(|| config.get("duration"))
                    .or_else(|| config.get("seconds"))
                    .and_then(|v| v.as_f64())
                    .unwrap_or(0.0) as f32;
                steps.push(WorkflowActionStep::Wait {
                    duration_sec: duration,
                });
            }
            other if !other.is_empty() => {
                steps.push(WorkflowActionStep::Custom {
                    name: other.to_string(),
                    payload: config,
                });
            }
            _ => {}
        }

        // Advance to next node
        if let Some(next_val) = op.get("next") {
            if let Some(next_str) = next_val.as_str() {
                current_id = next_str.to_string();
            } else if let Some(next_obj) = next_val.as_object() {
                if let Some(builtin) = next_obj.get("builtin").and_then(|v| v.as_str()) {
                    if builtin == "terminate" || builtin == "dispose" {
                        break;
                    }
                }
                break;
            } else if let Some(next_arr) = next_val.as_array() {
                if let Some(first) = next_arr.first().and_then(|v| v.as_str()) {
                    current_id = first.to_string();
                } else {
                    break;
                }
            } else {
                break;
            }
        } else {
            break;
        }
    }

    steps
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_parse_empty_and_whitespace() {
        assert!(parse_workflow("").is_empty());
        assert!(parse_workflow("   ").is_empty());
        assert!(parse_workflow("\n\t").is_empty());
    }

    #[test]
    fn test_parse_simple_strings() {
        let dock = parse_workflow("dock_conveyor_r1_c1");
        assert_eq!(
            dock,
            vec![WorkflowActionStep::Dock {
                dock_id: "dock_conveyor_r1_c1".to_string()
            }]
        );

        let undock = parse_workflow("undock");
        assert_eq!(undock, vec![WorkflowActionStep::Undock]);

        let custom = parse_workflow("some_random_task");
        assert_eq!(
            custom,
            vec![WorkflowActionStep::Custom {
                name: "some_random_task".to_string(),
                payload: serde_json::Value::Null,
            }]
        );
    }

    #[test]
    fn test_parse_single_json_action() {
        let dock_json = r#"{"action": "dock", "dock_id": "station_alpha"}"#;
        assert_eq!(
            parse_workflow(dock_json),
            vec![WorkflowActionStep::Dock {
                dock_id: "station_alpha".to_string()
            }]
        );

        let undock_json = r#"{"action": "undock"}"#;
        assert_eq!(
            parse_workflow(undock_json),
            vec![WorkflowActionStep::Undock]
        );

        let wait_json = r#"{"action": "wait", "duration_sec": 3.5}"#;
        assert_eq!(
            parse_workflow(wait_json),
            vec![WorkflowActionStep::Wait { duration_sec: 3.5 }]
        );

        let custom_json = r#"{"action": "custom", "name": "scanner", "payload": {"mode": "high"}}"#;
        assert_eq!(
            parse_workflow(custom_json),
            vec![WorkflowActionStep::Custom {
                name: "scanner".to_string(),
                payload: serde_json::json!({"mode": "high"}),
            }]
        );
    }

    #[test]
    fn test_parse_rmf_action_schema() {
        let rmf_dock =
            r#"{"category": "dock", "description": {"dock_name": "dock_conveyor_r1_c1"}}"#;
        assert_eq!(
            parse_workflow(rmf_dock),
            vec![WorkflowActionStep::Dock {
                dock_id: "dock_conveyor_r1_c1".to_string()
            }]
        );

        let rmf_undock = r#"{"category": "undock"}"#;
        assert_eq!(parse_workflow(rmf_undock), vec![WorkflowActionStep::Undock]);

        let rmf_wait = r#"{"category": "wait", "description": {"duration": 2.5}}"#;
        assert_eq!(
            parse_workflow(rmf_wait),
            vec![WorkflowActionStep::Wait { duration_sec: 2.5 }]
        );
    }

    #[test]
    fn test_parse_json_array_sequence() {
        let array_json = r#"[
            {"action": "dock", "dock_id": "dock_station_1"},
            {"action": "wait", "duration_sec": 4.0},
            {"action": "undock"}
        ]"#;

        let steps = parse_workflow(array_json);
        assert_eq!(steps.len(), 3);
        assert_eq!(
            steps[0],
            WorkflowActionStep::Dock {
                dock_id: "dock_station_1".to_string()
            }
        );
        assert_eq!(steps[1], WorkflowActionStep::Wait { duration_sec: 4.0 });
        assert_eq!(steps[2], WorkflowActionStep::Undock);
    }

    #[test]
    fn test_parse_rmf_sequence_wrapper() {
        let seq_json = r#"{
            "category": "sequence",
            "description": {
                "sequence": [
                    {"category": "dock", "description": {"dock_name": "dock_station_1"}},
                    {"category": "wait", "description": {"duration": 1.5}},
                    {"category": "undock"}
                ]
            }
        }"#;

        let steps = parse_workflow(seq_json);
        assert_eq!(steps.len(), 3);
        assert_eq!(
            steps[0],
            WorkflowActionStep::Dock {
                dock_id: "dock_station_1".to_string()
            }
        );
        assert_eq!(steps[1], WorkflowActionStep::Wait { duration_sec: 1.5 });
        assert_eq!(steps[2], WorkflowActionStep::Undock);
    }

    #[test]
    fn test_parse_crossflow_diagram() {
        let diagram_json = r#"{
            "version": "0.1.0",
            "start": "step_dock",
            "ops": {
                "step_dock": {
                    "type": "node",
                    "builder": "dock",
                    "config": { "dock_id": "dock_bay_2" },
                    "next": "step_wait"
                },
                "step_wait": {
                    "type": "node",
                    "builder": "wait",
                    "config": { "duration_sec": 5.0 },
                    "next": "step_undock"
                },
                "step_undock": {
                    "type": "node",
                    "builder": "undock",
                    "next": { "builtin": "terminate" }
                }
            }
        }"#;

        let steps = parse_workflow(diagram_json);
        assert_eq!(steps.len(), 3);
        assert_eq!(
            steps[0],
            WorkflowActionStep::Dock {
                dock_id: "dock_bay_2".to_string()
            }
        );
        assert_eq!(steps[1], WorkflowActionStep::Wait { duration_sec: 5.0 });
        assert_eq!(steps[2], WorkflowActionStep::Undock);
    }

    #[test]
    fn test_workflow_struct_helpers() {
        let wf = Workflow::parse(r#"[{"action": "dock", "dock_id": "d1"}, {"action": "undock"}]"#);
        assert_eq!(wf.len(), 2);
        assert!(!wf.is_empty());
        assert!(wf.steps[0].is_dock());
        assert_eq!(wf.steps[0].dock_id(), Some("d1"));
        assert!(wf.steps[1].is_undock());
    }
}
