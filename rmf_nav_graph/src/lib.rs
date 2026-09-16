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

use rmf_site_format::{Category, Site};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;

/// A special action (such as docking) to be executed at a navigation vertex.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub struct VertexAction {
    pub action_type: String,
    pub name: String,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub duration: Option<f32>,
}

/// Information about a discrete navigation vertex in the site nav graph.
#[derive(Clone, Debug, PartialEq)]
pub struct NavVertex {
    pub id: u32,
    pub name: Option<String>,
    pub position: [f32; 2],
    pub arrival_action: Option<VertexAction>,
    pub orientation: Option<f32>,
    pub undock_vertex_id: Option<u32>,
    pub undock_position: Option<[f32; 2]>,
}

/// In-memory representation of the navigation graph parsed from a .site.json file.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct NavGraphData {
    pub vertices_by_id: HashMap<u32, NavVertex>,
    pub vertices_by_name: HashMap<String, u32>,
}

impl NavGraphData {
    /// Load and parse a navigation graph from a .site.json file path.
    pub fn from_site_file(path: &str) -> Result<Self, Box<dyn std::error::Error>> {
        let data = std::fs::read(path)?;
        let site = Site::from_bytes_json(&data)?;
        Ok(Self::from_site(&site))
    }

    /// Construct a NavGraphData instance from an in-memory rmf_site_format::Site.
    pub fn from_site(site: &Site) -> Self {
        let mut vertices_by_id = HashMap::new();
        let mut vertices_by_name = HashMap::new();

        // 1. Extract all anchor positions across all levels
        let mut anchor_positions: HashMap<u32, [f32; 2]> = HashMap::new();
        for level in site.levels.values() {
            for (&anchor_id, anchor) in &level.anchors {
                let [x, y] = anchor.translation_for_category(Category::General);
                anchor_positions.insert(anchor_id, [x, y]);
            }
        }

        // Global site anchors if any
        for (&anchor_id, anchor) in &site.anchors {
            let [x, y] = anchor.translation_for_category(Category::General);
            anchor_positions.entry(anchor_id).or_insert([x, y]);
        }

        // 2. Initialize vertices from anchor coordinates
        for (&id, &position) in &anchor_positions {
            vertices_by_id.insert(
                id,
                NavVertex {
                    id,
                    name: None,
                    position,
                    arrival_action: None,
                    orientation: None,
                    undock_vertex_id: None,
                    undock_position: None,
                },
            );
        }

        // 3. Attach location names & tags directly from site format
        for location in site.navigation.guided.locations.values() {
            let anchor_id = location.anchor.0;
            let name = location.name.0.clone();
            if !name.is_empty() {
                vertices_by_name.insert(name.clone(), anchor_id);
                vertices_by_name.insert(name.to_lowercase(), anchor_id);
                if let Some(v) = vertices_by_id.get_mut(&anchor_id) {
                    v.name = Some(name);
                }
            }
        }

        // 4. Extract dock / arrival actions directly from rmf_site_format guided lanes
        let compute_heading = |from_id: u32,
                               to_id: u32,
                               constraint: &rmf_site_format::lane::OrientationConstraint|
         -> Option<f32> {
            match constraint {
                rmf_site_format::lane::OrientationConstraint::AbsoluteYaw(yaw) => {
                    Some(yaw.radians())
                }
                rmf_site_format::lane::OrientationConstraint::Backwards => {
                    let from_pos = anchor_positions.get(&from_id)?;
                    let to_pos = anchor_positions.get(&to_id)?;
                    let dx = to_pos[0] - from_pos[0];
                    let dy = to_pos[1] - from_pos[1];
                    Some(
                        (dy.atan2(dx) + std::f32::consts::PI)
                            .rem_euclid(2.0 * std::f32::consts::PI)
                            - std::f32::consts::PI,
                    )
                }
                _ => {
                    let from_pos = anchor_positions.get(&from_id)?;
                    let to_pos = anchor_positions.get(&to_id)?;
                    let dx = to_pos[0] - from_pos[0];
                    let dy = to_pos[1] - from_pos[1];
                    if dx.hypot(dy) > 1e-4 {
                        Some(dy.atan2(dx))
                    } else {
                        None
                    }
                }
            }
        };

        for lane in site.navigation.guided.lanes.values() {
            let [from_anchor, to_anchor] = lane.anchors.array();

            // In rmf_site_format, lane.forward.dock specifies a docking lane ending at to_anchor
            if let Some(dock) = &lane.forward.dock {
                let heading =
                    compute_heading(from_anchor, to_anchor, &lane.forward.orientation_constraint);
                if let Some(v) = vertices_by_id.get_mut(&to_anchor) {
                    v.arrival_action = Some(VertexAction {
                        action_type: "dock".to_string(),
                        name: dock.name.clone(),
                        duration: dock.duration,
                    });
                    if heading.is_some() {
                        v.orientation = heading;
                    }
                    v.undock_vertex_id = Some(from_anchor);
                    v.undock_position = anchor_positions.get(&from_anchor).copied();
                }
                vertices_by_name.insert(dock.name.clone(), to_anchor);
                vertices_by_name.insert(dock.name.to_lowercase(), to_anchor);
            }

            // In reverse motion, if a different motion specifies a dock arriving at from_anchor
            if let rmf_site_format::ReverseLane::Different(motion) = &lane.reverse {
                if let Some(dock) = &motion.dock {
                    let heading =
                        compute_heading(to_anchor, from_anchor, &motion.orientation_constraint);
                    if let Some(v) = vertices_by_id.get_mut(&from_anchor) {
                        v.arrival_action = Some(VertexAction {
                            action_type: "dock".to_string(),
                            name: dock.name.clone(),
                            duration: dock.duration,
                        });
                        if heading.is_some() {
                            v.orientation = heading;
                        }
                        v.undock_vertex_id = Some(to_anchor);
                        v.undock_position = anchor_positions.get(&to_anchor).copied();
                    }
                    vertices_by_name.insert(dock.name.clone(), from_anchor);
                    vertices_by_name.insert(dock.name.to_lowercase(), from_anchor);
                }
            }
        }

        Self {
            vertices_by_id,
            vertices_by_name,
        }
    }

    /// Look up a vertex in the nav graph by GraphElementKey (checking vertex id or name).
    pub fn find_vertex(
        &self,
        key: &ros_env::rmf_prototype_msgs::msg::GraphElementKey,
    ) -> Option<&NavVertex> {
        if let Some(&id) = key.key.first() {
            if id > 0 {
                if let Some(v) = self.vertices_by_id.get(&(id as u32)) {
                    return Some(v);
                }
            }
        }
        if let Some(name_seq) = key.name.first() {
            let name_str = name_seq.to_string();
            if let Some(v) = self.find_vertex_by_name(&name_str) {
                return Some(v);
            }
        }
        None
    }

    /// Look up a vertex by name (exact or case-insensitive), or matching arrival action name.
    pub fn find_vertex_by_name(&self, name: &str) -> Option<&NavVertex> {
        if name.is_empty() {
            return None;
        }
        // 1. Exact lookup
        if let Some(&id) = self.vertices_by_name.get(name) {
            return self.vertices_by_id.get(&id);
        }
        // 2. Case-insensitive lookup
        let lower = name.to_lowercase();
        if let Some(&id) = self.vertices_by_name.get(&lower) {
            return self.vertices_by_id.get(&id);
        }
        // 3. Match against dock action name directly
        for v in self.vertices_by_id.values() {
            if let Some(action) = &v.arrival_action {
                if action.name.eq_ignore_ascii_case(name) {
                    return Some(v);
                }
            }
        }
        None
    }

    /// Find a docking vertex that corresponds to target coordinates (x, y).
    ///
    /// This resolves coordinates that are either at the pre-dock waypoint itself,
    /// or forward along the approach corridor in the physical dock contact zone
    /// (e.g. at (0.0, 0.95) for conveyor_r1_c1 which pre-docks at (0.0, 1.5)).
    pub fn find_dock_vertex_by_proximity(
        &self,
        x: f32,
        y: f32,
        max_dist: f32,
    ) -> Option<&NavVertex> {
        let mut best_match: Option<(&NavVertex, f32)> = None;

        for v in self.vertices_by_id.values() {
            if v.arrival_action.is_none() {
                continue;
            }

            let dx = x - v.position[0];
            let dy = y - v.position[1];
            let dist = dx.hypot(dy);

            // Direct match at pre-dock waypoint
            if dist <= 0.15 {
                return Some(v);
            }

            if dist > max_dist {
                continue;
            }

            // Check if (x, y) lies in the approach/docking corridor along approach heading
            if let Some(heading) = v.orientation {
                let u_x = heading.cos();
                let u_y = heading.sin();

                // Longitudinal projection along heading vector
                let proj = dx * u_x + dy * u_y;
                // Lateral perpendicular distance
                let perp = (dx * u_y - dy * u_x).abs();

                // Point is forward in the docking lane towards the conveyor with small lateral deviation
                if proj >= -0.1 && proj <= max_dist && perp <= 0.35 {
                    if let Some((_, best_dist)) = best_match {
                        if dist < best_dist {
                            best_match = Some((v, dist));
                        }
                    } else {
                        best_match = Some((v, dist));
                    }
                }
            } else {
                if let Some((_, best_dist)) = best_match {
                    if dist < best_dist {
                        best_match = Some((v, dist));
                    }
                } else {
                    best_match = Some((v, dist));
                }
            }
        }

        best_match.map(|(v, _)| v)
    }

    /// If (x, y) is at or near a dock vertex or inside its docking approach corridor,
    /// return the undocked vertex position where the robot should start its plan.
    pub fn find_undocked_start_position(&self, x: f32, y: f32) -> Option<[f32; 2]> {
        if let Some(dock_vertex) = self.find_dock_vertex_by_proximity(x, y, 0.8) {
            return dock_vertex.undock_position.or(Some(dock_vertex.position));
        }
        None
    }
}
