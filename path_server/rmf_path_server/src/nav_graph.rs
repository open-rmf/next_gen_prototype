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
                },
            );
        }

        // 3. Attach location names & tags
        for location in site.navigation.guided.locations.values() {
            let anchor_id = location.anchor.0;
            let name = location.name.0.clone();
            if !name.is_empty() {
                vertices_by_name.insert(name.clone(), anchor_id);
                if let Some(v) = vertices_by_id.get_mut(&anchor_id) {
                    v.name = Some(name);
                }
            }
        }

        // 4. Extract dock / arrival actions from guided lanes
        for lane in site.navigation.guided.lanes.values() {
            let [from_anchor, to_anchor] = lane.anchors.array();

            // Forward motion docking action arrives at `to_anchor`
            if let Some(dock) = &lane.forward.dock {
                if let Some(v) = vertices_by_id.get_mut(&to_anchor) {
                    v.arrival_action = Some(VertexAction {
                        action_type: "dock".to_string(),
                        name: dock.name.clone(),
                        duration: dock.duration,
                    });
                }
            }

            // Reverse motion docking action arrives at `from_anchor`
            match &lane.reverse {
                rmf_site_format::ReverseLane::Different(motion) => {
                    if let Some(dock) = &motion.dock {
                        if let Some(v) = vertices_by_id.get_mut(&from_anchor) {
                            v.arrival_action = Some(VertexAction {
                                action_type: "dock".to_string(),
                                name: dock.name.clone(),
                                duration: dock.duration,
                            });
                        }
                    }
                }
                rmf_site_format::ReverseLane::Same => {
                    if let Some(dock) = &lane.forward.dock {
                        if let Some(v) = vertices_by_id.get_mut(&from_anchor) {
                            if v.arrival_action.is_none() {
                                v.arrival_action = Some(VertexAction {
                                    action_type: "dock".to_string(),
                                    name: dock.name.clone(),
                                    duration: dock.duration,
                                });
                            }
                        }
                    }
                }
                rmf_site_format::ReverseLane::Disable => {}
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
            if let Some(v) = self.vertices_by_id.get(&(id as u32)) {
                return Some(v);
            }
        }
        if let Some(name_seq) = key.name.first() {
            let name_str = name_seq.to_string();
            if let Some(&id) = self.vertices_by_name.get(&name_str) {
                return self.vertices_by_id.get(&id);
            }
        }
        None
    }
}
