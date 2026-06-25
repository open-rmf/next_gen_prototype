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

//! Configuration parsing for the reservation destination server.

use rmf_site_format::{legacy::building_map::BuildingMap, Anchor, Category, Site};
use ros_env::rmf_prototype_msgs::msg::Region;
use serde::Deserialize;
use std::fmt;
use std::path::Path;

/// Errors that can occur while loading or validating the configuration.
#[derive(Debug)]
pub enum ConfigError {
    /// The configuration file could not be read from disk.
    Io(std::io::Error),
    /// The configuration file could not be parsed as YAML.
    Parse(serde_yaml::Error),
    /// An RMF site or building map could not be parsed.
    Site(String),
    /// The configuration was parsed but is semantically invalid.
    Validation(String),
}

impl fmt::Display for ConfigError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            ConfigError::Io(e) => write!(f, "failed to read config file: {e}"),
            ConfigError::Parse(e) => write!(f, "failed to parse config file: {e}"),
            ConfigError::Site(e) => write!(f, "failed to parse RMF site: {e}"),
            ConfigError::Validation(msg) => write!(f, "invalid configuration: {msg}"),
        }
    }
}

impl std::error::Error for ConfigError {}

impl From<std::io::Error> for ConfigError {
    fn from(e: std::io::Error) -> Self {
        ConfigError::Io(e)
    }
}

impl From<serde_yaml::Error> for ConfigError {
    fn from(e: serde_yaml::Error) -> Self {
        ConfigError::Parse(e)
    }
}

/// A region as described in the configuration file.
#[derive(Debug, Clone, Deserialize)]
struct RawRegion {
    #[serde(default = "default_hint")]
    hint: String,
    points: Vec<f32>,
}

#[derive(Debug, Clone, Deserialize)]
struct RawSafeSet {
    name: String,
    region: RawRegion,
}

#[derive(Debug, Clone, Deserialize)]
struct RawParkingSpot {
    name: String,
    region: RawRegion,
}

#[derive(Debug, Clone, Deserialize)]
struct RawConfig {
    #[serde(default = "default_grid_size")]
    grid_size: f32,
    #[serde(default)]
    safe_sets: Vec<RawSafeSet>,
    #[serde(default)]
    parking_spots: Vec<RawParkingSpot>,
}

fn default_grid_size() -> f32 {
    0.5
}

fn default_hint() -> String {
    "axis_aligned_rectangle".to_string()
}

/// Translate a human readable hint into the numeric code from `Region.msg`.
fn parse_hint(hint: &str) -> Result<u8, ConfigError> {
    match hint.trim() {
        "unspecified" => Ok(Region::HINT_UNSPECIFIED),
        "point" => Ok(Region::HINT_POINT),
        "axis_aligned_rectangle" => Ok(Region::HINT_AXIS_ALIGNED_RECTANGLE),
        "rectangle" => Ok(Region::HINT_RECTANGLE),
        "convex_polygon" => Ok(Region::HINT_CONVEX_POLYGON),
        "polygon" => Ok(Region::HINT_POLYGON),
        other => Err(ConfigError::Validation(format!(
            "unknown region hint '{other}'"
        ))),
    }
}

/// A validated region from the configuration.
#[derive(Debug, Clone, PartialEq)]
pub struct ConfigRegion {
    pub hint: u8,
    pub points: Vec<f32>,
}

impl ConfigRegion {
    fn from_raw(raw: &RawRegion) -> Result<Self, ConfigError> {
        let hint = parse_hint(&raw.hint)?;
        let region = Self {
            hint,
            points: raw.points.clone(),
        };
        region.validate()?;
        Ok(region)
    }

    /// Validate the points for the given hint.
    fn validate(&self) -> Result<(), ConfigError> {
        if self.points.is_empty() || !self.points.len().is_multiple_of(2) {
            return Err(ConfigError::Validation(format!(
                "region must have a non-zero even number of coordinates, got {}",
                self.points.len()
            )));
        }
        if self.hint == Region::HINT_POINT && self.points.len() != 2 {
            return Err(ConfigError::Validation(format!(
                "a 'point' region must have exactly one (x, y) pair, got {} coordinates",
                self.points.len()
            )));
        }
        Ok(())
    }

    /// Axis-aligned bounding box of the region as `(min_x, min_y, max_x, max_y)`.
    pub fn bounds(&self) -> (f32, f32, f32, f32) {
        let mut min_x = f32::MAX;
        let mut min_y = f32::MAX;
        let mut max_x = f32::MIN;
        let mut max_y = f32::MIN;
        for pair in self.points.chunks_exact(2) {
            min_x = min_x.min(pair[0]);
            max_x = max_x.max(pair[0]);
            min_y = min_y.min(pair[1]);
            max_y = max_y.max(pair[1]);
        }
        (min_x, min_y, max_x, max_y)
    }

    /// Returns true if this region's bounding box lies entirely inside `outer`.
    fn is_within(&self, outer: &ConfigRegion) -> bool {
        let (in_min_x, in_min_y, in_max_x, in_max_y) = self.bounds();
        let (out_min_x, out_min_y, out_max_x, out_max_y) = outer.bounds();
        in_min_x >= out_min_x
            && in_min_y >= out_min_y
            && in_max_x <= out_max_x
            && in_max_y <= out_max_y
    }
}

/// A predefined safe set: a region of the map that robots are allowed to enter.
#[derive(Debug, Clone, PartialEq)]
pub struct SafeSet {
    pub name: String,
    pub region: ConfigRegion,
}

/// A backup parking space a robot can be diverted to while it waits.
#[derive(Debug, Clone, PartialEq)]
pub struct ParkingSpot {
    pub name: String,
    pub region: ConfigRegion,
}

/// The reservation configuration.
#[derive(Debug, Clone, PartialEq)]
pub struct ReservationConfig {
    /// Size of an individual occupancy grid cell, in meters.
    pub grid_size: f32,
    /// Predefined safe sets.
    pub safe_sets: Vec<SafeSet>,
    /// Backup parking spots.
    pub parking_spots: Vec<ParkingSpot>,
}

impl Default for ReservationConfig {
    fn default() -> Self {
        Self {
            grid_size: default_grid_size(),
            safe_sets: Vec::new(),
            parking_spots: Vec::new(),
        }
    }
}

impl ReservationConfig {
    /// Parse and validate a configuration from a YAML string.
    pub fn from_yaml_str(yaml: &str) -> Result<Self, ConfigError> {
        let raw: RawConfig = serde_yaml::from_str(yaml)?;

        if raw.grid_size <= 0.0 {
            return Err(ConfigError::Validation(format!(
                "grid_size must be positive, got {}",
                raw.grid_size
            )));
        }

        let mut safe_sets = Vec::with_capacity(raw.safe_sets.len());
        for raw_set in &raw.safe_sets {
            if raw_set.name.trim().is_empty() {
                return Err(ConfigError::Validation(
                    "every safe set must have a non-empty name".to_string(),
                ));
            }
            safe_sets.push(SafeSet {
                name: raw_set.name.clone(),
                region: ConfigRegion::from_raw(&raw_set.region)?,
            });
        }

        let config = Self {
            grid_size: raw.grid_size,
            safe_sets,
            parking_spots: parse_parking_spots(&raw)?,
        };

        config.validate()?;
        Ok(config)
    }

    /// Load a reservation YAML, native Site Editor project, or legacy building map.
    pub fn from_file<P: AsRef<Path>>(path: P) -> Result<Self, ConfigError> {
        let path = path.as_ref();
        let filename = path
            .file_name()
            .and_then(|name| name.to_str())
            .ok_or_else(|| ConfigError::Site("input path has no UTF-8 filename".to_string()))?;

        if filename.ends_with(".building.yaml") {
            let data = std::fs::read(path)?;
            let building =
                BuildingMap::from_bytes(&data).map_err(|err| ConfigError::Site(err.to_string()))?;
            let site = building
                .to_site()
                .map_err(|err| ConfigError::Site(err.to_string()))?;
            return Self::from_site(&site, default_grid_size());
        }

        if filename.ends_with(".site.json") {
            let data = std::fs::read(path)?;
            let site =
                Site::from_bytes_json(&data).map_err(|err| ConfigError::Site(err.to_string()))?;
            return Self::from_site(&site, default_grid_size());
        }

        let contents = std::fs::read_to_string(path)?;
        Self::from_yaml_str(&contents)
    }

    /// Convert a parsed RMF site into a reservation configuration.
    pub fn from_site(site: &Site, grid_size: f32) -> Result<Self, ConfigError> {
        if grid_size <= 0.0 {
            return Err(ConfigError::Validation(format!(
                "grid_size must be positive, got {grid_size}"
            )));
        }
        if site.levels.is_empty() {
            return Err(ConfigError::Validation(
                "RMF site has no levels".to_string(),
            ));
        }
        if site.levels.len() > 1 {
            return Err(ConfigError::Validation(format!(
                "multi-level RMF sites are not supported yet, got {} levels",
                site.levels.len()
            )));
        }

        let mut safe_sets = Vec::new();
        for level in site.levels.values() {
            let level_name = &level.properties.name.0;
            for (floor_index, floor) in level.floors.values().enumerate() {
                if floor.anchors.0.len() < 3 {
                    continue;
                }

                let mut points = Vec::with_capacity(floor.anchors.0.len() * 2);
                for anchor_id in &floor.anchors.0 {
                    let anchor = site.get_anchor(*anchor_id).ok_or_else(|| {
                        ConfigError::Validation(format!(
                            "floor references missing anchor {anchor_id}"
                        ))
                    })?;
                    let [x, y] = anchor_xy(anchor);
                    points.extend([rounded(x), rounded(y)]);
                }

                let name = if floor_index == 0 {
                    format!("{level_name}_floor")
                } else {
                    format!("{level_name}_floor_{floor_index}")
                };
                safe_sets.push(SafeSet {
                    name,
                    region: ConfigRegion {
                        hint: Region::HINT_POLYGON,
                        points,
                    },
                });
            }
        }

        let mut parking_spots = Vec::new();
        let mut unnamed_count = 0;
        for location in site.navigation.guided.locations.values() {
            if !location.tags.0.iter().any(|tag| tag.is_parking_spot()) {
                continue;
            }

            let anchor = site.get_anchor(location.anchor.0).ok_or_else(|| {
                ConfigError::Validation(format!(
                    "parking location references missing anchor {}",
                    location.anchor.0
                ))
            })?;
            let [x, y] = anchor_xy(anchor);
            unnamed_count += 1;
            let name = if location.name.0.trim().is_empty() {
                format!("parking_{unnamed_count}")
            } else {
                location.name.0.clone()
            };
            parking_spots.push(ParkingSpot {
                name,
                region: ConfigRegion {
                    hint: Region::HINT_AXIS_ALIGNED_RECTANGLE,
                    points: vec![
                        rounded(x - grid_size),
                        rounded(y - grid_size),
                        rounded(x + grid_size),
                        rounded(y + grid_size),
                    ],
                },
            });
        }

        let config = Self {
            grid_size,
            safe_sets,
            parking_spots,
        };
        config.validate()?;
        Ok(config)
    }

    /// If no safe sets are configured, the server accepts any requested region.
    pub fn enforces_safe_sets(&self) -> bool {
        !self.safe_sets.is_empty()
    }

    /// Returns true if the requested region lies entirely within at least one
    /// predefined safe set or if safe sets are not enforced.
    pub fn is_region_within_safe_sets(&self, hint: u8, points: &[f32]) -> bool {
        if !self.enforces_safe_sets() {
            return true;
        }
        let candidate = ConfigRegion {
            hint,
            points: points.to_vec(),
        };
        if candidate.validate().is_err() {
            return false;
        }
        self.safe_sets
            .iter()
            .any(|safe_set| candidate.is_within(&safe_set.region))
    }

    /// Validate cross-cutting invariants once everything is parsed.
    fn validate(&self) -> Result<(), ConfigError> {
        // Every parking spot should live inside a safe set, otherwise we could
        // never legally divert a robot to it.
        if self.enforces_safe_sets() {
            for spot in &self.parking_spots {
                if !self.is_region_within_safe_sets(spot.region.hint, &spot.region.points) {
                    return Err(ConfigError::Validation(format!(
                        "parking spot '{}' is not within any safe set",
                        spot.name
                    )));
                }
            }
        }
        Ok(())
    }
}

fn anchor_xy(anchor: &Anchor) -> [f32; 2] {
    anchor.translation_for_category(Category::General)
}

fn rounded(value: f32) -> f32 {
    (value * 10_000.0).round() / 10_000.0
}

fn parse_parking_spots(raw: &RawConfig) -> Result<Vec<ParkingSpot>, ConfigError> {
    let mut spots = Vec::with_capacity(raw.parking_spots.len());
    for raw_spot in &raw.parking_spots {
        if raw_spot.name.trim().is_empty() {
            return Err(ConfigError::Validation(
                "every parking spot must have a non-empty name".to_string(),
            ));
        }
        spots.push(ParkingSpot {
            name: raw_spot.name.clone(),
            region: ConfigRegion::from_raw(&raw_spot.region)?,
        });
    }
    Ok(spots)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::path::PathBuf;

    const SAMPLE: &str = r#"
grid_size: 0.5
safe_sets:
  - name: main_floor
    region:
      hint: axis_aligned_rectangle
      points: [0.0, 0.0, 20.0, 20.0]
parking_spots:
  - name: parking_a
    region:
      hint: axis_aligned_rectangle
      points: [0.5, 0.5, 1.5, 1.5]
  - name: parking_b
    region:
      hint: axis_aligned_rectangle
      points: [17.5, 17.5, 18.5, 18.5]
"#;

    fn map_fixture(name: &str) -> PathBuf {
        PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../../path_server/rmf_path_server_demo/maps")
            .join(name)
    }

    fn assert_points_close(left: &[f32], right: &[f32]) {
        assert_eq!(left.len(), right.len());
        for (left, right) in left.iter().zip(right) {
            assert!(
                (left - right).abs() < 0.05,
                "coordinate mismatch: {left} vs {right}"
            );
        }
    }

    #[test]
    fn loads_native_site_editor_project() {
        let config = ReservationConfig::from_file(map_fixture("office.site.json")).unwrap();

        assert_eq!(config.safe_sets.len(), 4);
        assert_eq!(
            config
                .parking_spots
                .iter()
                .map(|spot| spot.name.as_str())
                .collect::<Vec<_>>(),
            ["tinyRobot1_charger", "supplies", "tinyRobot2_charger"]
        );
    }

    #[test]
    fn legacy_building_and_native_site_are_equivalent() {
        let building = ReservationConfig::from_file(map_fixture("office.building.yaml")).unwrap();
        let native = ReservationConfig::from_file(map_fixture("office.site.json")).unwrap();

        assert_eq!(building.safe_sets.len(), native.safe_sets.len());
        assert_eq!(building.parking_spots.len(), native.parking_spots.len());

        for (building_floor, native_floor) in building.safe_sets.iter().zip(&native.safe_sets) {
            assert_eq!(building_floor.name, native_floor.name);
            assert_eq!(building_floor.region.hint, native_floor.region.hint);
            assert_points_close(&building_floor.region.points, &native_floor.region.points);
        }
        for (building_spot, native_spot) in building.parking_spots.iter().zip(&native.parking_spots)
        {
            assert_eq!(building_spot.name, native_spot.name);
            assert_eq!(building_spot.region.hint, native_spot.region.hint);
            assert_points_close(&building_spot.region.points, &native_spot.region.points);
        }
    }

    #[test]
    fn parses_sample_config() {
        let config = ReservationConfig::from_yaml_str(SAMPLE).unwrap();
        assert_eq!(config.grid_size, 0.5);
        assert_eq!(config.safe_sets.len(), 1);
        assert_eq!(config.safe_sets[0].name, "main_floor");
        assert_eq!(
            config.safe_sets[0].region.hint,
            Region::HINT_AXIS_ALIGNED_RECTANGLE
        );
        assert_eq!(config.parking_spots.len(), 2);
        assert_eq!(config.parking_spots[1].name, "parking_b");
    }

    #[test]
    fn region_inside_safe_set_is_allowed() {
        let config = ReservationConfig::from_yaml_str(SAMPLE).unwrap();
        assert!(config.is_region_within_safe_sets(
            Region::HINT_AXIS_ALIGNED_RECTANGLE,
            &[2.0, 2.0, 3.0, 3.0]
        ));
        assert!(config.is_region_within_safe_sets(Region::HINT_POINT, &[5.0, 5.0]));
        assert!(config
            .is_region_within_safe_sets(Region::HINT_POLYGON, &[2.0, 2.0, 8.0, 2.0, 5.0, 8.0]));
    }

    #[test]
    fn region_outside_safe_set_is_rejected() {
        let config = ReservationConfig::from_yaml_str(SAMPLE).unwrap();
        // Partially outside.
        assert!(!config.is_region_within_safe_sets(
            Region::HINT_AXIS_ALIGNED_RECTANGLE,
            &[19.0, 19.0, 25.0, 25.0]
        ));
        // Fully outside.
        assert!(!config.is_region_within_safe_sets(
            Region::HINT_AXIS_ALIGNED_RECTANGLE,
            &[100.0, 100.0, 101.0, 101.0]
        ));
        assert!(!config.is_region_within_safe_sets(Region::HINT_POINT, &[25.0, 5.0]));
        assert!(!config
            .is_region_within_safe_sets(Region::HINT_POLYGON, &[2.0, 2.0, 25.0, 2.0, 5.0, 8.0]));
    }

    #[test]
    fn matches_against_any_safe_set() {
        let yaml = r#"
safe_sets:
  - name: floor
    region:
      points: [0.0, 0.0, 10.0, 10.0]
  - name: dock
    region:
      points: [20.0, 20.0, 30.0, 30.0]
"#;
        let config = ReservationConfig::from_yaml_str(yaml).unwrap();
        // Inside the first safe set.
        assert!(config.is_region_within_safe_sets(
            Region::HINT_AXIS_ALIGNED_RECTANGLE,
            &[1.0, 1.0, 2.0, 2.0]
        ));
        // Inside the second safe set.
        assert!(config.is_region_within_safe_sets(
            Region::HINT_AXIS_ALIGNED_RECTANGLE,
            &[24.0, 24.0, 25.0, 25.0]
        ));
        // In the gap between the two safe sets.
        assert!(!config.is_region_within_safe_sets(
            Region::HINT_AXIS_ALIGNED_RECTANGLE,
            &[14.0, 14.0, 16.0, 16.0]
        ));
    }

    #[test]
    fn empty_safe_sets_are_permissive() {
        let config = ReservationConfig::from_yaml_str("grid_size: 1.0").unwrap();
        assert!(!config.enforces_safe_sets());
        assert!(config.is_region_within_safe_sets(
            Region::HINT_AXIS_ALIGNED_RECTANGLE,
            &[1000.0, 1000.0, 1001.0, 1001.0]
        ));
    }

    #[test]
    fn parking_spot_outside_safe_set_fails_validation() {
        let yaml = r#"
safe_sets:
  - name: main_floor
    region:
      points: [0.0, 0.0, 10.0, 10.0]
parking_spots:
  - name: bad_spot
    region:
      points: [49.0, 49.0, 51.0, 51.0]
"#;
        let err = ReservationConfig::from_yaml_str(yaml).unwrap_err();
        assert!(matches!(err, ConfigError::Validation(_)));
    }

    #[test]
    fn invalid_hint_fails() {
        let yaml = r#"
safe_sets:
  - name: main_floor
    region:
      hint: hexagon
      points: [0.0, 0.0, 10.0, 10.0]
"#;
        assert!(ReservationConfig::from_yaml_str(yaml).is_err());
    }

    #[test]
    fn hint_is_case_sensitive() {
        let yaml = r#"
safe_sets:
  - name: main_floor
    region:
      hint: Axis_Aligned_Rectangle
      points: [0.0, 0.0, 10.0, 10.0]
"#;
        assert!(ReservationConfig::from_yaml_str(yaml).is_err());
    }

    #[test]
    fn hint_tolerates_surrounding_whitespace() {
        let yaml = r#"
safe_sets:
  - name: main_floor
    region:
      hint: "  point  "
      points: [1.0, 1.0]
"#;
        let config = ReservationConfig::from_yaml_str(yaml).unwrap();
        assert_eq!(config.safe_sets[0].region.hint, Region::HINT_POINT);
    }

    #[test]
    fn odd_number_of_points_fails() {
        let yaml = r#"
safe_sets:
  - name: main_floor
    region:
      points: [0.0, 0.0, 10.0]
"#;
        assert!(ReservationConfig::from_yaml_str(yaml).is_err());
    }
}
