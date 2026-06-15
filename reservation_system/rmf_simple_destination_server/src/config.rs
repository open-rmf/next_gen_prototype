//! Configuration parsing for the simple destination server.

use rmf_prototype_msgs::msg::Region;
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
    /// The configuration was parsed but is semantically invalid.
    Validation(String),
}

impl fmt::Display for ConfigError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            ConfigError::Io(e) => write!(f, "failed to read config file: {e}"),
            ConfigError::Parse(e) => write!(f, "failed to parse config file: {e}"),
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
        if self.points.is_empty() || self.points.len() % 2 != 0 {
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

    /// Parse and validate a configuration from a file on disk.
    pub fn from_file<P: AsRef<Path>>(path: P) -> Result<Self, ConfigError> {
        let contents = std::fs::read_to_string(path)?;
        Self::from_yaml_str(&contents)
    }

    /// If no safe sets are configured the server runs accepts any requested region.
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
