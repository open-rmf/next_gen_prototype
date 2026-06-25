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

mod config;
mod reservation;

use config::ReservationConfig;
use rclrs::{Context, CreateBasicExecutor, IntoPrimitiveOptions, SpinOptions};
use reservation::{Outcome, ReservationState};
use ros_env::rmf_next_gen_reservation_msgs::msg::{
    NamedRegion, ReservationConfig as ReservationConfigMsg,
};
use ros_env::rmf_prototype_msgs::msg::{
    Destination, DestinationConstraints, DestinationError, DestinationGoal, Error, Region,
    TargetRegion,
};
use ros_env::unique_identifier_msgs::msg::UUID;
use std::collections::HashMap;
use std::sync::Arc;

#[derive(Hash, PartialEq, Eq, Clone, Copy, Debug)]
struct SessionUUID {
    uuid: [u8; 16],
}

impl SessionUUID {
    fn from_ros(ros_msg: &UUID) -> Self {
        Self { uuid: ros_msg.uuid }
    }
}

impl std::fmt::Display for SessionUUID {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "{:02x}{:02x}{:02x}{:02x}-{:02x}{:02x}-{:02x}{:02x}-{:02x}{:02x}-{:02x}{:02x}{:02x}{:02x}{:02x}{:02x}",
            self.uuid[0], self.uuid[1], self.uuid[2], self.uuid[3],
            self.uuid[4], self.uuid[5],
            self.uuid[6], self.uuid[7],
            self.uuid[8], self.uuid[9],
            self.uuid[10], self.uuid[11], self.uuid[12], self.uuid[13], self.uuid[14], self.uuid[15]
        )
    }
}

#[derive(Debug)]
enum ShapeErr {
    UnsupportedHint,
    InvalidPoints,
}

#[derive(Clone, Debug, PartialEq)]
struct DomainRegion {
    pub points: Vec<f32>,
    pub hint: u8,
}

impl DomainRegion {
    const HINT_POINT: u8 = 1;
    const HINT_AXIS_ALIGNED_RECTANGLE: u8 = 2;

    fn from_ros(ros: &Region) -> Self {
        Self {
            points: ros.points.clone(),
            hint: ros.hint,
        }
    }

    fn to_ros(&self) -> Region {
        Region {
            points: self.points.clone(),
            hint: self.hint,
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
struct DomainTargetRegion {
    pub tolerance: f32,
    pub region: DomainRegion,
}

impl DomainTargetRegion {
    fn from_ros(ros: &TargetRegion) -> Self {
        Self {
            tolerance: ros.tolerance,
            region: DomainRegion::from_ros(&ros.region),
        }
    }

    fn to_ros(&self) -> TargetRegion {
        TargetRegion {
            tolerance: self.tolerance,
            region: self.region.to_ros(),
            ..Default::default()
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
struct DomainDestinationConstraints {
    pub regions: Vec<DomainTargetRegion>,
}

impl DomainDestinationConstraints {
    fn from_ros(ros: &DestinationConstraints) -> Self {
        Self {
            regions: ros
                .regions
                .iter()
                .map(DomainTargetRegion::from_ros)
                .collect(),
        }
    }

    fn to_ros(&self) -> DestinationConstraints {
        DestinationConstraints {
            regions: self.regions.iter().map(|r| r.to_ros()).collect(),
            ..Default::default()
        }
    }
}

struct DomainDestinationGoal {
    pub one_of: Vec<DomainDestinationConstraints>,
    pub cost_bias: Vec<f32>,
    pub session: SessionUUID,
}

impl DomainDestinationGoal {
    fn from_ros(ros: &DestinationGoal) -> Self {
        Self {
            one_of: ros
                .one_of
                .iter()
                .map(DomainDestinationConstraints::from_ros)
                .collect(),
            cost_bias: ros.cost_bias.clone(),
            session: SessionUUID::from_ros(&ros.session),
        }
    }
}

struct DomainDestination {
    pub constraints: DomainDestinationConstraints,
    pub session: SessionUUID,
    /// The original goal session when this destination is a parking detour.
    pub detour_for_goal: Option<SessionUUID>,
}

impl DomainDestination {
    fn to_ros(&self) -> Destination {
        Destination {
            constraints: self.constraints.to_ros(),
            session: UUID {
                uuid: self.session.uuid,
            },
            detour_for_goal: UUID {
                uuid: match self.detour_for_goal {
                    Some(session) => session.uuid,
                    None => [0u8; 16],
                },
            },
            ..Default::default()
        }
    }
}

#[derive(Debug)]
struct DomainError {
    pub code: u32,
    pub message: String,
    pub parameters: String,
}

impl DomainError {
    fn to_ros(&self) -> Error {
        Error {
            code: self.code,
            message: self.message.clone(),
            parameters: self.parameters.clone(),
        }
    }
}

#[derive(Debug)]
struct DomainDestinationError {
    pub error: DomainError,
    pub session: SessionUUID,
}

impl DomainDestinationError {
    fn to_ros(&self) -> DestinationError {
        DestinationError {
            error: self.error.to_ros(),
            session: UUID {
                uuid: self.session.uuid,
            },
        }
    }
}

struct CurrentlyOccupiedDestinations {
    // Sparse occupancy grid keyed by signed cell coordinates.
    floor_space: HashMap<(i32, i32), SessionUUID>,

    // This helps with book keeping of the occupancy grid.
    session_to_location: HashMap<SessionUUID, Vec<(i32, i32)>>,

    // Each agent can have only one session
    agent_to_session: HashMap<String, SessionUUID>,
    /// Size of an indivudual cell.
    grid_size: f32,
}

impl CurrentlyOccupiedDestinations {
    fn new(grid_size: f32) -> Self {
        Self {
            floor_space: HashMap::new(),
            session_to_location: HashMap::new(),
            agent_to_session: HashMap::new(),
            grid_size,
        }
    }
}

impl Default for CurrentlyOccupiedDestinations {
    fn default() -> Self {
        Self::new(1.0)
    }
}

impl CurrentlyOccupiedDestinations {
    fn is_constraints_free(&self, constraints: &DomainDestinationConstraints) -> bool {
        constraints.regions.iter().all(|target_region| {
            self.check_if_region_free(&target_region.region)
                .unwrap_or(false)
        })
    }

    fn first_free_option(&self, options: &[DomainDestinationConstraints]) -> Option<usize> {
        options.iter().position(|c| self.is_constraints_free(c))
    }

    // Callers check that the constraints are free before reserving them.
    fn reserve_constraints(
        &mut self,
        agent: &str,
        session: &SessionUUID,
        constraints: &DomainDestinationConstraints,
    ) {
        for target_region in &constraints.regions {
            self.mark_region(session, &target_region.region);
        }
        self.agent_to_session.insert(agent.to_string(), *session);
    }

    fn release_agent(&mut self, agent: &str) {
        if let Some(session) = self.agent_to_session.remove(agent) {
            self.clear_old_uuid(&session);
        }
    }

    fn clear_old_uuid(&mut self, session: &SessionUUID) {
        let Some(locations) = self.session_to_location.remove(session) else {
            return;
        };
        for cell in locations {
            // Do not clear cells reassigned to another session.
            if self.floor_space.get(&cell) == Some(session) {
                self.floor_space.remove(&cell);
            }
        }
    }

    fn mark_region(&mut self, session: &SessionUUID, region: &DomainRegion) {
        let Some((start_x, end_x, start_y, end_y)) = self.get_region_grid_bounds(region) else {
            return;
        };

        for y in start_y..=end_y {
            for x in start_x..=end_x {
                self.floor_space.insert((x, y), *session);
                self.session_to_location
                    .entry(*session)
                    .or_default()
                    .push((x, y));
            }
        }
    }

    fn get_region_grid_bounds(&self, region: &DomainRegion) -> Option<(i32, i32, i32, i32)> {
        if region.hint == DomainRegion::HINT_AXIS_ALIGNED_RECTANGLE {
            if region.points.len() < 2 || !region.points.len().is_multiple_of(2) {
                return None;
            }
        } else if region.hint == DomainRegion::HINT_POINT {
            if region.points.len() != 2 {
                return None;
            }
        } else {
            return None;
        }

        let mut min_x = f32::MAX;
        let mut max_x = f32::MIN;
        let mut min_y = f32::MAX;
        let mut max_y = f32::MIN;

        for i in (0..region.points.len()).step_by(2) {
            let x = region.points[i];
            let y = region.points[i + 1];
            min_x = min_x.min(x);
            max_x = max_x.max(x);
            min_y = min_y.min(y);
            max_y = max_y.max(y);
        }

        let start_x = (min_x / self.grid_size).floor() as i32;
        let end_x = (max_x / self.grid_size).ceil() as i32;
        let start_y = (min_y / self.grid_size).floor() as i32;
        let end_y = (max_y / self.grid_size).ceil() as i32;

        Some((start_x, end_x, start_y, end_y))
    }

    fn check_if_region_free(&self, region: &DomainRegion) -> Result<bool, ShapeErr> {
        let Some((start_x, end_x, start_y, end_y)) = self.get_region_grid_bounds(region) else {
            if region.hint == DomainRegion::HINT_AXIS_ALIGNED_RECTANGLE
                || region.hint == DomainRegion::HINT_POINT
            {
                return Err(ShapeErr::InvalidPoints);
            }
            return Err(ShapeErr::UnsupportedHint);
        };

        for y_idx in start_y..=end_y {
            for x_idx in start_x..=end_x {
                if self.floor_space.contains_key(&(x_idx, y_idx)) {
                    return Ok(false);
                }
            }
        }
        Ok(true)
    }
}

struct RobotPublishers {
    goal: rclrs::Publisher<Destination>,
    error: rclrs::Publisher<DestinationError>,
}

struct DestinationsServer {
    state: ReservationState,
    robot_publishers: HashMap<String, RobotPublishers>,
    node: Arc<rclrs::Node>,
}

impl DestinationsServer {
    fn new(node: Arc<rclrs::Node>, config: Arc<ReservationConfig>) -> Self {
        Self {
            state: ReservationState::new(config),
            robot_publishers: HashMap::new(),
            node,
        }
    }

    fn register_robot(
        &mut self,
        robot_id: &str,
        goal: rclrs::Publisher<Destination>,
        error: rclrs::Publisher<DestinationError>,
    ) {
        self.robot_publishers
            .insert(robot_id.to_string(), RobotPublishers { goal, error });
    }

    fn deregister_robot(&mut self, robot_id: &str) {
        self.robot_publishers.remove(robot_id);
        let outcomes = self.state.remove(robot_id);
        self.dispatch(outcomes);
    }

    fn handle_goal(&mut self, robot_id: &str, goal_msg: DestinationGoal) {
        let goal = DomainDestinationGoal::from_ros(&goal_msg);
        rclrs::log!(
            self.node.logger(),
            "Received goal for {} (session UUID {})",
            robot_id,
            goal.session
        );
        let outcomes = self.state.request(robot_id, goal);
        self.dispatch(outcomes);
    }

    fn dispatch(&self, outcomes: Vec<Outcome>) {
        for outcome in outcomes {
            match outcome {
                Outcome::Reserve { agent, destination } => {
                    let Some(publishers) = self.robot_publishers.get(&agent) else {
                        rclrs::log_warn!(
                            self.node.logger(),
                            "No publisher registered for {}; dropping destination",
                            agent
                        );
                        continue;
                    };
                    match destination.detour_for_goal {
                        Some(goal_session) => rclrs::log!(
                            self.node.logger(),
                            "Diverting {} to a parking spot (detour session {}) while it waits \
                             for goal {}",
                            agent,
                            destination.session,
                            goal_session
                        ),
                        None => rclrs::log!(
                            self.node.logger(),
                            "Reserved goal for {} (session UUID {})",
                            agent,
                            destination.session
                        ),
                    }
                    let _ = publishers.goal.publish(destination.to_ros());
                }
                Outcome::Error { agent, error } => {
                    let Some(publishers) = self.robot_publishers.get(&agent) else {
                        continue;
                    };
                    rclrs::log_error!(
                        self.node.logger(),
                        "Failed to reserve destination for {} (session UUID {}): {}",
                        agent,
                        error.session,
                        error.error.message
                    );
                    let _ = publishers.error.publish(error.to_ros());
                }
            }
        }
    }
}

struct DiscoveryServer {
    node: Arc<rclrs::Node>,
    active_robots: HashMap<String, rclrs::WorkerSubscription<DestinationGoal, DestinationsServer>>,
    destinations_worker: Arc<rclrs::Worker<DestinationsServer>>,
}

impl DiscoveryServer {
    fn new(
        node: Arc<rclrs::Node>,
        destinations_worker: Arc<rclrs::Worker<DestinationsServer>>,
    ) -> Self {
        Self {
            node,
            active_robots: HashMap::new(),
            destinations_worker,
        }
    }
}

/// Load the reservation configuration from the given file path.
///
/// If no path is provided, fall back to the default configuration.
/// Throws an error if a path is provided but cannot be loaded.
fn load_config(
    node: &Arc<rclrs::Node>,
    path: &str,
) -> Result<Arc<ReservationConfig>, config::ConfigError> {
    if path.is_empty() {
        rclrs::log_warn!(
            node.logger(),
            "No 'config_file' parameter provided; running with a permissive default \
             configuration (all requested regions accepted)."
        );
        return Ok(Arc::new(ReservationConfig::default()));
    }

    let config = ReservationConfig::from_file(path).map_err(|err| {
        rclrs::log_error!(
            node.logger(),
            "Failed to load reservation config from '{}': {}.",
            path,
            err
        );
        err
    })?;

    rclrs::log!(
        node.logger(),
        "Loaded reservation config from '{}': {} safe set(s), {} parking spot(s), \
         grid_size {}.",
        path,
        config.safe_sets.len(),
        config.parking_spots.len(),
        config.grid_size
    );
    Ok(Arc::new(config))
}

/// Convert the loaded reservation config to its ROS message.
fn config_to_ros(config: &ReservationConfig) -> ReservationConfigMsg {
    let region_to_ros = |region: &config::ConfigRegion| Region {
        points: region.points.clone(),
        hint: region.hint,
    };

    ReservationConfigMsg {
        grid_size: config.grid_size,
        safe_sets: config
            .safe_sets
            .iter()
            .map(|set| NamedRegion {
                name: set.name.clone(),
                region: region_to_ros(&set.region),
            })
            .collect(),
        parking_spots: config
            .parking_spots
            .iter()
            .map(|spot| NamedRegion {
                name: spot.name.clone(),
                region: region_to_ros(&spot.region),
            })
            .collect(),
    }
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let context = Context::default_from_env().unwrap();
    let mut executor = context.create_basic_executor();
    let node = Arc::new(executor.create_node("reservation_destination_server")?);

    // Path to the reservation configuration file.
    let config_file = node
        .declare_parameter("config_file")
        .default(Arc::from(""))
        .mandatory()?;

    let config = load_config(&node, &config_file.get())?;

    // Transient local durability makes the config available to late subscribers.
    let config_publisher = node.create_publisher::<ReservationConfigMsg>(
        "/destination/reservation_config"
            .transient_local()
            .reliable(),
    )?;
    config_publisher.publish(config_to_ros(&config))?;
    rclrs::log!(
        node.logger(),
        "Published reservation config on /destination/reservation_config"
    );

    // 1. Create the Destinations worker (Data Plane)
    let destinations_worker = Arc::new(node.create_worker(DestinationsServer::new(
        Arc::clone(&node),
        Arc::clone(&config),
    )));

    // 2. Create the Discovery worker (Control Plane), passing a reference to the Destinations worker
    let discovery_worker = Arc::new(node.create_worker(DiscoveryServer::new(
        Arc::clone(&node),
        Arc::clone(&destinations_worker),
    )));

    // 3. Subscribe to discovery using the shared helper
    let _discovery_subscription = rmf_participant_discovery::create_discovery_subscription(
        &discovery_worker,
        "/destination/discovery",
        |server: &mut DiscoveryServer, robot_id: &str| {
            if !server.active_robots.contains_key(robot_id) {
                rclrs::log!(
                    server.node.logger(),
                    "Discovered new participant: {}",
                    robot_id
                );

                let goal_publisher = match server
                    .node
                    .create_publisher::<Destination>(&(robot_id.to_string() + "/destination"))
                {
                    Ok(pub_) => pub_,
                    Err(err) => {
                        rclrs::log_error!(
                            server.node.logger(),
                            "Failed to create destination publisher for {}: {:?}",
                            robot_id,
                            err
                        );
                        return;
                    }
                };

                let error_publisher = match server.node.create_publisher::<DestinationError>(
                    &(robot_id.to_string() + "/destination/error"),
                ) {
                    Ok(pub_) => pub_,
                    Err(err) => {
                        rclrs::log_error!(
                            server.node.logger(),
                            "Failed to create error publisher for {}: {:?}",
                            robot_id,
                            err
                        );
                        return;
                    }
                };

                let robot_id_clone = robot_id.to_string();

                // Queue registration first so a goal cannot arrive before its
                // publishers are available.
                let register_id = robot_id.to_string();
                std::mem::drop(server.destinations_worker.run(
                    move |dest_server: &mut DestinationsServer| {
                        dest_server.register_robot(&register_id, goal_publisher, error_publisher);
                    },
                ));

                // Create the goal subscription on the destinations_worker thread context!
                let subscription = match server
                    .destinations_worker
                    .create_subscription::<DestinationGoal, _>(
                        &(robot_id.to_string() + "/destination/goal"),
                        move |dest_server: &mut DestinationsServer, goal_msg: DestinationGoal| {
                            dest_server.handle_goal(&robot_id_clone, goal_msg);
                        },
                    ) {
                    Ok(sub) => sub,
                    Err(err) => {
                        rclrs::log_error!(
                            server.node.logger(),
                            "Failed to create goal subscription on DestinationsWorker for {}: {:?}",
                            robot_id,
                            err
                        );
                        return;
                    }
                };

                server
                    .active_robots
                    .insert(robot_id.to_string(), subscription);
            }
        },
        |server: &mut DiscoveryServer, robot_id: &str| {
            if server.active_robots.remove(robot_id).is_some() {
                rclrs::log!(server.node.logger(), "Participant left: {}", robot_id);
                let deregister_id = robot_id.to_string();
                std::mem::drop(server.destinations_worker.run(
                    move |dest_server: &mut DestinationsServer| {
                        dest_server.deregister_robot(&deregister_id);
                    },
                ));
            }
        },
    )?;

    rclrs::log!(
        node.logger(),
        "Reservation destination server started with multi-worker separation \
         (DiscoveryWorker + DestinationsWorker). Spinning..."
    );
    executor.spin(SpinOptions::default());
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn create_test_occupied() -> CurrentlyOccupiedDestinations {
        CurrentlyOccupiedDestinations::new(1.0)
    }

    #[test]
    fn test_mark_and_check_region() {
        let mut od = create_test_occupied();
        let session = SessionUUID { uuid: [1; 16] };
        let region = DomainRegion {
            hint: DomainRegion::HINT_AXIS_ALIGNED_RECTANGLE,
            points: vec![0.0, 0.0, 2.0, 2.0],
        };

        assert!(od.check_if_region_free(&region).unwrap());
        od.mark_region(&session, &region);
        assert!(!od.check_if_region_free(&region).unwrap());

        // Check if book-keeping works
        assert!(od.session_to_location.contains_key(&session));
        assert_eq!(od.session_to_location.get(&session).unwrap().len(), 9); // (0,0) to (2,2) inclusive is 3x3=9 cells
    }

    #[test]
    fn test_mark_and_check_negative_region() {
        let mut od = create_test_occupied();
        let session = SessionUUID { uuid: [7; 16] };
        let region = DomainRegion {
            hint: DomainRegion::HINT_AXIS_ALIGNED_RECTANGLE,
            points: vec![-8.0, -8.0, -7.0, -7.0],
        };

        assert!(od.check_if_region_free(&region).unwrap());
        od.mark_region(&session, &region);
        assert!(!od.check_if_region_free(&region).unwrap());

        let positive = DomainRegion {
            hint: DomainRegion::HINT_AXIS_ALIGNED_RECTANGLE,
            points: vec![1.0, 1.0, 2.0, 2.0],
        };
        assert!(od.check_if_region_free(&positive).unwrap());

        od.clear_old_uuid(&session);
        assert!(od.check_if_region_free(&region).unwrap());
    }

    #[test]
    fn test_clear_old_uuid() {
        let mut od = create_test_occupied();
        let session = SessionUUID { uuid: [1; 16] };
        let region = DomainRegion {
            hint: DomainRegion::HINT_AXIS_ALIGNED_RECTANGLE,
            points: vec![0.0, 0.0, 1.0, 1.0],
        };

        od.mark_region(&session, &region);
        assert!(!od.check_if_region_free(&region).unwrap());

        od.clear_old_uuid(&session);
        assert!(od.check_if_region_free(&region).unwrap());
        assert!(!od.session_to_location.contains_key(&session));
    }

    #[test]
    fn test_reserve_one_of() {
        let mut od = create_test_occupied();
        let agent = "robot_1";
        let session1 = SessionUUID { uuid: [1; 16] };
        let session2 = SessionUUID { uuid: [2; 16] };

        let constraints = vec![
            DomainDestinationConstraints {
                regions: vec![DomainTargetRegion {
                    tolerance: 0.0,
                    region: DomainRegion {
                        hint: DomainRegion::HINT_AXIS_ALIGNED_RECTANGLE,
                        points: vec![0.0, 0.0, 1.0, 1.0],
                    },
                }],
            },
            DomainDestinationConstraints {
                regions: vec![DomainTargetRegion {
                    tolerance: 0.0,
                    region: DomainRegion {
                        hint: DomainRegion::HINT_AXIS_ALIGNED_RECTANGLE,
                        points: vec![5.0, 5.0, 6.0, 6.0],
                    },
                }],
            },
        ];

        let id = od.first_free_option(&constraints).unwrap();
        assert_eq!(id, 0);
        od.release_agent(agent);
        od.reserve_constraints(agent, &session1, &constraints[id]);
        assert_eq!(od.agent_to_session.get(agent), Some(&session1));
        assert!(!od
            .check_if_region_free(&constraints[0].regions[0].region)
            .unwrap());

        let only_second = [constraints[1].clone()];
        let id = od.first_free_option(&only_second).unwrap();
        od.release_agent(agent);
        od.reserve_constraints(agent, &session2, &only_second[id]);
        assert_eq!(od.agent_to_session.get(agent), Some(&session2));
        assert!(od
            .check_if_region_free(&constraints[0].regions[0].region)
            .unwrap());
        assert!(!od
            .check_if_region_free(&constraints[1].regions[0].region)
            .unwrap());

        assert!(od.first_free_option(&only_second).is_none());
    }
}
