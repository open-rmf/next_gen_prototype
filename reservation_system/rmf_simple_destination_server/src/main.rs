use rclrs::{Context, CreateBasicExecutor, SpinOptions};
use rmf_prototype_msgs::msg::{
    Destination, DestinationConstraints, DestinationError, DestinationGoal, Error, Region,
    TargetRegion,
};
use std::collections::HashMap;

#[derive(Hash, PartialEq, Eq, Clone, Copy, Debug)]
struct SessionUUID {
    uuid: [u8; 16],
}

impl SessionUUID {
    fn from_ros(ros_msg: &unique_identifier_msgs::msg::UUID) -> Self {
        Self {
            uuid: ros_msg.uuid,
        }
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
            regions: ros.regions.iter().map(DomainTargetRegion::from_ros).collect(),
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
}

impl DomainDestination {
    fn to_ros(&self) -> Destination {
        Destination {
            constraints: self.constraints.to_ros(),
            session: unique_identifier_msgs::msg::UUID {
                uuid: self.session.uuid,
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
            session: unique_identifier_msgs::msg::UUID {
                uuid: self.session.uuid,
            },
        }
    }
}

struct CurrentlyOccupiedDestinations {

    // A simple occupancy grid. If None, the region is considered free, if Some it is allocated to the session
    floor_space: Vec<Vec<Option<SessionUUID>>>,

    // This helps with book keeping of the occupancy grid.
    session_to_location: HashMap<SessionUUID, Vec<(u32, u32)>>,

    // Each agent can have only one session
    agent_to_session: HashMap<String, SessionUUID>,
    /// Size of an indivudual cell.
    grid_size: f32,
}

#[derive(Debug, PartialEq)]
enum ReservationError {
    NoAvailableConstraints,
}

impl CurrentlyOccupiedDestinations {
    fn new(grid_size: f32) -> Self {
        Self {
            floor_space: Vec::new(),
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
    fn reserve_one_of(
        &mut self,
        agent: &str,
        constraints: &[DomainDestinationConstraints],
        session: &SessionUUID,
    ) -> Result<DomainDestinationConstraints, ReservationError> {
        let mut allocated_id = None;
        for (id, c) in constraints.iter().enumerate() {
            // check all  regions are free
            let all_free = c.regions.iter().all(|target_region| {
                self.check_if_region_free(&target_region.region).unwrap_or(false)
            });

            if all_free {
                allocated_id = Some(id);
                break;
            }
        }

        if let Some(id) = allocated_id {
            // Check if the agent holds a reservation
            if let Some(old_uuid) = self.agent_to_session.get(agent) {
                self.clear_old_uuid(&old_uuid.clone());
            }

            let selected_constraints = constraints[id].clone();

            // Mark the new regions
            for target_region in &selected_constraints.regions {
                self.mark_region(session, &target_region.region);
            }
            self.agent_to_session.insert(agent.to_string(), *session);
            Ok(selected_constraints)
        } else {
            Err(ReservationError::NoAvailableConstraints)
        }
    }

    fn clear_old_uuid(&mut self, session: &SessionUUID) {
        let Some(locations) = self.session_to_location.get(session) else {
            return;
        };
        for location in locations {
            if let Some(row) = self.floor_space.get_mut(location.1 as usize) {
                if let Some(cell) = row.get_mut(location.0 as usize) {
                    *cell = None;
                }
            }
        }

        self.session_to_location.remove(session);
    }

    fn mark_region(&mut self, session: &SessionUUID, region: &DomainRegion) {
        let Some((start_x, end_x, start_y, end_y)) = self.get_region_grid_bounds(region) else {
            return;
        };

        // Resize floor_space if needed
        if self.floor_space.len() <= end_y {
            self.floor_space.resize(end_y + 1, Vec::new());
        }

        for y in start_y..=end_y {
            let row = &mut self.floor_space[y];
            if row.len() <= end_x {
                row.resize(end_x + 1, None);
            }
            for x in start_x..=end_x {
                row[x] = Some(*session);
                self.session_to_location
                    .entry(*session)
                    .or_default()
                    .push((x as u32, y as u32));
            }
        }
    }

    fn get_region_grid_bounds(&self, region: &DomainRegion) -> Option<(usize, usize, usize, usize)> {
        if region.hint == DomainRegion::HINT_AXIS_ALIGNED_RECTANGLE {
            if region.points.len() < 2 || region.points.len() % 2 != 0 {
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

        if min_x < 0.0 || min_y < 0.0 {
            // As per instructions, we don't care about negative coords for now.
            // But let's at least clip them to 0 if they are partly negative,
            // or return None if fully negative.
            if max_x < 0.0 || max_y < 0.0 {
                return None;
            }
            min_x = min_x.max(0.0);
            min_y = min_y.max(0.0);
        }

        let start_x = (min_x / self.grid_size).floor() as usize;
        let end_x = (max_x / self.grid_size).ceil() as usize;
        let start_y = (min_y / self.grid_size).floor() as usize;
        let end_y = (max_y / self.grid_size).ceil() as usize;

        Some((start_x, end_x, start_y, end_y))
    }

    fn check_if_region_free(&self, region: &DomainRegion) -> Result<bool, ShapeErr> {
        let bounds = self.get_region_grid_bounds(region);
        let Some((start_x, end_x, start_y, end_y)) = bounds else {
            // If we can't get bounds (invalid points or negative), we treat it as an error
            // but for simplicity here we'll just check hint if it was invalid points.
            if region.hint == DomainRegion::HINT_AXIS_ALIGNED_RECTANGLE || region.hint == DomainRegion::HINT_POINT {
                if region.points.len() < 2 {
                    return Err(ShapeErr::InvalidPoints);
                }
            } else {
                return Err(ShapeErr::UnsupportedHint);
            }
            // If points were okay but negative, we can return true (it's "free" in our tracked positive space)
            return Ok(true);
        };

        for y_idx in start_y..=end_y {
            if let Some(row) = self.floor_space.get(y_idx) {
                for x_idx in start_x..=end_x {
                    if let Some(cell) = row.get(x_idx) {
                        if cell.is_some() {
                            return Ok(false);
                        }
                    }
                }
            }
        }
        Ok(true)
    }
}

struct DestinationServer {
    currently_occupied: CurrentlyOccupiedDestinations
}

impl DestinationServer {
    fn new(grid_size: f32) -> Self {
        Self {
            currently_occupied: CurrentlyOccupiedDestinations::new(grid_size),
        }
    }

    fn request_destination(
        &mut self,
        agent_id: &str,
        goals: &DomainDestinationGoal,
    ) -> Result<DomainDestination, DomainDestinationError> {
        if !goals.cost_bias.is_empty() && goals.one_of.len() != goals.cost_bias.len() {
            return Err(DomainDestinationError {
                error: DomainError {
                    code: 0, //TODO(arjoc): reserve a code,
                    message: "Number of goals and destinations do not match".into(),
                    parameters: "TODO".into(),
                },
                session: goals.session,
            });
        }

        let mut sorted_one_of = goals.one_of.clone();
        //TODO(arjoc): Sort goals by on_of length
        sorted_one_of.sort_by_key(|c| c.regions.len());

        if sorted_one_of.is_empty() {
            return Err(DomainDestinationError {
                error: DomainError {
                    code: 0,
                    message: "No goals provided".into(),
                    parameters: "one_of is empty".into(),
                },
                session: goals.session,
            });
        }

        match self
            .currently_occupied
            .reserve_one_of(agent_id, &sorted_one_of, &goals.session)
        {
            Ok(constraints) => Ok(DomainDestination {
                constraints,
                session: goals.session,
            }),
            Err(_) => Err(DomainDestinationError {
                error: DomainError {
                    code: 0,
                    message: "No available constraints".into(),
                    parameters: "".into(),
                },
                session: goals.session,
            }),
        }
    }
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let context = Context::default_from_env().unwrap();
    let mut executor = context.create_basic_executor();
    let node = executor.create_node("destination_server")?;

    let robots = ["robot_1", "robot_2"];

    let worker = node.create_worker(DestinationServer::new(0.5));

    let mut subscriptions = Vec::new();
    let mut goal_publishers = Vec::new();
    let mut error_publishers = Vec::new();

    for robot in robots {
        let goal_publisher = node.create_publisher::<Destination>(
            &(robot.to_owned() + "/destination"),
        )?;
        let error_publisher = node.create_publisher::<DestinationError>(
            &(robot.to_owned() + "/destination/error"),
        )?;
        
        let robot_id = robot.to_string();

        let goal_pub_clone = goal_publisher.clone();
        let error_pub_clone = error_publisher.clone();

        let subscription = worker.create_subscription::<DestinationGoal, _>(
            &(robot.to_owned() + "/destination/goal"),
            move |server: &mut DestinationServer, msg: DestinationGoal| {
                println!("Received goal for {}: {:?}", robot_id, msg);
                let domain_goal = DomainDestinationGoal::from_ros(&msg);
                match server.request_destination(&robot_id, &domain_goal) {
                    Ok(dest) => {
                        println!("Successfully reserved destination for {}", robot_id);
                        let _ = goal_pub_clone.publish(&dest.to_ros());
                    }
                    Err(err) => {
                        println!("Failed to reserve destination for {}: {:?}", robot_id, err);
                        let _ = error_pub_clone.publish(&err.to_ros());
                    }
                }
            },
        )?;
        subscriptions.push(subscription);
        goal_publishers.push(goal_publisher);
        error_publishers.push(error_publisher);
    }

    println!("Destination server started. Spinning...");
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

        // Reserve first option
        let res = od.reserve_one_of(agent, &constraints, &session1);
        assert!(res.is_ok());
        assert_eq!(res.unwrap(), constraints[0]);
        assert_eq!(od.agent_to_session.get(agent), Some(&session1));
        assert!(!od.check_if_region_free(&constraints[0].regions[0].region).unwrap());

        // Reserve second option (should clear first)
        let res = od.reserve_one_of(agent, &[constraints[1].clone()], &session2);
        assert!(res.is_ok());
        assert_eq!(res.unwrap(), constraints[1]);
        assert_eq!(od.agent_to_session.get(agent), Some(&session2));
        assert!(od.check_if_region_free(&constraints[0].regions[0].region).unwrap());
        assert!(!od.check_if_region_free(&constraints[1].regions[0].region).unwrap());

        // Try to reserve an occupied region
        let session3 = SessionUUID { uuid: [3; 16] };
        let res = od.reserve_one_of("robot_2", &[constraints[1].clone()], &session3);
        assert_eq!(res, Err(ReservationError::NoAvailableConstraints));
    }
}
