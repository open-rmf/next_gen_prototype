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

use super::{
    CurrentlyOccupiedDestinations, DomainDestination, DomainDestinationConstraints,
    DomainDestinationError, DomainDestinationGoal, DomainError, DomainRegion, DomainTargetRegion,
    SessionUUID,
};
use crate::config::{ParkingSpot, ReservationConfig};
use ros_env::rmf_prototype_msgs::msg::DestinationError;
use std::collections::HashMap;
use std::sync::Arc;

struct OccupiedReservation {
    constraints: DomainDestinationConstraints,
    session: SessionUUID,
}

struct AgentState {
    desired_options: Vec<DomainDestinationConstraints>,
    goal_session: SessionUUID,
    occupied: OccupiedReservation,
}

pub(super) enum Outcome {
    Reserve {
        agent: String,
        destination: DomainDestination,
    },
    Error {
        agent: String,
        error: DomainDestinationError,
    },
}

fn parking_constraints(spot: &ParkingSpot) -> DomainDestinationConstraints {
    DomainDestinationConstraints {
        regions: vec![DomainTargetRegion {
            tolerance: 0.0,
            region: DomainRegion {
                points: spot.region.points.clone(),
                hint: spot.region.hint,
            },
        }],
    }
}

pub(super) struct ReservationState {
    currently_occupied: CurrentlyOccupiedDestinations,
    config: Arc<ReservationConfig>,
    agents: HashMap<String, AgentState>,
    queue: Vec<String>,
    next_detour: u128,
}

impl ReservationState {
    pub(super) fn new(config: Arc<ReservationConfig>) -> Self {
        Self {
            currently_occupied: CurrentlyOccupiedDestinations::new(config.grid_size),
            config,
            agents: HashMap::new(),
            queue: Vec::new(),
            next_detour: 0,
        }
    }

    // Prefix detour IDs so they do not look like regular goal sessions.
    fn next_detour_session(&mut self) -> SessionUUID {
        self.next_detour += 1;
        let mut uuid = [0u8; 16];
        uuid[0..4].copy_from_slice(&[0xDE, 0x70, 0x18, 0x12]);
        uuid[8..16].copy_from_slice(&self.next_detour.to_be_bytes()[8..16]);
        SessionUUID { uuid }
    }

    fn reserve_free_parking(
        &mut self,
        agent: &str,
    ) -> Option<(SessionUUID, DomainDestinationConstraints)> {
        let constraints = self
            .config
            .parking_spots
            .iter()
            .map(parking_constraints)
            .find(|constraints| self.currently_occupied.is_constraints_free(constraints))?;
        let session = self.next_detour_session();
        self.currently_occupied
            .reserve_constraints(agent, &session, &constraints);
        Some((session, constraints))
    }

    pub(super) fn request(&mut self, agent: &str, goal: DomainDestinationGoal) -> Vec<Outcome> {
        let mut outcomes = Vec::new();

        if !goal.cost_bias.is_empty() && goal.one_of.len() != goal.cost_bias.len() {
            outcomes.push(Outcome::Error {
                agent: agent.to_string(),
                error: DomainDestinationError {
                    error: DomainError {
                        code: 0, //TODO(arjoc): reserve a code,
                        message: "Number of goals and destinations do not match".into(),
                        parameters: "TODO".into(),
                    },
                    session: goal.session,
                },
            });
            return outcomes;
        }

        if goal.one_of.is_empty() {
            outcomes.push(Outcome::Error {
                agent: agent.to_string(),
                error: DomainDestinationError {
                    error: DomainError {
                        code: 0,
                        message: "No goals provided".into(),
                        parameters: "one_of is empty".into(),
                    },
                    session: goal.session,
                },
            });
            return outcomes;
        }

        let mut options = goal.one_of;
        options.sort_by_key(|c| c.regions.len());

        if self.config.enforces_safe_sets() {
            for constraints in &options {
                for target_region in &constraints.regions {
                    let region = &target_region.region;
                    if !self
                        .config
                        .is_region_within_safe_sets(region.hint, &region.points)
                    {
                        outcomes.push(Outcome::Error {
                            agent: agent.to_string(),
                            error: DomainDestinationError {
                                error: DomainError {
                                    code: DestinationError::CODE_UNREACHABLE,
                                    message: "Requested region is outside the predefined safe sets"
                                        .into(),
                                    parameters: format!("{{\"points\": {:?}}}", region.points),
                                },
                                session: goal.session,
                            },
                        });
                        return outcomes;
                    }
                }
            }
        }

        let previous_state = self.agents.remove(agent);
        self.currently_occupied.release_agent(agent);

        if let Some(id) = self.currently_occupied.first_free_option(&options) {
            let constraints = options[id].clone();
            self.currently_occupied
                .reserve_constraints(agent, &goal.session, &constraints);
            self.agents.insert(
                agent.to_string(),
                AgentState {
                    desired_options: options,
                    goal_session: goal.session,
                    occupied: OccupiedReservation {
                        constraints: constraints.clone(),
                        session: goal.session,
                    },
                },
            );
            self.queue.retain(|a| a != agent);
            outcomes.push(Outcome::Reserve {
                agent: agent.to_string(),
                destination: DomainDestination {
                    constraints,
                    session: goal.session,
                    detour_for_goal: None,
                },
            });
        } else if let Some((detour_session, constraints)) = self.reserve_free_parking(agent) {
            if !self.queue.iter().any(|a| a == agent) {
                self.queue.push(agent.to_string());
            }
            self.agents.insert(
                agent.to_string(),
                AgentState {
                    desired_options: options,
                    goal_session: goal.session,
                    occupied: OccupiedReservation {
                        constraints: constraints.clone(),
                        session: detour_session,
                    },
                },
            );
            outcomes.push(Outcome::Reserve {
                agent: agent.to_string(),
                destination: DomainDestination {
                    constraints,
                    session: detour_session,
                    detour_for_goal: Some(goal.session),
                },
            });
        } else {
            if let Some(previous_state) = previous_state {
                self.currently_occupied.reserve_constraints(
                    agent,
                    &previous_state.occupied.session,
                    &previous_state.occupied.constraints,
                );
                self.agents.insert(agent.to_string(), previous_state);
            }
            outcomes.push(Outcome::Error {
                agent: agent.to_string(),
                error: DomainDestinationError {
                    error: DomainError {
                        code: 0,
                        message: "No available constraints".into(),
                        parameters: "no goal or parking spot available".into(),
                    },
                    session: goal.session,
                },
            });
        }

        // Releasing the old spot may have opened a goal for someone else.
        self.advance_queue(&mut outcomes);
        outcomes
    }

    pub(super) fn remove(&mut self, agent: &str) -> Vec<Outcome> {
        let mut outcomes = Vec::new();
        self.currently_occupied.release_agent(agent);
        self.agents.remove(agent);
        self.queue.retain(|a| a != agent);
        self.advance_queue(&mut outcomes);
        outcomes
    }

    // Keep going because moving one agent can free a goal for the next.
    fn advance_queue(&mut self, outcomes: &mut Vec<Outcome>) {
        loop {
            let mut progressed = false;
            for agent in self.queue.clone() {
                let Some(state) = self.agents.get(&agent) else {
                    continue;
                };
                let Some(id) = self
                    .currently_occupied
                    .first_free_option(&state.desired_options)
                else {
                    continue;
                };
                let constraints = state.desired_options[id].clone();
                let goal_session = state.goal_session;

                self.currently_occupied.release_agent(&agent);
                self.currently_occupied
                    .reserve_constraints(&agent, &goal_session, &constraints);
                if let Some(state) = self.agents.get_mut(&agent) {
                    state.occupied = OccupiedReservation {
                        constraints: constraints.clone(),
                        session: goal_session,
                    };
                }
                self.queue.retain(|a| a != &agent);
                outcomes.push(Outcome::Reserve {
                    agent: agent.clone(),
                    destination: DomainDestination {
                        constraints,
                        session: goal_session,
                        detour_for_goal: None,
                    },
                });
                progressed = true;
            }
            if !progressed {
                break;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::{ConfigRegion, SafeSet};
    use ros_env::rmf_prototype_msgs::msg::Region;

    fn queueing_config() -> Arc<ReservationConfig> {
        Arc::new(ReservationConfig {
            grid_size: 0.5,
            safe_sets: vec![SafeSet {
                name: "main_floor".to_string(),
                region: ConfigRegion {
                    hint: Region::HINT_AXIS_ALIGNED_RECTANGLE,
                    points: vec![0.0, 0.0, 20.0, 20.0],
                },
            }],
            parking_spots: vec![
                ParkingSpot {
                    name: "parking_a".to_string(),
                    region: ConfigRegion {
                        hint: Region::HINT_AXIS_ALIGNED_RECTANGLE,
                        points: vec![0.5, 0.5, 1.5, 1.5],
                    },
                },
                ParkingSpot {
                    name: "parking_b".to_string(),
                    region: ConfigRegion {
                        hint: Region::HINT_AXIS_ALIGNED_RECTANGLE,
                        points: vec![17.5, 17.5, 18.5, 18.5],
                    },
                },
            ],
        })
    }

    fn goal_for_region(session_byte: u8, points: Vec<f32>) -> DomainDestinationGoal {
        DomainDestinationGoal {
            one_of: vec![DomainDestinationConstraints {
                regions: vec![DomainTargetRegion {
                    tolerance: 0.0,
                    region: DomainRegion {
                        hint: DomainRegion::HINT_AXIS_ALIGNED_RECTANGLE,
                        points,
                    },
                }],
            }],
            cost_bias: vec![],
            session: SessionUUID {
                uuid: [session_byte; 16],
            },
        }
    }

    fn make_goal(session_byte: u8, x: f32, y: f32, size: f32) -> DomainDestinationGoal {
        goal_for_region(session_byte, vec![x, y, x + size, y + size])
    }

    fn reserved_for<'a>(outcomes: &'a [Outcome], agent: &str) -> Option<&'a DomainDestination> {
        outcomes.iter().find_map(|o| match o {
            Outcome::Reserve {
                agent: a,
                destination,
            } if a == agent => Some(destination),
            _ => None,
        })
    }

    fn errored_for<'a>(outcomes: &'a [Outcome], agent: &str) -> Option<&'a DomainDestinationError> {
        outcomes.iter().find_map(|o| match o {
            Outcome::Error { agent: a, error } if a == agent => Some(error),
            _ => None,
        })
    }

    #[test]
    fn goal_is_reserved_directly_when_free() {
        let mut state = ReservationState::new(queueing_config());
        let outcomes = state.request("robot_1", make_goal(1, 10.0, 10.0, 1.0));

        let dest = reserved_for(&outcomes, "robot_1").expect("robot_1 should get a destination");
        assert_eq!(dest.detour_for_goal, None);
        assert_eq!(dest.session, SessionUUID { uuid: [1; 16] });
    }

    #[test]
    fn second_robot_diverted_for_negative_goal() {
        let config = Arc::new(ReservationConfig {
            grid_size: 0.5,
            safe_sets: vec![SafeSet {
                name: "floor".to_string(),
                region: ConfigRegion {
                    hint: Region::HINT_AXIS_ALIGNED_RECTANGLE,
                    points: vec![-10.0, -10.0, 10.0, 10.0],
                },
            }],
            parking_spots: vec![ParkingSpot {
                name: "p_a".to_string(),
                region: ConfigRegion {
                    hint: Region::HINT_AXIS_ALIGNED_RECTANGLE,
                    points: vec![-8.5, -8.5, -7.5, -7.5],
                },
            }],
        });
        let mut state = ReservationState::new(config);

        let _ = state.request("robot_1", make_goal(1, -5.0, -5.0, 1.0));
        let outcomes = state.request("robot_2", make_goal(2, -5.0, -5.0, 1.0));

        let dest = reserved_for(&outcomes, "robot_2").expect("robot_2 should be diverted");
        assert_eq!(dest.detour_for_goal, Some(SessionUUID { uuid: [2; 16] }));
        assert!(state.queue.contains(&"robot_2".to_string()));
    }

    #[test]
    fn second_robot_is_diverted_to_parking() {
        let mut state = ReservationState::new(queueing_config());
        let _ = state.request("robot_1", make_goal(1, 10.0, 10.0, 1.0));
        let outcomes = state.request("robot_2", make_goal(2, 10.0, 10.0, 1.0));

        let dest = reserved_for(&outcomes, "robot_2").expect("robot_2 should be diverted");
        assert_eq!(dest.detour_for_goal, Some(SessionUUID { uuid: [2; 16] }));
        assert_ne!(dest.session, SessionUUID { uuid: [2; 16] });
        assert!(state.queue.contains(&"robot_2".to_string()));
    }

    #[test]
    fn queued_robot_advances_when_goal_frees_up() {
        let mut state = ReservationState::new(queueing_config());
        let goal_points = (10.0, 10.0, 1.0);

        let _ = state.request(
            "robot_1",
            make_goal(1, goal_points.0, goal_points.1, goal_points.2),
        );
        let _ = state.request(
            "robot_2",
            make_goal(2, goal_points.0, goal_points.1, goal_points.2),
        );
        assert!(state.queue.contains(&"robot_2".to_string()));

        let outcomes = state.request("robot_1", make_goal(3, 5.0, 5.0, 1.0));

        let r1 = reserved_for(&outcomes, "robot_1").expect("robot_1 should move");
        assert_eq!(r1.detour_for_goal, None);

        let r2 = reserved_for(&outcomes, "robot_2").expect("robot_2 should advance to its goal");
        assert_eq!(r2.detour_for_goal, None);
        assert_eq!(r2.session, SessionUUID { uuid: [2; 16] });
        assert!(!state.queue.contains(&"robot_2".to_string()));
    }

    #[test]
    fn robots_swapping_places_resolve_via_queue() {
        let mut state = ReservationState::new(queueing_config());
        let _ = state.request("robot_1", make_goal(1, 10.0, 10.0, 1.0));
        let _ = state.request("robot_2", make_goal(2, 5.0, 5.0, 1.0));

        let outcomes = state.request("robot_1", make_goal(3, 5.0, 5.0, 1.0));
        assert_eq!(
            reserved_for(&outcomes, "robot_1").unwrap().detour_for_goal,
            Some(SessionUUID { uuid: [3; 16] })
        );

        let outcomes = state.request("robot_2", make_goal(4, 10.0, 10.0, 1.0));
        let r2 = reserved_for(&outcomes, "robot_2").expect("robot_2 takes A");
        assert_eq!(r2.detour_for_goal, None);
        let r1 = reserved_for(&outcomes, "robot_1").expect("robot_1 advances to B");
        assert_eq!(r1.detour_for_goal, None);
        assert_eq!(r1.session, SessionUUID { uuid: [3; 16] });
        assert!(state.queue.is_empty());
    }

    #[test]
    fn diversion_only_happens_when_parking_is_configured() {
        let mut state = ReservationState::new(Arc::new(ReservationConfig::default()));
        let _ = state.request("robot_1", make_goal(1, 10.0, 10.0, 1.0));
        let outcomes = state.request("robot_2", make_goal(2, 10.5, 10.5, 1.0));

        assert!(reserved_for(&outcomes, "robot_2").is_none());
        assert!(errored_for(&outcomes, "robot_2").is_some());
    }

    #[test]
    fn failed_replacement_preserves_previous_reservation() {
        let mut state = ReservationState::new(Arc::new(ReservationConfig::default()));
        let old_goal = make_goal(1, 2.0, 2.0, 1.0);
        let old_constraints = old_goal.one_of[0].clone();
        let _ = state.request("robot_1", old_goal);
        let _ = state.request("robot_2", make_goal(2, 10.0, 10.0, 1.0));

        let outcomes = state.request("robot_1", make_goal(3, 10.0, 10.0, 1.0));

        assert!(errored_for(&outcomes, "robot_1").is_some());
        assert!(!state
            .currently_occupied
            .is_constraints_free(&old_constraints));
        assert_eq!(
            state.currently_occupied.agent_to_session.get("robot_1"),
            Some(&SessionUUID { uuid: [1; 16] })
        );
    }

    #[test]
    fn leaving_robot_frees_its_goal_for_the_queue() {
        let mut state = ReservationState::new(queueing_config());
        let _ = state.request("robot_1", make_goal(1, 10.0, 10.0, 1.0));
        let _ = state.request("robot_2", make_goal(2, 10.0, 10.0, 1.0));

        let outcomes = state.remove("robot_1");
        let r2 = reserved_for(&outcomes, "robot_2").expect("robot_2 should advance");
        assert_eq!(r2.detour_for_goal, None);
        assert!(state.queue.is_empty());
    }

    #[test]
    fn transitivity_preserved_when_occupied_spot_is_a_diversion() {
        let mut state = ReservationState::new(queueing_config());

        // Robot 1 requests A
        let a = state.request("robot_1", make_goal(1, 10.0, 10.0, 1.0));
        let r1 = reserved_for(&a, "robot_1").expect("robot_1 takes A");
        assert_eq!(r1.detour_for_goal, None);
        let spot_a = r1.constraints.regions[0].region.points.clone();

        // Robot 2 requests A => Robot 2 is diverted to B
        let outcomes = state.request("robot_2", make_goal(2, 10.0, 10.0, 1.0));
        let r2_diversion = reserved_for(&outcomes, "robot_2").expect("robot_2 diverted");
        assert_eq!(
            r2_diversion.detour_for_goal,
            Some(SessionUUID { uuid: [2; 16] })
        );
        let spot_b = r2_diversion.constraints.regions[0].region.points.clone();
        assert!(state.queue.contains(&"robot_2".to_string()));

        // Robot 3 requests B => Robot 3 is diverted elsewhere
        let outcomes = state.request("robot_3", goal_for_region(3, spot_b.clone()));
        let r3_diversion = reserved_for(&outcomes, "robot_3").expect("robot_3 diverted");
        assert_eq!(
            r3_diversion.detour_for_goal,
            Some(SessionUUID { uuid: [3; 16] })
        );
        assert_ne!(r3_diversion.constraints.regions[0].region.points, spot_b);
        assert!(state.queue.contains(&"robot_3".to_string()));

        // Robot 1 moves out of A
        let outcomes = state.request("robot_1", make_goal(4, 8.0, 8.0, 1.0));

        // Robot 2 and Robot 3 move into A and B respectively.
        let r2 = reserved_for(&outcomes, "robot_2").expect("robot_2 advances to A");
        assert_eq!(r2.detour_for_goal, None);
        assert_eq!(r2.session, SessionUUID { uuid: [2; 16] });
        assert_eq!(r2.constraints.regions[0].region.points, spot_a);
        let r3 = reserved_for(&outcomes, "robot_3").expect("robot_3 advances to B");
        assert_eq!(r3.detour_for_goal, None);
        assert_eq!(r3.session, SessionUUID { uuid: [3; 16] });
        assert_eq!(r3.constraints.regions[0].region.points, spot_b);

        assert!(state.queue.is_empty());
    }

    #[test]
    fn three_robots_same_goal_get_unique_parking_and_complete_in_turn() {
        // Three robots request the same goal A. Only the first is awarded it.
        // The remaining two are diverted to distinct parking spots. As each holder
        // of A leaves, the next queued robot advances into A, so every robot
        // eventually completes its original request.
        let mut state = ReservationState::new(queueing_config());

        // Robot 1 requests A and is awarded it
        let r1 = state.request("robot_1", make_goal(1, 10.0, 10.0, 1.0));
        let r1_dest = reserved_for(&r1, "robot_1").expect("robot_1 takes A");
        assert_eq!(r1_dest.detour_for_goal, None);
        let spot_a = r1_dest.constraints.regions[0].region.points.clone();

        // Robot 2 requests A => Robot 2 is diverted to a parking spot
        let r2 = state.request("robot_2", make_goal(2, 10.0, 10.0, 1.0));
        let r2_diversion = reserved_for(&r2, "robot_2").expect("robot_2 diverted");
        assert_eq!(
            r2_diversion.detour_for_goal,
            Some(SessionUUID { uuid: [2; 16] })
        );
        let r2_spot = r2_diversion.constraints.regions[0].region.points.clone();

        // Robot 3 requests A => Robot 3 is diverted to a different parking spot
        let r3 = state.request("robot_3", make_goal(3, 10.0, 10.0, 1.0));
        let r3_diversion = reserved_for(&r3, "robot_3").expect("robot_3 diverted");
        assert_eq!(
            r3_diversion.detour_for_goal,
            Some(SessionUUID { uuid: [3; 16] })
        );
        let r3_spot = r3_diversion.constraints.regions[0].region.points.clone();

        // The two diverted robots rest away from A and in distinct spots.
        assert_ne!(r2_spot, spot_a);
        assert_ne!(r3_spot, spot_a);
        assert_ne!(r2_spot, r3_spot);
        assert!(state.queue.contains(&"robot_2".to_string()));
        assert!(state.queue.contains(&"robot_3".to_string()));

        // Robot 1 leaves A, robot 2 advances into it, robot 3 keeps waiting.
        let outcomes = state.remove("robot_1");
        let r2 = reserved_for(&outcomes, "robot_2").expect("robot_2 advances to A");
        assert_eq!(r2.detour_for_goal, None);
        assert_eq!(r2.session, SessionUUID { uuid: [2; 16] });
        assert!(reserved_for(&outcomes, "robot_3").is_none());
        assert!(state.queue.contains(&"robot_3".to_string()));

        // Robot 2 leaves A, robot 3 completes its request.
        let outcomes = state.remove("robot_2");
        let r3 = reserved_for(&outcomes, "robot_3").expect("robot_3 advances to A");
        assert_eq!(r3.detour_for_goal, None);
        assert_eq!(r3.session, SessionUUID { uuid: [3; 16] });
        assert!(state.queue.is_empty());
    }
}
