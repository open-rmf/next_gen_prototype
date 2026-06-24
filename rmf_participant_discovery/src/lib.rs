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

use rclrs::IntoPrimitiveOptions;
use ros_env::rmf_prototype_msgs::msg::ParticipantList;
use std::collections::HashSet;

#[derive(Debug, Clone, Default)]
pub struct ParticipantTracker {
    active_participants: HashSet<String>,
}

impl ParticipantTracker {
    /// Creates a new empty participant tracker.
    pub fn new() -> Self {
        Self {
            active_participants: HashSet::new(),
        }
    }

    /// Updates the tracker with the latest ParticipantList.
    /// Returns a tuple of `(added, removed)` participant names.
    pub fn update(&mut self, msg: &ParticipantList) -> (Vec<String>, Vec<String>) {
        let incoming_participants: HashSet<String> =
            msg.participants.iter().map(|p| p.name.clone()).collect();

        // Find newly added participants: present in incoming, not in active_participants
        let added: Vec<String> = incoming_participants
            .iter()
            .filter(|name| !self.active_participants.contains(*name))
            .cloned()
            .collect();

        // Find removed participants: present in active_participants, not in incoming
        let removed: Vec<String> = self
            .active_participants
            .iter()
            .filter(|name| !incoming_participants.contains(*name))
            .cloned()
            .collect();

        // Apply updates to the stored set
        self.active_participants = incoming_participants;

        (added, removed)
    }

    /// Checks if a given participant is currently active.
    pub fn is_active(&self, name: &str) -> bool {
        self.active_participants.contains(name)
    }

    /// Returns a reference to the set of currently active participants.
    pub fn active_participants(&self) -> &HashSet<String> {
        &self.active_participants
    }
}

/// Helper to subscribe to a dynamic participant discovery topic.
/// It tracks additions and removals using `ParticipantTracker` under the hood
/// and invokes the supplied callback functions when those events occur.
pub fn create_discovery_subscription<T, AddFn, RemFn>(
    worker: &rclrs::Worker<T>,
    topic: &str,
    mut on_added: AddFn,
    mut on_removed: RemFn,
) -> Result<rclrs::WorkerSubscription<ParticipantList, T>, rclrs::RclrsError>
where
    T: Send + Sync + 'static,
    AddFn: FnMut(&mut T, &str) + Send + 'static,
    RemFn: FnMut(&mut T, &str) + Send + 'static,
{
    let mut tracker = ParticipantTracker::new();
    worker.create_subscription::<ParticipantList, _>(
        topic.transient_local().reliable().keep_all(),
        move |server: &mut T, msg: ParticipantList| {
            let (added, removed) = tracker.update(&msg);

            for robot_id in removed {
                on_removed(server, &robot_id);
            }

            for robot_id in added {
                on_added(server, &robot_id);
            }
        },
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use ros_env::rmf_prototype_msgs::msg::Participant;

    #[test]
    fn test_participant_tracker() {
        let mut tracker = ParticipantTracker::new();
        assert!(tracker.active_participants().is_empty());

        // 1. Add participants
        let msg1 = ParticipantList {
            participants: vec![
                Participant {
                    name: "robot_1".to_string(),
                    components: vec![],
                },
                Participant {
                    name: "robot_2".to_string(),
                    components: vec![],
                },
            ],
        };

        let (added, removed) = tracker.update(&msg1);
        assert_eq!(added.len(), 2);
        assert!(added.contains(&"robot_1".to_string()));
        assert!(added.contains(&"robot_2".to_string()));
        assert!(removed.is_empty());

        assert!(tracker.is_active("robot_1"));
        assert!(tracker.is_active("robot_2"));
        assert!(!tracker.is_active("robot_3"));

        // 2. Add one, remove one
        let msg2 = ParticipantList {
            participants: vec![
                Participant {
                    name: "robot_2".to_string(),
                    components: vec![],
                },
                Participant {
                    name: "robot_3".to_string(),
                    components: vec![],
                },
            ],
        };

        let (added, removed) = tracker.update(&msg2);
        assert_eq!(added, vec!["robot_3".to_string()]);
        assert_eq!(removed, vec!["robot_1".to_string()]);

        assert!(!tracker.is_active("robot_1"));
        assert!(tracker.is_active("robot_2"));
        assert!(tracker.is_active("robot_3"));
    }
}
