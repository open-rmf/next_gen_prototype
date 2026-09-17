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

use rclrs::{IntoPrimitiveOptions, MessageInfo};
use ros_env::rmf_prototype_msgs::msg::ParticipantList;
use std::collections::{HashMap, HashSet};

/// Identifies the publisher a roster came from.
///
/// This is the RMW publisher GID, carried as raw bytes rather than as
/// [`rclrs::PublisherGid`] because that type's `implementation_identifier` is a
/// `*const c_char` and takes part in its `Hash` and `Eq`. Hashing a raw pointer
/// would make the key depend on where a string happens to live rather than on
/// which publisher spoke.
pub type PublisherKey = Vec<u8>;

/// The publisher a message came from, in the form [`ParticipantTracker`] wants.
pub fn publisher_key(info: &MessageInfo) -> PublisherKey {
    info.publisher_gid.data.to_vec()
}

#[derive(Debug, Clone, Default)]
pub struct ParticipantTracker {
    /// The roster each publisher most recently announced.
    ///
    /// A `ParticipantList` describes the participants of the node that sent it,
    /// not the participants of the whole system. Treating one as the global
    /// truth means that with two publishers each message silently retires the
    /// other's robots, and the two then flap against each other forever, once
    /// per message. Keeping the rosters apart makes a roster able to retire
    /// only what that same publisher previously claimed.
    rosters: HashMap<PublisherKey, HashSet<String>>,
    /// The union of every roster, cached so that lookups stay cheap.
    active_participants: HashSet<String>,
}

impl ParticipantTracker {
    /// Creates a new empty participant tracker.
    pub fn new() -> Self {
        Self::default()
    }

    /// Updates the tracker with the latest ParticipantList from `publisher`.
    /// Returns a tuple of `(added, removed)` participant names.
    ///
    /// Both are reported against the union of all known rosters, so a
    /// participant counts as added only when nobody was already claiming it,
    /// and as removed only once nobody claims it at all.
    ///
    /// A publisher that dies without emptying its roster first leaves its
    /// participants active indefinitely. That is not new -- silence was never
    /// distinguishable from "nothing changed" here -- and it fails in the
    /// direction that keeps a robot visible to the planner rather than
    /// spuriously erasing it.
    pub fn update(
        &mut self,
        publisher: PublisherKey,
        msg: &ParticipantList,
    ) -> (Vec<String>, Vec<String>) {
        let incoming_participants: HashSet<String> =
            msg.participants.iter().map(|p| p.name.clone()).collect();
        self.rosters.insert(publisher, incoming_participants);

        let union: HashSet<String> = self.rosters.values().flatten().cloned().collect();

        let added: Vec<String> = union
            .difference(&self.active_participants)
            .cloned()
            .collect();
        let removed: Vec<String> = self
            .active_participants
            .difference(&union)
            .cloned()
            .collect();

        self.active_participants = union;

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
        topic.transient_local().reliable().keep_last(10),
        move |server: &mut T, msg: ParticipantList, info: MessageInfo| {
            let (added, removed) = tracker.update(publisher_key(&info), &msg);

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

    /// Distinct publisher keys. The real ones are RMW GIDs; only their
    /// inequality matters here.
    const PUB_A: &[u8] = &[1];
    const PUB_B: &[u8] = &[2];

    fn roster(names: &[&str]) -> ParticipantList {
        ParticipantList {
            participants: names
                .iter()
                .map(|name| Participant {
                    name: name.to_string(),
                    components: vec![],
                })
                .collect(),
        }
    }

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

        let (added, removed) = tracker.update(PUB_A.to_vec(), &msg1);
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

        let (added, removed) = tracker.update(PUB_A.to_vec(), &msg2);
        assert_eq!(added, vec!["robot_3".to_string()]);
        assert_eq!(removed, vec!["robot_1".to_string()]);

        assert!(!tracker.is_active("robot_1"));
        assert!(tracker.is_active("robot_2"));
        assert!(tracker.is_active("robot_3"));
    }

    /// Two publishers, each announcing only its own robots, must not retire
    /// each other's.
    ///
    /// This is the regression guard for the flapping that made four path server
    /// integration tests time out: every roster was read as the global truth,
    /// so each message removed the other publisher's robots and the two then
    /// added and removed each other forever.
    #[test]
    fn rosters_from_different_publishers_do_not_retire_each_other() {
        let mut tracker = ParticipantTracker::new();

        let (added, removed) = tracker.update(PUB_A.to_vec(), &roster(&["robot_a"]));
        assert_eq!(added, vec!["robot_a".to_string()]);
        assert!(removed.is_empty());

        let (added, removed) = tracker.update(PUB_B.to_vec(), &roster(&["robot_b"]));
        assert_eq!(added, vec!["robot_b".to_string()]);
        assert!(
            removed.is_empty(),
            "publisher B's roster must not retire publisher A's robot"
        );

        // Repeat both rosters. Nothing has changed, so nothing should be
        // reported: this is the step that used to flap.
        let (added, removed) = tracker.update(PUB_A.to_vec(), &roster(&["robot_a"]));
        assert!(added.is_empty());
        assert!(removed.is_empty());

        let (added, removed) = tracker.update(PUB_B.to_vec(), &roster(&["robot_b"]));
        assert!(added.is_empty());
        assert!(removed.is_empty());

        assert!(tracker.is_active("robot_a"));
        assert!(tracker.is_active("robot_b"));
    }

    /// A publisher dropping a robot someone else still claims is not a removal.
    #[test]
    fn a_robot_is_only_removed_once_no_publisher_claims_it() {
        let mut tracker = ParticipantTracker::new();

        tracker.update(PUB_A.to_vec(), &roster(&["shared"]));
        let (added, removed) = tracker.update(PUB_B.to_vec(), &roster(&["shared"]));
        assert!(
            added.is_empty(),
            "a second claim on a known robot is not an addition"
        );
        assert!(removed.is_empty());

        let (_, removed) = tracker.update(PUB_A.to_vec(), &roster(&[]));
        assert!(removed.is_empty(), "publisher B still claims it");
        assert!(tracker.is_active("shared"));

        let (_, removed) = tracker.update(PUB_B.to_vec(), &roster(&[]));
        assert_eq!(removed, vec!["shared".to_string()]);
        assert!(!tracker.is_active("shared"));
    }
}
