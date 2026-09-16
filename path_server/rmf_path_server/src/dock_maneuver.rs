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

//! Shaping of raw MAPF trajectories before conflict analysis.
//!
//! This module does two things, both purely to the *geometry* of the
//! trajectories. It computes no dependencies: `mapf_post` builds the whole ADG.
//!
//! 1. [`splice_dock_maneuvers`] appends the dock lane and prepends the undock
//!    lane, so the space a docking robot occupies is present in the trajectory.
//! 2. [`pad_to_equal_length`] extends every trajectory to the longest one, so a
//!    robot that finishes early keeps occupying its goal instead of vanishing.
//!
//! # Why splicing is needed
//!
//! A dock is not a point on the planning grid. `PibtPlanner` discretizes the
//! world at `MIN_PLANNING_RESOLUTION` (1.0 m), whereas a dock lane in the site is
//! typically 0.5 m long. Historically this was worked around by *hiding* the
//! maneuver from the planner: the start pose of a docked robot was teleported out
//! to the staging vertex, and the dock itself was reduced to a string stapled
//! onto the last waypoint after conflict analysis had finished. `mapf_post`
//! therefore never saw the space a docking robot occupies.
//!
//! It does not need the grid to. `mapf_post` derives its Type-2 dependencies by
//! sweeping the AABB between *consecutive poses of a trajectory*, so poses may be
//! spliced in at arbitrary sub-cell offsets. We decorate the planner's output;
//! the grid stays coarse and the trajectory becomes honest.
//!
//! # Why there is no dwell
//!
//! An earlier version modelled the dock as the dock pose repeated N times, with N
//! derived from the `duration` declared on the site's dock lane. That was wrong
//! in a way worth recording, because it is an attractive mistake:
//!
//! * The duration is a guess. Dock and undock take different amounts of time,
//!   neither is known in advance, and nav2 may retry internally.
//! * Worse, **a dwell built from repeated poses is unobservable.** Progress is
//!   reported by projecting the robot's pose onto its plan, and no pose can
//!   distinguish dwell step 2 from dwell step 3. The reported progress pinned
//!   itself to the first of the repeats and never advanced, so peers holding
//!   traffic dependencies against the later repeats waited forever.
//!
//! The real quantity was never duration, it was *completion*, which the robot
//! reports at runtime. So the dock is one segment appended after the plan ends.
//!
//! # Why there is no hand-built dock gate
//!
//! A second version added a `dock_entry_gate` that computed, for each peer
//! sweeping the dock corridor, a traffic dependency forcing the docking robot to
//! yield. The justification was that `mapf_post` orders conflicts by trajectory
//! index, so a peer crossing the corridor at an index *after* the dock would be
//! told to wait for a robot that never moves again.
//!
//! That justification was right about the symptom and wrong about the fix.
//! `mapf_post` emits `later` waits for `earlier`, so in that case the peer
//! already has a blocker against the docker. Adding the reverse edge closes a
//! cycle and deadlocks both robots, which is strictly worse than the
//! over-permissive behaviour it was trying to correct. In the opposite case
//! (peer crosses first) `mapf_post` already produces exactly the intended gate,
//! so the extra edge was redundant there too.
//!
//! The real gap was that a parked robot contributes no segments past the end of
//! its trajectory and so becomes invisible. [`pad_to_equal_length`] fixes that at
//! the input, where it belongs, and lets `mapf_post` reach its own conclusions.

use mapf_post::na::Isometry2;

/// Two poses closer than this (in metres) are treated as the same place when
/// deciding whether an intermediate waypoint is worth emitting.
const POSE_EPS: f32 = 1e-3;

/// The undock half of a maneuver: what the planner was not told about the
/// robot's real starting position.
///
/// There is no duration here. The undock occupies exactly one segment of the
/// trajectory, which is enough for the conflict analyser to see the corridor;
/// how long the robot actually spends in it is reported at runtime, not planned.
#[derive(Clone, Debug, PartialEq)]
pub struct UndockPrefix {
    /// Where the robot physically is right now, i.e. its odometry before it was
    /// teleported out to the staging vertex for the benefit of the grid planner.
    pub docked_pose: Isometry2<f32>,
    /// The dock vertex the robot must pass back through on its way out, if that
    /// is a distinct place from `docked_pose`. When the robot is parked exactly
    /// on the dock vertex this is redundant and will be dropped.
    pub via: Option<Isometry2<f32>>,
}

/// The dock half of a maneuver: a single segment appended after the planner's
/// last pose, running from the pre-dock (staging) vertex into the dock vertex.
#[derive(Clone, Debug, PartialEq)]
pub struct DockSuffix {
    /// Pose of the dock vertex itself. The planner's goal is the staging vertex,
    /// so this is strictly beyond the end of the planned path.
    pub dock_pose: Isometry2<f32>,
}

/// Everything that needs splicing into one agent's trajectory.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct DockManeuvers {
    /// Present when the robot begins the plan physically docked.
    pub undock: Option<UndockPrefix>,
    /// Present when the robot's destination resolved to a dock.
    pub dock: Option<DockSuffix>,
}

impl DockManeuvers {
    /// True when there is nothing to splice, so callers can skip the work.
    pub fn is_empty(&self) -> bool {
        self.undock.is_none() && self.dock.is_none()
    }

    /// How many poses `splice` will prepend. Callers need this to keep indices
    /// computed against the raw planner output pointing at the same waypoints.
    pub fn prefix_len(&self, first_planned_pose: Option<&Isometry2<f32>>) -> usize {
        match &self.undock {
            None => 0,
            Some(undock) => 1 + usize::from(undock.emits_via(first_planned_pose)),
        }
    }
}

impl UndockPrefix {
    /// Whether the `via` pose is distinct enough from both the docked pose and
    /// the planner's first pose to be worth emitting.
    fn emits_via(&self, first_planned_pose: Option<&Isometry2<f32>>) -> bool {
        let Some(via) = &self.via else {
            return false;
        };
        if same_place(via, &self.docked_pose) {
            return false;
        }
        match first_planned_pose {
            Some(first) => !same_place(via, first),
            None => true,
        }
    }
}

fn same_place(a: &Isometry2<f32>, b: &Isometry2<f32>) -> bool {
    (a.translation.vector - b.translation.vector).norm() <= POSE_EPS
}

/// Splice `maneuvers` into `traj` in place, returning the index of the dock
/// waypoint if one ended up in the trajectory.
///
/// The result is laid out as:
///
/// ```text
/// [docked_pose] [via?] <planner output, ending at the staging vertex> [dock_pose]
/// ```
///
/// A no-op on an empty trajectory, since there is nothing to anchor the splice
/// to.
pub fn splice_dock_maneuvers(
    traj: &mut Vec<Isometry2<f32>>,
    maneuvers: &DockManeuvers,
) -> Option<usize> {
    if traj.is_empty() || maneuvers.is_empty() {
        return None;
    }

    let mut docked = false;
    if let Some(dock) = &maneuvers.dock {
        // Only append if the planner did not already land us on the dock pose,
        // which would make the maneuver a zero-length segment.
        if traj
            .last()
            .is_none_or(|last| !same_place(last, &dock.dock_pose))
        {
            traj.push(dock.dock_pose);
        }
        docked = true;
    }

    if let Some(undock) = &maneuvers.undock {
        let emits_via = undock.emits_via(traj.first());
        let mut prefix = Vec::with_capacity(1 + usize::from(emits_via));
        prefix.push(undock.docked_pose);
        if emits_via {
            prefix.push(undock.via.expect("emits_via implies via is Some"));
        }
        traj.splice(0..0, prefix);
    }

    // Whether or not the pose was appended, the dock is the final waypoint: the
    // planner's goal is the staging vertex just before it.
    docked.then(|| traj.len() - 1)
}

/// Extend every trajectory to the length of the longest by repeating its final
/// pose, and return that length.
///
/// # Why
///
/// `mapf_post` sweeps segments per agent, so an agent contributes nothing past
/// the end of its own trajectory. A robot whose plan is shorter than its peers'
/// therefore *disappears* from conflict analysis the moment it arrives, and a
/// peer routed across its goal is told the space is free. That has always been
/// wrong; docking merely makes it obvious, because a docking robot's plan ends
/// early by construction and it is emphatically still there.
///
/// Repeating the final pose states the truth: within this plan, the robot stays
/// where it stopped.
///
/// # What this costs
///
/// The padded waypoints are duplicates, so they are unobservable to the
/// projection-based progress report in the same way the old dwell was. A
/// dependency landing on one of them can never be satisfied, and the robot
/// holding it will wait indefinitely.
///
/// That is the correct outcome — it is waiting for space that genuinely never
/// frees up — but it is indistinguishable from a bug at runtime, and it is only
/// resolved properly by not planning peers through occupied goals in the first
/// place. Erring towards a visible stall is the right direction while that is
/// outstanding: the alternative is driving into a parked robot.
pub fn pad_to_equal_length(trajectories: &mut [Vec<Isometry2<f32>>]) -> usize {
    let target = trajectories.iter().map(|t| t.len()).max().unwrap_or(0);

    for traj in trajectories.iter_mut() {
        // An empty trajectory has no final pose to hold, and inventing one would
        // put a footprint somewhere the robot has never been claimed to be.
        let Some(&last) = traj.last() else {
            continue;
        };
        traj.resize(target, last);
    }

    target
}

/// Where dock and undock actions land in the `Plan` message.
///
/// Indices refer to the trajectory *after* splicing and padding, which is what
/// the `Plan` message is built from.
#[derive(Clone, Copy, Debug, Default)]
pub struct DockAnnotations<'a> {
    /// Action to run before departing waypoint 0, e.g. `"undock"`. Set whenever
    /// the plan begins with an undock prefix.
    pub departure_action: Option<&'a str>,
    /// Action to run on arrival at the dock, e.g. `"dock_conveyor_r1_c1"`.
    pub arrival_action: Option<&'a str>,
    /// Index of the dock waypoint, from [`splice_dock_maneuvers`].
    ///
    /// Not simply the last waypoint: padding may sit beyond it, and the action
    /// must fire when the robot *arrives*, not on each padded repeat.
    pub dock_index: Option<usize>,
    /// Index of the waypoint the destination refers to.
    ///
    /// The last waypoint the robot was actually planned to, which is neither the
    /// dock (a separate commitment beyond the planner's goal) nor the padding.
    pub destination_index: usize,
}

#[cfg(test)]
mod tests {
    use super::*;

    fn p(x: f32, y: f32) -> Isometry2<f32> {
        Isometry2::translation(x, y)
    }

    #[test]
    fn no_maneuvers_leaves_trajectory_untouched() {
        let mut traj = vec![p(0.0, 0.0), p(1.0, 0.0)];
        let before = traj.clone();
        assert_eq!(
            splice_dock_maneuvers(&mut traj, &DockManeuvers::default()),
            None
        );
        assert_eq!(traj, before);
    }

    #[test]
    fn empty_trajectory_is_a_no_op() {
        let mut traj: Vec<Isometry2<f32>> = Vec::new();
        let dock_index = splice_dock_maneuvers(
            &mut traj,
            &DockManeuvers {
                undock: None,
                dock: Some(DockSuffix {
                    dock_pose: p(0.0, 1.5),
                }),
            },
        );
        assert!(traj.is_empty());
        assert_eq!(dock_index, None);
    }

    #[test]
    fn dock_is_appended_as_a_single_segment() {
        // The planner routes to the staging vertex (0, 2.0); the dock vertex
        // (0, 1.5) is beyond the end of the planned path.
        let mut traj = vec![p(0.0, 4.0), p(0.0, 3.0), p(0.0, 2.0)];
        let maneuvers = DockManeuvers {
            undock: None,
            dock: Some(DockSuffix {
                dock_pose: p(0.0, 1.5),
            }),
        };
        let dock_index = splice_dock_maneuvers(&mut traj, &maneuvers);

        assert_eq!(
            traj,
            vec![p(0.0, 4.0), p(0.0, 3.0), p(0.0, 2.0), p(0.0, 1.5)]
        );
        // No repeats: exactly one segment carries the maneuver.
        assert_eq!(dock_index, Some(3));
    }

    #[test]
    fn dock_is_not_appended_twice_if_the_planner_already_got_there() {
        let mut traj = vec![p(0.0, 2.0), p(0.0, 1.5)];
        let maneuvers = DockManeuvers {
            undock: None,
            dock: Some(DockSuffix {
                dock_pose: p(0.0, 1.5),
            }),
        };
        let dock_index = splice_dock_maneuvers(&mut traj, &maneuvers);
        assert_eq!(traj, vec![p(0.0, 2.0), p(0.0, 1.5)]);
        assert_eq!(dock_index, Some(1));
    }

    #[test]
    fn undock_prefix_passes_back_out_through_the_dock_vertex() {
        // Planner was handed the staging vertex (0, 2.0) as the start; the robot
        // is really sitting at the conveyor contact pose (0, 0.95).
        let mut traj = vec![p(0.0, 2.0), p(0.0, 3.0), p(0.0, 4.0)];
        let dock_index = splice_dock_maneuvers(
            &mut traj,
            &DockManeuvers {
                undock: Some(UndockPrefix {
                    docked_pose: p(0.0, 0.95),
                    via: Some(p(0.0, 1.5)),
                }),
                dock: None,
            },
        );
        assert_eq!(
            traj,
            vec![
                p(0.0, 0.95),
                p(0.0, 1.5),
                p(0.0, 2.0),
                p(0.0, 3.0),
                p(0.0, 4.0),
            ]
        );
        assert_eq!(dock_index, None);
    }

    #[test]
    fn undock_prefix_drops_a_redundant_via() {
        // Robot is parked exactly on the dock vertex, so `via` would be a
        // zero-length hop.
        let mut traj = vec![p(0.0, 2.0), p(0.0, 3.0)];
        splice_dock_maneuvers(
            &mut traj,
            &DockManeuvers {
                undock: Some(UndockPrefix {
                    docked_pose: p(0.0, 1.5),
                    via: Some(p(0.0, 1.5)),
                }),
                dock: None,
            },
        );
        assert_eq!(traj, vec![p(0.0, 1.5), p(0.0, 2.0), p(0.0, 3.0)]);
    }

    #[test]
    fn undock_and_dock_can_both_apply_to_one_plan() {
        // Undock from conveyor_r1_c1, drive across, dock at conveyor_r2_c3.
        let mut traj = vec![p(0.0, 2.0), p(2.5, 2.5), p(5.0, 3.0)];
        let maneuvers = DockManeuvers {
            undock: Some(UndockPrefix {
                docked_pose: p(0.0, 0.95),
                via: Some(p(0.0, 1.5)),
            }),
            dock: Some(DockSuffix {
                dock_pose: p(5.0, 3.5),
            }),
        };
        let prefix_len = maneuvers.prefix_len(traj.first());
        let dock_index = splice_dock_maneuvers(&mut traj, &maneuvers);

        assert_eq!(prefix_len, 2);
        assert_eq!(traj.len(), 2 + 3 + 1);
        assert_eq!(traj[prefix_len], p(0.0, 2.0));
        assert_eq!(*traj.last().unwrap(), p(5.0, 3.5));
        // The prefix shifts the dock along, so this cannot be assumed from the
        // planner's output length alone.
        assert_eq!(dock_index, Some(5));
    }

    #[test]
    fn padding_holds_a_short_plan_at_its_final_pose() {
        let mut trajectories = vec![
            vec![p(0.0, 2.0), p(0.0, 1.5)],
            vec![p(4.0, 0.0), p(3.0, 0.0), p(2.0, 0.0), p(1.0, 0.0)],
        ];

        let len = pad_to_equal_length(&mut trajectories);

        assert_eq!(len, 4);
        assert_eq!(
            trajectories[0],
            vec![p(0.0, 2.0), p(0.0, 1.5), p(0.0, 1.5), p(0.0, 1.5)],
            "the docked robot must keep occupying the dock"
        );
        assert_eq!(trajectories[1].len(), 4, "the longest plan is untouched");
    }

    #[test]
    fn padding_leaves_an_empty_trajectory_empty() {
        // There is no final pose to hold, and inventing one would claim space on
        // behalf of a robot we know nothing about.
        let mut trajectories = vec![Vec::new(), vec![p(0.0, 0.0), p(1.0, 0.0)]];
        pad_to_equal_length(&mut trajectories);
        assert!(trajectories[0].is_empty());
    }

    #[test]
    fn padding_is_idempotent() {
        let mut trajectories = vec![vec![p(0.0, 0.0), p(1.0, 0.0)], vec![p(5.0, 5.0)]];
        pad_to_equal_length(&mut trajectories);
        let once = trajectories.clone();
        pad_to_equal_length(&mut trajectories);
        assert_eq!(trajectories, once);
    }

    /// Regression guard for the deadlock the dwell caused. Every progress level
    /// a plan asks a robot to reach *within its own motion* must correspond to a
    /// distinct pose, otherwise a position projection can never report it.
    ///
    /// This deliberately checks the spliced trajectory only. Padding does append
    /// repeats, but they represent a robot that has stopped for good rather than
    /// a step it is expected to complete.
    #[test]
    fn spliced_plans_contain_no_unobservable_progress_levels() {
        let mut traj = vec![p(0.0, 4.0), p(0.0, 3.0), p(0.0, 2.0)];
        splice_dock_maneuvers(
            &mut traj,
            &DockManeuvers {
                undock: Some(UndockPrefix {
                    docked_pose: p(0.0, 0.95),
                    via: Some(p(0.0, 1.5)),
                }),
                dock: Some(DockSuffix {
                    dock_pose: p(0.0, 1.5),
                }),
            },
        );

        for pair in traj.windows(2) {
            assert!(
                !same_place(&pair[0], &pair[1]),
                "consecutive identical poses make progress unobservable: {:?}",
                traj
            );
        }
    }

    /// The property padding exists for, measured through `mapf_post` itself.
    ///
    /// Agent 0 parks at the origin on its second waypoint. Agent 1 crawls east
    /// along `y = 0` and reaches that same spot on its last step, long after
    /// agent 0 stopped moving.
    ///
    /// Unpadded, agent 0 owns a single segment and agent 1 is handed **no
    /// dependency at all**: nothing in the graph mentions the two of them
    /// together, so agent 1 is free to drive straight into a robot that is
    /// sitting on its path. A robot that finishes early simply stops existing.
    ///
    /// Padded, agent 0 keeps contributing stationary segments at the origin and
    /// agent 1 is held against a waypoint agent 0 only reaches at the very end.
    /// That progress level is unobservable by design, so agent 1 waits
    /// indefinitely — which is the truth, because the space never frees up.
    #[test]
    fn padding_keeps_a_parked_robot_visible_to_conflict_analysis() {
        use mapf_post::{shape, MapfResult, SemanticWaypoint, Trajectory};
        use std::sync::Arc;

        let parked = vec![p(0.0, 2.0), p(0.0, 0.0)];
        let peer: Vec<Isometry2<f32>> = (0..7).map(|i| p(-6.0 + i as f32, 0.0)).collect();

        /// How far into agent 0's plan agent 1 is ever required to wait.
        fn wait_depth(trajectories: Vec<Vec<Isometry2<f32>>>) -> Option<usize> {
            let footprint: Arc<dyn shape::Shape> = Arc::new(shape::Ball::new(0.4));
            let peer_len = trajectories[1].len();
            let plan = mapf_post::mapf_post(&MapfResult {
                trajectories: trajectories
                    .into_iter()
                    .map(|poses| Trajectory { poses })
                    .collect(),
                footprints: vec![footprint.clone(), footprint],
                discretization_timestep: 1.0,
            });

            (0..peer_len)
                .filter_map(|trajectory_index| {
                    plan.comes_before(&SemanticWaypoint {
                        agent: 1,
                        trajectory_index,
                    })
                })
                .flatten()
                .map(|&id| plan.waypoints[id])
                .filter(|wp| wp.agent == 0)
                .map(|wp| wp.trajectory_index)
                .max()
        }

        assert_eq!(
            wait_depth(vec![parked.clone(), peer.clone()]),
            None,
            "without padding the parked robot is invisible to its peer"
        );

        let mut padded = vec![parked, peer];
        pad_to_equal_length(&mut padded);
        assert_eq!(
            wait_depth(padded),
            Some(4),
            "padding must hold the peer against the parked robot's final waypoint"
        );
    }

    /// The order the planning thread applies these in, and why it survives.
    ///
    /// Padding runs *before* splicing as well as after. PIBT should already
    /// return trajectories of equal length, so the first pass is usually a
    /// no-op, but a short plan would otherwise reach conflict analysis with its
    /// agent missing from the tail of the horizon.
    ///
    /// The risk of going first is that the repeats land between the robot's
    /// arrival and its dock. They do, and it is harmless: the dock is appended
    /// after them so it remains the last real pose, and the arrival is still
    /// locatable as `prefix_len + <index recorded before padding>`, which is
    /// what the destination's constraints are hung on.
    #[test]
    fn pre_padding_does_not_bury_the_dock() {
        // A two step plan into the staging vertex, against a peer that is still
        // driving. The planner's goal is (0, 2.0); the dock lies beyond it.
        let mut trajectories = vec![
            vec![p(2.5, 2.0), p(0.0, 2.0)],
            vec![p(9.0, 0.0), p(8.0, 0.0), p(7.0, 0.0), p(6.0, 0.0)],
        ];
        let motion_end = trajectories[0].len() - 1;

        pad_to_equal_length(&mut trajectories);

        let maneuvers = DockManeuvers {
            undock: None,
            dock: Some(DockSuffix {
                dock_pose: p(0.0, 1.5),
            }),
        };
        let prefix = maneuvers.prefix_len(trajectories[0].first());
        let dock_index = splice_dock_maneuvers(&mut trajectories[0], &maneuvers);

        assert_eq!(
            trajectories[0],
            vec![
                p(2.5, 2.0),
                p(0.0, 2.0),
                p(0.0, 2.0),
                p(0.0, 2.0),
                p(0.0, 1.5)
            ],
            "the dock belongs after the repeats, not before them"
        );
        assert_eq!(dock_index, Some(4));

        // The waypoint the destination refers to is the *first* arrival at the
        // staging vertex, not the last repeat of it and not the dock.
        let destination = prefix + motion_end;
        assert_eq!(destination, 1);
        assert_eq!(trajectories[0][destination], p(0.0, 2.0));
    }
}
