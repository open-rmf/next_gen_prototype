use mapf_post::{
    na::{Isometry2, Vector2},
    WaypointFollower,
};
use ros_env::rmf_prototype_msgs::msg::{Plan, Progress};

// Match Follower type
#[derive(Copy, Clone, Debug)]
enum FollowType {
    WatchOdometry,
    WatchProgress,
}

// The defaulf WaypointFollower in mapf_post assumes
// that all progress along a plan can be inferred from the robot's
// position. This is not strictly true in some cases. For instance,
// the robot's progress along its docking trajectory may not strictly
// follow a single straight line as a robot may attempt to redock a
// few times.
pub(crate) struct ActionExecutingFollower {
    waypoint_follower: WaypointFollower,
    trajectory_follow_type: Vec<FollowType>,
    poses: Vec<Isometry2<f32>>,
}

impl ActionExecutingFollower {
    pub(crate) fn from_plan(agent_id: usize, plan: &Plan) -> Self {
        let mut trajectory_follow_type = vec![];
        let mut poses = vec![];
        for wp in &plan.waypoints {
            if wp.departure_action != "" || wp.arrival_action != "" {
                trajectory_follow_type.push(FollowType::WatchProgress);
            } else {
                trajectory_follow_type.push(FollowType::WatchOdometry);
            }
            poses.push(Isometry2::new(
                Vector2::new(wp.position[0], wp.position[1]),
                0.0,
            ));
        }

        let waypoint_follower = WaypointFollower::from_trajectory(
            agent_id,
            mapf_post::Trajectory {
                poses: poses.clone(),
            },
        );

        Self {
            waypoint_follower,
            trajectory_follow_type,
            poses,
        }
    }

    // TODO(arjoc): Fix mapf_post we dont need a mut
    fn current_follow_strategy(&mut self) -> FollowType {
        // Get current position index
        let wp_id = self.waypoint_follower.get_semantic_waypoint();
        self.trajectory_follow_type[wp_id.trajectory_index]
    }

    // Update the robot's progresss along its pre-computed trajectory using
    pub(crate) fn update_position_estimate(&mut self, pos: &Isometry2<f32>, uncertainty: f32) {
        match self.current_follow_strategy() {
            FollowType::WatchOdometry => {
                self.waypoint_follower
                    .update_position_estimate(pos, uncertainty);
            }
            FollowType::WatchProgress => {
                return;
            }
        }
    }

    pub(crate) fn update_progress_estimate(&mut self, progress: &Progress) {
        match self.current_follow_strategy() {
            FollowType::WatchOdometry => {
                return;
            }
            FollowType::WatchProgress => {
                if progress.reached_waypoint as usize
                    > self
                        .waypoint_follower
                        .get_semantic_waypoint()
                        .trajectory_index
                {
                    if progress.reached_waypoint as usize >= self.poses.len() {
                        // TODO(arjoc): Misbehaving fleetadapter
                        println!("Reached waypoint index is invalid. Assuming we have reached final destination");
                        return;
                    }
                    self.waypoint_follower.update_position_estimate(
                        &self.poses[progress.reached_waypoint as usize],
                        0.1,
                    );
                }
            }
        }
    }

    // Assume that the waypoint follower has the full picture,
    // just the estimate of progress is what changes.
    pub fn remaining_trajectory(&self) -> Vec<(f32, f32)> {
        self.waypoint_follower.remaining_trajectory()
    }

    // Get index along the plan
    pub fn get_index_along_plan(&mut self) -> usize {
        self.waypoint_follower
            .get_semantic_waypoint()
            .trajectory_index
    }
}
