# Progress.msg

# The continuous progress value of the robot along its current plan. This allows
# us to continuously identify when traffic dependencies are satisfied without
# being sensitive to the spacing of waypoints.
float32 progress

# The last waypoint of the plan that has been reached. This gives a firm
# confirmation of when a robot has executed its navigation to a waypoint and is
# ready for the next stage of the plan to execute.
uint64 reached_waypoint

# The plan that this progress is associated with. If this does not match the
# plan that is currently posted then the progress should be taken to be 0.0.
PlanId plan_id
