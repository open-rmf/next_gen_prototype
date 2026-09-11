# Progress.msg

# The continuous progress value of the robot along its current plan. This allows
# us to continuously identify when traffic dependencies are satisfied without
# being sensitive to the spacing of waypoints.
float32 progress

# The last waypoint of the plan that has been reached. This gives a firm
# confirmation of when a robot has executed its navigation to a waypoint and is
# ready for the next stage of the plan to execute.
uint64 reached_waypoint

# The waypoint that the robot is currently working to reach. When this is equal
# to reached_waypoint, the robot is waiting for an event to finish or has
# reached the end of the plan.
uint64 target_waypoint

# The departure_trajectory of reached_waypoint may have some number of graph
# element keys that they pass through. When each of those keys is reached, it
# will be added into this array.
#
# This array will be reset each time a new waypoint is reached in order to avoid
# excessive message growth.
GraphElementKey[] reached_keys

# The plan that this progress is associated with. If this does not match the
# plan that is currently posted then the progress should be taken to be 0.0.
PlanId plan_id


# The executor has not reported a state, or the state cannot be determined.
uint8 EXECUTION_STATE_UNKNOWN=0

# The robot has finished its plan (or has no plan) and is free to be given a new
# one. It is not holding any space that it needs to keep.
uint8 EXECUTION_STATE_IDLE=1

# The robot is actively moving along its plan.
uint8 EXECUTION_STATE_MOVING=2

# The robot has stopped short of the end of its plan because one or more of the
# departure_blockers of reached_waypoint have not been satisfied yet. It is able
# to move again as soon as those blockers clear, and it is safe to replan for it.
uint8 EXECUTION_STATE_HOLDING_FOR_TRAFFIC=3

# The robot is part-way through an arrival_action or departure_action and is
# physically committed to it - for example it is driving into or out of a dock
# under the control of a separate docking controller.
#
# While a robot reports this state:
# * It must NOT be given a new plan. Its current plan_id must remain valid until
#   it reports a different state.
# * The space it needs in order to finish the action must be treated as claimed,
#   which in general is larger than its footprint.
#
# This is the signal that a traffic planner should use to exclude the robot from
# multi-agent planning rather than trying to infer commitment from the waypoint
# indices above.
uint8 EXECUTION_STATE_EXECUTING_ACTION=4

# One of the EXECUTION_STATE_* constants above.
uint8 execution_state

# When execution_state is EXECUTION_STATE_EXECUTING_ACTION, this names the action
# that is currently in flight. It should match the arrival_action or
# departure_action string of the relevant waypoint where possible, so that the
# recipient can correlate it with the plan it posted.
#
# Empty for all other execution states.
string active_action
