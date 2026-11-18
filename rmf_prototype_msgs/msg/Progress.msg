# Progress.msg

uint8 MODE_IDLE = 0
uint8 MODE_NAVIGATING = 1
uint8 MODE_ACTION = 2
uint8 MODE_WAITING = 3
uint8 MODE_ERROR = 4

# Current estimated pose of the robot in the map/world frame.
geometry_msgs/PoseStamped pose

# Current operational mode of the robot (one of the MODE_* constants above).
uint8 mode

# Active safe zone identifier, if the robot is currently operating within an
# allocated SafeZone.
SafeZoneId[<=1] safe_zone_id

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
