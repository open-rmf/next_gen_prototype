# SafeZone.msg

# The furthest point or region along the current plan that the robot can aim for
# while remaining inside the safe zone that has been allocated for the AMR.
DestinationConstraints incremental_target

# A costmap that represents the safe zone that has been allocated for the AMR.
nav2_msgs/Costmap costmap

# If the incremental_target will bring the robot to a specific waypoint, this
# field indicates which waypoint that is.
uint64[<=1] target_waypoint

# The last waypoint that was reached before this target was sent. When updating
# ~/plan/progress, this can be used for the reached_waypoint field while making
# progress to the incremental_target.
#
# Once the incremental_target is reached, target_waypoint should be used for
# ~/plan/progress if a value was given for that field.
uint64 last_waypoint

# The progress level achieved by reaching the incremental_target.
#
# Prior to reaching the target, the progress level should be calculated by
# projecting the robot's position onto the departure_trajectory of last_waypoint
# without exceeding the value of this target_progress field. Once the
# incremental_target is reached, you can use the target_progress value.
#
# Note that if you report a progress value higher than this target_progress
# value, it could cause significant traffic disruption since the traffic system
# may be asking this robot to wait at or before incremental_target to allow
# another robot to pass. Violating that expectation could cause cascading errors
# that the system might or might not be able to automatically recover from.
float32 target_progress

# Unique ID that associates this costmap with the plan that it is meant for.
SafeZoneId id
