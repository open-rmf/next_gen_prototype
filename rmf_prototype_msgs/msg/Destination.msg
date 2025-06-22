# Destination.msg

# A description of the constraints that need to be satisfied for the destination
# to be considered reached.
DestinationConstraints constraints

# Timestamp of when this destination was posted.
builtin_interfaces/Time timestamp

# The unique session identifier for this destination. If this destination will
# bring the robot to a DestinationGoal then this will contain the same session
# ID as the goal itself.
unique_identifier_msgs/UUID session

# A destination server may need to direct a robot along a detour if its goal is
# currently occupied by another agent or temporarily unavailable. When that
# happens, the destination server will generate a unique ID for the session
# field, and this detour_for_goal field will contain the session ID of the goal.
#
# This can help operation dashboards to understand why a robot might appear to
# be going somewhere other than its goal. It can also be used by task execution
# systems to recognize that this destination is in service of the posted goal,
# but that the goal will not be reached when the robot reaches this destination.
#
# If this is not a detour then this will be zero-initialized.
unique_identifier_msgs/UUID detour_for_goal
