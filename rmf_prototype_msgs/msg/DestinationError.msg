# DestinationError.msg

# An error code to indicate what kind of problem has happened.
uint32 code

# There is no particular code that matches the error.
uint32 CODE_UNSPECIFIED=0

# The key or name of the destination is not recognized by the planner.
uint32 CODE_UNKNOWN_DESTINATION=1

# The destination is temporarily unreachable but may become reachable if
# conditions change. This error may happen when a traffic lane is temporarily
# closed.
uint32 CODE_TEMPORARILY_UNREACHABLE=2

# The destination is being blocked by a physical obstruction that is not
# controlled by the traffic system.
uint32 CODE_DESTINATION_BLOCKED=3

# The destination is not reachable according to the currently available graph
# information, and there is no known change in conditions which could make it
# available.
#
# With a freespace planner this may happen if the static portion of the cost map
# has no path from the robot's current location to the destination. With a
# discrete vertex/edge navigation graph, this may happen if there is no edge
# sequence that connects the robot's current location to the destination node.
uint32 CODE_UNREACHABLE=4

# The destination has a constraint that the planner is incompatible with.
#
# Some implementations might only support a subset of the features expressed by
# these interfaces, such as not supporting certain Region shapes. If you
# encounter this code, you may need to represent the destination differently or
# choose a different planner implementation.
uint32 CODE_INCOMPATIBLE_CONSTRAINT=5


# Additional human-readable information about the error.
string message


# Additional information about the error that may be parsed by a machine.
# It is recommended to use a JSON structure for this.
string parameters
