# DestinationConstraints.msg

# One or more spatial regions that the robot intends to reach. It is strongly
# recommended to always specify at least one region since regions can be
# visualized without access to any graph structure.
#
# If there are multiple regions that are non-overlapping, then the robot may
# sent to any one of them, and all regions are expected to be kept available
# for the robot. Some plan servers might not support non-overlapping regions,
# in which case they should post an error message.
TargetRegion[] regions

# Zero or more graph nodes that the robot intends to reach. This is meant to
# communicate the destination to graph-based planners and other tools that
# operate in terms of graph semantics.
#
# This can be used to set the destination to a named location.
TargetNode[] nodes
