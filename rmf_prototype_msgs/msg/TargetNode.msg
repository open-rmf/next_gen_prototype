# TargetNode.msg

# A key within a graph that the robot intends to reach.
GraphElementKey key

# Zero or more orientations that the robot should face once it has reached the
# target node.
#
# If no target orientations are specified, the robot may face any direction.
#
# If multiple orientations are specified, the robot may face towards any one of
# them to satisfy the orientation constraint.
TargetOrientation[] orientations
