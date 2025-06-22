# TargetRegion.msg

# Tolerance for how close the centerpoint of the robot needs to be to the region.
#
# The tolerance is measured by minimum distance from the region in meters.
#
# A value of 0.0 implies that the centerpoint needs to be inside the region or
# perfectly on the region's edge. If the region is a discrete point then a
# tolerance of 0.0 will make it impractical for the robot to reach the
# destination. A positive value for tolerance would effectively make a discrete
# point into a circle.
#
# Negative values are considered invalid and may be treated as 0.0 and/or result
# in an error being posted.
float32 tolerance

# If the centerpoint of the robot is within `tolerance` of this region, then
# this destination constraint will be satisfied.
Region region

# Zero or more orientations that the robot should face once it is inside the
# target region in order to satisfy the destination constraint.
#
# If no target orientations are specified, the robot may face in any direction.
#
# If multiple orientations are specified, the robot may face towards any one of
# them to satisfy the orientation constraint.
TargetOrientation[] orientations
