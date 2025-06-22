# TargetOrientation.msg

# Specify the center of the target orientation in radians.
float32 orientation_radians

# Specify a "spread" around the `orientation_radians` centerpoint of equally
# valid orientations for the robot to reach.
float32 spread_radians

# Specify a tolerance for how far in radians the robot's orientation can be
# from the target to still be accepted.
#
# This is different from `spread_radians` because any value within a distance
# of `spread_radians` from `orientation_radius` is an equally valid target for
# the robot to aim for, whereas `tolerance_radians` is a range that will be
# accepted as a last resort.
#
# Examples:
#
# ### Case 1
# - orientation_radians: pi (180-degrees)
# - spread_radians: pi/2 (90-degrees)
# - tolerance_radians: 0
#
# A robot currently facing 0-degrees should aim to reach an orientation of
# 90-degrees (plus OR minus) to satisfy the constraint, and must end facing
# somewhere in the left half-plane.
#
# ### Case 2
# - orientation_radians: pi (180-degrees)
# - spread_radians: 0
# - tolerance_radians: pi/18 (10-degrees)
#
# A robot currently facing 0-degrees should aim to reach an orientation of
# 180-degrees to satisfy the constraint. As long as it gets within 10-degrees of
# the target, the constraint will be satisfied.
#
# ### Case 3
# - orientation_radians: pi (180-degrees)
# - spread_radians: pi/2 (90-degrees)
# - tolerance_radians: pi/18 (10-degrees)
#
# A robot currently facing 0-degrees should aim to reach an orientation of
# 90-degrees (plus OR minus), but the constraint will be satisfied as long as
# its orientation is greater than 80-degrees or less than -80-degrees.
float32 tolerance_radians
