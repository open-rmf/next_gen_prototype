# Trajectory.msg

# The curve of this trajectory as NURBS.
Curve curve

# Value of the plan progress starting from the beginning of the curve's
# arclength.
#
# While the robot makes progress along the trajectory, it should be reported as
# progress along the plan. The curve's own parameterization will depend on the
# values of its knots, which are agnostic to the plan's concept of progress.
#
# This field says what the progress level of the robot is when it is at the
# start of the curve. The final_progress_level field says what the progress
# level of the robot is at the end of the curve. As the robot progresses along
# the curve, the overall plan progress should be reported as a linear
# interpolation from initial_progress_level to final_progress_level based on the
# robot's position along the curve.
#
# It is recommended that curves choose their knot values so that each unit of
# the parametric variable (u) represents one second of travel time along the
# curve. However the overall traffic system must not rely on this convention.
float32 initial_progress_level

# Used alongside initial_progress_level to calculate the plan progress of the
# robot as it advances along this curve.
float32 final_progress_level

# The name or names of any "maps" that this trajectory passes through.
#
# Multiple map names may be used at once if the trajectory is moving through
# multiple areas at once. For example, a robot moving from "Level_1" into
# "North_Lift_A" should have a trajectory that includes both of those as maps.
string[] maps

# If this trajectory passes through any elements of any graphs, their keys are
# listed here.
GraphElementKey[] keys
