# Curve.msg

# This message type represents a NURBS curve (Non-Uniform Rational B-Spline)
# https://en.wikipedia.org/wiki/Non-uniform_rational_B-spline
#
# A default-initialized curve is invalid.
#
# To create a piecewise linear trajectory, do the following:
# * set degree to 1
# * add each (x, y) vertex of the trajectory as an element in control_points
#   in order of arrival with weight=1.0 for each
# * For N control points, create N + 2 knots and set their values as follows:
#   * knots[0] = knots[1] = 0.0
#   * knots[N+1] = finish time
#   * knots[i+1] = arrival time at control_point[i], for i = [1, N-1]
#   The above structure will be interpreted as a series of straight-line
#   trajectories where the parametric variable is the time spent moving through
#   the trajectory.
#
# For curved trajectories, it is recommended to use graphical editing tools.

# The degree of the NURBS
uint32 degree

# The control points of the NURBS
ControlPoint[] control_points

# The knot values for the NURBS. For a standard NURBS this should contain a
# number of elements equal to len(control_points) + degree + 1 and values
# should be in ascending order.
float32[] knots
