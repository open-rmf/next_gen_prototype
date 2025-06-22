# Region.msg

# Pairs of coordinates that encompass the region. There must always be an even
# number of elements in this array. Each even index maps to an x-coordinate and
# each odd index maps to a y-coordinate.
#
# A single pair is interpreted as a point. Two pairs of points are interpreted
# as a line segment. Three pairs are a triangle, while more can be any type of
# polygon.
#
# The `hint` field can provide a hint about how to interpret the array. It is
# undefined behavior to give a hint that the data in the array are incompatible
# with.
float32[] points

# A hint for what type of data this region is meant to contain. Consumers of
# the message should still validate that data matches the hint. If the data does
# not match the hint then consumers are recommended to post an error.
uint8 hint

# The points need to be treated as potentially being any polygon
uint8 HINT_UNSPECIFIED=0

# There should only be exactly one point present.
uint8 HINT_POINT=1

# The points should form a rectangle aligned with the x/y axes.
uint8 HINT_AXIS_ALIGNED_RECTANGLE=2

# The points should form a rectangle which may be rotated.
uint8 HINT_RECTANGLE=3

# The points should form a convex polygon.
uint8 HINT_CONVEX_POLYGON=4

# The points may form any type of polygon.
uint8 HINT_POLYGON=5
