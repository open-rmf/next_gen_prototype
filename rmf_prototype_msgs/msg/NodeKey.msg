# NodeKey.msg

# Name of the graph that this target is referencing. The default standard is for
# the graph information to be published to the /{graph_name}/graph topic.
string graph_name

# The key for a node in the graph.
#
# If the graph uses discrete vertices and edges then this should be a single
# positive int64 value that refers to the index of the node vertex.
#
# If the graph uses a grid representation then this should be a pair of integers
# which may be positive or negative depending on the restrictions of the grid.
#
# Empty keys are only allowed if a `name` is specified, otherwise the graph node
# is undefined.
int64[<=2] key

# The semantic name of a node in the graph. This may be used instead of `key`.
# It may also be used in addition to `key` as long as `key` and `name` both
# refer to the same node.
string[<=1] name

# In some cases the key or name alone might not be sufficient to uniquely
# identify a node. Graphs may be broken down into parts that we call "maps".
# A map might represent a distinct area, room, or floor within the environment.
#
# For example in a multi-floor building, the coordinates key=[3, 12] may be
# valid grid coordinates across multiple different floors. To fully specify a
# location in the building, the graph may define each floor of the building as
# a map and have one grid covering each floor.
string[<=1] map
