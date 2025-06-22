# DestinationGoal.msg

# One of these destination constraints must be satisfied in order for the goal
# to be reached.
#
# The goal is invalid if this array is empty.
DestinationConstraints[] one_of

# A cost vector to bias the choice of destination.
#
# This must either be empty or contain the same number of elements as one_of.
# Otherwise this overall message is invalid.
#
# It is recommended to weigh the cost in terms of seconds spent reaching the
# destination. For example if you have cost_bias = [0.0, 60.0] then you are
# indicating that you would rather select one_of[0] unless one_of[1] could be
# reached 60 seconds faster than one_of[0].
#
# At exactly a 60 second difference they would tie, but if one_of[1] can be
# performed even sooner than 60 seconds before one_of[0], then one_of[1] is the
# winner.
float32[] cost_bias

# The unique session identifier for this goal.
unique_identifier_msgs/UUID session
