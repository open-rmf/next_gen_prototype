# TrafficDependency.msg

# A traffic dependency indicates that a plan is aware of a potential conflict
# with another traffic participant and plans to wait until that other
# participant has reached a point along its plan before continuing.

# The name of the other traffic participant that is leading to this dependency.
string name

# Unique identifier for the plan that is being depended on.
PlanId plan_id

# The point of progress that the plan dependency needs to reach before it stops
# being a blocker.
float32 required_progress
