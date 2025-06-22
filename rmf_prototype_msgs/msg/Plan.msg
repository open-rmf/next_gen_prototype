# Plan.msg

# A Plan message represents the intended route that a robot will travel from its
# initial location to a certain Destination. Each Plan is explicitly associated
# with a specific Destination via the destination_session field.

# The waypoints that a robot should follow.
Waypoint[] waypoints

# The time that this plan was posted. This should roughly correspond to when the
# plan begins executing.
builtin_interfaces/Time start_time

# The unique identifier for this plan instance
PlanId plan_id
