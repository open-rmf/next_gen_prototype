# Action.msg
#
# Published on ~/plan/action by the plan executor to trigger a waypoint's
# arrival_action or departure_action on a process executor node.

uint8 PHASE_ARRIVAL = 1
uint8 PHASE_DEPARTURE = 2

# The plan that this action belongs to.
PlanId plan_id

# The index of the waypoint within the plan where this action occurs.
uint64 waypoint_id

# Whether this is an arrival_action (PHASE_ARRIVAL) or departure_action (PHASE_DEPARTURE).
uint8 phase

# The action specification string (e.g. JSON or action identifier) from the Waypoint.
string action
