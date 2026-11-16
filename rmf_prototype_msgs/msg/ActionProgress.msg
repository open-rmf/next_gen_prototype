# ActionProgress.msg
#
# Published on ~/plan/action/progress by a process executor node to report the
# execution status of an Action requested on ~/plan/action.

uint8 STATUS_RUNNING = 1
uint8 STATUS_SUCCEEDED = 2
uint8 STATUS_FAILED = 3

# The plan that this action progress belongs to.
PlanId plan_id

# The index of the waypoint within the plan where this action occurs.
uint64 waypoint_id

# Whether this is an arrival_action (Action::PHASE_ARRIVAL) or departure_action (Action::PHASE_DEPARTURE).
uint8 phase

# Current execution status (one of the STATUS_* constants above).
uint8 status

# Error details if status == STATUS_FAILED.
Error error
