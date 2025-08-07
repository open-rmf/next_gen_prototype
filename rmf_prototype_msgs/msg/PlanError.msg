# PlanError.msg

# An error code to indicate what kind of problem has happened.
uint32 code

# There is no particular code that matches the error.
uint32 CODE_UNSPECIFIED=0

# A replan is being requested for unspecified reasons. This will trigger a
# replan to occur.
uint32 CODE_REPLAN_REQUEST=1

# A replan is being requested because the planned path is blocked. If possible,
# the planner will gather the latest occupancy information available and generate
# a new plan.
uint32 CODE_PATH_BLOCKED=2

# The plan cannot be executed because some action in the plan is not recognized
# by the plan executor. This indicates that the system is misconfigured.
uint32 CODE_UNRECOGNIZED_ACTION=3

# The robot or infrastructure are not compatible with one of the actions in the
# plan. This indicates that the system is misconfigured.
uint32 CODE_INCOMPATIBLE_ACTION=4


# Additional human-readable information about the error.
string message


# Additional information about the error that may be parsed by a machine.
# It is recommended to use a JSON structure for this.
string parameters


# What plan is this error asssociated with
PlanId plan_id
