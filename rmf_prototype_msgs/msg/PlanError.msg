# PlanError.msg

# A replan is being requested for unspecified reasons. This will trigger a
# replan to occur.
uint32 CODE_REPLAN_REQUEST=2001

# A replan is being requested because the planned path is blocked. If possible,
# the planner will gather the latest occupancy information available and generate
# a new plan.
uint32 CODE_PATH_BLOCKED=2002

# The plan cannot be executed because some action in the plan is not recognized
# by the plan executor. This indicates that the system is misconfigured.
uint32 CODE_UNRECOGNIZED_ACTION=2003

# The robot or infrastructure are not compatible with one of the actions in the
# plan. This indicates that the system is misconfigured.
uint32 CODE_INCOMPATIBLE_ACTION=2004


# Description of the error
Error error


# What plan is this error asssociated with
PlanId plan_id
