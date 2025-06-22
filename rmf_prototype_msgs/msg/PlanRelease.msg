# PlanRelease.msg

# This release message can be used by dashboards to visualize the portion of the
# plan that the robot is free to execute. It may also be used by VDA 5050 bridges
# to indicate how far the robots may travel through their order.


# The plan may be freely executed up to this waypoint based on the state of all
# blockers and actions within the plan.
uint64 waypoint_id


# Which plan does this release refer to.
PlanId plan_id
