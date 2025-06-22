# SafeZoneId.msg

# What specific plan this safe zone is associated with
PlanId plan_id

# What the version of this safe zone is. This will start from zero for each unique
# plan_id and increment as the safe zone changes.
#
# The zone is expected to change as the traffic conditions evolve---especially
# when conflicting traffic passes by. Eventually it should reach the destination
# of the robot.
uint64 safe_zone_version
