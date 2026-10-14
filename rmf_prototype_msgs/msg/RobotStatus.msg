# RobotStatus.msg
#
# Consolidated status telemetry for a traffic participant, combining spatial pose,
# operational execution mode, and optional active plan progress / safe zone context.

uint8 MODE_IDLE = 0
uint8 MODE_NAVIGATING = 1
uint8 MODE_ACTION = 2
uint8 MODE_WAITING = 3
uint8 MODE_ERROR = 4

# Current estimated pose of the robot in the map/world frame.
geometry_msgs/PoseStamped pose

# Current operational mode of the robot (one of the MODE_* constants above).
uint8 mode

# Active plan progress, if the robot is currently executing or holding a plan.
# Empty when the robot is idle and has no active plan.
Progress[<=1] progress

# Active safe zone identifier, if the robot is currently operating within an
# allocated SafeZone.
SafeZoneId[<=1] safe_zone_id
