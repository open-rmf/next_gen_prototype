# Waypoint.msg

# A waypoint represents a significant point within a robot's plan. Waypoints are
# intended to be locations where the robot may need to wait for other traffic or
# perform a special action before proceeding.
#
# For plan segments that do not need to perform any special action, it is
# recommended to consolidate them into the departure_trajectory of a single
# waypoint. However this might not be suitable for interfacing with all systems,
# so plan executors should be prepared to handle plans that are simple piecewise
# linear trajectories with or without trajectories, blockers, or actions at any
# waypoint.


# The (x, y) location of this waypoint, relative to the deployment's canonical
# origin point, measured in meters.
float32[2] position


# If this waypoint is associated with any nodes in any graphs, their keys are
# listed here.
NodeKey[] keys


# A value that indicates the progress level of a robot that has reached this
# waypoint. This value must increase with each subsequent Waypoint along a plan.
#
# Ideally this should be an estimate of the number of seconds that the robot
# will take to reach this point starting from the beginning of the plan. However
# the overall system should not assume that this estimate will be accurate, so
# the concept of "progress" should ultimately be treated as a semantic parameter.
float32 progress


# The name or names of any "maps" that this waypoint is in.
#
# Multiple map names may be used at once if the robot needs to consider
# conflicts against traffic participants from multiple areas at once. Any plan
# segment of any other robot that contains any one of the listed maps will be
# checked for conflicts against this plan.
#
# If this is left empty then the conflicts will only be examined against other
# agents who specify no maps.
string[] maps


# A set of traffic dependencies that block the robot from continuing past this
# waypoint. Every traffic participant in this list must reach or pass its
# required_progress point before the robot following this plan may depart this
# waypoint.
TrafficDependency[] departure_blockers


# The recommended trajectory to follow when leaving this waypoint.
#
# Multiple trajectories can be included at once to accommodate changes in what
# maps are being passed through. For example, if the departure_action
# encapsulates a process where a robot enters a lift named Lift_A at Level_1,
# waits for the lift to arrive at Level_2, and then exits the lift, then this
# departure_trajectory should consist of three elements:
# * A curve that includes both maps Level_1 and Lift_A
# * A curve that only includes Lift_A
# * A curve that includes both maps Lift_A and Level_2
#
# If this field is empty then it should be interpreted as requesting a
# straight-line approach to the next waypoint that includes all maps of both
# this waypoint and the next waypoint in the plan.
Trajectory[] departure_trajectory


# A particular action to be taken when the robot is allowed to depart.
#
# Recommended practice is for this to be a JSON structure that fits a standard
# action schema. Included among the standard action schemas are sequence,
# parallel, and workflow categories, which all allow multiple actions to take
# place from this waypoint.
#
# In some cases this action might trigger something external, such as a door to
# open or a lift to arrive. In those cases, the execution manager should wait
# for the action to finish and then command the robot to move towards the next
# waypoint.
#
# In other cases this action might bring the robot to the next waypoint as part
# of some workflow or docking sequence. Therefore the general process for the
# execution manager should be to trigger this action, wait until it is complete,
# check the progress of the robot, and then:
# * if the robot has arrived at the next waypoint then execute the next waypoint
#   of the plan, or
# * if the robot has not arrived at the next waypoint then issue a command to
#   move to the next waypoint and wait until it arrives.
#
# The departure action must be executed even if this is the last waypoint in a
# plan, since this action might be doing important cleanup, such as closing a
# door.
string departure_action
