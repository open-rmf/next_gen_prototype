# DockStatus.msg

# The latched state of one robot with respect to its dock.
#
# Published per robot on `<robot_id>/dock_status` with transient local
# durability, so that a node which starts late still learns the current state
# rather than waiting for the next transition.
#
# This is deliberately a topic of its own rather than a field on Progress.
# Progress describes how far a robot has got along a plan; being in a dock is a
# physical fact about the robot that outlives any particular plan, and has to
# remain readable by nodes that are not tracking plans at all.


# The robot is not in a dock and is free to navigate.
uint8 STATE_UNDOCKED=0

# The robot is driving into `dock_id` under the control of a docking
# controller. It is physically committed and cannot be rerouted.
uint8 STATE_DOCKING=1

# The robot has finished docking and is parked in `dock_id`.
#
# This is the state that persists indefinitely, and the reason this message
# exists: once docking completes the robot goes back to reporting an idle
# execution state with no active action, so nothing else on the wire
# distinguishes it from a robot parked in the open. A robot in this state
# cannot begin following a new plan without first reversing out of the dock.
uint8 STATE_DOCKED=2

# The robot is reversing out of `dock_id`. It is physically committed and
# cannot be rerouted.
uint8 STATE_UNDOCKING=3

# A dock or undock attempt failed. The robot's position with respect to the
# dock is not known: it may be anywhere in the dock lane, so `dock_id` is still
# populated and the lane should continue to be treated as occupied until a
# recovery puts the robot into a known state.
uint8 STATE_FAILED=4

# When this report was published.
#
# This is a liveness stamp, not the time of the transition. The publisher
# repeats the current state on a fixed wall-clock interval, so the stamp
# advances while nothing changes, and answers "is the reporter still alive and
# still asserting this?" rather than "when did this happen?".
#
# A consumer that has not seen a fresh stamp within its own timeout must treat
# the state as unknown rather than as STATE_UNDOCKED. A silent reporter says
# nothing about where the robot is, and assuming a robot has left its dock
# because its reporter died is how you plan a route out of a dock the robot is
# still sitting in.
#
# The interval is wall-clock and independent of odometry on purpose: tying it
# to sensor data would let an odometry dropout expire a dock state that is
# still perfectly true.
builtin_interfaces/Time stamp

# One of the STATE_* constants above.
uint8 state

# The dock being entered, occupied, left, or failed on.
#
# Where possible this matches the arrival_action string of the waypoint that
# sent the robot there, so that the recipient can correlate it with the plan it
# posted and with the corresponding vertex in its own graph.
#
# Empty only when state is STATE_UNDOCKED.
string dock_id

# The plan whose dock action produced this state.
#
# This is provenance, not a validity gate. It says which plan asked for the
# manoeuvre, so a recipient can tell that the destination it issued has been
# reached and retire it. It must NOT be used to decide whether to believe the
# state.
#
# Believing a STATE_DOCKED report only when its plan_id matches the plan the
# recipient currently considers active looks tempting and is wrong. A robot
# docks under plan {A,0}; an unrelated robot is then given a task; the first
# robot is swept into that negotiation and reissued as {A,1}; its dock report
# still says {A,0} and is discarded. The robot is now believed to be undocked
# while parked in a dock, which is the one belief guaranteed to produce an
# unusable plan. Use `stamp` to judge freshness and the robot's position to
# corroborate; use this only to match a completed manoeuvre to the request that
# caused it.
#
# Zero-valued for any state not produced by executing a plan, notably the
# initial STATE_UNDOCKED a robot reports before it has ever been given one.
#
# Note that a restart of the publishing node loses the dock state entirely and
# begins again from STATE_UNDOCKED. That is consistent with the rest of the
# stack, which also forgets: the agent's own dock state resets too, so it will
# not attempt to undock before its next navigation either. Recovering the true
# state after a restart would mean querying the docking controller at startup.
PlanId plan_id

# Human-readable detail, primarily for STATE_FAILED, where the state alone does
# not say what went wrong. Free-form and intended for operators and logs.
#
# Consumers must not parse this to determine state; use `state` for that.
string message
