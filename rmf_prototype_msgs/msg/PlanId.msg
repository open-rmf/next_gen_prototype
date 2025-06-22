# PlanId.msg

# Unique identifier for a plan that has been generated to reach a destination

# The session ID of the Destination that this plan is associated with
unique_identifier_msgs/UUID destination_session

# The version number of this plan. This value will start from 0 for any unique
# destination session value and then increment upwards each time a new plan is
# generated.
#
# New plans may be generated due to disruptions or changing traffic conditions.
# To ensure that traffic dependencies are tracked correctly, it is important to
# unqiuely identify each plan that gets produced.
uint64 plan_version
