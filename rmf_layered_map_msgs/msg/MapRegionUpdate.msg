# MapRegionUpdate.msg

# The regions should be interpreted as temporary occupied space.
uint8 UPDATE_OBSTACLE=1

# The regions should be interpreted as temporary free space.
uint8 UPDATE_CLEAR=2

# The update should remove previous temporary observations from this source.
uint8 UPDATE_RESET=3

# Metadata describing where the observation came from.
MapObservationSource source

# UPDATE_OBSTACLE, UPDATE_CLEAR, or UPDATE_RESET.
uint8 update_type

# OccupancyGrid-compatible value to write when this update is rasterized.
#
# Obstacle updates with values less than or equal to zero are treated as 100.
# Clear updates with negative values are treated as 0.
int8 occupancy_value

# Time-to-live for this update. If this is zero or negative, source.default_ttl
# is used instead. If both are zero or negative, the map server's configured
# default TTL is used.
builtin_interfaces/Duration ttl

# 2D geometric observation patches. Empty regions are valid only for
# UPDATE_RESET.
rmf_prototype_msgs/Region[] regions
