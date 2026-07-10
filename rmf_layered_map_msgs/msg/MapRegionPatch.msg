# MapRegionPatch.msg

# The regions should be interpreted as temporary occupied space.
uint8 UPDATE_OBSTACLE=1

# The regions should be interpreted as temporary free space.
uint8 UPDATE_CLEAR=2

# UPDATE_OBSTACLE or UPDATE_CLEAR.
uint8 update_type

# OccupancyGrid-compatible value to write when this patch is rasterized.
#
# Obstacle patches with values less than or equal to zero are treated as 100.
# Clear patches with negative values are treated as 0.
int8 occupancy_value

# Time-to-live in seconds for the regions in this patch. If this is zero or
# negative, source.default_ttl_sec is used instead. If both are zero or
# negative, the map server's configured default TTL is used. Clear patches use
# TTL in the same way as obstacle patches because they are temporary free-space
# observations.
float64 ttl_sec

# 2D geometric observation patches. The first prototype supports point and
# axis-aligned rectangle regions.
rmf_prototype_msgs/Region[] regions
