# MapSourceContribution.msg

# Source that owns these rasterized obstacle cells.
MapObservationSource source

# Row-major indices into MapSourceSnapshot.info. Each index has a matching
# OccupancyGrid-compatible value in occupancy_values.
uint64[] cell_indices
int8[] occupancy_values
