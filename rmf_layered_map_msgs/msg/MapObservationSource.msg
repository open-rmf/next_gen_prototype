# MapObservationSource.msg

# Unique source for this stream of observations. For a robot-local node this can
# be the robot namespace plus a stable sensor or node identifier.
string source_id

# Robot that produced this observation, if the source is tied to a robot. This
# may be empty for fixed sensors or synthetic map layers.
string robot_name

# Map or level that the observations belong to.
string map_name

# Coordinate frame for the regions.
string frame_id

# Time when this observation snapshot was produced.
builtin_interfaces/Time stamp

# Fallback time-to-live for updates from this source.
builtin_interfaces/Duration default_ttl
