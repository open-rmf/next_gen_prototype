# MapObservationSource.msg

# Frame and time for this observation snapshot.
std_msgs/Header header

# Unique source for this stream of observations. For a robot-local node this can
# be the robot namespace plus a stable sensor or node identifier.
string source_id

# Robot that produced this observation, if the source is tied to a robot. This
# may be empty for fixed sensors or synthetic map layers.
string robot_name

# Map or level that the observations belong to.
string map_name

# Pose of the robot when this observation was produced. Fixed sensors or
# synthetic map layers may leave this at its default value.
geometry_msgs/PoseStamped robot_pose

# Fallback time-to-live in seconds for patches from this source.
float64 default_ttl_sec
