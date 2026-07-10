# MapObservationSource.msg

# Global frame and time for this observation snapshot. The timestamp is required.
# Consumers should reject updates with a zero timestamp.
std_msgs/Header header

# Unique source for this stream of observations. For a robot-local node this can
# be the robot namespace plus a stable sensor or node identifier.
string source_id

# Robot that produced this observation, if the source is tied to a robot. This
# may be empty for fixed sensors or synthetic map layers.
string robot_name

# Map or level that the observations belong to.
string map_name

# Pose of the robot-local observation frame in header.frame_id when this
# snapshot was produced. Fixed sensors can use their sensor pose. Sources whose
# regions are already in header.frame_id should use the identity pose.
geometry_msgs/Pose robot_pose

# Fallback time-to-live in seconds for patches from this source.
float64 default_ttl_sec
