# MapSourceSnapshot.msg

# Global frame for the rasterized source contributions.
std_msgs/Header header

# Geometry shared by every source contribution in this snapshot.
nav_msgs/MapMetaData info

# Complete active contribution state, grouped by observation source.
MapSourceContribution[] sources
