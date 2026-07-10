[Draft for Discourse]

# Layered Global Map Observations

This post describes the observation interface for contributing temporary map
information to the next generation Open-RMF prototype.

The goal is to let robots and perception systems publish what they currently
observe without tying them to a specific central map implementation. The first
prototype consumes sparse 2D occupancy regions because that is enough for the
current global planning grid, but the interface is intentionally isolated in
`rmf_layered_map_msgs` so richer representations, such as height-aware or voxel
observations, can be added later.

# Quick Summary

* Observation sources publish sparse temporary map patches
* One message can contain both clear-space and occupied-space patches from the
  same sensor snapshot
* Clear-space patches have TTLs, just like obstacle patches
* A source can reset its previous observations without using a TTL
* Robot-mounted sources can include the robot pose at observation time
* The observation messages live in `rmf_layered_map_msgs`, leaving
  `rmf_prototype_msgs` unchanged

# Observation Topic

Observation sources publish dynamic map observations on:

* `/map/region_updates` - [`MapRegionUpdate.msg`](../rmf_layered_map_msgs/msg/MapRegionUpdate.msg)

The topic is an event stream. Updates are not latched because expired
observations should not be replayed to a restarted map service as if they were
current.

Each observation stream should use a stable `source_id`, such as
`robot_1/local_costmap`, `robot_1/front_lidar`, or `door_sensor/west_lobby`.
The map service tracks each source independently so one robot can update or
reset its own temporary observations without disturbing observations from other
sources.

# Messages

## `MapObservationSource`

[`MapObservationSource.msg`](../rmf_layered_map_msgs/msg/MapObservationSource.msg)
describes where an observation came from.

Important fields:

* `header`: global frame for the observation and its required, non-zero
  timestamp
* `source_id`: stable source identifier, usually the robot namespace plus the
  local source name
* `robot_name`: robot that produced this observation, if the source is mounted
  on a robot. This can be empty for fixed sensors or synthetic map layers
* `map_name`: map or level that the observations belong to
* `robot_pose`: pose of the robot-local observation frame in `header.frame_id`
  when the observation was produced, so consumers do not need to reconstruct
  it from a separate TF-like lookup
* `default_ttl_sec`: fallback TTL in seconds for patches from this source

## `MapRegionUpdate`

[`MapRegionUpdate.msg`](../rmf_layered_map_msgs/msg/MapRegionUpdate.msg)
describes one observation snapshot from a source.

Important fields:

* `source`: metadata for the observation source
* `reset_source`: remove active observations from the same `source_id` and
  `map_name` before applying the new patches
* `patches`: clear-space and occupied-space patches from this snapshot

`reset_source` is useful when a source publishes replacement snapshots, changes
maps, shuts down, or knows its previous observation state is no longer valid. It
does not have a TTL because it is a bookkeeping operation, not an active map
observation.

## `MapRegionPatch`

[`MapRegionPatch.msg`](../rmf_layered_map_msgs/msg/MapRegionPatch.msg)
describes one group of regions with the same action and TTL.

Patch types:

* `UPDATE_CLEAR`: regions are temporary free space
* `UPDATE_OBSTACLE`: regions are temporary occupied space

Clear patches and obstacle patches both require a TTL because both describe
temporary observations. If a patch TTL is zero or negative, the source
`default_ttl_sec` is used. If that is also zero or negative, the map service's
configured default TTL is used.

When a sensor snapshot includes both clear and occupied regions, publish them in
one `MapRegionUpdate` message. The map service applies clear patches before
obstacle patches, which gives deterministic "clear then mark" behavior without
relying on the arrival order of separate ROS messages.

The first prototype uses `rmf_prototype_msgs/Region` for sparse 2D geometry so
robots can publish compact patches. Region coordinates are robot-local. The map
service transforms them through `source.robot_pose` into
`source.header.frame_id` before rasterizing them. Fixed sensors can publish
sensor-local regions with their sensor pose, while synthetic sources whose
regions are already global can use the identity pose. The first implementation
accepts point and axis-aligned rectangle regions.

# Example Flow

A local costmap or LiDAR observation node can publish a replacement snapshot by:

1. Stamping the observation source with the global frame and observation time,
   and including the pose of the robot-local observation frame.
2. Setting `reset_source` if the new snapshot replaces the source's previous
   temporary observations.
3. Adding one or more clear patches for free space seen by the sensor.
4. Adding one or more obstacle patches for occupied space seen by the sensor.
5. Giving each patch a TTL long enough to survive normal publication jitter but
   short enough to decay when the observation is no longer refreshed.

The map service should ignore snapshots from a source if their timestamp is
older than a newer snapshot that has already been accepted. This prevents a late
clear/reset message from removing obstacle information that came from a newer
observation.

# Implemented Test Coverage

The first server tests cover:

* composing obstacle regions over a static planning grid
* point regions with non-zero map origins and non-1.0 resolutions
* transforming robot-local regions into the global map frame
* out-of-bounds regions, malformed point arrays, and unsupported region types
* rejecting updates without a timestamp
* pruning expired observations by TTL
* clear and obstacle patches in the same update
* late older snapshots being ignored
* reset updates removing observations from the same source and map
* multiple robot sources being stitched into one composed grid
