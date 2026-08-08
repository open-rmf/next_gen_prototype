[Draft for Discourse]

# Layered Global Map Observations

This post proposes an interface for robots and perception systems to share temporary map observations without depending on a specific map implementation.

The current interface uses sparse 2D occupancy regions. Future interfaces may add height-aware, voxel, or semantic observations where 2D data is insufficient.

# Quick Summary

* Observation sources publish `MapRegionUpdate` messages on `/map/region_updates`
* One message contains all clear and occupied regions from the same observation
* Clear and obstacle patches both expire after a TTL
* Sources include the observation frame pose and timestamp
* A source can reset its previous observations

# Region Updates

[`MapRegionUpdate`](../rmf_layered_map_msgs/msg/MapRegionUpdate.msg) describes one observation from one source. It contains:

* `source`: the source identity, frame, timestamp, pose, and default TTL
* `reset_source`: whether to discard the source's previous observations before applying this update
* `patches`: clear and occupied regions observed in the update

Publish all evidence from the same observation in one message. For example, a laser scan should contain both its clear rays and occupied endpoints so the result does not depend on the arrival order of separate messages.

The topic is an event stream and should not use transient-local durability. Replaying an old update after a subscriber restarts could make expired evidence appear current.

## Source (`MapObservationSource`)

[`MapObservationSource`](../rmf_layered_map_msgs/msg/MapObservationSource.msg) identifies where and when an observation was made. It contains:

* `header`: the global observation frame and timestamp
* `source_id`: a stable ID for the observation stream
* `robot_name`: the robot that owns the source, if any
* `map_name`: the map or level where the observation was made
* `robot_pose`: the observation frame's pose in the header frame
* `default_ttl_sec`: the fallback lifetime for patches

Use a stable `source_id`, such as `robot_1/front_lidar` or `door_sensor/west_lobby`, so observations from different sources can be managed independently.

The source header contains the global observation frame and a required, non-zero timestamp. Regions are expressed in the local observation frame, and `robot_pose` records that frame's pose in the header frame at the observation time. Fixed sensors can use their fixed pose, while sources that already publish global coordinates can use the identity pose.

## Patches (`MapRegionPatch`)

[`MapRegionPatch`](../rmf_layered_map_msgs/msg/MapRegionPatch.msg) groups regions that share an action, occupancy value, and TTL. It contains:

* `update_type`: `UPDATE_CLEAR` or `UPDATE_OBSTACLE`
* `occupancy_value`: the value assigned to the observed cells
* `ttl_sec`: how long the observation remains active
* `regions`: the observed points, rectangles, or convex polygons

The update types are:

* `UPDATE_CLEAR` reports temporary free space
* `UPDATE_OBSTACLE` reports temporary occupied space

Clear patches are applied before obstacle patches in the same update. This keeps an occupied endpoint marked after its clear ray is processed. Clear evidence removes older obstacle evidence only from the same source.

A positive patch TTL sets its lifetime. If the patch TTL is zero or negative, `source.default_ttl_sec` is used. If both values are zero or negative, the consumer may use its configured default.

The current message supports points, axis-aligned rectangles, and convex polygons. Producers should use the most compact shape that represents their observation.

## Resetting a Source

Set `reset_source` when an update replaces all earlier evidence from the same `source_id` and `map_name`, or when a source changes maps, shuts down, or invalidates its previous state. A reset is a bookkeeping operation and has no TTL. A reset-only message may omit patches.

Do not reset every incremental sensor update when older observations should remain active until they are cleared or expire.

## Example

A local costmap or LiDAR integration can publish an update by:

1. Stamping the source with the global frame and observation time.
2. Recording the observation frame's pose at that time.
3. Adding clear patches for observed free space.
4. Adding obstacle patches from the same observation.
5. Assigning TTLs long enough to tolerate publication jitter but short enough for stale evidence to expire.

Consumers should reject malformed or unstamped updates and ignore updates older than the latest accepted update from the same source. This prevents a late clear or reset from removing newer obstacle evidence.
