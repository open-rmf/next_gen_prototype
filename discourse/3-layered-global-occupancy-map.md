[Draft for Discourse]

# Layered Global Map Observations

This post describes the implemented observation interface and demos for
contributing temporary map information to the next generation Open-RMF
prototype.

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
* Robot-mounted sources include the observation-frame pose at observation time
* The Rust map server rasterizes incoming observations once and composes their active cell contributions with a static occupancy grid into `/map`
* The Nav2 demo converts scans from three robots into clear ray sectors and occupied endpoint regions, then displays the source contributions and map
* The replanning demo fuses two robots' scans and replans when an updated map blocks an active route
* The observation messages live in `rmf_layered_map_msgs`, leaving
  `rmf_prototype_msgs` unchanged

# Observation Topics

The layered map server uses these topics:

* `/map/static` - static `nav_msgs/OccupancyGrid`
* `/map/region_updates` - [`MapRegionUpdate.msg`](../rmf_layered_map_msgs/msg/MapRegionUpdate.msg)
* `/map` - composed `nav_msgs/OccupancyGrid`

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

# Layered Map Server

`rmf_layered_map_server` keeps the static occupancy grid separate from dynamic observations and publishes their composition on `/map`. It validates source timestamps and frames, ignores updates older than the latest accepted update from the same source, supports source resets, and removes observations after their TTL expires. Each region is transformed and rasterized on arrival. Repeated observations refresh cell contributions instead of stacking region snapshots. Older evidence is retained only when it outlives a newer contribution.

The server applies clear patches before obstacle patches from the same update. During composition, obstacle observations win over clear observations so occupied space is not accidentally erased by another active source. Updates received before the static grid are held in a bounded FIFO queue and rasterized when the grid arrives. A static grid geometry change discards active cell contributions because their indices refer to the old geometry.

# Three-Robot Nav2 Demo

The launch file starts three robots in different free corners of the warehouse, cycles them through fixed Nav2 goals, and spawns one deterministic Gazebo box near each robot. Each observation node subscribes to its robot's local `sensor_msgs/LaserScan`, filters invalid or out-of-range returns, and publishes free-space ray sectors as convex polygons and occupied endpoints as point regions.

Each scan adds temporary clear and obstacle patches. The map server rasterizes them on arrival and refreshes matching cell contributions rather than retaining region messages. It applies clear cells before obstacle cells so a measured hit remains occupied. Active obstacle evidence still wins over clear evidence until its TTL expires. An update may set `reset_source` so the new scan replaces all active observations from that source. The observation-frame pose is recorded in the shared `map` frame so the map server can transform scan-local regions before rasterizing them.

The demo launches Nav2 localization, planning, and control for the fixed goal loops, but does not launch RMF planning. Robot-local RViz windows show Nav2 state, while another RViz window displays the combined global `/map`. The combined view overlays incoming region updates as colored markers grouped by source, with the same retention behavior as the map contributions.

The scan-to-region conversion uses one convex sector per sampled beam instead of expanding a Bresenham line into many point regions. Scan sampling reduces the number of sectors, publication throttling limits the snapshot rate, and the observation range limits represented returns. The TTL controls how quickly stale observations expire. Each publisher logs its input beams and clear and obstacle regions so message density, update rate, and visual fidelity can be compared. The current demo remains a 2D occupancy approximation; it does not fuse probabilistic confidence or compress adjacent sectors into larger regions.

# Two-Robot Replanning Demo

The replanning launch starts with a simple room by default and retains the warehouse layout as a regression scenario. It fuses both robots' scans into `/map`, spawns a blocking bar, and displays robot-colored poses, goals, observations, and Nav2 paths alongside the green RMF plans.

The plan executor checks each remaining route against the composed map and publishes `CODE_PATH_BLOCKED` after a short debounce. The path server replans on a conservatively downsampled `1.0 m` grid, while the Nav2 bridge ignores superseded aborts and resends unchanged targets when a new plan requires them.

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

The map server ignores snapshots from a source if their timestamp is older than
a newer snapshot that has already been accepted. This prevents a late clear or
reset message from removing obstacle information that came from a newer
observation.

# Implemented Test Coverage

The committed server and demo tests cover:

* composing obstacle regions over a static planning grid
* point regions with non-zero map origins and non-1.0 resolutions
* transforming robot-local regions into the global map frame
* out-of-bounds regions, malformed point arrays, and unsupported region types
* rejecting updates without a timestamp or in a different global frame
* pruning expired observations by TTL
* clear and obstacle patches in the same update
* late older snapshots being ignored
* reset updates removing observations from the same source and map
* multiple robot sources being stitched into one composed grid
* converting laser beams into clear sectors and occupied endpoints
* filtering invalid and out-of-range laser returns
* preserving original beam angles when scan points are sampled
* converting point and rectangle updates into source-colored markers
* retaining markers until TTL expiry and replacing them on source reset
