# RMF Layered Map Server

`rmf_layered_map_server` composes a static `nav_msgs/OccupancyGrid` with temporary observation regions into the `/map` topic consumed by the path server.

Each robot or perception node publishes updates with its own `source_id`, and the server stitches all active observations into one composed global grid. Robot-mounted sources should also include `robot_name` metadata so the source can be correlated with the robot that observed it.

Default topics:

- `/map/static`: static `nav_msgs/OccupancyGrid`
- `/map/region_updates`: `rmf_layered_map_msgs/MapRegionUpdate`
- `/map`: composed `nav_msgs/OccupancyGrid`
- `/map/source_contributions`: active obstacle cells grouped by source

Dynamic observations expire according to their TTL, so transient obstacles do not remain in the global planning map forever. The server transforms and rasterizes each region on arrival, then stores cell contributions by source, map, update type, and cell. Repeated observations refresh matching cells instead of stacking region snapshots. Older evidence is retained only when it outlives a newer contribution.

`/map/source_contributions` publishes the active rasterized obstacle cells for each source whenever the composed map changes.

`MapRegionUpdate` messages contain a list of `MapRegionPatch` entries. A sensor snapshot that clears freespace and marks obstacles can publish clear and obstacle patches in the same message. The server applies clear patches before obstacle patches. Clear cells remove older obstacles from the same source, while current obstacle endpoints and obstacles reported by other sources remain occupied. Observations outside the clear regions remain active until their TTL expires. If the snapshot replaces the source's entire previous observation state, set `reset_source` so the server removes all old observations from that `source_id` and `map_name` before adding the new patches. Resetting has no TTL. Clear and obstacle patches both use TTLs in seconds.

Patch regions are expressed in the robot-local observation frame. The server
uses `source.robot_pose` to transform them into `source.header.frame_id`, which
must match the static occupancy grid frame. Updates need a non-zero source
timestamp. The current implementation accepts point and axis-aligned rectangle
regions. It logs and ignores other region types.

Region updates received before the static occupancy grid are held in a bounded FIFO queue and rasterized once the grid arrives. Replacing only the static grid data preserves active dynamic cells; changing its frame, dimensions, resolution, or origin discards them because their cell indices no longer describe the new grid.

## Performance Benchmark

The deterministic benchmark is installed by the normal colcon build and exercises the map server without ROS transport, visualization, or simulator overhead. It reports ingest, prune, composition, and total latency, plus throughput, peak RSS, and an output-grid checksum.

Build and source the workspace, then run each supported update pattern:

```bash
ros2 run rmf_layered_map_server layered_map_benchmark \
  --label candidate --scenario reset
ros2 run rmf_layered_map_server layered_map_benchmark \
  --label candidate --scenario rolling-overlap
ros2 run rmf_layered_map_server layered_map_benchmark \
  --label candidate --scenario rolling-moving
```

For an A/B comparison, run the same benchmark source and arguments against both revisions. Compare performance only when the checksums match.

## Visualization Smoke Test

Build and source the workspace:

```bash
cd /home/dell/workspaces/mapf_ws
colcon build --packages-select rmf_layered_map_msgs rmf_layered_map_server rmf_layered_map_server_demo
source install/setup.bash
```

Start the demo:

```bash
ros2 launch rmf_layered_map_server_demo demo.launch.py
```

The launch file starts RViz with a `Map` display on `/map` and the fixed frame set to `map`. The demo publisher sends a bordered static map and a temporary obstacle region with a 5 second TTL. The obstacle should appear in the composed map and then disappear when the TTL expires.
