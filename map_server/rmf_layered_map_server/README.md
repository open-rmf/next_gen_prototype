# RMF Layered Map Server

`rmf_layered_map_server` composes a static `nav_msgs/OccupancyGrid` with temporary observation regions into the `/map` topic consumed by the path server.

Each robot or perception node publishes updates with its own `source_id`, and the server stitches all active observations into one composed global grid. Robot-mounted sources should also include `robot_name` metadata so the source can be correlated with the robot that observed it.

Default topics:

- `/map/static`: static `nav_msgs/OccupancyGrid`
- `/map/region_updates`: `rmf_layered_map_msgs/MapRegionUpdate`
- `/map`: composed `nav_msgs/OccupancyGrid`

Dynamic observations expire according to their TTL, so transient local obstacles do not remain in the global planning map forever. A LiDAR or local costmap observer can keep refreshing an obstacle while it is still seen, then let the server decay it after the TTL.

`MapRegionUpdate` messages contain a list of `MapRegionPatch` entries. A sensor
snapshot that clears freespace and marks obstacles can publish clear and obstacle
patches in the same message. The server applies clear patches before obstacle
patches. If the snapshot replaces the source's previous observation state, set
`reset_source` so the server removes old observations from that `source_id` and
`map_name` before adding the new patches. Resetting has no TTL. Clear and
obstacle patches both use TTLs in seconds.

Updates with a zero source timestamp are rejected.

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
