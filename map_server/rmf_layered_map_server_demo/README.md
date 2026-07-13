# Layered Map Server Demos

This package contains two demonstrations of the layered map server.

## TTL Smoke Test

This demo publishes a synthetic static grid and a temporary rectangle with a five-second TTL:

```bash
ros2 launch rmf_layered_map_server_demo demo.launch.py
```

## Three-Robot Nav2 Observation Demo

This demo combines laser observations from three stationary Nav2 robots in the global `/map`:

```bash
ros2 launch rmf_layered_map_server_demo nav2_observations.launch.py
```

Use `use_nav2_rviz:=False` to open only the combined-map view. Use `spawn_clutter:=False` to run against the unmodified warehouse world.
