# Layered Map Server Demos

This package contains two demonstrations of the layered map server.

## TTL Smoke Test

This demo publishes a synthetic static grid and a temporary rectangle with a five-second TTL:

```bash
ros2 launch rmf_layered_map_server_demo demo.launch.py
```

Use `use_rviz:=False` for a headless run.

## Three-Robot Nav2 Observation Demo

This demo combines laser observations from three moving Nav2 robots in the global `/map`:

```bash
ros2 launch rmf_layered_map_server_demo nav2_observations.launch.py
```

![Three robot laser observations in the combined global map](docs/images/nav2_observations.png)

By default, the demo opens three robot-local RViz windows and one combined-map view, moves the robots between fixed goals, spawns demo obstacles, and retains scans for ten seconds.

* Set `use_nav2_rviz:=False` or `use_global_rviz:=False` to disable either RViz view.
* Set `move_robots:=False` to disable robot movement.
* Set `spawn_clutter:=False` to use the unmodified warehouse world.
* Set `reset_source:=True` to show only the latest scan from each robot.
* Use `map` and `params_file` to override the warehouse map and shared Nav2 parameters.
* `beam_stride:=1` and `publish_period_sec:=0.5` control scan sampling.
* `max_observation_range:=2.5` and `ttl_sec:=10.0` control range and retention.
