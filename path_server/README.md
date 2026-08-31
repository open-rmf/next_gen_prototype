# Path Server

This folder contains the path server and plan executor. The path server plans for new destinations and replans active robots that report `PlanError.CODE_PATH_BLOCKED`.

Planners supported:
- `pibt-grid-world` (default): Grid-world PIBT with external tracks (conservatively downsamples maps finer than `1.0 m`)
- `ccbs`: Continuous Conflict-Based Search (CCBS) with SIPP single-agent routing

## Path Server Demo Dashboard

There is a simple web-based dashboard that can be used to visualize the paths of the robots. It can be accessed at `http://localhost:8080`.

### 1. Direct Demo (PIBT by default, or CCBS)

```bash
# Default (PIBT)
ros2 launch rmf_path_server_demo demo.launch.py

# Using CCBS planner
ros2 launch rmf_path_server_demo demo.launch.py planner:=ccbs
```

### 2. Map Demo (With Pre-existing Map)

To run the demo with a pre-existing grid map:

```bash
# Default (1.0m resolution demo_grid.yaml)
ros2 launch rmf_path_server_demo demo_map.launch.py planner:=ccbs

# 0.1m fine-resolution demo map (demo_grid_0_1m.yaml) for footprint clearance testing
ros2 launch rmf_path_server_demo demo_map.launch.py planner:=ccbs map:=demo_grid_0_1m

# Using custom map file
ros2 launch rmf_path_server_demo demo_map.launch.py planner:=ccbs map_file:=/path/to/map.yaml
```

