# Path Server

This folder contains a path server implementation. It works by triggering a replan any time a new destination event comes in. The replan should only include robots that are actively moving.

Planners supported:
- `pibt-grid-world` (default): Grid-world PIBT with external tracks
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

To run the demo with a pre-existing 20x20 grid map (`demo_grid.yaml` / `demo_grid.png`) and obstacle layer:

```bash
# Default (PIBT) with pre-existing map
ros2 launch rmf_path_server_demo demo_map.launch.py

# Using CCBS planner with pre-existing map
ros2 launch rmf_path_server_demo demo_map.launch.py planner:=ccbs

# Using custom map file
ros2 launch rmf_path_server_demo demo_map.launch.py planner:=ccbs map_file:=/path/to/map.yaml
```

