# Path Server

This folder contains the path server and plan executor. The path server plans for new destinations and replans active robots that report `PlanError.CODE_PATH_BLOCKED`.

The plan executor checks each remaining route against updates to `/map`, with a 300 ms debounce and two-second cooldown. The grid-world PiBT planner conservatively downsamples maps finer than its configured planning resolution; other `MapfPlanner` implementations (such as `CcbsPlanner`) can be substituted.

## Supported Planners
- `pibt-grid-world` (default): Grid-world PIBT with external tracks
- `ccbs`: Continuous Conflict-Based Search (CCBS) with SIPP single-agent routing

## Planning parameters

The path server accepts these startup parameters:

| Parameter | Default | Effect |
| --- | --- | --- |
| `planner` | `pibt-grid-world` | MAPF planner to use (`pibt-grid-world` or `ccbs`). |
| `planning_grid_resolution` | `1.0` m | Planning grid cell size / resolution in meters. |

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
