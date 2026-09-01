# RMF Path Server Demo (`rmf_path_server_demo`)

This package contains a web-based dashboard and tools to visualize and interactively test RMF path planning and destination allocation.

## Usage Modes

The demo can be run in two modes:

### 1. Direct Path Server Mode (Default)
In this mode, the dashboard bypasses any destination allocation logic and sends destination goals directly to the path server.

Run the launch file (PIBT by default, or CCBS):
```bash
# Default (PIBT)
ros2 launch rmf_path_server_demo demo.launch.py

# Switch to CCBS planner
ros2 launch rmf_path_server_demo demo.launch.py planner:=ccbs
```
- Open `http://localhost:8080` in your web browser.
- **Add Robot**: Click "Add Robot" and place robots on the grid canvas.
- **Set Goal**: Select a robot and click the grid to place its single destination.
- **Send**: Click "Send Scenario" to start simulation and planning.

### 2. Destination Server Mode (Alternative Goals Testing)
In this mode, the spawner publishes a list of alternative candidate goals as
`DestinationGoal` to a selectable destination server, which forwards the
selected goal as a `Destination` message to the path planner.

Run the launch file:
```bash
ros2 launch rmf_path_server_demo demo_destination_server.launch.py planner:=ccbs
```

The launch file defaults to `destination_server:=reservation`. Use
`destination_server:=simple` to forward the first candidate without occupancy
tracking, safe sets, queueing, or parking detours. See
[`reservation_system/README.md`](../../reservation_system/README.md) for the
full comparison.

- Open `http://localhost:8080` in your web browser.
- **Add Robot**: Click "Add Robot" and place robots on the grid canvas.
- **Set Goals**: Click the grid multiple times to place alternative goal
  locations. Conflicting goals can be assigned because the reservation server
  resolves them.
- **Switch or Finish**: Select another robot in the sidebar to switch goal
  editing. Select the current robot again to finish.
- **Send**: Click "Send Scenario" to submit goals for resolution and start planning.
- The canvas will highlight the chosen goals in solid colors and fade out the unused alternatives.

### 3. Map Server Mode (Pre-existing Grid Map Testing)
In this mode, the launch file spins up a ROS 2 `map_server` loaded with a pre-existing grid map (`demo_grid.yaml` at 1.0m, or `demo_grid_0_1m.yaml` at 0.1m) and activates it. The dashboard subscribes to `/map` and visualizes it as a background layer, and the path server incorporates the map and footprint sizes into collision-free planning.

Run the launch file:
```bash
# Run with CCBS planner and default 1.0m map (demo_grid.yaml)
ros2 launch rmf_path_server_demo demo_map.launch.py planner:=ccbs

# Run with CCBS planner and 0.1m fine-resolution map (demo_grid_0_1m.yaml) to test footprint sizes
ros2 launch rmf_path_server_demo demo_map.launch.py planner:=ccbs map:=demo_grid_0_1m

# Run with PIBT planner and pre-existing map
ros2 launch rmf_path_server_demo demo_map.launch.py planner:=pibt-grid-world

# Run with a custom map yaml file
ros2 launch rmf_path_server_demo demo_map.launch.py planner:=ccbs map_file:=/path/to/custom_grid.yaml
```
- Open `http://localhost:8080` in your web browser.
- The canvas will automatically fit to the loaded grid map.
- You can add robots and set goals as in the default mode, and observe robots with different footprint sizes taking appropriate clearances around obstacles and through passages.

## Reservation Config Visualization

When the reservation destination server loads a configuration, it republishes
the parsed config on the transient-local topic
`/destination/reservation_config`. The dashboard subscribes to this topic and
draws the config as a background layer on the canvas.

## Loading Reservation Config from RMF Site Editor

The reservation destination server accepts three configuration formats through
its `config_file` parameter:

```text
*.site.json      # Native RMF Site Editor project
*.building.yaml  # Legacy Traffic Editor building map
*.yaml           # Explicit reservation configuration
```

For example, the command below loads the config at `maps/office.site.json`:

```bash
ros2 launch rmf_path_server_demo demo_destination_server.launch.py \
  destination_server:=reservation \
  config_file:=$(ros2 pkg prefix rmf_path_server_demo)/share/rmf_path_server_demo/maps/office.site.json
```

The package includes example configs adapted from the Open-RMF office demo:

- [`maps/office.building.yaml`](https://github.com/open-rmf/rmf_demos/blob/main/rmf_demos_maps/maps/office/office.building.yaml)
- [`maps/office.site.json`](https://github.com/open-rmf/rmf_site_ros2/blob/main/rmf_site_demos/maps/office/office.site.json)

The server derives safe sets from floor polygons and parking spots from
locations tagged as `ParkingSpot`, then republishes them for the dashboard.
