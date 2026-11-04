# RMF Path Server Demo (`rmf_path_server_demo`)

This package contains a web-based dashboard and tools to visualize and interactively test RMF path planning and destination allocation.

## Usage Modes

The demo can be run in two modes:

### 1. Direct Path Server Mode (Default)
In this mode, the dashboard bypasses any destination allocation logic and sends destination goals directly to the path server.

Run the launch file:
```bash
ros2 launch rmf_path_server_demo demo.launch.py
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
ros2 launch rmf_path_server_demo demo_destination_server.launch.py
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

### 3. Map Server Mode (Grid Map Testing)
In this mode, the launch file also spins up a ROS 2 `map_server` loaded with a custom 20x20 grid map (`demo_grid.png`) and activates it. The dashboard subscribes to `/map` and visualizes it as a background layer.

Run the launch file:
```bash
ros2 launch rmf_path_server_demo demo_map.launch.py
```
- Open `http://localhost:8080` in your web browser.
- The canvas will automatically fit to the 20x20 grid map.
- You can add robots and set goals as in the default mode, and see the map obstacles (a box in the center) drawn on the canvas.

### 4. Docking Corridor Mode (3x8 Corridor with Protruding Docks)
In this mode, the launch file loads a 3x8 central corridor `OccupancyGrid` (`corridor_grid.yaml`) with 6 small protruding 1x1 dock alcoves (`dock_north_1..3` at `y=2` and `dock_south_1..3` at `y=-2` for `x in {-1, 1, 3}`) alongside a matching RMF site file (`corridor_docks.site.json`). Each spawned `rmf_mock_robot_sim` loads `corridor_docks.site.json` and randomly varies the transition time when entering or leaving a dock.

Run the launch file:
```bash
ros2 launch rmf_path_server_demo demo_docking.launch.py
```
To also enable the reservation destination server (`rmf_reservation_destination_server`) with `corridor_docks.site.json`:
```bash
ros2 launch rmf_path_server_demo demo_docking.launch.py use_destination_server:=true
```

### 5. Nav2 + Gazebo (`gz-sim`) Docking Corridor Mode
This mode brings up the same 3x8 corridor + 6 protruding dock alcoves layout in Gazebo (`worlds/corridor_docks.sdf`) with multi-robot TurtleBot3 Nav2 stacks (`sp_demo_nav2_bringup`), Gazebo ground-truth localization (`urdf/gz_waffle_ground_truth.sdf.xacro` + `ground_truth_localizer`), a common `opennav_docking` dock database (`config/corridor_docks.yaml`), a 0.05m Nav2 static map (`maps/corridor_nav2.yaml`), the 1.0m RMF planning grid (`maps/corridor_grid.yaml`) on `/map`, `rmf_reservation_destination_server` loaded with `maps/corridor_docks.site.json`, `rmf_path_server`, `rmf_plan_executor`, and `rmf_nav2_traffic`.

Run the launch file:
```bash
ros2 launch rmf_path_server_demo demo_corridor_nav2.launch.py
```

#### Automated Corridor & Docking Scenarios
You can automatically dispatch multi-robot scenarios in Gazebo + Nav2 using the `scenario` launch argument (or via the `demo_corridor_swap.launch.py` wrapper, which defaults to `scenario:=swap` and `loop:=true`):

```bash
# Continuous two-robot corridor swap
ros2 launch rmf_path_server_demo demo_corridor_swap.launch.py

# Or select a specific scenario directly on demo_corridor_nav2.launch.py:
ros2 launch rmf_path_server_demo demo_corridor_nav2.launch.py scenario:=dock_rotate_pass
```

Available `scenario` values:
- `none`: Bring up the simulation and Nav2 stacks without auto-dispatching goals (default for `demo_corridor_nav2.launch.py`).
- `swap`: Swap `robot0` from `(-2, 0)` to `(5, 0)` and `robot1` from `(5, 0)` to `(-2, 0)` across the corridor.
- `swap_then_new_goals`: Perform the initial corridor swap, then immediately dispatch follow-up goals (`robot0 -> (-1, 1)`, `robot1 -> (4, 0)`).
- `mid_swap_retarget`: Start a corridor swap and inject a new goal mid-swap redirecting `robot0` to `(3, -1)` once it crosses `x >= 0.0`.
- `dock_pass`: Spawn `robot0` at `(-1, 1, pi/2)` and `robot1` at `(-2, 1, 0)` and dispatch both up-front so `robot0` docks into `dock_north_1` at `(-1, 2, pi/2)` while `robot1` passes to `(1, 1, 0)`.
- `dock_rotate_pass`: Spawn `robot0` at `(0, 1, 0)` (facing East, wrong orientation for docking North) and `robot1` at `(-2, 1, 0)`, and dispatch both up-front so `robot0` translates to `(-1, 1, pi/2)`, rotates 90 degrees in place (gating final progress until `pi/2` is achieved), and docks into `(-1, 2, pi/2)` while traffic management routes `robot1` around to `(1, 1, 0)`.

Additional launch arguments:
- `loop:=true|false`: Continuously repeat the scenario rounds.
- `second_robot_delay_sec:=<float>`: Optional delay in seconds before dispatching `robot1`'s initial goal.
- `use_rviz:=true|false`: Launch RViz2 for each robot namespace alongside Gazebo.

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
