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
In this mode, the spawner publishes a list of alternative candidate goals as `DestinationGoal` to the `rmf_simple_destination_server`. The destination server resolves conflicting bookings, allocates one of the goals to each robot, and forwards it as a `Destination` message to the path planner.

Run the launch file:
```bash
ros2 launch rmf_path_server_demo demo_destination_server.launch.py
```
- Open `http://localhost:8080` in your web browser.
- **Add Robot**: Click "Add Robot" and place robots on the grid canvas.
- **Set Goals**: Click the grid multiple times to place **multiple alternative goal locations** (labeled `G_<robot_id>.<index>`). Conflicting goals can be assigned as the destination server will resolve them automatically.
- **Switch or Finish**: Select another robot in the sidebar to switch goal editing to it. Select the current robot again in the sidebar list to finish. Canvas clicks always place goals while editing, including on another robot's current position.
- **Send**: Click "Send Scenario" to submit goals for resolution and start planning.
- The canvas will highlight the chosen goals in solid colors and fade out the unused alternatives.

## Reservation Config Visualization

When the destination server loads a reservation configuration, it republishes the parsed config on the transient-local topic `/destination/reservation_config`. The dashboard subscribes to this topic and draws the config as a background layer on the canvas.
