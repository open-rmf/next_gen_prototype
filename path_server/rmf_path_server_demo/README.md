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
- **Finish Goal Setup**: Click the robot's name in the sidebar list again to finish setting goals for it.
- **Send**: Click "Send Scenario" to submit goals for resolution and start planning.
- The canvas will highlight the chosen goals in solid colors and fade out the unused alternatives.