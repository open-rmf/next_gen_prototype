# rmf_path_server_test

This package provides system-level integration and launch tests for the Next-Gen RMF path server stack. It spins up the full path planning and plan execution pipeline to validate path planning coordination, dynamic path redirection, and collision avoidance under different scenarios.

---

## Launch Testing

The package includes python-based ROS 2 `launch_testing` scenarios under the `test/` directory. These tests run integrated environments containing the following nodes:
* `rmf_path_server` (Rust): The core destinations path planner.
* `rmf_plan_executor` (Rust): Coordinates plan tracking, semantics, and departure blocker verification.
* `rmf_mock_robot_sim` (Python): Simulates robot foot kinematics, publishes odometry, and follows planned paths.

### Test Scenarios
1. **Dynamic Path Redirection Scenario** (`test/test_path_server_scenario.py`):
   * Spawns two robots starting at `(0,0)` and `(3,0)`.
   * Targets `robot_1` to `(5,0)` and `robot_2` to `(3,0)`.
   * Once `robot_1` crosses `x >= 1.9` mid-route, the test dynamically publishes a new destination constraint targeting `(5.0, 5.0)` to verify the path server's capability to recalculate paths on-the-fly and safely redirect robots.
   * On success, saves the recorded timeline to `trajectories/scenario_test_trajectory.csv`.

2. **Follow & Collision Avoidance Scenario** (`test/test_path_server_follow.py`):
   * Spawns a fast robot (`robot_1`, speed `4.0`) behind a slow robot (`robot_2`, speed `1.0`).
   * Targets them to `(9.0, 0.0)` and `(10.0, 0.0)` respectively.
   * Asserts that `robot_1` safely follows `robot_2` without colliding (verifies that they maintain a distance of at least `0.95m` at all times, coordinating via departure blockers).
   * On success, saves the recorded timeline to `trajectories/follow_test_trajectory.csv`.

### Running the Tests
To build and run all tests (including flake8, copyright, and pep257 linters):
```bash
colcon build --packages-select rmf_path_server_test
source install/setup.bash
colcon test --packages-select rmf_path_server_test
```
To check test results:
```bash
colcon test-result --all --verbose
```

---

## Trajectory Visualizer

We provide a lightweight, interactive web-based visualizer tool to replay and analyze the trajectory CSV files produced during testing. This is particularly useful for developers ssh'd into remote development servers.

### Running the Visualizer
After sourcing your workspace, run the visualizer CLI script directly using `ros2 run`:
```bash
ros2 run rmf_path_server_test rmf_visualize_trajectory <path_to_csv_file>
```
Example:
```bash
ros2 run rmf_path_server_test rmf_visualize_trajectory src/next_gen_prototype/path_server/rmf_path_server_test/trajectories/scenario_test_trajectory.csv
```

#### CLI Options
* `--port <number>`: Port to run the server on (default: `9567`).
* `--open`: Attempt to open the visualizer in your local default browser automatically.

#### Remote Access (SSH Port Forwarding)
If running on a remote server, you can forward the visualizer port to your local machine using:
```bash
ssh -L 9567:localhost:9567 <user>@<remote_host>
```
Then navigate to `http://localhost:9567/` in your local web browser.

### Visualizer Features
* **Timeline Playback**: Play, pause, loop, or scrub through the timeline slider. Speed selectors let you adjust replaying speeds from `0.25x` to `10.0x`.
* **Interactive Navigation**: Drag inside the grid to pan, and scroll your mouse wheel to zoom (zooms centered on your mouse cursor). Double-click anywhere to auto-fit the view to the trajectories bounds.
* **Footprint Configurator**: Real-time slider to change the rendering footprints (radius) of the robots, making it easy to identify spatial conflicts or close-calls.
* **Camera Focus Tracking**: Click the target/crosshair icon next to any robot in the sidebar to auto-center and keep the camera locked on it as it travels.
* **Interactive Grid & Tooltips**: Displays coordinates under the cursor. Hovering over a robot reveals a tooltip showing its current spatial coordinate coordinates, status, and velocity (calculated in real-time between timestamps).
* **Trajectory Trails**: Toggles showing faint dotted path outlines to distinguish planned routes from traveled segments.