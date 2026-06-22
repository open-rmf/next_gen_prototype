# rmf_path_visualizer

The `rmf_path_visualizer` node listens to the `~/plan` topic for a specific robot and publishes visualization markers (a `MarkerArray`) to the `~/path_markers` topic so you can view the planned path and safe zones in RViz.

## Running the Node

You must run a separate instance of the visualizer for each robot you want to visualize. The executable accepts a single positional argument for the `robot_name` namespace.

**Usage:**
```bash
ros2 run rmf_path_visualizer rmf_path_visualizer <robot_name>
```

**Examples:**
```bash
# Run the visualizer for robot0
ros2 run rmf_path_visualizer rmf_path_visualizer robot0

# Run the visualizer for robot1 in a separate terminal
ros2 run rmf_path_visualizer rmf_path_visualizer robot1
```

If no `robot_name` is provided, it will default to `robot0`.
