# Path Server

This folder contains a path server implementation. It works by triggering a replan any time a new destination event comes in. The replan should only include robots that are actively moving.
Currently it uses a grid-world PIBT based planner, but it is designed to work with any MapfPlanner implementation.

## Path Server Demo Dashboard

There is a simple web-based dashboard that can be used to visualize the paths of the robots. It can be accessed at http://localhost:8080.
To start the dashboard, run the following command: 

```bash
ros2 launch rmf_path_server_demo demo.launch.py
```
Nor the path followers do not follow any of the traffic dependencies prescribed by the plan.
