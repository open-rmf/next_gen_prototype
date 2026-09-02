# Path Server

This folder contains the path server and plan executor. The path server plans for new destinations and replans active robots that report `PlanError.CODE_PATH_BLOCKED`.

The plan executor checks each remaining route against updates to `/map`, with a 300 ms debounce and two-second cooldown. The grid-world PiBT planner conservatively downsamples maps finer than its configured planning resolution; other `MapfPlanner` implementations can be substituted.

## Planning parameters

The path server accepts these read-only startup parameters:

| Parameter | Default | Effect |
| --- | --- | --- |
| `planning_grid_resolution` | `1.0` m | Minimum PiBT grid cell size. Lower values preserve more map detail but increase planning work. Coarser maps are not upsampled. |

## Path Server Demo Dashboard

There is a simple web-based dashboard that can be used to visualize the paths of the robots. It can be accessed at http://localhost:8080.
To start the dashboard, run the following command: 

```bash
ros2 launch rmf_path_server_demo demo.launch.py
```
