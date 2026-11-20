# Path Server

This folder contains the path server and plan executor. The path server plans for new destinations and replans active robots that report `PlanError.CODE_PATH_BLOCKED`.

The plan executor checks each remaining route against updates to `/map`, with a 300 ms debounce and two-second cooldown. The grid-world PiBT planner conservatively downsamples maps finer than its configured planning resolution; other `MapfPlanner` implementations can be substituted.

## Planning parameters

The path server accepts these read-only startup parameters:

| Parameter | Default | Effect |
| --- | --- | --- |
| `planning_grid_resolution` | `1.0` m | Minimum PiBT grid cell size. Lower values preserve more map detail but increase planning work. Coarser maps are not upsampled. |

The plan executor (`rmf_plan_executor`) accepts these startup parameters (and matching CLI flags `--grid-subdivision`, `--blur-radius`):

| Parameter | Default | Effect |
| --- | --- | --- |
| `grid_subdivision` | `1` | Integer subdivision factor (`1..=64`) applied to `/map` cells when constructing the SafeZone Voronoi allocation grid (automatically capped to keep total cells within `4,000,000`). |
| `blur_radius` | `1.0` cells | Footprint dilation / blur radius in subdivided grid cells (Chebyshev step count) passed to `mapf_post::Grid2D` during SafeZone allocation. |

### `{robot}/plan/error` QoS

`{robot}/plan/error` uses `RELIABLE` + `TRANSIENT_LOCAL` (`keep_last(1)`) so late-joining subscribers do not miss an active `CODE_PATH_BLOCKED` error; the path server validates `PlanError.plan_id` against the robot's active `PlanId` to ignore stale latched errors from superseded plans.

## Path Server Demo Dashboard

There is a simple web-based dashboard that can be used to visualize the paths of the robots. It can be accessed at http://localhost:8080.
To start the dashboard, run the following command: 

```bash
ros2 launch rmf_path_server_demo demo.launch.py
```
