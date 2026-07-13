# Map Server

This folder contains the layered map server packages.

## Demo

The demo launches the server, a small publisher that sends a static map and a temporary obstacle region, and RViz with a `Map` display on `/map`.

```bash
ros2 launch rmf_layered_map_server_demo demo.launch.py
```

Use `use_rviz:=False` for a headless run.

See [`rmf_layered_map_server_demo/README.md`](rmf_layered_map_server_demo/README.md) for the three-robot Nav2 observation demo.
