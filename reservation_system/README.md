# Destination server implementations

This directory contains two interchangeable destination servers. Both listen
for `DestinationGoal` messages on `<robot>/destination/goal` and publish the
selected `Destination` on `<robot>/destination`.

## Simple destination server

Package: `rmf_simple_destination_server`

The simple server is stateless. It forwards the first candidate supplied in
each goal. It does not track occupancy, enforce safe sets, queue robots, or
create parking detours.

```bash
ros2 run rmf_simple_destination_server rmf_simple_destination_server
```

## Reservation destination server

Package: `rmf_reservation_destination_server`

The reservation server tracks occupied destination regions, prevents
overlapping assignments, queues blocked robots, and can divert waiting robots
to configured parking spots. It also enforces configured safe sets and
publishes the loaded configuration on `/destination/reservation_config`.

Its `config_file` parameter accepts:

- Explicit reservation configuration: `*.yaml`
- Native RMF Site Editor project: `*.site.json`
- Legacy Traffic Editor map: `*.building.yaml`

```bash
ros2 run rmf_reservation_destination_server \
  rmf_reservation_destination_server \
  --ros-args -p config_file:=/path/to/map.site.json
```

## Selecting an implementation in the demo

The path server demo defaults to the reservation implementation:

```bash
ros2 launch rmf_path_server_demo demo_destination_server.launch.py
```

Select the stateless implementation with:

```bash
ros2 launch rmf_path_server_demo demo_destination_server.launch.py \
  destination_server:=simple
```

Select the reservation implementation and provide a map with:

```bash
ros2 launch rmf_path_server_demo demo_destination_server.launch.py \
  destination_server:=reservation \
  config_file:=/path/to/map.site.json
```

Only one destination server should run for a topic namespace at a time.
