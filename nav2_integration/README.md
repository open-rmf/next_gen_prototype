# Nav2 Integration for Next-Gen RMF

![image](./docs/images/nav2_traffic_demo.gif)

This folder contains the various packages required for Nav2 integration and demonstrates how Next Gen prototype messages and traffic management design is used for multi-robot coordination. The core `rmf_nav2_traffic` plugin manages incoming navigation requests and plans produced by the path server, and publishes action goals to the Nav2 server accordingly. We also incorporate [spatio_temporal_partition_layer](https://github.com/arjo129/spatio_temporal_partitioning) into the `rmf_nav2_traffic` plugin, where safe zone data is used to update the Nav2 costmap and provide partitions to facilitate path-planning.

## InnerNavigationServices Workflow Diagram

This document describes the workflow in `InnerNavigationServices`, specifically the `await_and_send_goal` workflow, implemented using Crossflow.

### Workflow Diagram

```mermaid
graph TD
    Start([scope.start]) --> ForkClone{Fork Clone}
    
    ForkClone --> AwaitReq[Await Requests]
    ForkClone --> AwaitExtCancel[Await External Cancellation]
    
    AwaitExtCancel -->|stream| CancelGoal[Cancel Goal]
    AwaitReq -->|stream| CheckGoal[Check Goal]
    
    CheckGoal -->|Ok| HandleExisting[Handle Existing Goal Result]
    CheckGoal -->|Err| Drop[Drop Outdated Request]
    
    HandleExisting -->|Ok| CancelGoal
    CancelGoal -->|Ok| RequestGoal[Request Goal]
    
    HandleExisting -->|Err| RequestGoal
    
    RequestGoal -->|Ok| UpdateGoal[Update Goal Client]
    UpdateGoal --> MonitorGoal[Monitor Navigation]
    MonitorGoal --> ProcessNav[Process Navigation Result]

    ProcessNav -->|Ok| RequestGoal
    ProcessNav -->|Err| CleanupGoal[Cleanup Goal Client]
    
    CancelGoal -->|Err| LogError[Log Error]
    RequestGoal -->|Err| LogError

    AwaitReq -->|output| Terminate([scope.terminate])
```

### Node Descriptions

- **`await_new_requests`**: A continuous service that listens for `InnerNavigationTarget` events and streams `InnerNavigationRequest` objects.
- **`await_external_cancellation`**: A continuous service that listens for `CancelInnerForAgent` events and streams cancellation requests.
- **`check_existing_goal`**: Checks if the agent already has an active goal. If the incoming request is outdated/duplicate, it returns an error and gets dropped. Otherwise it returns ok.
- **`handle_existing_goal_result`**: Routes the valid checked result into either a cancellation of the current goal or a direct request for a new goal.
- **`async_cancel_goal`**: An asynchronous service that cancels the existing Nav2 goal.
- **`async_request_new_goal`**: An asynchronous service that sends a new `NavigateToPose` goal to Nav2.
- **`update_goal_client`**: Updates the `InnerNavigationClient` component with the new goal handle.
- **`async_monitor_ongoing_navigation`**: Monitors the progress of the navigation goal, handling feedback and final results (Succeeded, Aborted, Cancelled).
- **`process_navigation_result`**: Processes the final result of the navigation request. If aborted, it prepares to retry by requesting a new goal; otherwise, it passes the result for cleanup.
- **`cleanup_goal_client`**: Cleans up the goal client state in the component upon completion or failure.
- **`log_inner_navigation_error`**: Logs any errors encountered during goal cancellation or request.


## NavigationServices Workflow Diagram

This document describes the workflow in `NavigationServices`, implemented using Crossflow.

### Workflow Diagram

```mermaid
graph TD
    Start([scope.start]) --> ForkClone{Fork Clone}
    
    ForkClone --> MonitorClients[Monitor Clients]
    ForkClone --> MonitorFeedback[Monitor Feedback]
    
    MonitorFeedback -- streams --> FeedbackBuffer[(Feedback Buffer)]
    
    MonitorClients -- streams --> BufferAccess[Buffer Access]
    FeedbackBuffer -.-> BufferAccess
    
    BufferAccess --> PublishFeedback[Publish Feedback]
    
    MonitorClients -- output --> Terminate([scope.terminate])
```

### Node Descriptions

- **`monitor_inner_navigation_clients`**: A continuous service that manages incoming navigation requests and responds to orders when the entire navigation is completed.
- **`monitor_inner_navigation_feedback`**: A continuous service that monitors feedback from inner navigation clients.
- **`FeedbackBuffer`**: A buffer that keeps the last 10 `InnerNavigationFeedback` items.
- **`Buffer Access`**: Accesses the `FeedbackBuffer` to retrieve feedback for requests coming from `monitor_inner_navigation_clients`.
- **`publish_navigation_feedback`**: A service that publishes the navigation feedback.


## Try out the demo


### Setup

Do a fresh update & upgrade:
```
sudo apt update && sudo apt upgrade -y
```

Set up a fresh workspace
```
mkdir ~/nav2_traffic_ws/src -p
cd ~/nav2_traffic_ws/src
git clone https://github.com/open-rmf/next_gen_prototype.git
```

Deps
```
# Install Rust (see https://rustup.rs/)
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh

# Install required system packages
sudo apt install -y git libclang-dev python3-pip

# Install colcon plugins for Rust
sudo apt install python3-colcon-cargo
pip3 install --break-system-packages colcon-ros-cargo
```

```
rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -yr
```

Build
```
colcon build --packages-up-to sp_demo_nav2_bringup rmf_nav2_traffic rmf_path_server_demo rmf_path_server_test
```

### Run

With the workspace built and sourced, run the following nodes:

Spin up the Nav2 simulation:
```
ros2 launch sp_demo_nav2_bringup cloned_multi_tb3_simulation_launch.py   robots:="robot0={x: 0.0, y: 5.0, yaw: 0.0}; robot1={x: 3.0, y: 5.0, yaw: 0.0};"
```

In a separate terminal, spin up the Nav2 traffic node:
```
ros2 run rmf_nav2_traffic nav2_traffic --ros-args -p use_sim_time:=true
```

Run the demo launch file containing the path server, plan executor, destination server, and path visualizer nodes:
```
ros2 launch rmf_path_server_demo demo_viz.launch.py robots:="robot0 robot1"
```

Send action goals to the robots:

```
ros2 action send_goal robot0/navigate_to_pose nav2_msgs/action/NavigateToPose "{
  pose: {
    header: {
      stamp: {sec: 0, nanosec: 0},
      frame_id: 'map'
    },
    pose: {
      position: {x: 5.0, y: 5.0, z: 0.0},
      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
    }
  }
}"
```

```
ros2 action send_goal robot1/navigate_to_pose nav2_msgs/action/NavigateToPose "{
  pose: {
    header: {
      stamp: {sec: 0, nanosec: 0},
      frame_id: 'map'
    },
    pose: {
      position: {x: 0.0, y: 3.0, z: 0.0},
      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
    }
  }
}"
```

Alternate navigation goals:

```
ros2 action send_goal robot0/navigate_to_pose nav2_msgs/action/NavigateToPose "{
  pose: {
    header: {
      stamp: {sec: 0, nanosec: 0},
      frame_id: 'map'
    },
    pose: {
      position: {x: 9.0, y: 3.0, z: 0.0},
      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
    }
  }
}"
```

```
ros2 action send_goal robot1/navigate_to_pose nav2_msgs/action/NavigateToPose "{
  pose: {
    header: {
      stamp: {sec: 0, nanosec: 0},
      frame_id: 'map'
    },
    pose: {
      position: {x: 9.0, y: 6.0, z: 0.0},
      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
    }
  }
}"
```