# rmf_nav2_traffic


# InnerNavigationServices Workflow Diagram

This document describes the workflow in `InnerNavigationServices`, specifically the `await_and_send_goal` workflow, implemented using Crossflow.

## Workflow Diagram

```mermaid
graph TD
    Start([scope.start]) --> ForkClone{Fork Clone}
    
    ForkClone --> AwaitReq[Await Requests]
    ForkClone --> AwaitExtCancel[Await External Cancellation]
    
    AwaitExtCancel -->|stream| CancelGoal[Cancel Goal]
    AwaitReq -->|stream| CheckGoal[Check Goal]
    
    CheckGoal -->|Ok| CancelGoal
    CancelGoal -->|Ok| RequestGoal[Request Goal]
    
    CheckGoal -->|Err| RequestGoal
    
    RequestGoal -->|Ok| UpdateGoal[Update Goal Client]
    UpdateGoal --> MonitorGoal[Monitor Navigation]
    MonitorGoal --> ProcessNav[Process Navigation Result]

    ProcessNav -->|Ok| RequestGoal
    ProcessNav -->|Err| CleanupGoal[Cleanup Goal Client]
    
    CancelGoal -->|Err| LogError[Log Error]
    RequestGoal -->|Err| LogError

    AwaitReq -->|output| Terminate([scope.terminate])
```

## Node Descriptions

- **`await_new_requests`**: A continuous service that listens for `InnerNavigationTarget` events and streams `InnerNavigationRequest` objects.
- **`await_external_cancellation`**: A continuous service that listens for `CancelInnerForAgent` events and streams cancellation requests.
- **`check_existing_goal`**: Checks if the agent already has an active goal. If so, it proceeds to cancel it; otherwise, it requests a new goal.
- **`async_cancel_goal`**: An asynchronous service that cancels the existing Nav2 goal.
- **`async_request_new_goal`**: An asynchronous service that sends a new `NavigateToPose` goal to Nav2.
- **`update_goal_client`**: Updates the `InnerNavigationClient` component with the new goal handle.
- **`async_monitor_ongoing_navigation`**: Monitors the progress of the navigation goal, handling feedback and final results (Succeeded, Aborted, Cancelled).
- **`process_navigation_result`**: Processes the final result of the navigation request. If aborted, it prepares to retry by requesting a new goal; otherwise, it passes the result for cleanup.
- **`cleanup_goal_client`**: Cleans up the goal client state in the component upon completion or failure.
- **`log_inner_navigation_error`**: Logs any errors encountered during goal cancellation or request.


# NavigationServices Workflow Diagram

This document describes the workflow in `NavigationServices`, implemented using Crossflow.

## Workflow Diagram

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

## Node Descriptions

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
```

Clone this repository to the workspace and import the relevant repositories
```
cd ~/nav2_traffic_ws/src
git clone https://github.com/open-rmf/next_gen_prototype.git -b xiyu/nav2
cd ~/nav2_traffic_ws
vcs import src < src/next_gen_prototype/rmf_nav2_traffic/nav2_traffic.repos
```

Deps
```
rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
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
ros2 launch rmf_path_server_demo demo.launch.py robots:="robot0 robot1"
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
}" --feedback
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
}" --feedback
```