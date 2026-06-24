# Open-RMF Next Generation Prototyping

This repo is a temporary space for drafting prototype data structures and design
documents for the next generation of Open-RMF.

If you would like to give feedback or input on the designs, you are welcome to
open issue tickets, submit pull requests, or post to the [ideas board](https://discourse.ros.org/c/open-rmf/open-rmf-ideas/105)
on discourse.


# Setup & Building

:warning: As this is a project under construction, these instructions are likely to change rapidly.

## Pre-requisites

* ROS 2 jazzy
* The latest Rust compiler.


## Build Instructions


1. Install `rust` and `colcon-cargo`.

   **Method 1: Using a machine with ros2-jazzy installed**
   
   ```
   # Install Rust (see https://rustup.rs/)
   curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh

   # Install required system packages
   sudo apt install -y git libclang-dev python3-pip python3-vcstool

   # Install colcon plugins for Rust
   sudo apt install python3-colcon-cargo 
   ```

   **Method 2: Using a container**
   We pre-build base containers with all the dependencies you can use `rocker`, `distrobox`, `podman` or
   any other oci compliant tool to get a base dev environment. For the purposes of this set of instructions we use distrobox:
   ```
   distrobox create --image ghcr.io/open-rmf/ros2_rust_base:jazzy -n ros2_rust_base
   distrobox enter ros2_rust_base
   ```

2. **Import Workspace Dependencies (via `.repos` file)**  
   Create a workspace and clone the repo:
   ```bash
   mkdir -p rmf_ws/src
   cd rmf_ws/src
   git clone https://github.com/open-rmf/next_gen_prototype.git
   cd ..
   ```

3. **Build the PR Packages**  
   Ensure you are at the workspace root inside the `jazzy` distrobox container, then build the relevant packages:
   ```bash
   colcon build
   ```

## Running Automated Integration Tests
Verify core scenario coordination and robust following behavior:
```bash
colcon test --packages-select rmf_path_server_test --event-handlers console_direct+
```

## Running the Interactive Web Demonstration

https://github.com/user-attachments/assets/d5793eee-515d-49a9-a92c-9926617b48ed


Launch the fully standalone path server dashboard:
```bash
ros2 launch rmf_path_server_demo demo.launch.py
```
1. Open `http://localhost:8080` in your web browser.
2. Click **Add Robot** to drop active participants onto the canvas.
3. Select a robot and click a cell to place its goal.
4. Click **Send Scenario** to observe multi-agent trajectory generation and live execution progress.
