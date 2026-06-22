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

1. Install `rust`, `colcon-cargo` and `colcon-cargo-ros`:
```
# Install Rust (see https://rustup.rs/)
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh

# Install required system packages
sudo apt install -y git libclang-dev python3-pip python3-vcstool

# Install colcon plugins for Rust
pip install --break-system-packages colcon-cargo colcon-ros-cargo

```

2. **Import Workspace Dependencies (via `.repos` file)**  
   Create a workspace and import the required ROS 2 Rust and navigation dependencies using this [`setup.repos`](https://gist.github.com/arjo129/1711acb6fdf2956640f022a1ebde3869):
   ```bash
   mkdir -p rmf_ws/src
   cd rmf_ws
   vcs import src --input https://gist.githubusercontent.com/arjo129/1711acb6fdf2956640f022a1ebde3869/raw/5786aa3d5e87a556c5c6769cc0983264b06a20d7/gistfile1.txt
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
