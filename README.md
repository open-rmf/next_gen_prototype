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
* Cargo extensions for colcon


## Build Instructions


### 1. Install `ROS 2`, `rust` and `colcon-cargo`.

<details>
   <summary><b>Method 1: On your local system</b></summary>

> a. [Install ROS 2 Jazzy](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html). Make sure to [install the development tools](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html#install-development-tools-optional).
> b. Install `rust`:
>    ```bash
>    curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
>    ```
> c. Install colcon extensions:
>    ```bash
>    # Install required system packages
>    sudo apt install -y git libclang-dev python3-pip
>
>    # Install colcon plugins for Rust
>    sudo apt install python3-colcon-cargo
>    pip3 install --break-system-packages colcon-ros-cargo
>    cargo install cargo-ament-build
>    ```
</details>

<details>
   <summary><b>Method 2: Using a container</b></summary>

> We pre-build base containers with all the dependencies you can use `rocker`, `distrobox`, `podman` or any other oci compliant tool to get a base dev environment. For the purposes of this set of instructions we use distrobox:
> ```bash
> distrobox create --image ghcr.io/open-rmf/ros2_rust_base:jazzy -n ros2_rust_base
> distrobox enter ros2_rust_base
> ```
</details>

### 2. Import Workspace Dependencies
Create a workspace and clone the repo:
```bash
mkdir -p rmf_ws/src
cd rmf_ws/src
git clone https://github.com/open-rmf/next_gen_prototype.git
cd ..
```

### 3. Install ROS package dependencies
```bash
rosdep install --from-paths src --ignore-src --rosdistro jazzy -yr
```

### 4. Build the PR Packages
Ensure you are at the workspace root inside the `jazzy` distrobox container, then build the relevant packages:
```bash
source /opt/ros/jazzy/setup.bash
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
