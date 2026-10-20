# Copyright 2026 Open Source Robotics Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnShutdown
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from nav2_common.launch import ParseMultiRobotPose, RewrittenYaml


def generate_launch_description():
    """Launch the 3x8 corridor + 6 dock alcoves environment in gz-sim with Nav2."""
    demo_share = get_package_share_directory('rmf_path_server_demo')
    nav2_share = get_package_share_directory('sp_demo_nav2_bringup')

    default_nav2_map = os.path.join(demo_share, 'maps', 'corridor_nav2.yaml')
    default_planning_map = os.path.join(
        demo_share, 'maps', 'corridor_grid.yaml'
    )
    default_world = os.path.join(demo_share, 'worlds', 'corridor_docks.sdf')
    default_robot_sdf = os.path.join(
        demo_share, 'urdf', 'gz_waffle_ground_truth.sdf.xacro'
    )
    default_config = os.path.join(
        demo_share, 'maps', 'corridor_docks.site.json'
    )
    default_docks = os.path.join(
        demo_share, 'config', 'corridor_docks.yaml'
    )
    default_params = os.path.join(
        nav2_share, 'params', 'nav2_multirobot_params_all.yaml'
    )
    default_swap_robots = (
        'robot0={x: -2.0, y: 0.0, yaw: 0.0}; '
        'robot1={x: 5.0, y: 0.0, yaw: 3.14159}'
    )
    default_dock_pass_robots = (
        'robot0={x: -1.0, y: 1.0, yaw: 1.5708}; '
        'robot1={x: -2.0, y: 1.0, yaw: 0.0}'
    )
    default_dock_rotate_pass_robots = (
        'robot0={x: 0.0, y: 1.0, yaw: 0.0}; '
        'robot1={x: -2.0, y: 1.0, yaw: 0.0}'
    )

    map_file = LaunchConfiguration('map')
    planning_map_file = LaunchConfiguration('planning_map')
    world_file = LaunchConfiguration('world')
    robot_sdf_file = LaunchConfiguration('robot_sdf')
    config_file = LaunchConfiguration('config_file')
    docks_file = LaunchConfiguration('docks_file')
    destination_server = LaunchConfiguration('destination_server')
    robots = LaunchConfiguration('robots')
    params_file = LaunchConfiguration('params_file')
    use_rviz = LaunchConfiguration('use_rviz')
    staged_startup = LaunchConfiguration('staged_startup')
    scenario = LaunchConfiguration('scenario')
    swap_loop = LaunchConfiguration('swap_loop')
    second_robot_delay_sec = LaunchConfiguration('second_robot_delay_sec')
    grid_subdivision = LaunchConfiguration('grid_subdivision')
    blur_radius = LaunchConfiguration('blur_radius')

    default_robots = PythonExpression([
        f'"{default_dock_pass_robots}" if "',
        scenario,
        f'" == "dock_pass" else ("{default_dock_rotate_pass_robots}" if "',
        scenario,
        f'" == "dock_rotate_pass" else "{default_swap_robots}")',
    ])

    rewritten_params = RewrittenYaml(
        source_file=params_file,
        param_rewrites={
            'amcl.ros__parameters.tf_broadcast': 'False',
            'amcl.ros__parameters.update_min_d': '10000.0',
            'amcl.ros__parameters.update_min_a': '10000.0',
            (
                'controller_server.ros__parameters.general_goal_checker.'
                'stateful'
            ): 'False',
            (
                'controller_server.ros__parameters.general_goal_checker.'
                'xy_goal_tolerance'
            ): '0.10',
            (
                'controller_server.ros__parameters.FollowPath.'
                'xy_goal_tolerance'
            ): '0.10',
            (
                'global_costmap.global_costmap.ros__parameters.'
                'inflation_layer.inflation_radius'
            ): '0.30',
            (
                'global_costmap.global_costmap.ros__parameters.'
                'update_frequency'
            ): '20.0',
            (
                'global_costmap.global_costmap.ros__parameters.'
                'publish_frequency'
            ): '5.0',
            'docking_server.ros__parameters.dock_database': docks_file,
            (
                'docking_server.ros__parameters.simple_charging_dock.'
                'use_external_detection_pose'
            ): 'False',
            (
                'docking_server.ros__parameters.simple_charging_dock.'
                'staging_x_offset'
            ): '-1.0',
        },
        convert_types=True,
    )

    declarations = [
        DeclareLaunchArgument(
            'map',
            default_value=default_nav2_map,
            description='High-resolution static map for Nav2 localization and costmaps.',
        ),
        DeclareLaunchArgument(
            'planning_map',
            default_value=default_planning_map,
            description='1.0m OccupancyGrid map published on /map for RMF planning.',
        ),
        DeclareLaunchArgument(
            'world',
            default_value=default_world,
            description='Gazebo world SDF for the 3x8 corridor with 6 protruding docks.',
        ),
        DeclareLaunchArgument(
            'robot_sdf',
            default_value=default_robot_sdf,
            description='Robot SDF xacro with Gazebo ground-truth OdometryPublisher.',
        ),
        DeclareLaunchArgument(
            'config_file',
            default_value=default_config,
            description=(
                'Reservation config the destination server loads '
                '(*.yaml, *.site.json, or *.building.yaml).'
            ),
        ),
        DeclareLaunchArgument(
            'docks_file',
            default_value=default_docks,
            description=(
                'Common dock_database YAML file loaded by opennav_docking.'
            ),
        ),
        DeclareLaunchArgument(
            'destination_server',
            default_value='reservation',
            description='Destination implementation to run: simple or reservation.',
            choices=['simple', 'reservation'],
        ),
        DeclareLaunchArgument(
            'scenario',
            default_value='swap',
            description=(
                'Corridor scenario to execute: '
                'swap, dock_pass, dock_rotate_pass, swap_then_new_goals, '
                'mid_swap_retarget, or none.'
            ),
            choices=[
                'swap',
                'dock_pass',
                'dock_rotate_pass',
                'swap_then_new_goals',
                'mid_swap_retarget',
                'none',
            ],
        ),
        DeclareLaunchArgument(
            'robots',
            default_value=default_robots,
            description='Semicolon-separated robot initial poses for sp_demo_nav2_bringup.',
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Base Nav2 parameters shared by the simulated robots.',
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='True',
            description='Whether to open per-robot Nav2 RViz instances.',
        ),
        DeclareLaunchArgument(
            'staged_startup',
            default_value='True',
            description='Start each robot Nav2 stack after its transforms are ready.',
        ),
        DeclareLaunchArgument(
            'swap_loop',
            default_value='False',
            description='Continuously repeat the corridor swap back and forth.',
        ),
        DeclareLaunchArgument(
            'second_robot_delay_sec',
            default_value='0.0',
            description=(
                'Optional delay in seconds before sending robot1 goal '
                '(e.g. >0 to exercise staggered cross-plan replanning).'
            ),
        ),
        DeclareLaunchArgument(
            'grid_subdivision',
            default_value='5',
            description=(
                'Odd subdivision factor per planning map cell for safe zone '
                'spatial allocation (e.g. 5 gives 0.2m resolution from 1.0m).'
            ),
        ),
        DeclareLaunchArgument(
            'blur_radius',
            default_value='2.0',
            description=(
                'Blur radius in subdivided grid cells for safe zone expansion.'
            ),
        ),
    ]

    robots_list = ParseMultiRobotPose('robots').value()
    if not robots_list:
        robots_list = {
            'robot0': {'x': -2.0, 'y': 0.0, 'yaw': 0.0},
            'robot1': {'x': 5.0, 'y': 0.0, 'yaw': 3.14159},
        }

    ground_truth_localizers = [
        Node(
            package='rmf_path_server_demo',
            executable='ground_truth_localizer',
            name='ground_truth_localizer',
            namespace=f'{robot_name}/inner',
            output='screen',
            parameters=[{'use_sim_time': True}],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
        )
        for robot_name in robots_list
    ]

    # 1. Start Gazebo + multi-robot TurtleBot3 Nav2 simulation
    nav2_simulation = ExecuteProcess(
        cmd=[
            'ros2',
            'launch',
            'sp_demo_nav2_bringup',
            'cloned_multi_tb3_simulation_launch.py',
            ['robots:=', robots],
            ['map:=', map_file],
            ['world:=', world_file],
            ['robot_sdf:=', robot_sdf_file],
            ['params_file:=', rewritten_params],
            ['use_rviz:=', use_rviz],
            'use_navigation:=True',
            ['staged_startup:=', staged_startup],
        ],
        output='screen',
    )

    # 2. Start root map_server publishing the 1.0m planning grid on /map
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            {'yaml_filename': planning_map_file},
            {'use_sim_time': True},
        ],
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': ['map_server'],
        }],
    )

    # 3. Start the RMF path server
    path_server = Node(
        package='rmf_path_server',
        executable='rmf_path_server',
        name='rmf_path_server',
        output='both',
    )

    # 4. Start the visualizer node
    visualizer_node = Node(
        package='rmf_path_visualizer',
        executable='rmf_path_visualizer',
        name='rmf_path_visualizer',
        output='both',
    )

    # 5. Start the selected destination server
    simple_destination_server = Node(
        package='rmf_simple_destination_server',
        executable='rmf_simple_destination_server',
        name='rmf_simple_destination_server',
        output='both',
        condition=IfCondition(
            PythonExpression(["'", destination_server, "' == 'simple'"])
        ),
    )
    reservation_destination_server = Node(
        package='rmf_reservation_destination_server',
        executable='rmf_reservation_destination_server',
        name='rmf_reservation_destination_server',
        output='both',
        parameters=[{'config_file': config_file}],
        condition=IfCondition(
            PythonExpression(["'", destination_server, "' == 'reservation'"])
        ),
    )

    # 6. Start the plan executor
    plan_executor = Node(
        package='rmf_plan_executor',
        executable='rmf_plan_executor',
        name='rmf_plan_executor',
        output='both',
        parameters=[{
            'grid_subdivision': ParameterValue(
                grid_subdivision, value_type=int
            ),
            'blur_radius': ParameterValue(blur_radius, value_type=float),
        }],
    )

    # 7. Start nav2_traffic
    nav2_traffic = Node(
        package='rmf_nav2_traffic',
        executable='nav2_traffic',
        name='nav2_traffic',
        output='both',
        parameters=[{'use_sim_time': True}],
    )

    # 8. Optionally start the automatic corridor scenario client node
    corridor_swap_demo = Node(
        package='rmf_path_server_demo',
        executable='corridor_swap_demo',
        name='corridor_swap_demo',
        output='screen',
        parameters=[{
            'scenario': scenario,
            'loop': ParameterValue(swap_loop, value_type=bool),
            'second_robot_delay_sec': ParameterValue(
                second_robot_delay_sec, value_type=float
            ),
        }],
        condition=IfCondition(
            PythonExpression(["'", scenario, "' != 'none'"])
        ),
    )

    cleanup_on_shutdown = RegisterEventHandler(
        event_handler=OnShutdown(
            on_shutdown=[
                OpaqueFunction(
                    function=lambda _: (
                        os.system(
                            'pkill -f "[g]z sim|[c]omponent_container_isolated|'
                            '[p]arameter_bridge|[s]taged_nav2_init" '
                            '2>/dev/null || true'
                        ),
                        None,
                    )[1]
                )
            ]
        )
    )

    return LaunchDescription([
        *declarations,
        nav2_simulation,
        *ground_truth_localizers,
        map_server,
        lifecycle_manager,
        path_server,
        visualizer_node,
        simple_destination_server,
        reservation_destination_server,
        plan_executor,
        nav2_traffic,
        corridor_swap_demo,
        cleanup_on_shutdown,
    ])
