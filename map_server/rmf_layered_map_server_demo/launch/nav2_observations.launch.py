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
    EmitEvent,
    ExecuteProcess,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


ROBOTS = (
    ('robot0', -12.0, -21.0, 0.7854),
    ('robot1', 10.0, -21.0, 2.3562),
    ('robot2', -12.0, 21.0, -0.7854),
)


def robots_argument():
    """Format the three warehouse-corner robot poses for Nav2 bringup."""
    return '; '.join(
        f'{name}={{x: {x}, y: {y}, yaw: {yaw}}}'
        for name, x, y, yaw in ROBOTS
    )


def generate_launch_description():
    demo_share = get_package_share_directory('rmf_layered_map_server_demo')
    nav2_share = get_package_share_directory('sp_demo_nav2_bringup')

    map_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_nav2_rviz = LaunchConfiguration('use_nav2_rviz')
    use_global_rviz = LaunchConfiguration('use_global_rviz')
    spawn_clutter = LaunchConfiguration('spawn_clutter')
    beam_stride = LaunchConfiguration('beam_stride')
    publish_period_sec = LaunchConfiguration('publish_period_sec')
    ttl_sec = LaunchConfiguration('ttl_sec')
    max_observation_range = LaunchConfiguration('max_observation_range')

    declarations = [
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(nav2_share, 'maps', 'warehouse.yaml'),
            description='Static map used by each Nav2 robot.',
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(
                nav2_share, 'params', 'nav2_multirobot_params_all.yaml'
            ),
            description='Nav2 parameters shared by the three robots.',
        ),
        DeclareLaunchArgument(
            'use_nav2_rviz',
            default_value='True',
            description='Whether to open one local Nav2 RViz window per robot.',
        ),
        DeclareLaunchArgument(
            'use_global_rviz',
            default_value='True',
            description='Whether to open the combined global-map RViz window.',
        ),
        DeclareLaunchArgument(
            'spawn_clutter',
            default_value='True',
            description='Whether to spawn one deterministic obstacle near each robot.',
        ),
        DeclareLaunchArgument(
            'beam_stride',
            default_value='1',
            description='Publish every Nth valid laser return as a point region.',
        ),
        DeclareLaunchArgument(
            'publish_period_sec',
            default_value='0.5',
            description='Minimum period between region snapshots from each robot.',
        ),
        DeclareLaunchArgument(
            'ttl_sec',
            default_value='1.0',
            description='Lifetime of each robot observation snapshot.',
        ),
        DeclareLaunchArgument(
            'max_observation_range',
            default_value='2.5',
            description='Maximum laser return distance represented globally.',
        ),
    ]

    # ParseMultiRobotPose in the Nav2 demo reads sys.argv directly.
    # Launch a managed child process so the robot list reaches that parser.
    nav2_simulation = ExecuteProcess(
        cmd=[
            'ros2',
            'launch',
            'sp_demo_nav2_bringup',
            'cloned_multi_tb3_simulation_launch.py',
            f'robots:={robots_argument()}',
            ['map:=', map_file],
            ['params_file:=', params_file],
            ['use_rviz:=', use_nav2_rviz],
            'use_navigation:=False',
        ],
        output='screen',
    )

    layered_map_server = Node(
        package='rmf_layered_map_server',
        executable='rmf_layered_map_server',
        output='screen',
        remappings=[('/map/static', '/robot0/inner/map')],
    )

    observation_nodes = []
    for robot_name, _, _, _ in ROBOTS:
        observation_nodes.append(
            Node(
                package='rmf_layered_map_server_demo',
                executable='scan_region_publisher',
                namespace=f'{robot_name}/inner',
                output='screen',
                parameters=[{
                    'robot_name': robot_name,
                    'map_frame': 'map',
                    'map_name': 'warehouse',
                    'scan_topic': f'/{robot_name}/inner/scan',
                    'beam_stride': ParameterValue(beam_stride, value_type=int),
                    'publish_period_sec': ParameterValue(
                        publish_period_sec, value_type=float
                    ),
                    'ttl_sec': ParameterValue(ttl_sec, value_type=float),
                    'max_observation_range': ParameterValue(
                        max_observation_range, value_type=float
                    ),
                }],
                remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
            )
        )

    clutter_spawner = TimerAction(
        period=5.0,
        actions=[
            Node(
                condition=IfCondition(spawn_clutter),
                package='rmf_layered_map_server_demo',
                executable='layered_map_demo_clutter_spawner',
                output='screen',
            ),
        ],
    )

    global_rviz = Node(
        condition=IfCondition(use_global_rviz),
        package='rviz2',
        executable='rviz2',
        name='layered_global_map_rviz',
        arguments=[
            '-d',
            os.path.join(demo_share, 'rviz', 'nav2_observations.rviz'),
        ],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )
    global_rviz_exit_handler = RegisterEventHandler(
        condition=IfCondition(use_global_rviz),
        event_handler=OnProcessExit(
            target_action=global_rviz,
            on_exit=EmitEvent(event=Shutdown(reason='global RViz exited')),
        ),
    )

    return LaunchDescription([
        *declarations,
        nav2_simulation,
        layered_map_server,
        *observation_nodes,
        clutter_spawner,
        global_rviz,
        global_rviz_exit_handler,
    ])
