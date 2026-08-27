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
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup_launch(context, *args, **kwargs):
    package_name = 'rmf_path_server_demo'
    maps_dir = os.path.join(get_package_share_directory(package_name), 'maps')

    map_file_arg = LaunchConfiguration('map_file').perform(context)
    map_name_arg = LaunchConfiguration('map').perform(context)

    if map_file_arg:
        resolved_map_file = map_file_arg
    elif map_name_arg:
        if not map_name_arg.endswith('.yaml'):
            map_name_arg = f'{map_name_arg}.yaml'
        resolved_map_file = os.path.join(maps_dir, map_name_arg)
    else:
        resolved_map_file = os.path.join(maps_dir, 'demo_grid.yaml')

    planner_config = LaunchConfiguration('planner')

    # 1. Start the RMF path server
    path_server = Node(
        package='rmf_path_server',
        executable='rmf_path_server',
        name='rmf_path_server',
        output='both',
        parameters=[{'planner': planner_config}],
        arguments=['--planner', planner_config],
    )

    # 2. Start the web spawner & web dashboard hosting server (REST & SSE Bridge)
    robot_spawner = Node(
        package='rmf_path_server_demo',
        executable='robot_spawner',
        name='robot_spawner',
        output='both',
    )

    # 3. Start the plan executor
    plan_executor = Node(
        package='rmf_plan_executor',
        executable='rmf_plan_executor',
        name='rmf_plan_executor',
        output='both',
    )

    # 4. Start the map server
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': resolved_map_file}],
    )

    # 5. Start the lifecycle manager to activate the map server
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': ['map_server'],
        }],
    )

    return [
        path_server,
        robot_spawner,
        plan_executor,
        map_server,
        lifecycle_manager,
    ]


def generate_launch_description():
    declare_planner = DeclareLaunchArgument(
        'planner',
        default_value='pibt-grid-world',
        description='MAPF Planner to use: pibt-grid-world or ccbs',
        choices=['pibt-grid-world', 'pibt', 'ccbs'],
    )

    declare_map = DeclareLaunchArgument(
        'map',
        default_value='demo_grid',
        description='Pre-existing map name to load: demo_grid (1.0m) or demo_grid_0_1m (0.1m)',
    )

    declare_map_file = DeclareLaunchArgument(
        'map_file',
        default_value='',
        description='Explicit full path to custom map yaml file (overrides map argument)',
    )

    return LaunchDescription([
        declare_planner,
        declare_map,
        declare_map_file,
        OpaqueFunction(function=setup_launch),
    ])
