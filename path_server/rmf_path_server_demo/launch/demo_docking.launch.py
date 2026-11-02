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
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_name = 'rmf_path_server_demo'
    share_dir = get_package_share_directory(package_name)

    default_map_yaml = os.path.join(share_dir, 'maps', 'corridor_grid.yaml')
    default_site_file = os.path.join(
        share_dir, 'maps', 'corridor_docks.site.json'
    )

    map_yaml_file = LaunchConfiguration('map_yaml_file')
    site_file = LaunchConfiguration('site_file')
    use_destination_server = LaunchConfiguration('use_destination_server')

    declare_map_yaml = DeclareLaunchArgument(
        'map_yaml_file',
        default_value=default_map_yaml,
        description='OccupancyGrid YAML map for the 3x8 corridor with protruding docks.',
    )
    declare_site_file = DeclareLaunchArgument(
        'site_file',
        default_value=default_site_file,
        description='RMF Site file (*.site.json) with dock lanes and locations.',
    )
    declare_use_destination_server = DeclareLaunchArgument(
        'use_destination_server',
        default_value='false',
        description='Whether to route goals through rmf_reservation_destination_server.',
    )

    # 1. Start the RMF path server
    path_server = Node(
        package='rmf_path_server',
        executable='rmf_path_server',
        name='rmf_path_server',
        output='both',
    )

    # 2. Start the web spawner & web dashboard hosting server (REST & SSE Bridge)
    robot_spawner = Node(
        package='rmf_path_server_demo',
        executable='robot_spawner',
        name='robot_spawner',
        output='both',
        parameters=[{
            'site_file': site_file,
            'use_destination_server': use_destination_server,
        }],
    )

    # 3. Optionally start the reservation destination server
    reservation_destination_server = Node(
        package='rmf_reservation_destination_server',
        executable='rmf_reservation_destination_server',
        name='rmf_reservation_destination_server',
        output='both',
        parameters=[{'config_file': site_file}],
        condition=IfCondition(use_destination_server),
    )

    # 4. Start the plan executor
    plan_executor = Node(
        package='rmf_plan_executor',
        executable='rmf_plan_executor',
        name='rmf_plan_executor',
        output='both',
    )

    # 5. Start the map server with the 3x8 corridor + protruding docks map
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_yaml_file}],
    )

    # 6. Start the lifecycle manager to activate the map server
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

    return LaunchDescription([
        declare_map_yaml,
        declare_site_file,
        declare_use_destination_server,
        path_server,
        robot_spawner,
        reservation_destination_server,
        plan_executor,
        map_server,
        lifecycle_manager,
    ])
