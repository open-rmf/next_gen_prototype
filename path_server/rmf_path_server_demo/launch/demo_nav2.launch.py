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
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    # Reservation config the destination server loads and republishes for the
    # dashboard to visualize.
    default_config = os.path.join(
        get_package_share_directory('rmf_path_server_demo'),
        'config',
        'reservation_config.yaml',
    )
    config_file = LaunchConfiguration('config_file')
    destination_server = LaunchConfiguration('destination_server')
    declare_destination_server = DeclareLaunchArgument(
        'destination_server',
        default_value='reservation',
        description=(
            'Destination implementation to run: simple or reservation.'
        ),
        choices=['simple', 'reservation'],
    )
    declare_config_file = DeclareLaunchArgument(
        'config_file',
        default_value=default_config,
        description='Reservation config the destination server loads '
                    '(*.yaml, *.site.json, or *.building.yaml).',
    )

    # 1. Start the RMF path server
    path_server = Node(
        package='rmf_path_server',
        executable='rmf_path_server',
        name='rmf_path_server',
        output='both'
    )

    # 2. Start the visualizer node
    visualizer_node = Node(
        package='rmf_path_visualizer',
        executable='rmf_path_visualizer',
        name='rmf_path_visualizer',
        output='both'
    )

    # 3. Start exactly one selectable destination implementation.
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

    # 4. Start the plan executor
    plan_executor = Node(
        package='rmf_plan_executor',
        executable='rmf_plan_executor',
        name='rmf_plan_executor',
        output='both'
    )

    # 5. Start nav2_traffic
    nav2_traffic = Node(
        package='rmf_nav2_traffic',
        executable='nav2_traffic',
        name='nav2_traffic',
        output='both',
        arguments=['--config', os.path.join(
            get_package_share_directory('rmf_nav2_traffic'),
            'config',
            'config.yaml',
        )],
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        declare_destination_server,
        declare_config_file,
        path_server,
        visualizer_node,
        simple_destination_server,
        reservation_destination_server,
        plan_executor,
        nav2_traffic,
    ])
