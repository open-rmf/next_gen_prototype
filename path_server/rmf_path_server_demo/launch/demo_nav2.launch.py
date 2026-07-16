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
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def launch_nav2_traffic_instances(context, *args, **kwargs):
    robots_str = LaunchConfiguration('robots').perform(context)
    plugins_str = LaunchConfiguration('nav2_traffic_plugins').perform(context)
    use_nav2_traffic = LaunchConfiguration('use_nav2_traffic').perform(context).lower() in ['true', '1', 'yes']

    if not use_nav2_traffic:
        return []

    # Parse robot names from robots_str
    # Supports formats like:
    # "robot0 robot1"
    # "robot0, robot1"
    # "robot0={x: 0.0, y: 5.0, yaw: 0.0}; robot1={x: 3.0, y: 5.0, yaw: 0.0};"
    robot_names = []
    if '=' in robots_str:
        for part in robots_str.split(';'):
            part = part.strip()
            if '=' in part:
                name = part.split('=')[0].strip()
                if name:
                    robot_names.append(name)
    else:
        for token in robots_str.replace(',', ' ').split():
            token = token.strip()
            if token:
                robot_names.append(token)

    if not robot_names:
        robot_names = ['robot0', 'robot1']

    nodes = []
    for robot_name in robot_names:
        params = [
            {'use_sim_time': True},
            {'agent_name': robot_name},
        ]
        if plugins_str:
            params.append({'plugins': plugins_str})

        nodes.append(
            Node(
                package='rmf_nav2_traffic',
                executable='nav2_traffic',
                name=f'{robot_name}_nav2_traffic',
                output='both',
                parameters=params,
            )
        )
    return nodes


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
    use_path_server = LaunchConfiguration('use_path_server')
    use_visualizer = LaunchConfiguration('use_visualizer')
    use_destination_server = LaunchConfiguration('use_destination_server')
    use_plan_executor = LaunchConfiguration('use_plan_executor')

    declare_robots = DeclareLaunchArgument(
        'robots',
        default_value='robot0 robot1',
        description='Space, comma, or dict-formatted list of robots to launch nav2_traffic for.',
    )
    declare_nav2_traffic_plugins = DeclareLaunchArgument(
        'nav2_traffic_plugins',
        default_value='',
        description='Comma or space-separated list of nav2_traffic sub-plugins to enable '
                    '(e.g., destination_goal_publisher, safe_zone_subscription, '
                    'inner_navigation_client, navigation_server, nav2_agent).',
    )
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
    declare_use_path_server = DeclareLaunchArgument(
        'use_path_server', default_value='true', description='Whether to start path server'
    )
    declare_use_visualizer = DeclareLaunchArgument(
        'use_visualizer', default_value='true', description='Whether to start visualizer'
    )
    declare_use_destination_server = DeclareLaunchArgument(
        'use_destination_server', default_value='true', description='Whether to start destination server'
    )
    declare_use_plan_executor = DeclareLaunchArgument(
        'use_plan_executor', default_value='true', description='Whether to start plan executor'
    )
    declare_use_nav2_traffic = DeclareLaunchArgument(
        'use_nav2_traffic', default_value='true', description='Whether to start nav2_traffic instance(s)'
    )

    # 1. Start the RMF path server
    path_server = Node(
        package='rmf_path_server',
        executable='rmf_path_server',
        name='rmf_path_server',
        output='both',
        condition=IfCondition(use_path_server),
    )

    # 2. Start the visualizer node
    visualizer_node = Node(
        package='rmf_path_visualizer',
        executable='rmf_path_visualizer',
        name='rmf_path_visualizer',
        output='both',
        condition=IfCondition(use_visualizer),
    )

    # 3. Start exactly one selectable destination implementation.
    simple_destination_server = Node(
        package='rmf_simple_destination_server',
        executable='rmf_simple_destination_server',
        name='rmf_simple_destination_server',
        output='both',
        condition=IfCondition(
            PythonExpression([
                "'", use_destination_server, "' == 'true' and '",
                destination_server, "' == 'simple'"
            ])
        ),
    )
    reservation_destination_server = Node(
        package='rmf_reservation_destination_server',
        executable='rmf_reservation_destination_server',
        name='rmf_reservation_destination_server',
        output='both',
        parameters=[{'config_file': config_file}],
        condition=IfCondition(
            PythonExpression([
                "'", use_destination_server, "' == 'true' and '",
                destination_server, "' == 'reservation'"
            ])
        ),
    )

    # 4. Start the plan executor
    plan_executor = Node(
        package='rmf_plan_executor',
        executable='rmf_plan_executor',
        name='rmf_plan_executor',
        output='both',
        condition=IfCondition(use_plan_executor),
    )

    return LaunchDescription([
        declare_robots,
        declare_nav2_traffic_plugins,
        declare_destination_server,
        declare_config_file,
        declare_use_path_server,
        declare_use_visualizer,
        declare_use_destination_server,
        declare_use_plan_executor,
        declare_use_nav2_traffic,
        path_server,
        visualizer_node,
        simple_destination_server,
        reservation_destination_server,
        plan_executor,
        OpaqueFunction(function=launch_nav2_traffic_instances),
    ])
