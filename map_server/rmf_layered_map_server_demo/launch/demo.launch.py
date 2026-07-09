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
from launch.actions import EmitEvent
from launch.actions import RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('rmf_layered_map_server_demo')
    rviz_config = LaunchConfiguration('rviz_config')
    use_rviz = LaunchConfiguration('use_rviz')

    declare_rviz_config = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(package_share, 'rviz', 'layered_map.rviz'),
        description='Full path to the RViz config file to use.',
    )
    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        description='Whether to start RViz.',
    )

    layered_map_server = Node(
        package='rmf_layered_map_server',
        executable='rmf_layered_map_server',
        output='screen',
    )
    demo_publisher = Node(
        package='rmf_layered_map_server_demo',
        executable='layered_map_demo_publisher',
        output='screen',
    )
    rviz = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
    )
    rviz_exit_handler = RegisterEventHandler(
        condition=IfCondition(use_rviz),
        event_handler=OnProcessExit(
            target_action=rviz,
            on_exit=EmitEvent(event=Shutdown(reason='rviz exited')),
        ),
    )

    return LaunchDescription([
        declare_rviz_config,
        declare_use_rviz,
        layered_map_server,
        demo_publisher,
        rviz,
        rviz_exit_handler,
    ])
