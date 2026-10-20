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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Launch the Nav2 + gz-sim 3x8 corridor environment and run the swap."""
    demo_share = get_package_share_directory('rmf_path_server_demo')
    corridor_nav2_launch = os.path.join(
        demo_share, 'launch', 'demo_corridor_nav2.launch.py'
    )

    use_rviz = LaunchConfiguration('use_rviz')
    destination_server = LaunchConfiguration('destination_server')
    scenario = LaunchConfiguration('scenario')
    swap_loop = LaunchConfiguration('swap_loop')
    second_robot_delay_sec = LaunchConfiguration('second_robot_delay_sec')
    grid_subdivision = LaunchConfiguration('grid_subdivision')
    blur_radius = LaunchConfiguration('blur_radius')

    declarations = [
        DeclareLaunchArgument(
            'use_rviz',
            default_value='True',
            description='Whether to open per-robot Nav2 RViz instances.',
        ),
        DeclareLaunchArgument(
            'destination_server',
            default_value='reservation',
            description=(
                'Destination implementation to run: simple or reservation.'
            ),
            choices=['simple', 'reservation'],
        ),
        DeclareLaunchArgument(
            'scenario',
            default_value='swap',
            description=(
                'Corridor scenario to execute: '
                'swap, dock_pass, swap_then_new_goals, or mid_swap_retarget.'
            ),
            choices=[
                'swap',
                'dock_pass',
                'swap_then_new_goals',
                'mid_swap_retarget',
                'none',
            ],
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

    corridor_nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(corridor_nav2_launch),
        launch_arguments={
            'use_rviz': use_rviz,
            'destination_server': destination_server,
            'scenario': scenario,
            'swap_loop': swap_loop,
            'second_robot_delay_sec': second_robot_delay_sec,
            'grid_subdivision': grid_subdivision,
            'blur_radius': blur_radius,
        }.items(),
    )

    return LaunchDescription([
        *declarations,
        corridor_nav2,
    ])
