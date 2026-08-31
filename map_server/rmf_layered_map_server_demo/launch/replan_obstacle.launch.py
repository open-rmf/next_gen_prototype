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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

from rmf_layered_map_server_demo.replan_obstacle_launch import (
    generate_replan_launch_description,
)


def _launch_scenario(context):
    scenario = LaunchConfiguration('scenario').perform(context)
    return generate_replan_launch_description(scenario).entities


def generate_launch_description():
    """Launch a replanning scenario."""
    return LaunchDescription([
        DeclareLaunchArgument(
            'scenario',
            default_value='simple',
            choices=['simple', 'warehouse'],
            description='Replanning environment to launch.',
        ),
        OpaqueFunction(function=_launch_scenario),
    ])
