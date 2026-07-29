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
from nav2_common.launch import RewrittenYaml

from .replan_scenarios import get_scenario


def _robots_argument(scenario):
    """Format robot poses for Nav2 bringup."""
    return '; '.join(
        f'{robot.name}={{x: {robot.pose[0]}, y: {robot.pose[1]}, '
        f'yaw: {robot.pose[2]}}}'
        for robot in scenario.robots
    )


def generate_replan_launch_description(scenario_name):
    """Build the replanning launch description."""
    scenario_config = get_scenario(scenario_name)
    demo_share = get_package_share_directory('rmf_layered_map_server_demo')
    nav2_share = get_package_share_directory('sp_demo_nav2_bringup')

    if scenario_name == 'simple':
        default_map = os.path.join(demo_share, 'maps', 'single_room.yaml')
        default_world = os.path.join(demo_share, 'worlds', 'single_room.sdf')
    else:
        default_map = os.path.join(nav2_share, 'maps', 'warehouse.yaml')
        default_world = os.path.join(
            get_package_share_directory('nav2_minimal_tb4_sim'),
            'worlds',
            'warehouse.sdf',
        )

    map_file = LaunchConfiguration('map')
    world_file = LaunchConfiguration('world')
    world_name = LaunchConfiguration('world_name')
    params_file = LaunchConfiguration('params_file')
    use_nav2_rviz = LaunchConfiguration('use_nav2_rviz')
    use_global_rviz = LaunchConfiguration('use_global_rviz')
    spawn_delay_sec = LaunchConfiguration('spawn_delay_sec')
    scenario_timeout_sec = LaunchConfiguration('scenario_timeout_sec')
    ttl_sec = LaunchConfiguration('ttl_sec')
    self_filter_radius = LaunchConfiguration('self_filter_radius')
    demo_params_file = RewrittenYaml(
        source_file=params_file,
        param_rewrites={
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
        },
        convert_types=True,
    )

    declarations = [
        DeclareLaunchArgument(
            'map',
            default_value=default_map,
            description=f'Static map for the {scenario_name} scenario.',
        ),
        DeclareLaunchArgument(
            'world',
            default_value=default_world,
            description=f'Gazebo world for the {scenario_name} scenario.',
        ),
        DeclareLaunchArgument(
            'world_name',
            default_value=scenario_config.world_name,
            description='Gazebo world used by the obstacle spawner.',
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(
                nav2_share, 'params', 'nav2_multirobot_params_all.yaml'
            ),
            description='Nav2 parameters shared by both robots.',
        ),
        DeclareLaunchArgument(
            'use_nav2_rviz',
            default_value='False',
            description='Whether to open one local Nav2 RViz per robot.',
        ),
        DeclareLaunchArgument(
            'use_global_rviz',
            default_value='True',
            description='Whether to open the combined map and plan view.',
        ),
        DeclareLaunchArgument(
            'spawn_delay_sec',
            default_value='1.0',
            description='Delay between initial plans and obstacle spawn.',
        ),
        DeclareLaunchArgument(
            'scenario_timeout_sec',
            default_value='180.0',
            description='Timeout for observing a replan.',
        ),
        DeclareLaunchArgument(
            'ttl_sec',
            default_value=str(scenario_config.ttl_sec),
            description='Lifetime of observations that are not re-observed.',
        ),
        DeclareLaunchArgument(
            'self_filter_radius',
            default_value='0.22',
            description='Radius excluded from each robot scan.',
        ),
    ]

    nav2_simulation = ExecuteProcess(
        cmd=[
            'ros2',
            'launch',
            'sp_demo_nav2_bringup',
            'cloned_multi_tb3_simulation_launch.py',
            f'robots:={_robots_argument(scenario_config)}',
            ['map:=', map_file],
            ['world:=', world_file],
            ['params_file:=', demo_params_file],
            ['use_rviz:=', use_nav2_rviz],
            'use_navigation:=True',
            'staged_startup:=True',
        ],
        output='screen',
    )

    layered_map_server = Node(
        package='rmf_layered_map_server',
        executable='rmf_layered_map_server',
        output='screen',
        remappings=[('/map/static', '/robot0/inner/map')],
    )
    region_visualizer = Node(
        condition=IfCondition(use_global_rviz),
        package='rmf_layered_map_server_demo',
        executable='region_update_visualizer',
        output='screen',
        parameters=[{
            'output_topic': '/map/scan_region_markers',
            'show_obstacles': False,
        }],
    )
    source_contribution_visualizer = Node(
        condition=IfCondition(use_global_rviz),
        package='rmf_layered_map_server_demo',
        executable='source_contribution_visualizer',
        output='screen',
    )
    observation_nodes = [
        Node(
            package='rmf_layered_map_server_demo',
            executable='scan_region_publisher',
            namespace=f'{robot.name}/inner',
            output='screen',
            parameters=[{
                'robot_name': robot.name,
                'map_frame': 'map',
                'map_name': scenario_config.map_name,
                'scan_topic': f'/{robot.name}/inner/scan',
                'beam_stride': 1,
                'publish_period_sec': 0.5,
                'ttl_sec': ParameterValue(ttl_sec, value_type=float),
                'self_filter_radius': ParameterValue(
                    self_filter_radius, value_type=float
                ),
                'max_observation_range': scenario_config.scan_radius,
            }],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
        )
        for robot in scenario_config.robots
    ]

    path_server = Node(
        package='rmf_path_server',
        executable='rmf_path_server',
        name='rmf_path_server',
        output='both',
    )
    destination_server = Node(
        package='rmf_simple_destination_server',
        executable='rmf_simple_destination_server',
        name='rmf_simple_destination_server',
        output='both',
    )
    plan_executor = Node(
        package='rmf_plan_executor',
        executable='rmf_plan_executor',
        name='rmf_plan_executor',
        output='both',
    )
    nav2_traffic = Node(
        package='rmf_nav2_traffic',
        executable='nav2_traffic',
        name='nav2_traffic',
        output='both',
        parameters=[{'use_sim_time': True}],
    )
    path_visualizer = Node(
        condition=IfCondition(use_global_rviz),
        package='rmf_path_visualizer',
        executable='rmf_path_visualizer',
        name='rmf_path_visualizer',
        output='both',
    )
    scenario = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='rmf_layered_map_server_demo',
                executable='replan_obstacle_demo',
                output='screen',
                parameters=[{
                    'scenario': scenario_name,
                    'world_name': ParameterValue(world_name, value_type=str),
                    'spawn_delay_sec': ParameterValue(
                        spawn_delay_sec, value_type=float
                    ),
                    'scenario_timeout_sec': ParameterValue(
                        scenario_timeout_sec, value_type=float
                    ),
                }],
            ),
        ],
    )

    global_rviz = Node(
        condition=IfCondition(use_global_rviz),
        package='rviz2',
        executable='rviz2',
        name='replan_obstacle_rviz',
        arguments=[
            '-d',
            os.path.join(demo_share, 'rviz', 'nav2_observations.rviz'),
        ],
        parameters=[{'use_sim_time': False}],
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
        region_visualizer,
        source_contribution_visualizer,
        *observation_nodes,
        destination_server,
        path_server,
        plan_executor,
        nav2_traffic,
        path_visualizer,
        scenario,
        global_rviz,
        global_rviz_exit_handler,
    ])
