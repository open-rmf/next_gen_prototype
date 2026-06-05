from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # 1. Start the RMF path server
    path_server = Node(
        package='rmf_path_server',
        executable='rmf_path_server',
        name='rmf_path_server',
        output='both'
    )

    # 2. Start the web spawner & web dashboard hosting server (REST & SSE Bridge)
    # with use_destination_server set to True
    robot_spawner = Node(
        package='rmf_path_server_demo',
        executable='robot_spawner',
        name='robot_spawner',
        output='both',
        parameters=[{'use_destination_server': True}]
    )

    # 3. Start the simple destination server
    destination_server = Node(
        package='rmf_simple_destination_server',
        executable='rmf_simple_destination_server',
        name='rmf_simple_destination_server',
        output='both'
    )

    # 4. Start the plan executor
    plan_executor = Node(
        package='rmf_plan_executor',
        executable='rmf_plan_executor',
        name='rmf_plan_executor',
        output='both'
    )

    return LaunchDescription([
        path_server,
        robot_spawner,
        destination_server,
        plan_executor
    ])
