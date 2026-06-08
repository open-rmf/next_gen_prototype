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
    # robot_spawner = Node(
    #     package='rmf_path_server_demo',
    #     executable='robot_spawner',
    #     name='robot_spawner',
    #     output='both'
    # )

    # 3. Start the plan executor
    plan_executor = Node(
        package='rmf_plan_executor',
        executable='rmf_plan_executor',
        name='rmf_plan_executor',
        output='both'
    )

    return LaunchDescription([
        path_server,
        robot_spawner,
        plan_executor
    ])
