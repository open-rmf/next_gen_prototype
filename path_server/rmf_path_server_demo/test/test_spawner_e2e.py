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

import time
import unittest

from launch import LaunchDescription
import launch_ros.actions
import launch_testing.actions
from nav_msgs.msg import Odometry
import pytest
import rclpy

from rmf_path_server_demo.test_client import SpawnerTestClient


@pytest.mark.launch_test
def generate_test_description():
    path_server = launch_ros.actions.Node(
        package='rmf_path_server',
        executable='rmf_path_server',
        output='screen'
    )

    plan_executor = launch_ros.actions.Node(
        package='rmf_plan_executor',
        executable='rmf_plan_executor',
        output='screen'
    )

    robot_spawner = launch_ros.actions.Node(
        package='rmf_path_server_demo',
        executable='robot_spawner',
        output='screen',
        parameters=[{
            'port': 8088
        }]
    )

    return LaunchDescription([
        path_server,
        plan_executor,
        robot_spawner,
        launch_testing.actions.ReadyToTest(),
    ]), {
        'path_server': path_server,
        'plan_executor': plan_executor,
        'robot_spawner': robot_spawner,
    }


class TestRobotSpawnerE2E(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_robot_spawner_e2e_node')
        self.client = SpawnerTestClient(host='http://localhost:8088')

    def tearDown(self):
        try:
            self.client.reset()
        except Exception:
            pass
        self.node.destroy_node()

    def test_spawner_two_robot_scenario(self):
        # 1. Wait until robot_spawner HTTP server is ready
        server_ready = self.client.wait_until_ready(timeout=10.0)
        self.assertTrue(server_ready, 'robot_spawner HTTP server failed to become ready')

        # 2. Check initial config endpoint
        config = self.client.get_config()
        self.assertIn('default_radius', config)

        # 3. Spawn robot_1 at (0.0, 0.0) and robot_2 at (3.0, 0.0)
        spawn_res1 = self.client.spawn_robot('robot_1', 0.0, 0.0)
        self.assertEqual(spawn_res1.get('status'), 'spawned')
        self.assertEqual(spawn_res1.get('name'), 'robot_1')

        spawn_res2 = self.client.spawn_robot('robot_2', 3.0, 0.0)
        self.assertEqual(spawn_res2.get('status'), 'spawned')
        self.assertEqual(spawn_res2.get('name'), 'robot_2')

        # 4. Set destinations via HTTP client: robot_1 to (5.0, 0.0), robot_2 to (8.0, 0.0)
        dest_res1 = self.client.set_destination('robot_1', 5.0, 0.0)
        self.assertEqual(dest_res1.get('status'), 'goals_set')

        dest_res2 = self.client.set_destination('robot_2', 8.0, 0.0)
        self.assertEqual(dest_res2.get('status'), 'goals_set')

        # 5. Connect SSE listener to monitor events sent by robot_spawner
        self.client.start_sse_listener()

        # 6. Subscribe to robot odometry via ROS to double check positions
        r1_odoms = []
        r2_odoms = []

        def r1_odom_cb(msg):
            r1_odoms.append(msg.pose.pose.position)

        def r2_odom_cb(msg):
            r2_odoms.append(msg.pose.pose.position)

        self.node.create_subscription(Odometry, 'robot_1/odom', r1_odom_cb, qos_profile=10)
        self.node.create_subscription(Odometry, 'robot_2/odom', r2_odom_cb, qos_profile=10)

        # 7. Trigger scenario execution via HTTP test client
        scenario_res = self.client.send_scenario()
        self.assertEqual(scenario_res.get('status'), 'scenario_sent')

        # 8. Spin and verify that robot_spawner component responded and both robots
        # reach target positions
        start_time = time.time()
        timeout = 30.0
        r1_reached = False
        r2_reached = False

        while time.time() - start_time < timeout:
            rclpy.spin_once(self.node, timeout_sec=0.1)

            if r1_odoms:
                last_pos1 = r1_odoms[-1]
                if abs(last_pos1.x - 5.0) < 0.3 and abs(last_pos1.y - 0.0) < 0.3:
                    r1_reached = True

            if r2_odoms:
                last_pos2 = r2_odoms[-1]
                if abs(last_pos2.x - 8.0) < 0.3 and abs(last_pos2.y - 0.0) < 0.3:
                    r2_reached = True

            if r1_reached and r2_reached:
                break

            time.sleep(0.1)

        self.client.stop_sse_listener()

        r1_last = r1_odoms[-1] if r1_odoms else 'None'
        self.assertTrue(
            r1_reached,
            f'robot_1 did not reach target (5.0, 0.0). Last pos: {r1_last}'
        )
        r2_last = r2_odoms[-1] if r2_odoms else 'None'
        self.assertTrue(
            r2_reached,
            f'robot_2 did not reach target (8.0, 0.0). Last pos: {r2_last}'
        )

        # Check that SSE listener received odom events from robot_spawner
        sse_events = self.client.received_events
        odom_events = [e for e in sse_events if e.get('type') == 'odom']
        self.assertGreater(len(odom_events), 0, 'No odom events received via SSE stream')

        # 9. Verify reset endpoint works
        reset_res = self.client.reset()
        self.assertEqual(reset_res.get('status'), 'reset')
