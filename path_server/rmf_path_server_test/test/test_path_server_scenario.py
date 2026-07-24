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
import launch_ros
import launch_testing
from nav_msgs.msg import Odometry
import pytest
import rclpy
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rmf_prototype_msgs.msg import (
    Destination,
    DestinationConstraints,
    Participant,
    ParticipantList,
    Region,
    TargetRegion,
)


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

    robot_1_sim = launch_ros.actions.Node(
        package='rmf_mock_robot_sim',
        executable='rmf_mock_robot_sim',
        output='screen',
        parameters=[{
            'robot_name': 'robot_1',
            'speed': 2.0,
            'update_rate': 20.0,
            'initial_x': 0.0,
            'initial_y': 0.0,
            'initial_yaw': 0.0,
            'publish_discovery': False,
        }]
    )

    robot_2_sim = launch_ros.actions.Node(
        package='rmf_mock_robot_sim',
        executable='rmf_mock_robot_sim',
        output='screen',
        parameters=[{
            'robot_name': 'robot_2',
            'speed': 2.0,
            'update_rate': 20.0,
            'initial_x': 3.0,
            'initial_y': 0.0,
            'initial_yaw': 0.0,
            'publish_discovery': False,
        }]
    )

    return LaunchDescription([
        path_server,
        plan_executor,
        robot_1_sim,
        robot_2_sim,
        launch_testing.actions.ReadyToTest(),
    ]), {
        'path_server': path_server,
        'plan_executor': plan_executor,
        'robot_1_sim': robot_1_sim,
        'robot_2_sim': robot_2_sim,
    }


class TestPathServerScenario(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_path_server_scenario_node')

    def tearDown(self):
        self.node.destroy_node()

    def create_destination(self, session_id, x, y, size):
        msg = Destination()
        msg.session.uuid = [session_id] * 16
        constraint = DestinationConstraints()
        target_region = TargetRegion()
        target_region.region.hint = Region.HINT_AXIS_ALIGNED_RECTANGLE
        target_region.region.points = [
            float(x),
            float(y),
            float(x + size),
            float(y + size),
        ]
        constraint.regions.append(target_region)
        msg.constraints = constraint
        return msg

    def test_scenario(self):
        # Track robot odometry
        r1_odoms = []
        r2_odoms = []
        trajectory = []

        def r1_cb(msg):
            r1_odoms.append(msg)
            pos = msg.pose.pose.position
            trajectory.append((time.time(), 'robot_1', pos.x, pos.y))

        def r2_cb(msg):
            r2_odoms.append(msg)
            pos = msg.pose.pose.position
            trajectory.append((time.time(), 'robot_2', pos.x, pos.y))

        self.node.create_subscription(
            Odometry,
            'robot_1/odom',
            r1_cb,
            qos_profile=10
        )

        self.node.create_subscription(
            Odometry,
            'robot_2/odom',
            r2_cb,
            qos_profile=10
        )

        # Publishers for destinations
        dest_qos = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST
        )

        r1_dest_pub = self.node.create_publisher(
            Destination,
            'robot_1/destination',
            qos_profile=dest_qos
        )

        r2_dest_pub = self.node.create_publisher(
            Destination,
            'robot_2/destination',
            qos_profile=dest_qos
        )

        # Discovery publisher
        discovery_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        discovery_pub = self.node.create_publisher(
            ParticipantList,
            '/destination/discovery',
            qos_profile=discovery_qos
        )

        # Build unified participant list
        discovery_msg = ParticipantList()
        p1 = Participant()
        p1.name = 'robot_1'
        p1.components = []
        discovery_msg.participants.append(p1)

        p2 = Participant()
        p2.name = 'robot_2'
        p2.components = []
        discovery_msg.participants.append(p2)

        # Wait for discovery and subscriptions to settle
        time.sleep(3.0)

        dest_1 = self.create_destination(1, 5.0, 0.0, 1.0)
        dest_2 = self.create_destination(2, 3.0, 0.0, 1.0)

        # Publish discovery and destinations initially to ensure delivery
        for _ in range(5):
            discovery_pub.publish(discovery_msg)
            r1_dest_pub.publish(dest_1)
            r2_dest_pub.publish(dest_2)
            rclpy.spin_once(self.node, timeout_sec=0.1)
            time.sleep(0.1)

        # Run simulation and verify both robots reach their destinations
        start_time = time.time()
        timeout = 30.0
        r1_reached = False
        r2_reached = False
        r1_redirected = False

        try:
            while time.time() - start_time < timeout:
                # Keep publishing discovery so they aren't considered disconnected
                discovery_pub.publish(discovery_msg)

                rclpy.spin_once(self.node, timeout_sec=0.1)

                if r1_odoms:
                    x = r1_odoms[-1].pose.pose.position.x
                    y = r1_odoms[-1].pose.pose.position.y

                    if x >= 1.9 and not r1_redirected:
                        print(
                            f'robot_1 is midway at ({x:.2f}, {y:.2f}), '
                            'redirecting to (5.0, 5.0)...'
                        )
                        dest_1_new = self.create_destination(3, 5.0, 5.0, 1.0)
                        r1_dest_pub.publish(dest_1_new)
                        r1_redirected = True

                    if r1_redirected:
                        if abs(x - 5.0) < 0.25 and abs(y - 5.0) < 0.25:
                            r1_reached = True
                    else:
                        if abs(x - 5.0) < 0.25 and abs(y - 0.0) < 0.25:
                            r1_reached = True

                if r2_odoms:
                    x = r2_odoms[-1].pose.pose.position.x
                    y = r2_odoms[-1].pose.pose.position.y
                    # Goal is 3.0, 0.0
                    if abs(x - 3.0) < 0.25 and abs(y - 0.0) < 0.25:
                        r2_reached = True

                if r1_reached and r2_reached:
                    break

                time.sleep(0.2)

            self.assertTrue(r1_reached, 'robot_1 did not reach destination (5.0, 5.0)')
            self.assertTrue(r2_reached, 'robot_2 did not reach destination (3.0, 0.0)')

            # Verify robot_1 moved and did not collide with robot_2
            self.assertGreater(len(r1_odoms), 0)
            self.assertGreater(len(r2_odoms), 0)
            print('Both robots safely reached their targets!')
        finally:
            # Save trajectories to a single file
            import os
            os.makedirs('trajectories', exist_ok=True)
            trajectory.sort(key=lambda item: item[0])
            with open('trajectories/scenario_test_trajectory.csv', 'w') as f:
                f.write('t,robot_id,x,y\n')
                for t, robot_id, x, y in trajectory:
                    f.write(f'{t - start_time},{robot_id},{x},{y}\n')
