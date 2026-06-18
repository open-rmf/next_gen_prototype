# Copyright 2026 OSRA
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
from rmf_prototype_msgs.msg import ParticipantList, Plan, PlanId, PlanRelease, Progress, Waypoint


@pytest.mark.launch_test
def generate_test_description():
    sim_node = launch_ros.actions.Node(
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
        }]
    )

    return LaunchDescription([
        sim_node,
        launch_testing.actions.ReadyToTest(),
    ]), {'sim_node': sim_node}


class TestMockRobotSim(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_mock_robot_sim_node')

    def tearDown(self):
        self.node.destroy_node()

    def test_robot_sim_behavior(self):
        robot_name = 'robot_1'

        # 1. Discovery checks
        discovery_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST
        )

        received_dest_discovery = []
        received_plan_discovery = []

        self.node.create_subscription(
            ParticipantList,
            '/destination/discovery',
            lambda msg: received_dest_discovery.append(msg),
            qos_profile=discovery_qos
        )

        self.node.create_subscription(
            ParticipantList,
            '/plan/discovery',
            lambda msg: received_plan_discovery.append(msg),
            qos_profile=discovery_qos
        )

        # Wait for discovery message from mock_robot_sim
        start_time = time.time()
        timeout = 5.0
        discovered = False
        while time.time() - start_time < timeout:
            rclpy.spin_once(self.node, timeout_sec=0.1)

            # Check if robot_1 is in both discovery lists
            dest_ok = any(
                any(p.name == robot_name for p in msg.participants)
                for msg in received_dest_discovery
            )
            plan_ok = any(
                any(p.name == robot_name for p in msg.participants)
                for msg in received_plan_discovery
            )
            if dest_ok and plan_ok:
                discovered = True
                break
            time.sleep(0.1)

        self.assertTrue(discovered, 'Robot did not advertise itself on discovery topics')

        # 2. Setup plan and progress tracking
        received_odoms = []
        received_progress = []

        self.node.create_subscription(
            Odometry,
            f'{robot_name}/odom',
            lambda msg: received_odoms.append(msg),
            10
        )

        self.node.create_subscription(
            Progress,
            f'{robot_name}/plan/progress',
            lambda msg: received_progress.append(msg),
            10
        )

        reliable_transient_qos = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        plan_pub = self.node.create_publisher(
            Plan,
            f'{robot_name}/plan',
            qos_profile=reliable_transient_qos
        )

        release_pub = self.node.create_publisher(
            PlanRelease,
            f'{robot_name}/plan/release',
            qos_profile=reliable_transient_qos
        )

        # 3. Construct and publish Plan
        plan_msg = Plan()

        # Setup PlanId
        plan_id = PlanId()
        plan_id.destination_session.uuid = [7] * 16
        plan_id.plan_version = 42
        plan_msg.plan_id = plan_id

        # Waypoint 1: (2.0, 0.0)
        wp1 = Waypoint()
        wp1.position = [2.0, 0.0]
        wp1.progress = 1.0

        # Waypoint 2: (4.0, 0.0)
        wp2 = Waypoint()
        wp2.position = [4.0, 0.0]
        wp2.progress = 2.0

        plan_msg.waypoints = [wp1, wp2]

        # Create and publish PlanRelease message releasing up to waypoint index 1
        release_msg = PlanRelease()
        release_msg.waypoint_id = 1
        release_msg.plan_id = plan_id

        # Let's publish the plan repeatedly for a short time to ensure delivery
        start_time = time.time()
        while time.time() - start_time < 0.5:
            plan_pub.publish(plan_msg)
            release_pub.publish(release_msg)
            rclpy.spin_once(self.node, timeout_sec=0.05)

        # 4. Run simulation and verify movement and progress updates
        start_time = time.time()
        timeout = 6.0
        target_reached = False

        while time.time() - start_time < timeout:
            rclpy.spin_once(self.node, timeout_sec=0.1)

            if received_odoms:
                latest_odom = received_odoms[-1]
                current_x = latest_odom.pose.pose.position.x
                # If close enough to final waypoint (4.0, 0.0)
                if abs(current_x - 4.0) < 0.15:
                    target_reached = True
                    break
            time.sleep(0.1)

        self.assertTrue(target_reached, 'Robot did not reach the final waypoint in the plan')

        # Verify odometry positions and twist in between
        self.assertTrue(len(received_odoms) > 0)
        first_x = received_odoms[0].pose.pose.position.x
        last_x = received_odoms[-1].pose.pose.position.x
        self.assertLess(first_x, last_x, 'Robot did not move forward in x-direction')

        # Verify progress messages were received during travel
        self.assertTrue(len(received_progress) > 0, 'No progress messages received')

        # Verify PlanId matches in progress messages
        for prog in received_progress:
            expected_uuid = list(plan_id.destination_session.uuid)
            self.assertEqual(list(prog.plan_id.destination_session.uuid), expected_uuid)
            self.assertEqual(prog.plan_id.plan_version, plan_id.plan_version)
            self.assertTrue(0 <= prog.reached_waypoint <= 1)
            self.assertTrue(0 <= prog.target_waypoint <= 1)
            self.assertTrue(0.0 <= prog.progress <= 2.0)

        print(
            f'Test finished. Received {len(received_odoms)} odom and '
            f'{len(received_progress)} progress messages.'
        )
