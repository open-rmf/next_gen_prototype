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

        reliable_transient_qos = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        self.node.create_subscription(
            Progress,
            f'{robot_name}/plan/progress',
            lambda msg: received_progress.append(msg),
            qos_profile=reliable_transient_qos
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

        # 3. Construct and publish Plan (initially released only up to waypoint 0)
        plan_msg = Plan()

        # Setup PlanId
        plan_id = PlanId()
        plan_id.destination_session.uuid = [7] * 16
        plan_id.plan_version = 42
        plan_msg.plan_id = plan_id

        # Waypoint 0: (2.0, 0.0), progress 1.0
        wp0 = Waypoint()
        wp0.position = [2.0, 0.0]
        wp0.progress = 1.0

        # Waypoint 1: (2.0, 0.0), progress 2.0 (stationary wait waypoint)
        wp1 = Waypoint()
        wp1.position = [2.0, 0.0]
        wp1.progress = 2.0

        # Waypoint 2: (4.0, 0.0), progress 3.0
        wp2 = Waypoint()
        wp2.position = [4.0, 0.0]
        wp2.progress = 3.0

        plan_msg.waypoints = [wp0, wp1, wp2]
        plan_pub.publish(plan_msg)

        # 4. Wait until robot reaches wp0 and reports blocked progress at wp0
        start_time = time.time()
        timeout = 4.0
        blocked_at_wp0 = False

        while time.time() - start_time < timeout:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if received_progress and received_odoms:
                latest_prog = received_progress[-1]
                latest_x = received_odoms[-1].pose.pose.position.x
                if (
                    abs(latest_x - 2.0) < 0.05
                    and latest_prog.reached_waypoint == 0
                    and latest_prog.target_waypoint == 0
                    and abs(latest_prog.progress - 1.0) < 1e-3
                ):
                    blocked_at_wp0 = True
                    break

        self.assertTrue(
            blocked_at_wp0,
            'Robot did not hold at waypoint 0 with reached_waypoint=0, '
            'target_waypoint=0, progress=1.0 while unreleased'
        )

        # Verify that before reaching wp0, progress interpolated between 0.0 and 1.0
        pre_wp0_progress = [p.progress for p in received_progress if p.progress < 0.99]
        self.assertGreater(
            len(pre_wp0_progress),
            0,
            'Expected intermediate progress values (< 1.0) before reaching waypoint 0'
        )

        # 5. Release up to final waypoint (index 2) and wait for completion
        release_msg = PlanRelease()
        release_msg.waypoint_id = 2
        release_msg.plan_id = plan_id
        release_pub.publish(release_msg)

        start_time = time.time()
        timeout = 8.0
        plan_completed = False

        while time.time() - start_time < timeout:
            rclpy.spin_once(self.node, timeout_sec=0.05)

            if received_odoms and received_progress:
                latest_odom = received_odoms[-1]
                latest_prog = received_progress[-1]
                current_x = latest_odom.pose.pose.position.x
                if (
                    abs(current_x - 4.0) < 0.05
                    and latest_prog.reached_waypoint == 2
                    and latest_prog.target_waypoint == 2
                    and abs(latest_prog.progress - 3.0) < 1e-3
                ):
                    plan_completed = True
                    break

        self.assertTrue(
            plan_completed,
            'Robot did not complete the plan with reached_waypoint=2, '
            'target_waypoint=2, progress=3.0'
        )

        # Verify odometry positions moved forward in x-direction
        self.assertTrue(len(received_odoms) > 0)
        first_x = received_odoms[0].pose.pose.position.x
        last_x = received_odoms[-1].pose.pose.position.x
        self.assertLess(first_x, last_x, 'Robot did not move forward in x-direction')

        # Verify PlanId, bounds, and monotonic non-decreasing progress across all messages
        expected_uuid = list(plan_id.destination_session.uuid)
        prev_progress_val = 0.0
        saw_stationary_segment = False
        saw_final_segment = False

        for prog in received_progress:
            self.assertEqual(list(prog.plan_id.destination_session.uuid), expected_uuid)
            self.assertEqual(prog.plan_id.plan_version, plan_id.plan_version)
            self.assertTrue(0 <= prog.reached_waypoint <= 2)
            self.assertTrue(0 <= prog.target_waypoint <= 2)
            self.assertLessEqual(prog.reached_waypoint, prog.target_waypoint)
            self.assertTrue(0.0 <= prog.progress <= 3.0)
            self.assertGreaterEqual(
                prog.progress + 1e-4,
                prev_progress_val,
                f'Progress decreased from {prev_progress_val} to {prog.progress}'
            )
            prev_progress_val = prog.progress

            if prog.reached_waypoint == 0 and prog.target_waypoint == 1:
                saw_stationary_segment = True
                self.assertTrue(1.0 - 1e-4 <= prog.progress <= 2.0 + 1e-4)
            if prog.reached_waypoint == 1 and prog.target_waypoint == 2:
                saw_final_segment = True
                self.assertTrue(2.0 - 1e-4 <= prog.progress <= 3.0 + 1e-4)

        self.assertTrue(saw_stationary_segment, 'Did not observe progress on segment 0 -> 1')
        self.assertTrue(saw_final_segment, 'Did not observe progress on segment 1 -> 2')

        print(
            f'Test finished. Received {len(received_odoms)} odom and '
            f'{len(received_progress)} progress messages.'
        )
