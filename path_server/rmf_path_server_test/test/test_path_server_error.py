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
import pytest
import rclpy
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rmf_prototype_msgs.msg import (
    Destination,
    DestinationConstraints,
    Participant,
    ParticipantList,
    Plan,
    PlanError,
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

    return LaunchDescription([
        path_server,
        robot_1_sim,
        launch_testing.actions.ReadyToTest(),
    ]), {
        'path_server': path_server,
        'robot_1_sim': robot_1_sim,
    }


class TestPathServerError(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_path_server_error_node')

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

    def test_plan_error_behavior(self):
        received_plans = []

        reliable_transient_qos = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        def plan_cb(msg):
            received_plans.append(msg)

        self.node.create_subscription(
            Plan,
            'robot_1/plan',
            plan_cb,
            qos_profile=reliable_transient_qos
        )

        dest_pub = self.node.create_publisher(
            Destination,
            'robot_1/destination',
            qos_profile=reliable_transient_qos
        )

        error_pub = self.node.create_publisher(
            PlanError,
            'robot_1/plan/error',
            qos_profile=reliable_transient_qos
        )

        discovery_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST
        )

        discovery_pub = self.node.create_publisher(
            ParticipantList,
            '/destination/discovery',
            qos_profile=discovery_qos
        )

        discovery_msg = ParticipantList()
        p1 = Participant()
        p1.name = 'robot_1'
        p1.components = []
        discovery_msg.participants.append(p1)

        time.sleep(2.0)

        dest = self.create_destination(1, 5.0, 0.0, 1.0)

        # 1. Publish discovery and destination to receive initial plan
        for _ in range(5):
            discovery_pub.publish(discovery_msg)
            dest_pub.publish(dest)
            rclpy.spin_once(self.node, timeout_sec=0.1)
            time.sleep(0.1)

        start_time = time.time()
        timeout = 10.0
        while time.time() - start_time < timeout:
            discovery_pub.publish(discovery_msg)
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if received_plans:
                break
            time.sleep(0.1)

        self.assertGreater(
            len(received_plans), 0, 'Did not receive initial plan for robot_1'
        )
        initial_plan = received_plans[-1]
        initial_version = initial_plan.plan_id.plan_version

        # 2. Test publishing an unhandled error code (e.g. CODE_UNRECOGNIZED_ACTION)
        # Verify that it does NOT trigger a replan
        unhandled_error_msg = PlanError()
        unhandled_error_msg.error.code = PlanError.CODE_UNRECOGNIZED_ACTION
        unhandled_error_msg.error.message = 'Unrecognized action test'
        unhandled_error_msg.plan_id = initial_plan.plan_id

        error_pub.publish(unhandled_error_msg)
        spin_start = time.time()
        while time.time() - spin_start < 1.0:
            discovery_pub.publish(discovery_msg)
            rclpy.spin_once(self.node, timeout_sec=0.1)
            time.sleep(0.1)

        self.assertEqual(
            received_plans[-1].plan_id.plan_version,
            initial_version,
            'Unhandled error code should not trigger a replan'
        )

        # 3. Publish CODE_PATH_BLOCKED PlanError to trigger replanning
        plan_error_msg = PlanError()
        plan_error_msg.error.code = PlanError.CODE_PATH_BLOCKED
        plan_error_msg.error.message = 'Path is blocked'
        plan_error_msg.plan_id = initial_plan.plan_id

        num_plans_before_error = len(received_plans)
        error_pub.publish(plan_error_msg)

        start_time = time.time()
        new_plan_received = False
        while time.time() - start_time < timeout:
            discovery_pub.publish(discovery_msg)
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if len(received_plans) > num_plans_before_error:
                latest_plan = received_plans[-1]
                if latest_plan.plan_id.plan_version > initial_version:
                    new_plan_received = True
                    break
            time.sleep(0.1)

        self.assertTrue(
            new_plan_received,
            'Expected new plan with incremented version after publishing CODE_PATH_BLOCKED'
        )
        latest_plan = received_plans[-1]
        self.assertEqual(
            latest_plan.plan_id.plan_version,
            initial_version + 1,
            f'Plan version should be incremented from {initial_version} to {initial_version + 1}'
        )
        self.assertEqual(
            list(latest_plan.plan_id.destination_session.uuid),
            list(initial_plan.plan_id.destination_session.uuid),
            'Session UUID should remain the same across replans for the same destination session'
        )
