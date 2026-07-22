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

import launch
import launch_ros
import launch_testing
import pytest
import rclpy
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rmf_prototype_msgs.msg import (
    Destination,
    DestinationConstraints,
    DestinationError,
    DestinationGoal,
    Participant,
    ParticipantList,
    Region,
    TargetRegion,
)


@pytest.mark.launch_test
def generate_test_description():
    server_node = launch_ros.actions.Node(
        package='rmf_reservation_destination_server',
        executable='rmf_reservation_destination_server',
        output='screen'
    )

    return launch.LaunchDescription([
        server_node,
        launch_testing.actions.ReadyToTest(),
    ]), {'server_node': server_node}


class TestOverlap(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_overlap_node')

    def tearDown(self):
        self.node.destroy_node()

    def create_goal(self, session_id, x, y, size):
        msg = DestinationGoal()
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
        msg.one_of.append(constraint)
        return msg

    def test_overlapping_reservation(self):
        # Use robot_1 and robot_2 as they are supported by the server
        r1_name = 'robot_1'
        r2_name = 'robot_2'

        r1_received = []
        r2_errors = []

        reliable_transient_qos = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        self.node.create_subscription(
            Destination,
            f'{r1_name}/destination',
            lambda msg: r1_received.append(msg),
            qos_profile=reliable_transient_qos,
        )
        self.node.create_subscription(
            DestinationError,
            f'{r2_name}/destination/error',
            lambda msg: r2_errors.append(msg),
            qos_profile=reliable_transient_qos,
        )

        pub1 = self.node.create_publisher(
            DestinationGoal, f'{r1_name}/destination/goal', qos_profile=reliable_transient_qos
        )
        pub2 = self.node.create_publisher(
            DestinationGoal, f'{r2_name}/destination/goal', qos_profile=reliable_transient_qos
        )

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

        time.sleep(2.0)

        parts = ParticipantList()
        p1 = Participant()
        p1.name = r1_name
        parts.participants.append(p1)
        p2 = Participant()
        p2.name = r2_name
        parts.participants.append(p2)
        discovery_pub.publish(parts)

        time.sleep(0.5)

        # Robot 1 goal
        goal1 = self.create_goal(10, 10.0, 10.0, 1.0)

        # Wait for robot 1 to succeed
        start_time = time.time()
        while time.time() - start_time < 5.0:
            discovery_pub.publish(parts)
            pub1.publish(goal1)
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if r1_received:
                break
            time.sleep(0.5)

        self.assertTrue(
            len(r1_received) > 0,
            'Robot 1 did not receive Destination message',
        )

        # Robot 2 goal (overlapping with robot 1)
        goal2 = self.create_goal(11, 10.5, 10.5, 1.0)

        # Wait for robot 2 to fail
        start_time = time.time()
        while time.time() - start_time < 5.0:
            discovery_pub.publish(parts)
            pub2.publish(goal2)
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if r2_errors:
                break
            time.sleep(0.5)

        self.assertTrue(
            len(r2_errors) > 0,
            'Robot 2 did not receive DestinationError message',
        )
