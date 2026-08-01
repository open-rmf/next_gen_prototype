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


class TestReservation(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_reservation_node')

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

    def test_single_reservation(self):
        robot_name = 'robot_1'
        received_dest = []

        reliable_transient_qos = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        self.node.create_subscription(
            Destination,
            f'{robot_name}/destination',
            lambda msg: received_dest.append(msg),
            qos_profile=reliable_transient_qos
        )

        pub = self.node.create_publisher(
            DestinationGoal,
            f'{robot_name}/destination/goal',
            qos_profile=reliable_transient_qos
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

        # Wait for discovery and server to be ready
        time.sleep(2.0)

        parts = ParticipantList()
        p = Participant()
        p.name = robot_name
        parts.participants.append(p)
        discovery_pub.publish(parts)

        time.sleep(0.5)

        goal = self.create_goal(1, 0.0, 0.0, 1.0)

        start_time = time.time()
        timeout = 5.0
        while time.time() - start_time < timeout:
            discovery_pub.publish(parts)
            pub.publish(goal)
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if received_dest:
                break
            time.sleep(0.5)

        self.assertTrue(
            len(received_dest) > 0,
            'Did not receive Destination message',
        )
