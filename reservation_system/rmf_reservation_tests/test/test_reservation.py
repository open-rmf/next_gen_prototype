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
import uuid

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
    GraphElementKey,
    Participant,
    ParticipantList,
    Region,
    TargetNode,
    TargetOrientation,
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

    def create_goal(self, session_id, x, y, size, orientation_rad=-1.5707963):
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
        target_ori = TargetOrientation()
        target_ori.orientation_radians = float(orientation_rad)
        target_region.orientations.append(target_ori)
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

        goal = self.create_goal(1, 0.0, 0.0, 1.0, orientation_rad=-1.5707963)

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
        dest_msg = received_dest[-1]
        self.assertGreater(len(dest_msg.constraints.regions), 0)
        self.assertGreater(len(dest_msg.constraints.regions[0].orientations), 0)
        self.assertAlmostEqual(
            dest_msg.constraints.regions[0].orientations[0].orientation_radians,
            -1.5707963,
            places=5,
        )

    def test_reservation_forwards_graph_key(self):
        robot_name = 'robot_graph'
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
            qos_profile=reliable_transient_qos,
        )

        pub = self.node.create_publisher(
            DestinationGoal,
            f'{robot_name}/destination/goal',
            qos_profile=reliable_transient_qos,
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
            qos_profile=discovery_qos,
        )

        time.sleep(1.0)

        parts = ParticipantList()
        p = Participant()
        p.name = robot_name
        parts.participants.append(p)
        discovery_pub.publish(parts)

        time.sleep(0.5)

        goal = DestinationGoal()
        session_uuid = list(uuid.uuid4().bytes)
        goal.session.uuid = session_uuid

        constraint = DestinationConstraints()
        target_region = TargetRegion()
        target_region.region.hint = Region.HINT_AXIS_ALIGNED_RECTANGLE
        target_region.region.points = [5.0, 5.0, 6.0, 6.0]
        constraint.regions.append(target_region)

        target_node = TargetNode()
        target_node.key = GraphElementKey()
        target_node.key.key = [42]
        target_node.key.name = ['station_alpha']
        constraint.nodes.append(target_node)

        goal.one_of.append(constraint)

        start_time = time.time()
        timeout = 5.0
        while time.time() - start_time < timeout:
            discovery_pub.publish(parts)
            pub.publish(goal)
            for _ in range(5):
                rclpy.spin_once(self.node, timeout_sec=0.1)
            if any(
                list(d.session.uuid) == list(goal.session.uuid)
                for d in received_dest
            ):
                break

        matching_dests = [
            d
            for d in received_dest
            if list(d.session.uuid) == list(goal.session.uuid)
        ]
        self.assertTrue(
            len(matching_dests) > 0,
            'Did not receive Destination message with matching session UUID',
        )
        dest = matching_dests[-1]
        self.assertEqual(len(dest.constraints.nodes), 1)
        self.assertEqual(list(dest.constraints.nodes[0].key.key), [42])
        self.assertEqual(
            list(dest.constraints.nodes[0].key.name), ['station_alpha']
        )
