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

"""Tests for diversion, queue advancement and cancellation."""

import os
import time
import unittest

from ament_index_python.packages import get_package_share_directory
import launch
import launch_ros
import launch_testing
import pytest
import rclpy
from rmf_prototype_msgs.msg import (
    Destination,
    DestinationConstraints,
    DestinationGoal,
    Participant,
    ParticipantList,
    Region,
    TargetRegion,
)

ROBOT_NAMES = ['robot_1', 'robot_2']
ZERO_UUID = [0] * 16


@pytest.mark.launch_test
def generate_test_description():
    config_file = os.path.join(
        get_package_share_directory('rmf_reservation_tests'),
        'config',
        'queue.yaml',
    )

    server_node = launch_ros.actions.Node(
        package='rmf_reservation_destination_server',
        executable='rmf_reservation_destination_server',
        output='screen',
        parameters=[{'config_file': config_file}],
    )

    return launch.LaunchDescription(
        [
            server_node,
            launch_testing.actions.ReadyToTest(),
        ]
    ), {'server_node': server_node}


class TestQueue(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('test_queue_node')

        cls.destinations = {name: [] for name in ROBOT_NAMES}
        cls.goal_pubs = {}
        for name in ROBOT_NAMES:
            cls.node.create_subscription(
                Destination,
                f'{name}/destination',
                lambda msg, n=name: cls.destinations[n].append(msg),
                10,
            )
            cls.goal_pubs[name] = cls.node.create_publisher(
                DestinationGoal, f'{name}/destination/goal', 10
            )

        cls.discovery_pub = cls.node.create_publisher(
            ParticipantList, '/destination/discovery', 10
        )

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def setUp(self):
        # Reset the shared server so tests are order-independent: deregister
        # every robot, re-register them, and drop any leftover messages.

        self.assertTrue(
            self._drive_discovery_until(
                [], lambda: all(not self._robot_matched(n) for n in ROBOT_NAMES)
            ),
            'robots never deregistered between tests',
        )
        self.assertTrue(
            self._drive_discovery_until(
                ROBOT_NAMES, lambda: all(self._robot_matched(n) for n in ROBOT_NAMES)
            ),
            'server never matched all per-robot topics',
        )
        for buffer in self.destinations.values():
            buffer.clear()

    @classmethod
    def _robot_matched(cls, name):
        return (
            cls.node.count_subscribers(f'{name}/destination/goal') >= 1
            and cls.node.count_publishers(f'{name}/destination') >= 1
        )

    def _drive_discovery_until(self, names, predicate, timeout=30.0):
        """Publish a discovery list of `names` until `predicate()` holds."""
        parts = ParticipantList()
        for name in names:
            participant = Participant()
            participant.name = name
            parts.participants.append(participant)

        deadline = time.time() + timeout
        while time.time() < deadline:
            self.discovery_pub.publish(parts)
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if predicate():
                return True
            time.sleep(0.1)
        return False

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

    def _publish_until(self, robot_name, goal, predicate):
        """Publish `goal` for `robot_name` until `predicate()` holds."""
        goal_pub = self.goal_pubs[robot_name]
        deadline = time.time() + 15.0
        while time.time() < deadline:
            goal_pub.publish(goal)
            rclpy.spin_once(self.node, timeout_sec=0.2)
            if predicate():
                break
            time.sleep(0.2)

    def test_diversion_and_queue_advance(self):
        r1 = self.destinations['robot_1']
        r2 = self.destinations['robot_2']

        # robot_1 claims goal A directly.
        self._publish_until(
            'robot_1', self.create_goal(10, 10.0, 10.0, 1.0), lambda: len(r1) > 0
        )
        self.assertTrue(r1, 'robot_1 did not receive its goal destination')
        self.assertEqual(list(r1[-1].detour_for_goal.uuid), ZERO_UUID)

        # robot_2 wants the same goal and is diverted to a parking spot.
        self._publish_until(
            'robot_2', self.create_goal(11, 10.0, 10.0, 1.0), lambda: len(r2) > 0
        )
        self.assertTrue(r2, 'robot_2 was not diverted to a parking spot')
        divert = r2[-1]
        self.assertEqual(list(divert.detour_for_goal.uuid), [11] * 16)
        self.assertNotEqual(list(divert.session.uuid), [11] * 16)

        # robot_1 leaves A, robot_2 advances into it.
        r2.clear()
        self._publish_until(
            'robot_1',
            self.create_goal(12, 5.0, 5.0, 1.0),
            lambda: any(list(m.detour_for_goal.uuid) == ZERO_UUID for m in r2),
        )
        advanced = [m for m in r2 if list(m.detour_for_goal.uuid) == ZERO_UUID]
        self.assertTrue(
            advanced, 'robot_2 did not advance to its goal after it freed up'
        )
        self.assertEqual(list(advanced[-1].session.uuid), [11] * 16)

    def test_cancellation_frees_goal_for_queued_robot(self):
        r1 = self.destinations['robot_1']
        r2 = self.destinations['robot_2']

        # robot_1 claims goal A directly.
        self._publish_until(
            'robot_1', self.create_goal(20, 10.0, 10.0, 1.0), lambda: len(r1) > 0
        )
        self.assertTrue(r1, 'robot_1 did not receive its goal destination')
        self.assertEqual(list(r1[-1].detour_for_goal.uuid), ZERO_UUID)

        # robot_2 wants the same goal and is diverted to a parking spot.
        self._publish_until(
            'robot_2', self.create_goal(21, 10.0, 10.0, 1.0), lambda: len(r2) > 0
        )
        self.assertTrue(r2, 'robot_2 was not diverted to a parking spot')
        divert = r2[-1]
        self.assertEqual(list(divert.detour_for_goal.uuid), [21] * 16)
        self.assertNotEqual(list(divert.session.uuid), [21] * 16)

        # Cancel robot_1 by dropping it from discovery.
        # robot_2 should advance from parking into goal A.
        r2.clear()
        advanced = self._drive_discovery_until(
            ['robot_2'],
            lambda: any(list(m.detour_for_goal.uuid) == ZERO_UUID for m in r2),
        )
        self.assertTrue(
            advanced, 'robot_2 did not advance after robot_1 was cancelled'
        )
        arrived = [m for m in r2 if list(m.detour_for_goal.uuid) == ZERO_UUID]
        self.assertEqual(list(arrived[-1].session.uuid), [21] * 16)
