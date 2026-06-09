"""Test that a robot waits in parking while another robot occupies its goal."""

import os
import time
import unittest

import launch
import launch_ros
import launch_testing
import pytest
import rclpy
from ament_index_python.packages import get_package_share_directory
from rmf_prototype_msgs.msg import (
    Destination,
    DestinationConstraints,
    DestinationGoal,
    Participant,
    ParticipantList,
    Region,
    TargetRegion,
)

ROBOT_NAMES = ["robot_1", "robot_2"]
ZERO_UUID = [0] * 16


@pytest.mark.launch_test
def generate_test_description():
    config_file = os.path.join(
        get_package_share_directory("rmf_reservation_tests"),
        "config",
        "queue.yaml",
    )

    server_node = launch_ros.actions.Node(
        package="rmf_simple_destination_server",
        executable="rmf_simple_destination_server",
        output="screen",
        parameters=[{"config_file": config_file}],
    )

    return launch.LaunchDescription(
        [
            server_node,
            launch_testing.actions.ReadyToTest(),
        ]
    ), {"server_node": server_node}


class TestQueue(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("test_queue_node")

        cls.destinations = {name: [] for name in ROBOT_NAMES}
        cls.goal_pubs = {}
        for name in ROBOT_NAMES:
            cls.node.create_subscription(
                Destination,
                f"{name}/destination",
                lambda msg, n=name: cls.destinations[n].append(msg),
                10,
            )
            cls.goal_pubs[name] = cls.node.create_publisher(
                DestinationGoal, f"{name}/destination/goal", 10
            )

        cls.discovery_pub = cls.node.create_publisher(
            ParticipantList, "/destination/discovery", 10
        )

        # Discovery messages may arrive before the server is ready, so keep
        # publishing until every robot has matched topics.
        parts = ParticipantList()
        for name in ROBOT_NAMES:
            p = Participant()
            p.name = name
            parts.participants.append(p)

        deadline = time.time() + 30.0
        while time.time() < deadline:
            cls.discovery_pub.publish(parts)
            rclpy.spin_once(cls.node, timeout_sec=0.1)
            if all(cls._robot_matched(name) for name in ROBOT_NAMES):
                break
            time.sleep(0.1)
        else:
            raise AssertionError("Server never matched all per-robot topics")

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def _robot_matched(cls, name):
        return (
            cls.node.count_subscribers(f"{name}/destination/goal") >= 1
            and cls.node.count_publishers(f"{name}/destination") >= 1
        )

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
        r1 = self.destinations["robot_1"]
        r2 = self.destinations["robot_2"]

        self._publish_until(
            "robot_1", self.create_goal(10, 10.0, 10.0, 1.0), lambda: len(r1) > 0
        )
        self.assertTrue(r1, "robot_1 did not receive its goal destination")
        self.assertEqual(list(r1[-1].detour_for_goal.uuid), ZERO_UUID)

        self._publish_until(
            "robot_2", self.create_goal(11, 10.0, 10.0, 1.0), lambda: len(r2) > 0
        )
        self.assertTrue(r2, "robot_2 was not diverted to a parking spot")
        divert = r2[-1]
        self.assertEqual(list(divert.detour_for_goal.uuid), [11] * 16)
        self.assertNotEqual(list(divert.session.uuid), [11] * 16)

        r2.clear()
        self._publish_until(
            "robot_1",
            self.create_goal(12, 5.0, 5.0, 1.0),
            lambda: any(list(m.detour_for_goal.uuid) == ZERO_UUID for m in r2),
        )
        advanced = [m for m in r2 if list(m.detour_for_goal.uuid) == ZERO_UUID]
        self.assertTrue(
            advanced, "robot_2 did not advance to its goal after it freed up"
        )
        self.assertEqual(list(advanced[-1].session.uuid), [11] * 16)
