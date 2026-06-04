"""Launch test for configuration of rmf_simple_destination_server.

The server is started with config/safe_sets.yaml.
"""

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
    DestinationError,
    DestinationGoal,
    Participant,
    ParticipantList,
    Region,
    TargetRegion,
)

ROBOT_NAMES = [
    "robot_rect_in",
    "robot_rect_out",
    "robot_point_in",
    "robot_point_out",
    "robot_dock",
]


@pytest.mark.launch_test
def generate_test_description():
    config_file = os.path.join(
        get_package_share_directory("rmf_reservation_tests"),
        "config",
        "safe_sets.yaml",
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


class TestSafeSets(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("test_safe_sets_node")

        # Per-robot message buffers and publishers.
        cls.destinations = {name: [] for name in ROBOT_NAMES}
        cls.errors = {name: [] for name in ROBOT_NAMES}
        cls.goal_pubs = {}

        for name in ROBOT_NAMES:
            cls.node.create_subscription(
                Destination,
                f"{name}/destination",
                lambda msg, n=name: cls.destinations[n].append(msg),
                10,
            )
            cls.node.create_subscription(
                DestinationError,
                f"{name}/destination/error",
                lambda msg, n=name: cls.errors[n].append(msg),
                10,
            )
            cls.goal_pubs[name] = cls.node.create_publisher(
                DestinationGoal, f"{name}/destination/goal", 10
            )

        cls.discovery_pub = cls.node.create_publisher(
            ParticipantList, "/destination/discovery", 10
        )

        # Advertise every robot in a single discovery message and keep
        # re-publishing until the server has created and matched all of the
        # per-robot topics.
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
            and cls.node.count_publishers(f"{name}/destination/error") >= 1
        )

    def _goal_from_region(self, session_id, hint, points):
        msg = DestinationGoal()
        msg.session.uuid = [session_id] * 16
        constraint = DestinationConstraints()
        target_region = TargetRegion()
        target_region.region.hint = hint
        target_region.region.points = [float(v) for v in points]
        constraint.regions.append(target_region)
        msg.one_of.append(constraint)
        return msg

    def rect_goal(self, session_id, x, y, size):
        """Axis-aligned rectangle goal with a corner at (x, y)."""
        return self._goal_from_region(
            session_id,
            Region.HINT_AXIS_ALIGNED_RECTANGLE,
            [x, y, x + size, y + size],
        )

    def point_goal(self, session_id, x, y):
        """Single-point goal at (x, y)."""
        return self._goal_from_region(session_id, Region.HINT_POINT, [x, y])

    def _run_goal(self, robot_name, goal):
        """Publish a goal until a Destination or DestinationError comes back.

        Returns a tuple (destinations, errors) for that robot.
        """
        destinations = self.destinations[robot_name]
        errors = self.errors[robot_name]
        destinations.clear()
        errors.clear()

        goal_pub = self.goal_pubs[robot_name]
        deadline = time.time() + 15.0
        while time.time() < deadline:
            goal_pub.publish(goal)
            rclpy.spin_once(self.node, timeout_sec=0.2)
            if destinations or errors:
                break
            time.sleep(0.2)

        return destinations, errors

    def assert_accepted(self, robot_name, goal, msg):
        destinations, errors = self._run_goal(robot_name, goal)
        self.assertTrue(len(destinations) > 0, msg)
        self.assertEqual(len(errors), 0, f"Did not expect an error: {robot_name}")

    def assert_rejected(self, robot_name, goal, msg):
        destinations, errors = self._run_goal(robot_name, goal)
        self.assertTrue(len(errors) > 0, msg)
        self.assertEqual(errors[0].error.code, DestinationError.CODE_UNREACHABLE)
        self.assertEqual(
            len(destinations), 0, f"Did not expect a Destination: {robot_name}"
        )

    # --- rectangle regions

    def test_rect_goal_in_test_floor_is_accepted(self):
        self.assert_accepted(
            "robot_rect_in",
            self.rect_goal(1, 1.0, 1.0, 1.0),
            "Expected a Destination for a rectangle inside the test_floor safe set",
        )

    def test_rect_goal_outside_safe_set_is_rejected(self):
        self.assert_rejected(
            "robot_rect_out",
            self.rect_goal(2, 50.0, 50.0, 1.0),
            "Expected a DestinationError for a rectangle outside the safe sets",
        )

    def test_rect_goal_in_dock_is_accepted(self):
        self.assert_accepted(
            "robot_dock",
            self.rect_goal(5, 24.0, 24.0, 1.0),
            "Expected a Destination for a rectangle inside the dock safe set",
        )

    # --- point regions

    def test_point_goal_in_test_floor_is_accepted(self):
        self.assert_accepted(
            "robot_point_in",
            self.point_goal(3, 5.0, 5.0),
            "Expected a Destination for a point inside the test_floor safe set",
        )

    def test_point_goal_outside_safe_set_is_rejected(self):
        self.assert_rejected(
            "robot_point_out",
            self.point_goal(4, 15.0, 15.0),
            "Expected a DestinationError for a point outside the safe sets",
        )
