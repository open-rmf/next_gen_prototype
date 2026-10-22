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

import math
import os
import time
import unittest

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import launch_ros
import launch_testing
from nav_msgs.msg import OccupancyGrid, Odometry
import pytest
import rclpy
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rmf_prototype_msgs.msg import (
    Destination,
    DestinationConstraints,
    Participant,
    ParticipantList,
    Plan,
    Region,
    TargetRegion,
)


@pytest.mark.launch_test
def generate_test_description():
    site_file = os.path.join(
        get_package_share_directory('rmf_path_server_demo'),
        'maps',
        'corridor_docks.site.json',
    )

    path_server = launch_ros.actions.Node(
        package='rmf_path_server',
        executable='rmf_path_server',
        output='screen',
    )

    plan_executor = launch_ros.actions.Node(
        package='rmf_plan_executor',
        executable='rmf_plan_executor',
        output='screen',
    )

    # Two robots opposite each other in the center row (y=0.0) of the dock corridor
    robot_1_sim = launch_ros.actions.Node(
        package='rmf_mock_robot_sim',
        executable='rmf_mock_robot_sim',
        output='screen',
        parameters=[{
            'robot_name': 'robot_1',
            'speed': 1.0,
            'update_rate': 20.0,
            'initial_x': -1.0,
            'initial_y': 0.0,
            'initial_yaw': 0.0,
            'publish_discovery': False,
            'site_file': site_file,
        }],
    )

    robot_2_sim = launch_ros.actions.Node(
        package='rmf_mock_robot_sim',
        executable='rmf_mock_robot_sim',
        output='screen',
        parameters=[{
            'robot_name': 'robot_2',
            'speed': 1.0,
            'update_rate': 20.0,
            'initial_x': 4.0,
            'initial_y': 0.0,
            'initial_yaw': math.pi,
            'publish_discovery': False,
            'site_file': site_file,
        }],
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


class TestCorridorSwap(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_corridor_swap_node')

    def tearDown(self):
        self.node.destroy_node()

    def create_destination(self, session_id, x, y, size=1.0):
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

    def create_corridor_map_msg(self):
        msg = OccupancyGrid()
        msg.header.frame_id = 'map'
        msg.info.resolution = 1.0
        msg.info.width = 14
        msg.info.height = 9
        msg.info.origin.position.x = -5.0
        msg.info.origin.position.y = -4.0
        msg.info.origin.orientation.w = 1.0

        data = [100] * (14 * 9)
        for iy in range(3, 6):
            for ix in range(3, 11):
                data[iy * 14 + ix] = 0
        for ix in (4, 6, 8):
            data[2 * 14 + ix] = 0
            data[6 * 14 + ix] = 0
        msg.data = data
        return msg

    def test_opposite_robots_swap_in_corridor_without_collision(self):
        r1_positions = []
        r2_positions = []
        r1_plans = []
        r2_plans = []
        collisions = []

        transient_qos = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        def check_collision():
            if not r1_positions or not r2_positions:
                return
            pos1 = r1_positions[-1]
            pos2 = r2_positions[-1]
            dist = math.hypot(pos1[0] - pos2[0], pos1[1] - pos2[1])
            if dist < 0.95:
                collisions.append((time.time(), pos1, pos2, dist))

        def r1_odom_cb(msg):
            pos = msg.pose.pose.position
            r1_positions.append((pos.x, pos.y))
            check_collision()

        def r2_odom_cb(msg):
            pos = msg.pose.pose.position
            r2_positions.append((pos.x, pos.y))
            check_collision()

        def r1_plan_cb(msg):
            r1_plans.append(msg)
            print(f'--- robot_1 plan (version {msg.plan_id.plan_version}) ---')
            for i, wp in enumerate(msg.waypoints):
                blockers = [
                    f'{b.name} progress >= {b.required_progress}'
                    for b in wp.departure_blockers
                ]
                print(
                    f'  wp {i}: pos {wp.position}, '
                    f'progress {wp.progress}, blockers: {blockers}'
                )

        def r2_plan_cb(msg):
            r2_plans.append(msg)
            print(f'--- robot_2 plan (version {msg.plan_id.plan_version}) ---')
            for i, wp in enumerate(msg.waypoints):
                blockers = [
                    f'{b.name} progress >= {b.required_progress}'
                    for b in wp.departure_blockers
                ]
                print(
                    f'  wp {i}: pos {wp.position}, '
                    f'progress {wp.progress}, blockers: {blockers}'
                )

        self.node.create_subscription(Odometry, 'robot_1/odom', r1_odom_cb, 10)
        self.node.create_subscription(Odometry, 'robot_2/odom', r2_odom_cb, 10)
        self.node.create_subscription(
            Plan, 'robot_1/plan', r1_plan_cb, qos_profile=transient_qos
        )
        self.node.create_subscription(
            Plan, 'robot_2/plan', r2_plan_cb, qos_profile=transient_qos
        )

        map_pub = self.node.create_publisher(
            OccupancyGrid, '/map', qos_profile=transient_qos
        )
        r1_dest_pub = self.node.create_publisher(
            Destination, 'robot_1/destination', qos_profile=transient_qos
        )
        r2_dest_pub = self.node.create_publisher(
            Destination, 'robot_2/destination', qos_profile=transient_qos
        )

        discovery_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        discovery_pub = self.node.create_publisher(
            ParticipantList, '/destination/discovery', qos_profile=discovery_qos
        )

        discovery_msg = ParticipantList()
        for name in ('robot_1', 'robot_2'):
            p = Participant()
            p.name = name
            p.components = []
            discovery_msg.participants.append(p)

        map_msg = self.create_corridor_map_msg()
        map_pub.publish(map_msg)

        # Wait for discovery and map subscriptions to settle
        start_wait = time.time()
        while time.time() - start_wait < 2.0:
            map_pub.publish(map_msg)
            discovery_pub.publish(discovery_msg)
            rclpy.spin_once(self.node, timeout_sec=0.1)
            time.sleep(0.05)

        # Place final goals in opposite directions across the corridor center
        # so a swap must take place:
        # robot_1: (-1.0, 0.0) -> (3.0, 0.0)
        # robot_2: (4.0, 0.0) -> (0.0, 0.0)
        dest_r1 = self.create_destination(1, 3.0, 0.0)
        dest_r2 = self.create_destination(2, 0.0, 0.0)
        r1_dest_pub.publish(dest_r1)
        r2_dest_pub.publish(dest_r2)

        r1_reached = False
        r2_reached = False
        min_distance = float('inf')
        deadline = time.time() + 25.0

        while time.time() < deadline:
            discovery_pub.publish(discovery_msg)
            rclpy.spin_once(self.node, timeout_sec=0.05)

            if r1_positions and r2_positions:
                pos1 = r1_positions[-1]
                pos2 = r2_positions[-1]
                dist = math.hypot(pos1[0] - pos2[0], pos1[1] - pos2[1])
                if dist < min_distance:
                    min_distance = dist

                if abs(pos1[0] - 3.0) < 0.25 and abs(pos1[1] - 0.0) < 0.25:
                    r1_reached = True
                if abs(pos2[0] - 0.0) < 0.25 and abs(pos2[1] - 0.0) < 0.25:
                    r2_reached = True

            if r1_reached and r2_reached:
                break
            time.sleep(0.02)

        r1_last = r1_positions[-1] if r1_positions else None
        r2_last = r2_positions[-1] if r2_positions else None
        first_collision = collisions[0] if collisions else None
        self.assertTrue(
            r1_reached,
            f'robot_1 did not reach (3.0, 0.0); last pos: {r1_last}',
        )
        self.assertTrue(
            r2_reached,
            f'robot_2 did not reach (0.0, 0.0); last pos: {r2_last}',
        )
        self.assertEqual(
            len(collisions),
            0,
            f'Collision detected! Min distance={min_distance:.3f}m, '
            f'first collision={first_collision}',
        )
