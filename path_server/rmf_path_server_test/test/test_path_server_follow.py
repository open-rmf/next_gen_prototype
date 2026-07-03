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
import time
import unittest

from launch import LaunchDescription
import launch_ros
import launch_testing
from nav_msgs.msg import Odometry
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
    path_server = launch_ros.actions.Node(
        package='rmf_path_server',
        executable='rmf_path_server',
        output='screen'
    )

    plan_executor = launch_ros.actions.Node(
        package='rmf_plan_executor',
        executable='rmf_plan_executor',
        output='screen'
    )

    # Robot 1 is fast: speed 4.0, starting at x=0.0
    robot_1_sim = launch_ros.actions.Node(
        package='rmf_mock_robot_sim',
        executable='rmf_mock_robot_sim',
        output='screen',
        parameters=[{
            'robot_name': 'robot_1',
            'speed': 4.0,
            'update_rate': 20.0,
            'initial_x': 0.0,
            'initial_y': 0.0,
            'initial_yaw': 0.0,
            'publish_discovery': False,
        }]
    )

    # Robot 2 is slow: speed 1.0, starting at x=3.0 (in front of Robot 1)
    robot_2_sim = launch_ros.actions.Node(
        package='rmf_mock_robot_sim',
        executable='rmf_mock_robot_sim',
        output='screen',
        parameters=[{
            'robot_name': 'robot_2',
            'speed': 1.0,
            'update_rate': 20.0,
            'initial_x': 3.0,
            'initial_y': 0.0,
            'initial_yaw': 0.0,
            'publish_discovery': False,
        }]
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


class TestPathServerFollow(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_path_server_follow_node')

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

    def test_follow_scenario(self):
        # Track robot positions
        r1_positions = []
        r2_positions = []
        trajectory = []

        reliable_transient_qos = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        def r1_odom_cb(msg):
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            r1_positions.append((x, y))
            trajectory.append((time.time(), 'robot_1', x, y))

        def r2_odom_cb(msg):
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            r2_positions.append((x, y))
            trajectory.append((time.time(), 'robot_2', x, y))

        def r1_plan_cb(msg):
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

        self.node.create_subscription(
            Odometry, 'robot_1/odom', r1_odom_cb,
            qos_profile=reliable_transient_qos
        )
        self.node.create_subscription(
            Odometry, 'robot_2/odom', r2_odom_cb,
            qos_profile=reliable_transient_qos
        )
        self.node.create_subscription(
            Plan, 'robot_1/plan', r1_plan_cb,
            qos_profile=reliable_transient_qos
        )
        self.node.create_subscription(
            Plan, 'robot_2/plan', r2_plan_cb,
            qos_profile=reliable_transient_qos
        )

        # Publishers for destinations
        r1_dest_pub = self.node.create_publisher(
            Destination,
            'robot_1/destination',
            qos_profile=reliable_transient_qos
        )
        r2_dest_pub = self.node.create_publisher(
            Destination,
            'robot_2/destination',
            qos_profile=reliable_transient_qos
        )

        # Discovery publisher
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

        # Build unified participant list
        discovery_msg = ParticipantList()
        p1 = Participant()
        p1.name = 'robot_1'
        p1.components = []
        discovery_msg.participants.append(p1)

        p2 = Participant()
        p2.name = 'robot_2'
        p2.components = []
        discovery_msg.participants.append(p2)

        # Wait for discovery and subscriptions to settle
        time.sleep(3.0)

        # Destinations: robot 1 goes to (9.0, 0.0), robot 2 goes to (10.0, 0.0)
        dest_1 = self.create_destination(1, 9.0, 0.0, 1.0)
        dest_2 = self.create_destination(2, 10.0, 0.0, 1.0)

        # Publish discovery and destinations initially to ensure delivery
        for _ in range(5):
            discovery_pub.publish(discovery_msg)
            r1_dest_pub.publish(dest_1)
            r2_dest_pub.publish(dest_2)
            rclpy.spin_once(self.node, timeout_sec=0.1)
            time.sleep(0.1)

        # Run simulation and check distances dynamically
        start_time = time.time()
        timeout = 40.0
        r1_reached = False
        r2_reached = False
        min_distance = float('inf')
        distances = []

        try:
            while time.time() - start_time < timeout:
                discovery_pub.publish(discovery_msg)
                rclpy.spin_once(self.node, timeout_sec=0.1)

                if r1_positions and r2_positions:
                    pos1 = r1_positions[-1]
                    pos2 = r2_positions[-1]
                    dx = pos1[0] - pos2[0]
                    dy = pos1[1] - pos2[1]
                    dist = math.hypot(dx, dy)
                    distances.append(dist)
                    if dist < min_distance:
                        min_distance = dist

                    # Goal checks (within 0.25m from the target points)
                    if abs(pos1[0] - 9.0) < 0.25 and abs(pos1[1] - 0.0) < 0.25:
                        r1_reached = True
                    if abs(pos2[0] - 10.0) < 0.25 and abs(pos2[1] - 0.0) < 0.25:
                        r2_reached = True

                if r1_reached and r2_reached:
                    break

                time.sleep(0.2)

            # Asserts
            self.assertTrue(
                r1_reached,
                f'robot_1 did not reach destination (9.0, 0.0). '
                f'Last pos: {r1_positions[-1] if r1_positions else "None"}'
            )
            self.assertTrue(
                r2_reached,
                f'robot_2 did not reach destination (10.0, 0.0). '
                f'Last pos: {r2_positions[-1] if r2_positions else "None"}'
            )

            print(f'Recorded distances between robots: {distances}')
            print(f'Minimum distance recorded: {min_distance}')

            # Footprint check: radius of each is 0.49.
            # Sum of radii = 0.98.
            # If they get closer than 0.95 (with minor tolerances), we consider it a collision.
            # Let's verify that the minimum distance is at least 0.95.
            self.assertGreaterEqual(
                min_distance,
                0.95,
                f'Collision detected! Minimum distance was {min_distance}, expected >= 0.95'
            )
        finally:
            # Save trajectories to a single file
            import os
            os.makedirs('trajectories', exist_ok=True)
            trajectory.sort(key=lambda item: item[0])
            with open('trajectories/follow_test_trajectory.csv', 'w') as f:
                f.write('t,robot_id,x,y\n')
                for t, robot_id, x, y in trajectory:
                    f.write(f'{t - start_time},{robot_id},{x},{y}\n')
