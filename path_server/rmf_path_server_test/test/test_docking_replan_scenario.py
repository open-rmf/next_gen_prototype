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

    # robot_1 starts at dock_north_1 staging (-1.0, 1.0) with delayed docking (2.0x = 6.0s)
    robot_1_sim = launch_ros.actions.Node(
        package='rmf_mock_robot_sim',
        executable='rmf_mock_robot_sim',
        output='screen',
        parameters=[{
            'robot_name': 'robot_1',
            'speed': 2.0,
            'update_rate': 20.0,
            'initial_x': -1.0,
            'initial_y': 1.0,
            'initial_yaw': 0.0,
            'publish_discovery': False,
            'site_file': site_file,
            'action_time_min_scale': 2.0,
            'action_time_max_scale': 2.0,
        }],
    )

    # robot_2 starts right next to robot_1 at (-2.0, 1.0)
    robot_2_sim = launch_ros.actions.Node(
        package='rmf_mock_robot_sim',
        executable='rmf_mock_robot_sim',
        output='screen',
        parameters=[{
            'robot_name': 'robot_2',
            'speed': 2.0,
            'update_rate': 20.0,
            'initial_x': -2.0,
            'initial_y': 1.0,
            'initial_yaw': 0.0,
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


class TestDockingReplanScenario(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_docking_replan_scenario_node')

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

    def test_delayed_docking_with_passing_robot_replan(self):
        r1_odoms = []
        r2_odoms = []
        r1_plans = []
        r2_plans = []

        self.node.create_subscription(
            Odometry, 'robot_1/odom', lambda msg: r1_odoms.append(msg), 10
        )
        self.node.create_subscription(
            Odometry, 'robot_2/odom', lambda msg: r2_odoms.append(msg), 10
        )

        transient_qos = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        self.node.create_subscription(
            Plan,
            'robot_1/plan',
            lambda msg: r1_plans.append(msg),
            qos_profile=transient_qos,
        )
        self.node.create_subscription(
            Plan,
            'robot_2/plan',
            lambda msg: r2_plans.append(msg),
            qos_profile=transient_qos,
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

        # Wait for discovery and subscriptions to settle
        start_wait = time.time()
        while time.time() - start_wait < 2.0:
            map_pub.publish(map_msg)
            discovery_pub.publish(discovery_msg)
            rclpy.spin_once(self.node, timeout_sec=0.1)
            time.sleep(0.05)

        # 1. Command robot_1 to dock at dock_north_1 (-1.0, 2.0) and robot_2 to hold at (-2.0, 1.0)
        dest_r1_dock = self.create_destination(1, -1.0, 2.0)
        dest_r2_hold = self.create_destination(2, -2.0, 1.0)
        r1_dest_pub.publish(dest_r1_dock)
        r2_dest_pub.publish(dest_r2_hold)

        # 2. Wait until robot_1 is actively mid-docking between y=1.15 and y=1.75
        mid_docking_reached = False
        deadline = time.time() + 10.0
        while time.time() < deadline:
            discovery_pub.publish(discovery_msg)
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if r1_odoms and r1_plans:
                y = r1_odoms[-1].pose.pose.position.y
                if 1.15 <= y <= 1.75:
                    mid_docking_reached = True
                    break
            time.sleep(0.05)

        self.assertTrue(
            mid_docking_reached,
            'robot_1 did not enter mid-docking state between (-1.0, 1.0) and (-1.0, 2.0)',
        )

        r1_plans_before_pass = len(r1_plans)
        r2_plans_before_pass = len(r2_plans)

        # 3. While robot_1 is delayed mid-docking, robot_2 requests to pass
        # through (-1.0, 1.0) to (1.0, 1.0)
        dest_r2_pass = self.create_destination(3, 1.0, 1.0)
        r2_dest_pub.publish(dest_r2_pass)

        # 4. Wait for robot_2's replan to be published and for both robots to
        # reach their destinations
        r1_reached = False
        r2_reached = False
        deadline = time.time() + 20.0
        while time.time() < deadline:
            discovery_pub.publish(discovery_msg)
            rclpy.spin_once(self.node, timeout_sec=0.05)

            if r1_odoms:
                p1 = r1_odoms[-1].pose.pose.position
                if abs(p1.x - (-1.0)) < 0.2 and abs(p1.y - 2.0) < 0.2:
                    r1_reached = True

            if r2_odoms and len(r2_plans) > r2_plans_before_pass:
                p2 = r2_odoms[-1].pose.pose.position
                if abs(p2.x - 1.0) < 0.2 and abs(p2.y - 1.0) < 0.2:
                    r2_reached = True

            if r1_reached and r2_reached:
                break
            time.sleep(0.05)

        self.assertGreater(
            len(r2_plans),
            r2_plans_before_pass,
            'robot_2 did not receive a new plan for its passing goal (1.0, 1.0)',
        )

        r1_replans_during_docking = r1_plans[r1_plans_before_pass:]
        self.assertEqual(
            len(r1_replans_during_docking),
            0,
            f'Docking robot_1 should not be replanned while mid-docking, '
            f'but received {len(r1_replans_during_docking)} new plan(s) '
            f'(latest plan_version={r1_plans[-1].plan_id.plan_version})',
        )

        self.assertTrue(r1_reached, 'robot_1 did not finish docking at (-1.0, 2.0)')
        self.assertTrue(r2_reached, 'robot_2 did not pass to (1.0, 1.0)')
