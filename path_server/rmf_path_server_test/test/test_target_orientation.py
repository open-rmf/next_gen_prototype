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
import threading
import time
import unittest

from geometry_msgs.msg import PoseStamped
from launch import LaunchDescription
import launch_ros.actions
import launch_testing.actions
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
import pytest
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rmf_prototype_msgs.msg import (
    Destination,
    DestinationConstraints,
    DestinationGoal,
    GraphElementKey,
    Plan,
    Region,
    SafeZone,
    TargetNode,
    TargetOrientation,
    TargetRegion,
)


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


@pytest.mark.launch_test
def generate_test_description():
    reservation_server = launch_ros.actions.Node(
        package='rmf_reservation_destination_server',
        executable='rmf_reservation_destination_server',
        output='screen',
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

    nav2_traffic = launch_ros.actions.Node(
        package='rmf_nav2_traffic',
        executable='nav2_traffic',
        output='screen',
    )

    return LaunchDescription([
        reservation_server,
        path_server,
        plan_executor,
        nav2_traffic,
        launch_testing.actions.ReadyToTest(),
    ]), {
        'reservation_server': reservation_server,
        'path_server': path_server,
        'plan_executor': plan_executor,
        'nav2_traffic': nav2_traffic,
    }


class TestTargetOrientationForwarding(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_target_orientation_forwarding_node')
        self.cb_group = ReentrantCallbackGroup()
        self.executor = MultiThreadedExecutor()
        self.executor.add_node(self.node)
        self.spin_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.spin_thread.start()

    def tearDown(self):
        self.executor.shutdown()
        self.spin_thread.join(timeout=2.0)
        self.node.destroy_node()

    def test_end_to_end_orientation_and_exact_goal_forwarding(self):
        target_x = 2.35
        target_y = 1.75
        target_yaw = -math.pi / 2.0

        received_destinations = []
        received_plans = []
        received_safe_zones = []
        inner_goals = []

        transient_qos = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        self.node.create_subscription(
            Destination,
            '/robot0/destination',
            lambda msg: received_destinations.append(msg),
            qos_profile=transient_qos,
            callback_group=self.cb_group,
        )

        self.node.create_subscription(
            Plan,
            '/robot0/plan',
            lambda msg: received_plans.append(msg),
            qos_profile=transient_qos,
            callback_group=self.cb_group,
        )

        self.node.create_subscription(
            SafeZone,
            '/robot0/plan/safe_zone',
            lambda msg: received_safe_zones.append(msg),
            qos_profile=transient_qos,
            callback_group=self.cb_group,
        )

        def execute_inner(goal_handle):
            inner_goals.append(goal_handle.request.pose)
            goal_handle.succeed()
            return NavigateToPose.Result()

        inner_server = ActionServer(
            self.node,
            NavigateToPose,
            '/robot0/inner/navigate_to_pose',
            execute_callback=execute_inner,
            goal_callback=lambda _: GoalResponse.ACCEPT,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=self.cb_group,
        )

        odom_pub = self.node.create_publisher(Odometry, '/robot0/odom', 10)
        odom_msg = Odometry()
        odom_msg.header.frame_id = 'map'
        odom_msg.pose.pose.position.x = 0.0
        odom_msg.pose.pose.position.y = 0.0
        odom_msg.pose.pose.orientation.w = 1.0

        outer_client = ActionClient(
            self.node,
            NavigateToPose,
            '/robot0/navigate_to_pose',
            callback_group=self.cb_group,
        )
        self.assertTrue(
            outer_client.wait_for_server(timeout_sec=10.0),
            'Timed out waiting for /robot0/navigate_to_pose action server',
        )

        # Wait for discovery and subscriptions to settle while publishing odometry
        start_wait = time.time()
        while time.time() - start_wait < 2.0:
            odom_pub.publish(odom_msg)
            time.sleep(0.1)

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.pose.position.x = target_x
        goal_msg.pose.pose.position.y = target_y
        goal_msg.pose.pose.orientation.z = math.sin(target_yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(target_yaw / 2.0)

        send_future = outer_client.send_goal_async(goal_msg)

        deadline = time.time() + 15.0
        while time.time() < deadline and not (
            received_destinations
            and received_plans
            and received_safe_zones
            and inner_goals
        ):
            odom_pub.publish(odom_msg)
            time.sleep(0.1)

        try:
            self.assertTrue(send_future.done(), 'Outer goal was not accepted')
            self.assertGreater(
                len(received_destinations),
                0,
                'Did not receive /robot0/destination from reservation server',
            )
            dest = received_destinations[-1]
            self.assertAlmostEqual(
                dest.constraints.regions[0].orientations[0].orientation_radians,
                target_yaw,
                places=4,
            )

            self.assertGreater(
                len(received_plans),
                0,
                'Did not receive /robot0/plan from path server',
            )
            plan = received_plans[-1]
            last_wp = plan.waypoints[-1]
            self.assertAlmostEqual(last_wp.position[0], target_x, places=4)
            self.assertAlmostEqual(last_wp.position[1], target_y, places=4)
            self.assertAlmostEqual(
                last_wp.arrival_constraints.regions[0].orientations[0].orientation_radians,
                target_yaw,
                places=4,
            )

            self.assertGreater(
                len(received_safe_zones),
                0,
                'Did not receive /robot0/plan/safe_zone from plan executor',
            )
            sz = received_safe_zones[-1]
            self.assertAlmostEqual(
                sz.incremental_target.regions[0].orientations[0].orientation_radians,
                target_yaw,
                places=4,
            )

            self.assertGreater(
                len(inner_goals),
                0,
                'Did not receive inner NavigateToPose goal on /robot0/inner/navigate_to_pose',
            )
            inner_pose = inner_goals[-1]
            self.assertAlmostEqual(inner_pose.pose.position.x, target_x, places=4)
            self.assertAlmostEqual(inner_pose.pose.position.y, target_y, places=4)
            inner_yaw = yaw_from_quaternion(inner_pose.pose.orientation)
            self.assertAlmostEqual(inner_yaw, target_yaw, places=4)

            # Now verify TargetNode / GraphElementKey propagation through
            # Destination -> Plan -> SafeZone
            goal_pub = self.node.create_publisher(
                DestinationGoal,
                '/robot0/destination/goal',
                qos_profile=transient_qos,
            )
            node_session = [7] * 16
            node_yaw = math.pi / 2.0
            dest_goal = DestinationGoal()
            dest_goal.session.uuid = node_session
            constraint = DestinationConstraints()
            target_region = TargetRegion()
            target_region.region.hint = Region.HINT_POINT
            target_region.region.points = [3.0, 0.0]
            constraint.regions.append(target_region)

            target_node = TargetNode()
            target_node.key = GraphElementKey()
            target_node.key.key = [42]
            target_node.key.name = ['station_alpha']
            node_ori = TargetOrientation()
            node_ori.orientation_radians = node_yaw
            target_node.orientations.append(node_ori)
            constraint.nodes.append(target_node)
            dest_goal.one_of.append(constraint)

            goal_pub.publish(dest_goal)

            deadline = time.time() + 15.0
            while time.time() < deadline:
                odom_pub.publish(odom_msg)
                has_dest = any(
                    list(d.session.uuid) == node_session
                    for d in received_destinations
                )
                has_plan = any(
                    list(p.plan_id.destination_session.uuid) == node_session
                    for p in received_plans
                )
                has_sz = any(
                    list(s.id.plan_id.destination_session.uuid) == node_session
                    for s in received_safe_zones
                )
                if has_dest and has_plan and has_sz:
                    break
                time.sleep(0.1)

            node_dests = [
                d
                for d in received_destinations
                if list(d.session.uuid) == node_session
            ]
            self.assertGreater(
                len(node_dests),
                0,
                'Did not receive Destination with TargetNode session',
            )
            self.assertEqual(len(node_dests[-1].constraints.nodes), 1)
            self.assertEqual(
                list(node_dests[-1].constraints.nodes[0].key.key), [42]
            )
            self.assertEqual(
                list(node_dests[-1].constraints.nodes[0].key.name),
                ['station_alpha'],
            )

            node_plans = [
                p
                for p in received_plans
                if list(p.plan_id.destination_session.uuid) == node_session
            ]
            self.assertGreater(
                len(node_plans),
                0,
                'Did not receive Plan with TargetNode session',
            )
            plan_last_wp = node_plans[-1].waypoints[-1]
            self.assertEqual(len(plan_last_wp.arrival_constraints.nodes), 1)
            self.assertEqual(
                list(plan_last_wp.arrival_constraints.nodes[0].key.key), [42]
            )
            self.assertEqual(
                list(plan_last_wp.arrival_constraints.nodes[0].key.name),
                ['station_alpha'],
            )

            node_szs = [
                s
                for s in received_safe_zones
                if list(s.id.plan_id.destination_session.uuid) == node_session
            ]
            self.assertGreater(
                len(node_szs),
                0,
                'Did not receive SafeZone with TargetNode session',
            )
            self.assertEqual(len(node_szs[-1].incremental_target.nodes), 1)
            self.assertEqual(
                list(node_szs[-1].incremental_target.nodes[0].key.key), [42]
            )
            self.assertEqual(
                list(node_szs[-1].incremental_target.nodes[0].key.name),
                ['station_alpha'],
            )
            self.assertAlmostEqual(
                node_szs[-1]
                .incremental_target.regions[0]
                .orientations[0]
                .orientation_radians,
                node_yaw,
                places=4,
            )
        finally:
            inner_server.destroy()
            outer_client.destroy()
