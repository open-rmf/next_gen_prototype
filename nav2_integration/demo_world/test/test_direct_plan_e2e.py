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

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from launch import LaunchDescription
import launch_ros.actions
import launch_testing.actions
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
import pytest
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rmf_prototype_msgs.msg import (
    Destination,
    DestinationConstraints,
    Region,
    TargetRegion,
)


@pytest.mark.launch_test
def generate_test_description():
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
        path_server,
        plan_executor,
        nav2_traffic,
        launch_testing.actions.ReadyToTest(),
    ]), {
        'path_server': path_server,
        'plan_executor': plan_executor,
        'nav2_traffic': nav2_traffic,
    }


class MockNav2AgentHarness:

    def __init__(self, node, cb_group, name, initial_x, initial_y, speed=4.0):
        self.node = node
        self.name = name
        self.x = float(initial_x)
        self.y = float(initial_y)
        self.speed = float(speed)
        self.lock = threading.Lock()
        self.received_targets = []
        self.odoms = []

        transient_qos = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        self.amcl_pub = self.node.create_publisher(
            PoseWithCovarianceStamped,
            f'{self.name}/inner/amcl_pose',
            qos_profile=transient_qos,
        )

        self.node.create_subscription(
            Odometry,
            f'{self.name}/odom',
            self._odom_cb,
            qos_profile=10,
            callback_group=cb_group,
        )

        self.action_server = ActionServer(
            self.node,
            NavigateToPose,
            f'{self.name}/inner/navigate_to_pose',
            execute_callback=self._execute_cb,
            goal_callback=lambda _: GoalResponse.ACCEPT,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=cb_group,
        )

    def _odom_cb(self, msg):
        self.odoms.append((msg.pose.pose.position.x, msg.pose.pose.position.y))

    def publish_amcl_pose(self):
        with self.lock:
            x, y = self.x, self.y
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.w = 1.0
        self.amcl_pub.publish(msg)

    def _execute_cb(self, goal_handle):
        tx = float(goal_handle.request.pose.pose.position.x)
        ty = float(goal_handle.request.pose.pose.position.y)
        self.received_targets.append((tx, ty))

        dt = 0.05
        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return NavigateToPose.Result()

            with self.lock:
                dx = tx - self.x
                dy = ty - self.y
                dist = math.hypot(dx, dy)
                if dist <= self.speed * dt:
                    self.x = tx
                    self.y = ty
                    reached = True
                else:
                    self.x += (dx / dist) * self.speed * dt
                    self.y += (dy / dist) * self.speed * dt
                    reached = False
                cx, cy = self.x, self.y

            self.publish_amcl_pose()

            feedback = NavigateToPose.Feedback()
            feedback.current_pose = PoseStamped()
            feedback.current_pose.header.frame_id = 'map'
            feedback.current_pose.pose.position.x = cx
            feedback.current_pose.pose.position.y = cy
            feedback.current_pose.pose.orientation.w = 1.0
            goal_handle.publish_feedback(feedback)

            if reached:
                goal_handle.succeed()
                return NavigateToPose.Result()

            time.sleep(dt)

        goal_handle.abort()
        return NavigateToPose.Result()

    def destroy(self):
        self.action_server.destroy()


class TestDirectPlanE2E(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_direct_plan_e2e_node')
        self.cb_group = ReentrantCallbackGroup()
        self.executor = MultiThreadedExecutor()
        self.executor.add_node(self.node)
        self.spin_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.spin_thread.start()

    def tearDown(self):
        self.executor.shutdown()
        self.spin_thread.join(timeout=2.0)
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

    def test_direct_destination_drives_nav2_agents_to_goal(self):
        r0 = MockNav2AgentHarness(self.node, self.cb_group, 'robot0', 0.0, 0.0)
        r1 = MockNav2AgentHarness(self.node, self.cb_group, 'robot1', 0.0, 3.0)

        pose_timer = self.node.create_timer(
            0.1,
            lambda: (r0.publish_amcl_pose(), r1.publish_amcl_pose()),
            callback_group=self.cb_group,
        )

        dest_qos = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        r0_dest_pub = self.node.create_publisher(
            Destination,
            'robot0/destination',
            qos_profile=dest_qos,
        )
        r1_dest_pub = self.node.create_publisher(
            Destination,
            'robot1/destination',
            qos_profile=dest_qos,
        )

        try:
            # Wait until nav2_traffic converts inner/amcl_pose into /odom for both robots
            deadline = time.time() + 10.0
            while time.time() < deadline:
                if r0.odoms and r1.odoms:
                    break
                time.sleep(0.1)

            self.assertGreater(len(r0.odoms), 0, 'Expected robot0/odom from nav2_traffic')
            self.assertGreater(len(r1.odoms), 0, 'Expected robot1/odom from nav2_traffic')

            # Command both robots directly via ~/destination without calling ~/navigate_to_pose
            dest_r0 = self.create_destination(1, 3.0, 0.0)
            dest_r1 = self.create_destination(2, 3.0, 3.0)

            for _ in range(5):
                r0_dest_pub.publish(dest_r0)
                r1_dest_pub.publish(dest_r1)
                time.sleep(0.1)

            r0_reached = False
            r1_reached = False
            deadline = time.time() + 25.0
            while time.time() < deadline:
                if r0.odoms:
                    x0, y0 = r0.odoms[-1]
                    if abs(x0 - 3.0) < 0.25 and abs(y0 - 0.0) < 0.25:
                        r0_reached = True
                if r1.odoms:
                    x1, y1 = r1.odoms[-1]
                    if abs(x1 - 3.0) < 0.25 and abs(y1 - 3.0) < 0.25:
                        r1_reached = True
                if r0_reached and r1_reached:
                    break
                time.sleep(0.1)

            self.assertTrue(
                r0_reached,
                f'robot0 did not reach (3.0, 0.0); last odom={r0.odoms[-1] if r0.odoms else None}',
            )
            self.assertTrue(
                r1_reached,
                f'robot1 did not reach (3.0, 3.0); last odom={r1.odoms[-1] if r1.odoms else None}',
            )
            self.assertGreater(
                len(r0.received_targets),
                0,
                'Expected inner NavigateToPose targets for robot0',
            )
            self.assertGreater(
                len(r1.received_targets),
                0,
                'Expected inner NavigateToPose targets for robot1',
            )
        finally:
            pose_timer.cancel()
            self.node.destroy_timer(pose_timer)
            r0.destroy()
            r1.destroy()
