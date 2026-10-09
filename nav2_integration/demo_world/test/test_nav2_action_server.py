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

import threading
import time
import unittest

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from launch import LaunchDescription
import launch_ros.actions
import launch_testing.actions
from nav2_msgs.action import NavigateToPose
import pytest
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rmf_prototype_msgs.msg import (
    DestinationConstraints,
    DestinationGoal,
    Region,
    SafeZone,
    TargetOrientation,
    TargetRegion,
)


@pytest.mark.launch_test
def generate_test_description():
    nav2_traffic = launch_ros.actions.Node(
        package='rmf_nav2_traffic',
        executable='nav2_traffic',
        output='screen',
    )

    return LaunchDescription([
        nav2_traffic,
        launch_testing.actions.ReadyToTest(),
    ]), {
        'nav2_traffic': nav2_traffic,
    }


class TestNav2ActionServer(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_nav2_action_server_node')
        self.cb_group = ReentrantCallbackGroup()
        self.executor = MultiThreadedExecutor()
        self.executor.add_node(self.node)
        self.spin_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.spin_thread.start()

    def tearDown(self):
        self.executor.shutdown()
        self.spin_thread.join(timeout=2.0)
        self.node.destroy_node()

    def make_safe_zone(self, session_byte, plan_version, safe_zone_version, x, y, yaw=0.0):
        msg = SafeZone()
        msg.id.plan_id.destination_session.uuid = [session_byte] * 16
        msg.id.plan_id.plan_version = plan_version
        msg.id.safe_zone_version = safe_zone_version

        target_region = TargetRegion()
        target_region.tolerance = 0.2
        target_region.region.hint = Region.HINT_POINT
        target_region.region.points = [float(x), float(y)]
        orientation = TargetOrientation()
        orientation.orientation_radians = float(yaw)
        target_region.orientations.append(orientation)

        constraints = DestinationConstraints()
        constraints.regions.append(target_region)
        msg.incremental_target = constraints

        msg.target_waypoint = [1]
        msg.last_waypoint = 0
        msg.target_progress = 1.0
        return msg

    def test_outer_navigate_to_pose_completion_and_pre_feedback_cancel(self):
        dest_goals_r0 = []
        outer_feedback_r0 = []
        inner_cancelled_r1 = threading.Event()
        inner_started_r1 = threading.Event()

        def execute_r0_inner(goal_handle):
            target_pose = goal_handle.request.pose
            feedback = NavigateToPose.Feedback()
            feedback.current_pose = target_pose
            for _ in range(5):
                goal_handle.publish_feedback(feedback)
                time.sleep(0.1)
            goal_handle.succeed()
            return NavigateToPose.Result()

        def execute_r1_inner(goal_handle):
            inner_started_r1.set()
            # Intentionally do NOT publish any feedback so robot1 has no AgentPose
            deadline = time.time() + 10.0
            while time.time() < deadline:
                if goal_handle.is_cancel_requested:
                    inner_cancelled_r1.set()
                    goal_handle.canceled()
                    return NavigateToPose.Result()
                time.sleep(0.05)
            goal_handle.succeed()
            return NavigateToPose.Result()

        inner_server_r0 = ActionServer(
            self.node,
            NavigateToPose,
            'robot0/inner/navigate_to_pose',
            execute_callback=execute_r0_inner,
            goal_callback=lambda _: GoalResponse.ACCEPT,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=self.cb_group,
        )
        inner_server_r1 = ActionServer(
            self.node,
            NavigateToPose,
            'robot1/inner/navigate_to_pose',
            execute_callback=execute_r1_inner,
            goal_callback=lambda _: GoalResponse.ACCEPT,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=self.cb_group,
        )

        transient_qos = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        self.node.create_subscription(
            DestinationGoal,
            'robot0/destination/goal',
            lambda msg: dest_goals_r0.append(msg),
            qos_profile=transient_qos,
            callback_group=self.cb_group,
        )

        sz_pub_r0 = self.node.create_publisher(
            SafeZone,
            'robot0/plan/safe_zone',
            qos_profile=transient_qos,
        )
        sz_pub_r1 = self.node.create_publisher(
            SafeZone,
            'robot1/plan/safe_zone',
            qos_profile=transient_qos,
        )

        outer_client_r0 = ActionClient(
            self.node,
            NavigateToPose,
            'robot0/navigate_to_pose',
            callback_group=self.cb_group,
        )
        outer_client_r1 = ActionClient(
            self.node,
            NavigateToPose,
            'robot1/navigate_to_pose',
            callback_group=self.cb_group,
        )

        try:
            self.assertTrue(
                outer_client_r0.wait_for_server(timeout_sec=10.0),
                'robot0/navigate_to_pose action server not available',
            )
            self.assertTrue(
                outer_client_r1.wait_for_server(timeout_sec=10.0),
                'robot1/navigate_to_pose action server not available',
            )

            # Part 1: Send outer NavigateToPose goal to robot0 -> verify DestinationGoal,
            # feedback forwarding, and action completion.
            goal_r0 = NavigateToPose.Goal()
            goal_r0.pose = PoseStamped()
            goal_r0.pose.header.frame_id = 'map'
            goal_r0.pose.pose.position.x = 3.0
            goal_r0.pose.pose.position.y = 1.0
            goal_r0.pose.pose.orientation.w = 1.0

            send_future_r0 = outer_client_r0.send_goal_async(
                goal_r0,
                feedback_callback=lambda fb: outer_feedback_r0.append(fb.feedback),
            )
            deadline = time.time() + 10.0
            while time.time() < deadline and not send_future_r0.done():
                time.sleep(0.05)
            self.assertTrue(send_future_r0.done(), 'robot0 send_goal timed out')
            goal_handle_r0 = send_future_r0.result()
            self.assertTrue(goal_handle_r0.accepted, 'robot0 outer goal was not accepted')

            deadline = time.time() + 5.0
            while time.time() < deadline and not dest_goals_r0:
                time.sleep(0.05)
            self.assertGreater(len(dest_goals_r0), 0, 'Expected DestinationGoal on robot0')
            pts = dest_goals_r0[-1].one_of[0].regions[0].region.points
            self.assertAlmostEqual(pts[0], 3.0, places=3)
            self.assertAlmostEqual(pts[1], 1.0, places=3)

            # Publish SafeZone so inner navigation runs and feeds back pose at (3.0, 1.0)
            sz_pub_r0.publish(self.make_safe_zone(1, 1, 1, 3.0, 1.0))

            result_future_r0 = goal_handle_r0.get_result_async()
            deadline = time.time() + 10.0
            while time.time() < deadline and not result_future_r0.done():
                time.sleep(0.05)

            self.assertTrue(result_future_r0.done(), 'robot0 outer action did not complete')
            self.assertEqual(result_future_r0.result().status, GoalStatus.STATUS_SUCCEEDED)
            self.assertGreater(len(outer_feedback_r0), 0, 'Expected outer feedback on robot0')

            # Part 2: Send outer NavigateToPose goal to robot1 and cancel BEFORE any
            # inner feedback (so robot1 has no AgentPose component).
            goal_r1 = NavigateToPose.Goal()
            goal_r1.pose = PoseStamped()
            goal_r1.pose.header.frame_id = 'map'
            goal_r1.pose.pose.position.x = 6.0
            goal_r1.pose.pose.position.y = 2.0
            goal_r1.pose.pose.orientation.w = 1.0

            send_future_r1 = outer_client_r1.send_goal_async(goal_r1)
            deadline = time.time() + 10.0
            while time.time() < deadline and not send_future_r1.done():
                time.sleep(0.05)
            goal_handle_r1 = send_future_r1.result()
            self.assertTrue(goal_handle_r1.accepted, 'robot1 outer goal was not accepted')

            sz_pub_r1.publish(self.make_safe_zone(2, 1, 1, 6.0, 2.0))
            self.assertTrue(
                inner_started_r1.wait(timeout=10.0),
                'robot1 inner goal did not start',
            )

            cancel_future_r1 = goal_handle_r1.cancel_goal_async()
            deadline = time.time() + 10.0
            while time.time() < deadline and not cancel_future_r1.done():
                time.sleep(0.05)

            self.assertTrue(
                inner_cancelled_r1.wait(timeout=10.0),
                'Expected robot1 inner goal to be cancelled even without AgentPose',
            )
        finally:
            inner_server_r0.destroy()
            inner_server_r1.destroy()
            outer_client_r0.destroy()
            outer_client_r1.destroy()
