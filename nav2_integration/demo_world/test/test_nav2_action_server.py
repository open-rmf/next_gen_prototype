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

    def make_safe_zone(self, session_uuid, plan_version, safe_zone_version, x, y, yaw=0.0):
        msg = SafeZone()
        if isinstance(session_uuid, int):
            msg.id.plan_id.destination_session.uuid = [session_uuid] * 16
        else:
            msg.id.plan_id.destination_session.uuid = list(session_uuid)
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
        dest_goals_r1 = []
        outer_feedback_r0 = []
        r0_inner_targets = []
        hold_r0_inner = threading.Event()
        inner_cancelled_r1 = threading.Event()
        inner_started_r1 = threading.Event()
        succeed_r1_inner = threading.Event()

        def execute_r0_inner(goal_handle):
            target_pose = goal_handle.request.pose
            r0_inner_targets.append(
                (target_pose.pose.position.x, target_pose.pose.position.y)
            )
            if hold_r0_inner.is_set():
                deadline = time.time() + 10.0
                while time.time() < deadline and hold_r0_inner.is_set():
                    if goal_handle.is_cancel_requested:
                        goal_handle.canceled()
                        return NavigateToPose.Result()
                    time.sleep(0.05)
            feedback = NavigateToPose.Feedback()
            feedback.current_pose = target_pose
            for _ in range(5):
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    return NavigateToPose.Result()
                goal_handle.publish_feedback(feedback)
                time.sleep(0.1)
            goal_handle.succeed()
            return NavigateToPose.Result()

        def execute_r1_inner(goal_handle):
            inner_started_r1.set()
            if succeed_r1_inner.is_set():
                target_pose = goal_handle.request.pose
                feedback = NavigateToPose.Feedback()
                feedback.current_pose = target_pose
                for _ in range(5):
                    goal_handle.publish_feedback(feedback)
                    time.sleep(0.1)
                goal_handle.succeed()
                return NavigateToPose.Result()
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
        self.node.create_subscription(
            DestinationGoal,
            'robot1/destination/goal',
            lambda msg: dest_goals_r1.append(msg),
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

            # Publish SafeZone matching DestinationGoal.session and initial plan_version=0
            sz_pub_r0.publish(
                self.make_safe_zone(dest_goals_r0[-1].session.uuid, 0, 1, 3.0, 1.0)
            )

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

            deadline = time.time() + 5.0
            while time.time() < deadline and not dest_goals_r1:
                time.sleep(0.05)
            self.assertGreater(len(dest_goals_r1), 0, 'Expected DestinationGoal on robot1')

            sz_pub_r1.publish(
                self.make_safe_zone(dest_goals_r1[-1].session.uuid, 0, 1, 6.0, 2.0)
            )
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
            result_future_r1 = goal_handle_r1.get_result_async()
            deadline = time.time() + 10.0
            while time.time() < deadline and not result_future_r1.done():
                time.sleep(0.05)
            self.assertTrue(result_future_r1.done(), 'robot1 cancel result timed out')
            self.assertEqual(result_future_r1.result().status, GoalStatus.STATUS_CANCELED)

            # Part 3: Preempt an active outer NavigateToPose goal on robot0 with an
            # external SafeZone (different destination_session UUID).
            hold_r0_inner.set()
            prev_dest_count_r0 = len(dest_goals_r0)
            goal_r0_preempt = NavigateToPose.Goal()
            goal_r0_preempt.pose = PoseStamped()
            goal_r0_preempt.pose.header.frame_id = 'map'
            goal_r0_preempt.pose.pose.position.x = 7.0
            goal_r0_preempt.pose.pose.position.y = 4.0
            goal_r0_preempt.pose.pose.orientation.w = 1.0

            send_future_r0_p = outer_client_r0.send_goal_async(goal_r0_preempt)
            deadline = time.time() + 10.0
            while time.time() < deadline and not send_future_r0_p.done():
                time.sleep(0.05)
            goal_handle_r0_p = send_future_r0_p.result()
            self.assertTrue(goal_handle_r0_p.accepted)

            deadline = time.time() + 5.0
            while time.time() < deadline and len(dest_goals_r0) <= prev_dest_count_r0:
                time.sleep(0.05)
            self.assertGreater(len(dest_goals_r0), prev_dest_count_r0)

            sz_pub_r0.publish(
                self.make_safe_zone(dest_goals_r0[-1].session.uuid, 0, 1, 7.0, 4.0)
            )
            deadline = time.time() + 5.0
            while time.time() < deadline and len(r0_inner_targets) < 2:
                time.sleep(0.05)

            # A replan for the same destination_session with incremented plan_version
            # (e.g. when a second robot receives a goal) must NOT cancel the outer goal.
            result_future_r0_p = goal_handle_r0_p.get_result_async()
            sz_pub_r0.publish(
                self.make_safe_zone(dest_goals_r0[-1].session.uuid, 1, 1, 7.0, 4.0)
            )
            time.sleep(0.3)
            self.assertFalse(
                result_future_r0_p.done(),
                'Replan with incremented plan_version for same session '
                'should not cancel outer goal',
            )

            # Publish an external SafeZone with a different session UUID -> cancels outer goal
            hold_r0_inner.clear()
            sz_pub_r0.publish(self.make_safe_zone(99, 0, 1, 9.0, 5.0))

            deadline = time.time() + 10.0
            while time.time() < deadline and not result_future_r0_p.done():
                time.sleep(0.05)
            self.assertTrue(
                result_future_r0_p.done(),
                'Expected preempted robot0 outer NavigateToPose goal to terminate',
            )
            self.assertEqual(
                result_future_r0_p.result().status,
                GoalStatus.STATUS_CANCELED,
            )

            deadline = time.time() + 5.0
            while time.time() < deadline and len(r0_inner_targets) < 4:
                time.sleep(0.05)
            self.assertAlmostEqual(r0_inner_targets[-1][0], 9.0, places=3)
            self.assertAlmostEqual(r0_inner_targets[-1][1], 5.0, places=3)

            # Part 4: Preempt an active outer NavigateToPose goal on robot1 by sending
            # a second outer NavigateToPose goal on robot1.
            inner_started_r1.clear()
            inner_cancelled_r1.clear()
            prev_dest_count_r1 = len(dest_goals_r1)

            goal_r1_first = NavigateToPose.Goal()
            goal_r1_first.pose = PoseStamped()
            goal_r1_first.pose.header.frame_id = 'map'
            goal_r1_first.pose.pose.position.x = 8.0
            goal_r1_first.pose.pose.position.y = 3.0
            goal_r1_first.pose.pose.orientation.w = 1.0

            send_future_r1_first = outer_client_r1.send_goal_async(goal_r1_first)
            deadline = time.time() + 10.0
            while time.time() < deadline and not send_future_r1_first.done():
                time.sleep(0.05)
            goal_handle_r1_first = send_future_r1_first.result()
            self.assertTrue(goal_handle_r1_first.accepted)

            deadline = time.time() + 5.0
            while time.time() < deadline and len(dest_goals_r1) <= prev_dest_count_r1:
                time.sleep(0.05)

            first_r1_session_uuid = list(dest_goals_r1[-1].session.uuid)
            sz_pub_r1.publish(
                self.make_safe_zone(first_r1_session_uuid, 0, 1, 8.0, 3.0)
            )
            self.assertTrue(inner_started_r1.wait(timeout=10.0))

            # Send a second outer NavigateToPose goal to robot1 while first is still active
            succeed_r1_inner.set()
            prev_dest_count_r1 = len(dest_goals_r1)
            goal_r1_second = NavigateToPose.Goal()
            goal_r1_second.pose = PoseStamped()
            goal_r1_second.pose.header.frame_id = 'map'
            goal_r1_second.pose.pose.position.x = 2.0
            goal_r1_second.pose.pose.position.y = 4.0
            goal_r1_second.pose.pose.orientation.w = 1.0

            send_future_r1_second = outer_client_r1.send_goal_async(goal_r1_second)
            deadline = time.time() + 10.0
            while time.time() < deadline and not send_future_r1_second.done():
                time.sleep(0.05)
            goal_handle_r1_second = send_future_r1_second.result()
            self.assertTrue(goal_handle_r1_second.accepted)

            result_future_r1_first = goal_handle_r1_first.get_result_async()
            deadline = time.time() + 10.0
            while time.time() < deadline and not result_future_r1_first.done():
                time.sleep(0.05)
            self.assertTrue(
                result_future_r1_first.done(),
                'Expected first robot1 outer NavigateToPose goal to be cancelled when superseded',
            )
            self.assertEqual(
                result_future_r1_first.result().status,
                GoalStatus.STATUS_CANCELED,
            )
            self.assertTrue(
                inner_cancelled_r1.wait(timeout=10.0),
                'Expected first robot1 inner goal to be cancelled when superseded',
            )

            deadline = time.time() + 5.0
            while time.time() < deadline and len(dest_goals_r1) <= prev_dest_count_r1:
                time.sleep(0.05)

            # An in-flight SafeZone for the superseded session with a bumped plan_version
            # (e.g. triggered by another robot's replan before the new session finishes planning)
            # must be ignored and must NOT cancel the new goal.
            sz_pub_r1.publish(
                self.make_safe_zone(first_r1_session_uuid, 1, 2, 8.0, 3.0)
            )
            time.sleep(0.2)

            sz_pub_r1.publish(
                self.make_safe_zone(dest_goals_r1[-1].session.uuid, 0, 1, 2.0, 4.0)
            )

            result_future_r1_second = goal_handle_r1_second.get_result_async()
            deadline = time.time() + 10.0
            while time.time() < deadline and not result_future_r1_second.done():
                time.sleep(0.05)
            self.assertTrue(result_future_r1_second.done())
            self.assertEqual(
                result_future_r1_second.result().status,
                GoalStatus.STATUS_SUCCEEDED,
            )
        finally:
            inner_server_r0.destroy()
            inner_server_r1.destroy()
            outer_client_r0.destroy()
            outer_client_r1.destroy()

    def test_cancel_follower_in_two_robot_scenario(self):
        dest_goals_r0 = []
        dest_goals_r1 = []
        r0_inner_targets = []
        r1_inner_targets = []
        r0_inner_started = threading.Event()
        r0_inner_cancelled = threading.Event()
        r0_lock = threading.Lock()
        r1_lock = threading.Lock()
        r0_pos = [0.0, 0.0]
        r1_pos = [3.0, 0.0]

        def execute_r0_inner(goal_handle):
            tx = float(goal_handle.request.pose.pose.position.x)
            ty = float(goal_handle.request.pose.pose.position.y)
            r0_inner_targets.append((tx, ty))
            r0_inner_started.set()
            dt = 0.05
            speed = 1.0
            while rclpy.ok():
                if goal_handle.is_cancel_requested:
                    r0_inner_cancelled.set()
                    goal_handle.canceled()
                    return NavigateToPose.Result()
                with r0_lock:
                    dx = tx - r0_pos[0]
                    dy = ty - r0_pos[1]
                    dist = (dx * dx + dy * dy) ** 0.5
                    if dist <= speed * dt:
                        r0_pos[0] = tx
                        r0_pos[1] = ty
                        reached = True
                    else:
                        r0_pos[0] += (dx / dist) * speed * dt
                        r0_pos[1] += (dy / dist) * speed * dt
                        reached = False
                    cx, cy = r0_pos[0], r0_pos[1]

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

        def execute_r1_inner(goal_handle):
            tx = float(goal_handle.request.pose.pose.position.x)
            ty = float(goal_handle.request.pose.pose.position.y)
            r1_inner_targets.append((tx, ty))
            dt = 0.05
            speed = 5.0
            while rclpy.ok():
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    return NavigateToPose.Result()
                with r1_lock:
                    dx = tx - r1_pos[0]
                    dy = ty - r1_pos[1]
                    dist = (dx * dx + dy * dy) ** 0.5
                    if dist <= speed * dt:
                        r1_pos[0] = tx
                        r1_pos[1] = ty
                        reached = True
                    else:
                        r1_pos[0] += (dx / dist) * speed * dt
                        r1_pos[1] += (dy / dist) * speed * dt
                        reached = False
                    cx, cy = r1_pos[0], r1_pos[1]

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
        self.node.create_subscription(
            DestinationGoal,
            'robot1/destination/goal',
            lambda msg: dest_goals_r1.append(msg),
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
            self.assertTrue(outer_client_r0.wait_for_server(timeout_sec=10.0))
            self.assertTrue(outer_client_r1.wait_for_server(timeout_sec=10.0))

            # Clear any transient-local DestinationGoal messages from prior tests
            time.sleep(0.2)
            prev_r0_goals = len(dest_goals_r0)
            prev_r1_goals = len(dest_goals_r1)

            # Two-robot follower scenario:
            # robot1 (leader) starts at (3.0, 0.0) and goes to (10.0, 0.0)
            # robot0 (follower) starts at (0.0, 0.0) and goes to (9.0, 0.0)
            goal_r1 = NavigateToPose.Goal()
            goal_r1.pose.header.frame_id = 'map'
            goal_r1.pose.pose.position.x = 10.0
            goal_r1.pose.pose.position.y = 0.0
            goal_r1.pose.pose.orientation.w = 1.0

            goal_r0 = NavigateToPose.Goal()
            goal_r0.pose.header.frame_id = 'map'
            goal_r0.pose.pose.position.x = 9.0
            goal_r0.pose.pose.position.y = 0.0
            goal_r0.pose.pose.orientation.w = 1.0

            send_future_r1 = outer_client_r1.send_goal_async(goal_r1)
            deadline = time.time() + 10.0
            while time.time() < deadline and not send_future_r1.done():
                time.sleep(0.05)
            goal_handle_r1 = send_future_r1.result()
            self.assertTrue(goal_handle_r1.accepted)

            deadline = time.time() + 5.0
            while time.time() < deadline and len(dest_goals_r1) <= prev_r1_goals:
                time.sleep(0.05)
            r1_session = list(dest_goals_r1[-1].session.uuid)

            send_future_r0 = outer_client_r0.send_goal_async(goal_r0)
            deadline = time.time() + 10.0
            while time.time() < deadline and not send_future_r0.done():
                time.sleep(0.05)
            goal_handle_r0 = send_future_r0.result()
            self.assertTrue(goal_handle_r0.accepted)

            deadline = time.time() + 5.0
            while time.time() < deadline and len(dest_goals_r0) <= prev_r0_goals:
                time.sleep(0.05)
            r0_session = list(dest_goals_r0[-1].session.uuid)

            # Leader (robot1) gets SafeZone to (10.0, 0.0);
            # Follower (robot0) is initially gated behind robot1 at (2.0, 0.0).
            sz_pub_r1.publish(self.make_safe_zone(r1_session, 1, 1, 10.0, 0.0))
            sz_pub_r0.publish(self.make_safe_zone(r0_session, 1, 1, 2.0, 0.0))

            self.assertTrue(
                r0_inner_started.wait(timeout=10.0),
                'Follower robot0 inner goal did not start',
            )
            # Wait for follower robot0 to begin moving toward (2.0, 0.0)
            deadline = time.time() + 5.0
            while time.time() < deadline:
                with r0_lock:
                    if r0_pos[0] > 0.15:
                        break
                time.sleep(0.05)

            # Cancel the follower robot0's outer task
            cancel_future_r0 = goal_handle_r0.cancel_goal_async()
            deadline = time.time() + 10.0
            while time.time() < deadline and not cancel_future_r0.done():
                time.sleep(0.05)
            self.assertTrue(cancel_future_r0.done(), 'Follower cancel_goal_async timed out')

            result_future_r0 = goal_handle_r0.get_result_async()
            deadline = time.time() + 10.0
            while time.time() < deadline and not result_future_r0.done():
                time.sleep(0.05)
            self.assertTrue(result_future_r0.done(), 'Follower outer goal result timed out')
            self.assertEqual(
                result_future_r0.result().status,
                GoalStatus.STATUS_CANCELED,
                'Follower outer action goal status should be CANCELED',
            )

            self.assertTrue(
                r0_inner_cancelled.wait(timeout=10.0),
                'Expected follower inner navigation goal to be cancelled',
            )
            with r0_lock:
                stopped_x, stopped_y = r0_pos[0], r0_pos[1]
            stopped_target_count = len(r0_inner_targets)

            # As the leader advances, rmf_plan_executor unblocks subsequent SafeZones
            # for the follower's cancelled session. These must be ignored so the
            # follower remains stationary.
            sz_pub_r0.publish(self.make_safe_zone(r0_session, 1, 2, 5.0, 0.0))
            time.sleep(0.2)
            sz_pub_r0.publish(self.make_safe_zone(r0_session, 1, 3, 9.0, 0.0))
            time.sleep(0.3)

            with r0_lock:
                final_x, final_y = r0_pos[0], r0_pos[1]
            self.assertAlmostEqual(
                final_x,
                stopped_x,
                places=4,
                msg='Follower robot0 should remain stationary after cancellation',
            )
            self.assertAlmostEqual(
                final_y,
                stopped_y,
                places=4,
                msg='Follower robot0 should remain stationary after cancellation',
            )
            self.assertEqual(
                len(r0_inner_targets),
                stopped_target_count,
                'Follower robot0 must not receive new inner goals for its cancelled session',
            )

            # Meanwhile leader robot1 should complete its goal normally
            result_future_r1 = goal_handle_r1.get_result_async()
            deadline = time.time() + 10.0
            while time.time() < deadline and not result_future_r1.done():
                time.sleep(0.05)
            self.assertTrue(result_future_r1.done(), 'Leader outer goal did not complete')
            self.assertEqual(
                result_future_r1.result().status,
                GoalStatus.STATUS_SUCCEEDED,
            )
        finally:
            inner_server_r0.destroy()
            inner_server_r1.destroy()
            outer_client_r0.destroy()
            outer_client_r1.destroy()
