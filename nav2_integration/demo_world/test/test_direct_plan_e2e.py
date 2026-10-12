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

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
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
    Plan,
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
        self.plan_sessions = []

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

        self.node.create_subscription(
            Plan,
            f'{self.name}/plan',
            self._plan_cb,
            qos_profile=transient_qos,
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

    def _plan_cb(self, msg):
        with self.lock:
            self.plan_sessions.append(list(msg.plan_id.destination_session.uuid))

    def has_plan_for_session(self, session_uuid):
        if session_uuid is None:
            return False
        with self.lock:
            return session_uuid in self.plan_sessions

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

    def set_pose_and_speed(self, x, y, speed):
        with self.lock:
            self.x = float(x)
            self.y = float(y)
            self.speed = float(speed)
        self.publish_amcl_pose()

    def get_pose(self):
        with self.lock:
            return self.x, self.y

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
            # and rmf_path_server subscribes to both destination topics.
            deadline = time.time() + 10.0
            while time.time() < deadline:
                if (
                    r0.odoms
                    and r1.odoms
                    and r0_dest_pub.get_subscription_count() >= 1
                    and r1_dest_pub.get_subscription_count() >= 1
                ):
                    break
                time.sleep(0.1)

            self.assertGreater(len(r0.odoms), 0, 'Expected robot0/odom from nav2_traffic')
            self.assertGreater(len(r1.odoms), 0, 'Expected robot1/odom from nav2_traffic')

            # Command both robots directly via ~/destination without calling ~/navigate_to_pose
            dest_r0 = self.create_destination(1, 3.0, 0.0)
            dest_r1 = self.create_destination(2, 3.0, 3.0)

            r0_dest_pub.publish(dest_r0)
            deadline = time.time() + 10.0
            while time.time() < deadline and not r0.has_plan_for_session(
                list(dest_r0.session.uuid)
            ):
                time.sleep(0.05)
            self.assertTrue(
                r0.has_plan_for_session(list(dest_r0.session.uuid)),
                'Expected rmf_path_server to publish initial plan for robot0',
            )

            r1_dest_pub.publish(dest_r1)
            deadline = time.time() + 10.0
            while time.time() < deadline and not r1.has_plan_for_session(
                list(dest_r1.session.uuid)
            ):
                time.sleep(0.05)
            self.assertTrue(
                r1.has_plan_for_session(list(dest_r1.session.uuid)),
                'Expected rmf_path_server to publish initial plan for robot1',
            )

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

            # Part 2: Exact two-stage happy-path scenario from nav2_integration/README.md.
            # Initial poses: robot0=(0.0, 5.0), robot1=(3.0, 5.0)
            # First destination set: robot0 -> (5.0, 5.0), robot1 -> (0.0, 3.0)
            # Second destination set: robot0 -> (9.0, 3.0), robot1 -> (9.0, 6.0)
            forwarded_sessions = {'robot0': None, 'robot1': None}

            def forward_goal(robot_name, pub, goal_msg):
                if not goal_msg.one_of:
                    return
                dest = Destination()
                dest.constraints = goal_msg.one_of[0]
                dest.session = goal_msg.session
                forwarded_sessions[robot_name] = list(goal_msg.session.uuid)
                pub.publish(dest)

            self.node.create_subscription(
                DestinationGoal,
                'robot0/destination/goal',
                lambda msg: forward_goal('robot0', r0_dest_pub, msg),
                qos_profile=dest_qos,
                callback_group=self.cb_group,
            )
            self.node.create_subscription(
                DestinationGoal,
                'robot1/destination/goal',
                lambda msg: forward_goal('robot1', r1_dest_pub, msg),
                qos_profile=dest_qos,
                callback_group=self.cb_group,
            )

            outer_r0 = ActionClient(
                self.node,
                NavigateToPose,
                'robot0/navigate_to_pose',
                callback_group=self.cb_group,
            )
            outer_r1 = ActionClient(
                self.node,
                NavigateToPose,
                'robot1/navigate_to_pose',
                callback_group=self.cb_group,
            )
            self.assertTrue(outer_r0.wait_for_server(timeout_sec=10.0))
            self.assertTrue(outer_r1.wait_for_server(timeout_sec=10.0))

            r0.set_pose_and_speed(0.0, 5.0, 0.0)
            r1.set_pose_and_speed(3.0, 5.0, 0.0)
            deadline = time.time() + 5.0
            while time.time() < deadline:
                if (
                    r0.odoms
                    and r1.odoms
                    and math.hypot(r0.odoms[-1][0] - 0.0, r0.odoms[-1][1] - 5.0) < 0.1
                    and math.hypot(r1.odoms[-1][0] - 3.0, r1.odoms[-1][1] - 5.0) < 0.1
                ):
                    break
                time.sleep(0.05)

            def make_nav_goal(x, y):
                goal = NavigateToPose.Goal()
                goal.pose.header.frame_id = 'map'
                goal.pose.pose.position.x = float(x)
                goal.pose.pose.position.y = float(y)
                goal.pose.pose.orientation.w = 1.0
                return goal

            # Stage 1: robot0 -> (5.0, 5.0), robot1 -> (0.0, 3.0)
            fb_r0_stage1 = []
            fb_r1_stage1 = []
            forwarded_sessions['robot0'] = None
            forwarded_sessions['robot1'] = None

            r0.set_pose_and_speed(0.0, 5.0, 4.0)
            send_r0 = outer_r0.send_goal_async(
                make_nav_goal(5.0, 5.0),
                feedback_callback=lambda fb: fb_r0_stage1.append(fb.feedback),
            )
            deadline = time.time() + 10.0
            while time.time() < deadline and not send_r0.done():
                time.sleep(0.05)
            handle_r0 = send_r0.result()
            self.assertTrue(handle_r0.accepted)

            deadline = time.time() + 10.0
            while time.time() < deadline and not r0.has_plan_for_session(
                forwarded_sessions['robot0']
            ):
                time.sleep(0.05)
            self.assertTrue(
                r0.has_plan_for_session(forwarded_sessions['robot0']),
                'Expected rmf_path_server to publish Stage 1 plan for robot0',
            )

            r1.set_pose_and_speed(3.0, 5.0, 4.0)
            send_r1 = outer_r1.send_goal_async(
                make_nav_goal(0.0, 3.0),
                feedback_callback=lambda fb: fb_r1_stage1.append(fb.feedback),
            )
            deadline = time.time() + 10.0
            while time.time() < deadline and not send_r1.done():
                time.sleep(0.05)
            handle_r1 = send_r1.result()
            self.assertTrue(handle_r1.accepted)

            res_r0 = handle_r0.get_result_async()
            res_r1 = handle_r1.get_result_async()
            deadline = time.time() + 30.0
            while time.time() < deadline and (not res_r0.done() or not res_r1.done()):
                time.sleep(0.1)

            self.assertTrue(res_r0.done(), 'Stage 1 robot0 outer NavigateToPose did not finish')
            self.assertTrue(res_r1.done(), 'Stage 1 robot1 outer NavigateToPose did not finish')
            self.assertEqual(
                res_r0.result().status,
                GoalStatus.STATUS_SUCCEEDED,
                f'Stage 1 robot0 outer goal failed with status {res_r0.result().status}',
            )
            self.assertEqual(
                res_r1.result().status,
                GoalStatus.STATUS_SUCCEEDED,
                f'Stage 1 robot1 outer goal failed with status {res_r1.result().status}',
            )
            self.assertGreater(len(fb_r0_stage1), 0, 'Expected Stage 1 feedback for robot0')
            self.assertGreater(len(fb_r1_stage1), 0, 'Expected Stage 1 feedback for robot1')

            # Stage 2 ("Alternate navigation goals" in README):
            # robot0 -> (9.0, 3.0), robot1 -> (9.0, 6.0)
            fb_r0_stage2 = []
            fb_r1_stage2 = []
            forwarded_sessions['robot0'] = None
            forwarded_sessions['robot1'] = None

            send_r0_s2 = outer_r0.send_goal_async(
                make_nav_goal(9.0, 3.0),
                feedback_callback=lambda fb: fb_r0_stage2.append(fb.feedback),
            )
            deadline = time.time() + 10.0
            while time.time() < deadline and not send_r0_s2.done():
                time.sleep(0.05)
            handle_r0_s2 = send_r0_s2.result()
            self.assertTrue(handle_r0_s2.accepted)

            deadline = time.time() + 10.0
            while time.time() < deadline and not r0.has_plan_for_session(
                forwarded_sessions['robot0']
            ):
                time.sleep(0.05)
            self.assertTrue(
                r0.has_plan_for_session(forwarded_sessions['robot0']),
                'Expected rmf_path_server to publish Stage 2 plan for robot0',
            )

            send_r1_s2 = outer_r1.send_goal_async(
                make_nav_goal(9.0, 6.0),
                feedback_callback=lambda fb: fb_r1_stage2.append(fb.feedback),
            )
            deadline = time.time() + 10.0
            while time.time() < deadline and not send_r1_s2.done():
                time.sleep(0.05)
            handle_r1_s2 = send_r1_s2.result()
            self.assertTrue(handle_r1_s2.accepted)

            res_r0_s2 = handle_r0_s2.get_result_async()
            res_r1_s2 = handle_r1_s2.get_result_async()
            deadline = time.time() + 30.0
            while time.time() < deadline and (not res_r0_s2.done() or not res_r1_s2.done()):
                time.sleep(0.1)

            self.assertTrue(
                res_r0_s2.done(), 'Stage 2 robot0 outer NavigateToPose did not finish'
            )
            self.assertTrue(
                res_r1_s2.done(), 'Stage 2 robot1 outer NavigateToPose did not finish'
            )
            self.assertEqual(
                res_r0_s2.result().status,
                GoalStatus.STATUS_SUCCEEDED,
                f'Stage 2 robot0 outer goal failed with status {res_r0_s2.result().status}',
            )
            self.assertEqual(
                res_r1_s2.result().status,
                GoalStatus.STATUS_SUCCEEDED,
                f'Stage 2 robot1 outer goal failed with status {res_r1_s2.result().status}',
            )
            self.assertGreater(len(fb_r0_stage2), 0, 'Expected Stage 2 feedback for robot0')
            self.assertGreater(len(fb_r1_stage2), 0, 'Expected Stage 2 feedback for robot1')

            # Part 3: Two-robot follower scenario where the follower's outer
            # ~/navigate_to_pose goal is cancelled mid-execution.
            # Position robot0 (follower) at (0.0, 0.0) and robot1 (leader) at (3.0, 0.0).
            r0.set_pose_and_speed(0.0, 0.0, 0.0)
            r1.set_pose_and_speed(3.0, 0.0, 0.0)
            deadline = time.time() + 5.0
            while time.time() < deadline:
                if (
                    r0.odoms
                    and r1.odoms
                    and math.hypot(r0.odoms[-1][0] - 0.0, r0.odoms[-1][1] - 0.0) < 0.1
                    and math.hypot(r1.odoms[-1][0] - 3.0, r1.odoms[-1][1] - 0.0) < 0.1
                ):
                    break
                time.sleep(0.05)

            leader_goal = NavigateToPose.Goal()
            leader_goal.pose.header.frame_id = 'map'
            leader_goal.pose.pose.position.x = 10.0
            leader_goal.pose.pose.position.y = 0.0
            leader_goal.pose.pose.orientation.w = 1.0

            follower_goal = NavigateToPose.Goal()
            follower_goal.pose.header.frame_id = 'map'
            follower_goal.pose.pose.position.x = 9.0
            follower_goal.pose.pose.position.y = 0.0
            follower_goal.pose.pose.orientation.w = 1.0

            forwarded_sessions['robot0'] = None
            forwarded_sessions['robot1'] = None

            r1.set_pose_and_speed(3.0, 0.0, 2.5)
            send_leader = outer_r1.send_goal_async(leader_goal)
            deadline = time.time() + 10.0
            while time.time() < deadline and not send_leader.done():
                time.sleep(0.05)
            handle_leader = send_leader.result()
            self.assertTrue(handle_leader.accepted)

            deadline = time.time() + 10.0
            while time.time() < deadline and not r1.has_plan_for_session(
                forwarded_sessions['robot1']
            ):
                time.sleep(0.05)
            self.assertTrue(
                r1.has_plan_for_session(forwarded_sessions['robot1']),
                'Expected rmf_path_server to publish leader plan for robot1',
            )

            r0.set_pose_and_speed(0.0, 0.0, 1.0)
            send_follower = outer_r0.send_goal_async(follower_goal)
            deadline = time.time() + 10.0
            while time.time() < deadline and not send_follower.done():
                time.sleep(0.05)
            handle_follower = send_follower.result()
            self.assertTrue(handle_follower.accepted)

            # Wait until follower robot0 has received its plan and started moving along y=0.0
            deadline = time.time() + 10.0
            while time.time() < deadline:
                fx, _ = r0.get_pose()
                if r0.has_plan_for_session(forwarded_sessions['robot0']) and fx > 0.15:
                    break
                time.sleep(0.05)
            self.assertGreater(r0.get_pose()[0], 0.1, 'Follower robot0 did not start moving')

            # Cancel the follower robot0's outer task
            cancel_follower = handle_follower.cancel_goal_async()
            deadline = time.time() + 10.0
            while time.time() < deadline and not cancel_follower.done():
                time.sleep(0.05)
            self.assertTrue(cancel_follower.done(), 'Follower cancel_goal_async timed out')

            res_follower = handle_follower.get_result_async()
            deadline = time.time() + 10.0
            while time.time() < deadline and not res_follower.done():
                time.sleep(0.05)
            self.assertTrue(res_follower.done(), 'Follower outer goal did not terminate')
            self.assertEqual(
                res_follower.result().status,
                GoalStatus.STATUS_CANCELED,
                'Follower outer goal should terminate with STATUS_CANCELED',
            )

            # Allow inner cancellation to settle and record the follower's stopped pose
            time.sleep(0.3)
            stopped_x, stopped_y = r0.get_pose()
            stopped_targets = len(r0.received_targets)

            # Wait for the leader robot1 to finish reaching (10.0, 0.0)
            res_leader = handle_leader.get_result_async()
            deadline = time.time() + 25.0
            while time.time() < deadline and not res_leader.done():
                time.sleep(0.1)
            self.assertTrue(res_leader.done(), 'Leader outer goal did not finish')
            self.assertEqual(
                res_leader.result().status,
                GoalStatus.STATUS_SUCCEEDED,
            )

            # Give rmf_plan_executor a brief window after leader completion and verify
            # the cancelled follower remained completely stationary.
            time.sleep(0.3)
            final_x, final_y = r0.get_pose()
            self.assertAlmostEqual(
                final_x,
                stopped_x,
                places=3,
                msg='Follower robot0 should remain stationary after task cancellation',
            )
            self.assertAlmostEqual(
                final_y,
                stopped_y,
                places=3,
                msg='Follower robot0 should remain stationary after task cancellation',
            )
            self.assertEqual(
                len(r0.received_targets),
                stopped_targets,
                'Follower robot0 must not receive new inner targets after cancellation',
            )
            self.assertLess(
                final_x,
                5.0,
                'Cancelled follower robot0 should have stopped well before its (9.0, 0.0) goal',
            )

            outer_r0.destroy()
            outer_r1.destroy()
        finally:
            pose_timer.cancel()
            self.node.destroy_timer(pose_timer)
            r0.destroy()
            r1.destroy()
