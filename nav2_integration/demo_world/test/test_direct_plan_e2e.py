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
from nav2_msgs.msg import Costmap
import pytest
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rmf_prototype_msgs.msg import (
    Destination,
    DestinationConstraints,
    Plan,
    Progress,
    Region,
    TargetRegion,
)
from std_msgs.msg import Header


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
        self.progress_poses = []
        self.plan_sessions = []

        transient_qos = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        self.amcl_pub = self.node.create_publisher(
            PoseWithCovarianceStamped,
            f'{self.name}/amcl_pose',
            qos_profile=transient_qos,
        )

        self.node.create_subscription(
            Progress,
            f'{self.name}/plan/progress',
            self._progress_cb,
            qos_profile=transient_qos,
            callback_group=cb_group,
        )

        self.node.create_subscription(
            Plan,
            f'{self.name}/plan',
            self._plan_cb,
            qos_profile=transient_qos,
            callback_group=cb_group,
        )

        self.costmap_ack_pub = self.node.create_publisher(
            Header,
            f'{self.name}/global_costmap/plan/costmap_updated',
            qos_profile=transient_qos,
        )
        self.node.create_subscription(
            Costmap,
            f'{self.name}/global_costmap/plan/costmap',
            lambda msg: self.costmap_ack_pub.publish(msg.header),
            qos_profile=transient_qos,
            callback_group=cb_group,
        )

        self.action_server = ActionServer(
            self.node,
            NavigateToPose,
            f'{self.name}/navigate_to_pose',
            execute_callback=self._execute_cb,
            goal_callback=lambda _: GoalResponse.ACCEPT,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=cb_group,
        )

    def _progress_cb(self, msg):
        self.progress_poses.append((msg.pose.pose.position.x, msg.pose.pose.position.y))

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

    def wait_for_robots_to_reach(self, r0, target0, r1, target1, timeout_sec=30.0):
        r0_reached = False
        r1_reached = False
        deadline = time.time() + timeout_sec
        while time.time() < deadline:
            if r0.progress_poses:
                x0, y0 = r0.progress_poses[-1]
                if abs(x0 - target0[0]) < 0.25 and abs(y0 - target0[1]) < 0.25:
                    r0_reached = True
            if r1.progress_poses:
                x1, y1 = r1.progress_poses[-1]
                if abs(x1 - target1[0]) < 0.25 and abs(y1 - target1[1]) < 0.25:
                    r1_reached = True
            if r0_reached and r1_reached:
                return True, True
            time.sleep(0.1)
        return r0_reached, r1_reached

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
            # Wait until nav2_traffic publishes /plan/progress for both robots
            # and rmf_path_server subscribes to both destination topics.
            deadline = time.time() + 10.0
            while time.time() < deadline:
                if (
                    r0.progress_poses
                    and r1.progress_poses
                    and r0_dest_pub.get_subscription_count() >= 1
                    and r1_dest_pub.get_subscription_count() >= 1
                ):
                    break
                time.sleep(0.1)

            self.assertGreater(
                len(r0.progress_poses), 0, 'Expected robot0/plan/progress from nav2_traffic'
            )
            self.assertGreater(
                len(r1.progress_poses), 0, 'Expected robot1/plan/progress from nav2_traffic'
            )

            # Part 1: Command both robots directly via ~/destination
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

            r0_reached, r1_reached = self.wait_for_robots_to_reach(
                r0, (3.0, 0.0), r1, (3.0, 3.0), timeout_sec=25.0
            )
            self.assertTrue(
                r0_reached,
                f'robot0 did not reach (3.0, 0.0); last pose={r0.progress_poses[-1]}',
            )
            self.assertTrue(
                r1_reached,
                f'robot1 did not reach (3.0, 3.0); last pose={r1.progress_poses[-1]}',
            )
            self.assertGreater(
                len(r0.received_targets),
                0,
                'Expected NavigateToPose targets for robot0',
            )
            self.assertGreater(
                len(r1.received_targets),
                0,
                'Expected NavigateToPose targets for robot1',
            )

            # Part 2: Two-stage happy-path scenario from nav2_integration/README.md.
            # Initial poses: robot0=(0.0, 5.0), robot1=(3.0, 5.0)
            # Stage 1: robot0 -> (5.0, 5.0), robot1 -> (0.0, 3.0)
            # Stage 2: robot0 -> (9.0, 3.0), robot1 -> (9.0, 6.0)
            r0.set_pose_and_speed(0.0, 5.0, 4.0)
            r1.set_pose_and_speed(3.0, 5.0, 4.0)
            deadline = time.time() + 5.0
            while time.time() < deadline:
                if (
                    math.hypot(r0.progress_poses[-1][0] - 0.0, r0.progress_poses[-1][1] - 5.0)
                    < 0.1
                    and math.hypot(
                        r1.progress_poses[-1][0] - 3.0, r1.progress_poses[-1][1] - 5.0
                    )
                    < 0.1
                ):
                    break
                time.sleep(0.05)

            dest_r0_s1 = self.create_destination(3, 5.0, 5.0, size=0.1)
            dest_r1_s1 = self.create_destination(4, 0.0, 3.0, size=0.1)
            r0_dest_pub.publish(dest_r0_s1)
            deadline = time.time() + 10.0
            while time.time() < deadline and not r0.has_plan_for_session(
                list(dest_r0_s1.session.uuid)
            ):
                time.sleep(0.05)
            r1_dest_pub.publish(dest_r1_s1)

            r0_s1_ok, r1_s1_ok = self.wait_for_robots_to_reach(
                r0, (5.0, 5.0), r1, (0.0, 3.0), timeout_sec=30.0
            )
            self.assertTrue(r0_s1_ok, 'Stage 1 robot0 did not reach (5.0, 5.0)')
            self.assertTrue(r1_s1_ok, 'Stage 1 robot1 did not reach (0.0, 3.0)')

            # Stage 2: robot0 -> (9.0, 3.0), robot1 -> (9.0, 6.0)
            dest_r0_s2 = self.create_destination(5, 9.0, 3.0, size=0.1)
            dest_r1_s2 = self.create_destination(6, 9.0, 6.0, size=0.1)
            r0_dest_pub.publish(dest_r0_s2)
            deadline = time.time() + 10.0
            while time.time() < deadline and not r0.has_plan_for_session(
                list(dest_r0_s2.session.uuid)
            ):
                time.sleep(0.05)
            r1_dest_pub.publish(dest_r1_s2)

            r0_s2_ok, r1_s2_ok = self.wait_for_robots_to_reach(
                r0, (9.0, 3.0), r1, (9.0, 6.0), timeout_sec=30.0
            )
            self.assertTrue(r0_s2_ok, 'Stage 2 robot0 did not reach (9.0, 3.0)')
            self.assertTrue(r1_s2_ok, 'Stage 2 robot1 did not reach (9.0, 6.0)')
        finally:
            pose_timer.cancel()
            self.node.destroy_timer(pose_timer)
            r0.destroy()
            r1.destroy()
