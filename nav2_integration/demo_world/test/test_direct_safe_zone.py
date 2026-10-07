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
    DestinationConstraints,
    PlanError,
    Progress,
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


class TestDirectSafeZone(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_direct_safe_zone_node')
        self.cb_group = ReentrantCallbackGroup()
        self.executor = MultiThreadedExecutor()
        self.executor.add_node(self.node)
        self.spin_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.spin_thread.start()

    def tearDown(self):
        self.executor.shutdown()
        self.spin_thread.join(timeout=2.0)
        self.node.destroy_node()

    def make_safe_zone(
        self,
        session_byte,
        plan_version,
        safe_zone_version,
        x,
        y,
        yaw=0.0,
        last_wp=0,
        target_wp=1,
        progress=1.0,
    ):
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

        msg.target_waypoint = [int(target_wp)]
        msg.last_waypoint = int(last_wp)
        msg.target_progress = float(progress)
        return msg

    def test_direct_safe_zone_commands_inner_nav2(self):
        received_goals = []
        cancelled_goals = []
        received_costmaps = []
        received_progress = []
        received_errors = []
        hold_first_goal = threading.Event()
        abort_next_goal = threading.Event()

        def execute_cb(goal_handle):
            pos = goal_handle.request.pose.pose.position
            received_goals.append((pos.x, pos.y))

            if abort_next_goal.is_set():
                goal_handle.abort()
                return NavigateToPose.Result()

            # Hold the first goal active until superseded by a newer SafeZone
            if len(received_goals) == 1:
                deadline = time.time() + 10.0
                while time.time() < deadline and not hold_first_goal.is_set():
                    if goal_handle.is_cancel_requested:
                        cancelled_goals.append((pos.x, pos.y))
                        goal_handle.canceled()
                        return NavigateToPose.Result()
                    time.sleep(0.05)

            goal_handle.succeed()
            return NavigateToPose.Result()

        inner_server = ActionServer(
            self.node,
            NavigateToPose,
            'robot0/inner/navigate_to_pose',
            execute_callback=execute_cb,
            goal_callback=lambda _: GoalResponse.ACCEPT,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=self.cb_group,
        )

        self.node.create_subscription(
            Costmap,
            'robot0/inner/global_costmap/plan/costmap',
            lambda msg: received_costmaps.append(msg),
            qos_profile=10,
            callback_group=self.cb_group,
        )
        self.node.create_subscription(
            Progress,
            'robot0/plan/progress',
            lambda msg: received_progress.append(msg),
            qos_profile=10,
            callback_group=self.cb_group,
        )
        self.node.create_subscription(
            PlanError,
            'robot0/plan/error',
            lambda msg: received_errors.append(msg),
            qos_profile=10,
            callback_group=self.cb_group,
        )

        safe_zone_qos = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        safe_zone_pub = self.node.create_publisher(
            SafeZone,
            'robot0/plan/safe_zone',
            qos_profile=safe_zone_qos,
        )

        time.sleep(2.0)

        try:
            # 1. Publish a SafeZone directly without any outer NavigateToPose request
            sz1 = self.make_safe_zone(1, 1, 1, 4.0, 2.0, last_wp=0, target_wp=1, progress=1.0)
            safe_zone_pub.publish(sz1)

            deadline = time.time() + 10.0
            while time.time() < deadline:
                if received_goals and received_costmaps and received_progress:
                    break
                time.sleep(0.1)

            self.assertEqual(len(received_goals), 1, 'Expected initial inner NavigateToPose goal')
            self.assertAlmostEqual(received_goals[0][0], 4.0, places=3)
            self.assertAlmostEqual(received_goals[0][1], 2.0, places=3)
            self.assertGreater(len(received_costmaps), 0, 'Expected Costmap publication')
            self.assertGreater(len(received_progress), 0, 'Expected Progress publication')
            self.assertEqual(received_progress[-1].reached_waypoint, 0)
            self.assertEqual(received_progress[-1].target_waypoint, 1)

            # 2. Publish an updated SafeZone with a moved target -> cancels first goal
            sz2 = self.make_safe_zone(1, 1, 2, 8.0, 2.0, last_wp=1, target_wp=2, progress=2.0)
            safe_zone_pub.publish(sz2)

            deadline = time.time() + 10.0
            while time.time() < deadline:
                if len(received_goals) >= 2 and len(cancelled_goals) >= 1:
                    break
                time.sleep(0.1)

            hold_first_goal.set()
            self.assertEqual(len(cancelled_goals), 1, 'Expected first inner goal to be cancelled')
            self.assertGreaterEqual(len(received_goals), 2, 'Expected second inner goal')
            self.assertAlmostEqual(received_goals[1][0], 8.0, places=3)
            self.assertAlmostEqual(received_goals[1][1], 2.0, places=3)

            # 3. Abort a subsequent direct SafeZone goal -> publishes CODE_PATH_BLOCKED
            time.sleep(0.5)
            abort_next_goal.set()
            sz3 = self.make_safe_zone(1, 2, 1, 10.0, 2.0, last_wp=2, target_wp=3, progress=3.0)
            safe_zone_pub.publish(sz3)

            deadline = time.time() + 10.0
            while time.time() < deadline:
                if received_errors:
                    break
                time.sleep(0.1)

            self.assertGreater(
                len(received_errors),
                0,
                'Expected PlanError when inner NavigateToPose aborts',
            )
            self.assertEqual(received_errors[-1].error.code, PlanError.CODE_PATH_BLOCKED)
            self.assertEqual(received_errors[-1].plan_id.plan_version, 2)
        finally:
            inner_server.destroy()
