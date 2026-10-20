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
import subprocess
import time
import unittest

from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav2_msgs.msg import Costmap
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.action import ActionClient
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rmf_path_server_demo.corridor_swap_demo import make_destination_goal
from rmf_prototype_msgs.msg import (
    DestinationGoal,
    Plan,
    PlanError,
    RobotStatus,
    SafeZone,
)
from std_msgs.msg import Header


def stamp_to_seq(stamp) -> int:
    """Decode the 64-bit costmap handshake sequence number from a ROS Time stamp."""
    return ((int(stamp.sec) & 0xFFFFFFFF) << 32) | (
        int(stamp.nanosec) & 0xFFFFFFFF
    )


def cleanup_gazebo_processes():
    """Ensure gz sim and isolated Nav2 containers are terminated after a test."""
    subprocess.run(
        [
            'pkill',
            '-9',
            '-f',
            '[g]z sim|[c]omponent_container_isolated|[p]arameter_bridge',
        ],
        check=False,
    )


class CorridorNav2E2ETestBase(unittest.TestCase):
    """Shared base test harness for Nav2 + gz-sim corridor E2E launch tests."""

    @classmethod
    def setUpClass(cls):
        """Initialize rclpy and subscribe to status, plan, safe zone, and costmap topics."""
        rclpy.init()
        cls.node = rclpy.create_node(f'{cls.__name__.lower()}_observer')

        latched_qos = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        cls.map_ready = False
        cls.statuses = {}
        cls.plans = {'robot0': [], 'robot1': []}
        cls.safe_zones = {'robot0': [], 'robot1': []}
        cls.plan_errors = {'robot0': [], 'robot1': []}
        cls.costmap_seqs = {'robot0': [], 'robot1': []}
        cls.acked_seqs = {'robot0': [], 'robot1': []}
        cls.goal_pubs = {}
        cls.lifecycle_clients = {}
        cls.inner_nav_clients = {}
        cls.seed_counter = 20

        cls.node.create_subscription(
            OccupancyGrid,
            '/map',
            cls._on_map,
            latched_qos,
        )

        for robot in ('robot0', 'robot1'):
            cls.goal_pubs[robot] = cls.node.create_publisher(
                DestinationGoal,
                f'/{robot}/destination/goal',
                latched_qos,
            )
            cls.lifecycle_clients[robot] = cls.node.create_client(
                GetState,
                f'/{robot}/inner/bt_navigator/get_state',
            )
            cls.inner_nav_clients[robot] = ActionClient(
                cls.node,
                NavigateToPose,
                f'/{robot}/inner/navigate_to_pose',
            )
            cls.node.create_subscription(
                RobotStatus,
                f'/{robot}/status',
                lambda msg, r=robot: cls.statuses.__setitem__(r, msg),
                latched_qos,
            )
            cls.node.create_subscription(
                Plan,
                f'/{robot}/plan',
                lambda msg, r=robot: cls.plans[r].append(msg),
                latched_qos,
            )
            cls.node.create_subscription(
                SafeZone,
                f'/{robot}/plan/safe_zone',
                lambda msg, r=robot: cls.safe_zones[r].append(msg),
                latched_qos,
            )
            cls.node.create_subscription(
                PlanError,
                f'/{robot}/plan/error',
                lambda msg, r=robot: cls.plan_errors[r].append(msg),
                latched_qos,
            )
            cls.node.create_subscription(
                Costmap,
                f'/{robot}/inner/global_costmap/plan/costmap',
                lambda msg, r=robot: cls.costmap_seqs[r].append(
                    stamp_to_seq(msg.header.stamp)
                ),
                latched_qos,
            )
            cls.node.create_subscription(
                Header,
                f'/{robot}/inner/global_costmap/plan/costmap_updated',
                lambda msg, r=robot: cls.acked_seqs[r].append(
                    stamp_to_seq(msg.stamp)
                ),
                latched_qos,
            )

    @classmethod
    def tearDownClass(cls):
        """Destroy the observer node and shut down rclpy."""
        cls.node.destroy_node()
        rclpy.try_shutdown()

    @classmethod
    def _on_map(cls, msg: OccupancyGrid):
        cls.map_ready = msg.info.width > 0 and msg.info.height > 0

    def _wait_for_nav2_ready(self, timeout_sec: float = 60.0):
        """Wait until /map, both robot statuses, and both Nav2 stacks are active."""
        nav2_active = set()
        pending_futures = {}
        start = time.monotonic()

        while time.monotonic() - start < timeout_sec:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            now = time.monotonic()

            for robot, client in self.lifecycle_clients.items():
                entry = pending_futures.get(robot)
                if entry is not None:
                    fut, sent_at = entry
                    if fut.done():
                        try:
                            if (
                                fut.result().current_state.id
                                == State.PRIMARY_STATE_ACTIVE
                            ):
                                nav2_active.add(robot)
                        except Exception:
                            pass
                        del pending_futures[robot]
                    elif now - sent_at > 2.0:
                        client.remove_pending_request(fut)
                        del pending_futures[robot]

                if (
                    robot not in nav2_active
                    and robot not in pending_futures
                    and client.service_is_ready()
                ):
                    pending_futures[robot] = (
                        client.call_async(GetState.Request()),
                        now,
                    )

            action_ready = all(
                c.server_is_ready() for c in self.inner_nav_clients.values()
            )
            if (
                self.map_ready
                and len(self.statuses) == 2
                and action_ready
                and len(nav2_active) == 2
            ):
                # Allow 1.0s for Nav2 costmaps and TF to settle after activation
                settle_until = time.monotonic() + 1.0
                while time.monotonic() < settle_until:
                    rclpy.spin_once(self.node, timeout_sec=0.1)
                return

        self.fail(
            f'Nav2 stack did not become ready within {timeout_sec}s '
            f'(map_ready={self.map_ready}, statuses={list(self.statuses.keys())}, '
            f'nav2_active={nav2_active})'
        )

    def _publish_goal(self, robot: str, x: float, y: float, yaw: float) -> int:
        """Publish a DestinationGoal for a robot with a fresh session UUID."""
        type(self).seed_counter += 1
        seed = type(self).seed_counter
        goal = make_destination_goal(x, y, yaw, seed)
        self.goal_pubs[robot].publish(goal)
        return seed

    def _robot_pos(self, robot: str):
        pos = self.statuses[robot].pose.pose.position
        return (pos.x, pos.y)

    def _robot_yaw(self, robot: str) -> float:
        q = self.statuses[robot].pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def _angular_diff(a: float, b: float) -> float:
        diff = abs(a - b) % (2.0 * math.pi)
        return min(diff, 2.0 * math.pi - diff)

    def _wait_for_targets(
        self,
        targets: dict,
        timeout_sec: float = 80.0,
        tolerance: float = 0.35,
        yaw_tolerance: float = 0.30,
        step_hook=None,
    ):
        """Spin until all robots in `targets` reach their (x, y[, yaw]) within tolerance."""
        start = time.monotonic()
        while time.monotonic() - start < timeout_sec:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if step_hook is not None:
                step_hook()

            all_reached = True
            for r, target in targets.items():
                if r not in self.statuses:
                    all_reached = False
                    break
                tx, ty = target[0], target[1]
                rx, ry = self._robot_pos(r)
                if math.hypot(rx - tx, ry - ty) >= tolerance:
                    all_reached = False
                    break
                if len(target) >= 3 and target[2] is not None:
                    ryaw = self._robot_yaw(r)
                    if self._angular_diff(ryaw, target[2]) >= yaw_tolerance:
                        all_reached = False
                        break
            if all_reached:
                return

        current_poses = {
            r: (self._robot_pos(r), self._robot_yaw(r))
            if r in self.statuses
            else None
            for r in targets
        }
        self.fail(
            f'Robots failed to reach targets {targets} within {timeout_sec}s. '
            f'Current poses: {current_poses}'
        )

    def _assert_no_blocked_errors(self):
        blocked_errors = {
            robot: [
                err.error.code
                for err in errs
                if err.error.code == PlanError.CODE_PATH_BLOCKED
            ]
            for robot, errs in self.plan_errors.items()
        }
        self.assertEqual(
            len(blocked_errors['robot0']),
            0,
            f'robot0 emitted unexpected CODE_PATH_BLOCKED errors: {blocked_errors}',
        )
        self.assertEqual(
            len(blocked_errors['robot1']),
            0,
            f'robot1 emitted unexpected CODE_PATH_BLOCKED errors: {blocked_errors}',
        )
