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

from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rmf_prototype_msgs.msg import (
    DestinationConstraints,
    DestinationGoal,
    Region,
    RobotStatus,
    SafeZone,
    TargetOrientation,
    TargetRegion,
)
from unique_identifier_msgs.msg import UUID


def make_destination_goal(x: float, y: float, yaw: float, seed: int):
    """Build a DestinationGoal message for a point target."""
    goal = DestinationGoal()
    constraints = DestinationConstraints()
    target_region = TargetRegion()
    target_region.tolerance = 0.25
    target_region.region.hint = Region.HINT_POINT
    target_region.region.points = [float(x), float(y)]
    orientation = TargetOrientation()
    orientation.orientation_radians = float(yaw)
    target_region.orientations = [orientation]
    constraints.regions = [target_region]
    goal.one_of = [constraints]
    uuid_bytes = [0] * 16
    uuid_bytes[0] = seed & 0xFF
    goal.session = UUID(uuid=uuid_bytes)
    return goal


class CorridorSwapDemo(Node):
    """Automatically dispatch and monitor multi-robot corridor scenarios."""

    def __init__(self):
        """Initialize the corridor scenario node."""
        super().__init__('corridor_swap_demo')
        self.scenario = str(self.declare_parameter('scenario', 'swap').value)
        self.second_robot_delay_sec = float(
            self.declare_parameter('second_robot_delay_sec', 0.0).value
        )
        self.goal_tolerance = float(
            self.declare_parameter('goal_tolerance', 0.35).value
        )
        self.loop = bool(self.declare_parameter('loop', False).value)

        self.endpoints = [
            (-2.0, 0.0, 0.0),
            (5.0, 0.0, math.pi),
        ]
        self.swap_round = 0
        self.seed_counter = 10
        self.state = 'waiting_for_ready'
        self.ready_since = None
        self.second_goal_due = None
        self.round_started_at = None
        self.last_progress_log = 0.0
        self.active_targets = {}
        self.retargeted = False

        self.map_ready = False
        self.status = {}
        self.safe_zones = {}
        self.nav2_active = set()
        self.nav2_state_futures = {}

        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self._on_map, qos
        )
        self.goal_pubs = {}
        self.inner_nav_clients = {}
        self.lifecycle_clients = {}
        self._robot_subs = []

        for robot_name in ('robot0', 'robot1'):
            self.goal_pubs[robot_name] = self.create_publisher(
                DestinationGoal, f'/{robot_name}/destination/goal', qos
            )
            self.inner_nav_clients[robot_name] = ActionClient(
                self, NavigateToPose, f'/{robot_name}/inner/navigate_to_pose'
            )
            self.lifecycle_clients[robot_name] = self.create_client(
                GetState, f'/{robot_name}/inner/bt_navigator/get_state'
            )
            self._robot_subs.append(
                self.create_subscription(
                    RobotStatus,
                    f'/{robot_name}/status',
                    lambda msg, r=robot_name: self._on_status(r, msg),
                    qos,
                )
            )
            self._robot_subs.append(
                self.create_subscription(
                    SafeZone,
                    f'/{robot_name}/plan/safe_zone',
                    lambda msg, r=robot_name: self._on_safe_zone(r, msg),
                    qos,
                )
            )

        self.timer = self.create_timer(0.25, self._tick)
        self.get_logger().info(
            f'CorridorSwapDemo (scenario={self.scenario}) started; '
            'waiting for robots and Nav2 to activate'
        )

    def _on_map(self, msg: OccupancyGrid):
        self.map_ready = msg.info.width > 0 and msg.info.height > 0

    def _on_status(self, robot_name: str, msg: RobotStatus):
        self.status[robot_name] = msg

    def _on_safe_zone(self, robot_name: str, msg: SafeZone):
        self.safe_zones[robot_name] = msg

    def _robot_yaw(self, robot_name: str) -> float:
        q = self.status[robot_name].pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def _angular_diff(a: float, b: float) -> float:
        diff = abs(a - b) % (2.0 * math.pi)
        return min(diff, 2.0 * math.pi - diff)

    def _initial_targets(self):
        if self.scenario == 'dock_pass':
            if self.swap_round % 2 == 0:
                return {
                    'robot0': (-1.0, 2.0, math.pi / 2.0),
                    'robot1': (1.0, 1.0, 0.0),
                }
            return {
                'robot0': (-1.0, 1.0, math.pi / 2.0),
                'robot1': (-2.0, 1.0, 0.0),
            }

        if self.scenario == 'dock_rotate_pass':
            if self.swap_round % 2 == 0:
                return {
                    'robot0': (-1.0, 1.0, math.pi / 2.0),
                    'robot1': (1.0, 1.0, 0.0),
                }
            return {
                'robot0': (-1.0, 1.0, -math.pi / 2.0),
                'robot1': (1.0, 1.0, 0.0),
            }

        if self.swap_round % 2 == 0:
            return {
                'robot0': self.endpoints[1][:2] + (0.0,),
                'robot1': self.endpoints[0][:2] + (math.pi,),
            }
        return {
            'robot0': self.endpoints[0][:2] + (math.pi,),
            'robot1': self.endpoints[1][:2] + (0.0,),
        }

    def _check_nav2_active(self) -> bool:
        now = time.monotonic()
        for robot_name, client in self.lifecycle_clients.items():
            entry = self.nav2_state_futures.get(robot_name)
            if entry is not None:
                future, sent_at = entry
                if future.done():
                    try:
                        response = future.result()
                    except Exception:
                        pass
                    else:
                        if (
                            response.current_state.id
                            == State.PRIMARY_STATE_ACTIVE
                        ):
                            self.nav2_active.add(robot_name)
                    del self.nav2_state_futures[robot_name]
                elif now - sent_at > 2.0:
                    client.remove_pending_request(future)
                    del self.nav2_state_futures[robot_name]

            if (
                robot_name not in self.nav2_active
                and robot_name not in self.nav2_state_futures
                and client.service_is_ready()
            ):
                self.nav2_state_futures[robot_name] = (
                    client.call_async(GetState.Request()),
                    now,
                )

        return len(self.nav2_active) == len(self.lifecycle_clients)

    def _publish_goal_for(
        self, robot_name: str, target_override=None
    ):
        if target_override is not None:
            self.active_targets[robot_name] = target_override
        gx, gy, gyaw = self.active_targets[robot_name]
        self.seed_counter += 1
        goal = make_destination_goal(gx, gy, gyaw, self.seed_counter)
        self.goal_pubs[robot_name].publish(goal)
        self.get_logger().info(
            f'Published DestinationGoal for {robot_name} -> '
            f'({gx:.1f}, {gy:.1f}, yaw={gyaw:.2f})'
        )

    def _log_progress(self, now: float, d0: float, d1: float):
        if now - self.last_progress_log < 2.0:
            return
        p0 = self.status['robot0'].pose.pose.position
        p1 = self.status['robot1'].pose.pose.position
        sz0 = self.safe_zones.get('robot0')
        sz1 = self.safe_zones.get('robot1')
        tw0 = list(sz0.target_waypoint) if sz0 else []
        tw1 = list(sz1.target_waypoint) if sz1 else []
        lw0 = sz0.last_waypoint if sz0 else -1
        lw1 = sz1.last_waypoint if sz1 else -1
        elapsed = now - self.round_started_at
        self.get_logger().info(
            f'[{elapsed:4.1f}s] '
            f'r0=({p0.x:5.2f},{p0.y:5.2f}) d={d0:.2f} '
            f'wp={lw0}->{tw0} | '
            f'r1=({p1.x:5.2f},{p1.y:5.2f}) d={d1:.2f} '
            f'wp={lw1}->{tw1}'
        )
        self.last_progress_log = now

    def _tick(self):
        now = time.monotonic()

        if self.state == 'waiting_for_ready':
            action_ready = all(
                client.server_is_ready()
                for client in self.inner_nav_clients.values()
            )
            nav2_active = self._check_nav2_active()
            if (
                self.map_ready
                and len(self.status) == 2
                and action_ready
                and nav2_active
            ):
                if self.ready_since is None:
                    self.ready_since = now
                    self.get_logger().info(
                        'Both robots localized and Nav2 active; '
                        f'dispatching {self.scenario} scenario in 2.0s'
                    )
                elif now - self.ready_since >= 2.0:
                    self.active_targets = self._initial_targets()
                    self.retargeted = False
                    self.round_started_at = now
                    self._publish_goal_for('robot0')
                    if self.scenario == 'dock_rotate_pass':
                        if self.swap_round % 2 == 0:
                            # Dispatch both robots up-front: robot0 stages at (-1, 1, pi/2)
                            # from (0, 1, 0) before docking into (-1, 2, pi/2), while
                            # robot1 is immediately dispatched to (1, 1, 0).
                            self._publish_goal_for('robot1')
                            self.state = 'waiting_for_dock_stage'
                        else:
                            # Undock robot0 straight South to (-1, 1, -pi/2) before
                            # resetting to (0, 1, 0) and (-2, 1, 0).
                            self.state = 'waiting_for_undock'
                    elif self.second_robot_delay_sec > 0.0:
                        self.second_goal_due = (
                            now + self.second_robot_delay_sec
                        )
                        self.state = 'waiting_for_second_goal'
                    else:
                        self._publish_goal_for('robot1')
                        self.state = 'running_swap'
            return

        if self.state == 'waiting_for_dock_stage':
            p0 = self.status['robot0'].pose.pose.position
            p1 = self.status['robot1'].pose.pose.position
            d0 = math.hypot(
                p0.x - self.active_targets['robot0'][0],
                p0.y - self.active_targets['robot0'][1],
            )
            d1 = math.hypot(
                p1.x - self.active_targets['robot1'][0],
                p1.y - self.active_targets['robot1'][1],
            )
            self._log_progress(now, d0, d1)
            yaw_err = self._angular_diff(
                self._robot_yaw('robot0'), self.active_targets['robot0'][2]
            )
            if d0 < self.goal_tolerance and yaw_err < 0.30:
                self.get_logger().info(
                    f'robot0 staged and rotated at ({p0.x:.2f}, {p0.y:.2f}, '
                    f'yaw_err={yaw_err:.2f}); dispatching dock goal -> (-1.0, 2.0)'
                )
                self._publish_goal_for(
                    'robot0',
                    target_override=(-1.0, 2.0, math.pi / 2.0),
                )
                self.state = 'running_swap'
            return

        if self.state == 'waiting_for_undock':
            p0 = self.status['robot0'].pose.pose.position
            p1 = self.status['robot1'].pose.pose.position
            d0 = math.hypot(
                p0.x - self.active_targets['robot0'][0],
                p0.y - self.active_targets['robot0'][1],
            )
            d1 = math.hypot(
                p1.x - self.active_targets['robot1'][0],
                p1.y - self.active_targets['robot1'][1],
            )
            self._log_progress(now, d0, d1)
            if d0 < self.goal_tolerance:
                self._publish_goal_for(
                    'robot0',
                    target_override=(0.0, 1.0, 0.0),
                )
                self._publish_goal_for(
                    'robot1',
                    target_override=(-2.0, 1.0, 0.0),
                )
                self.state = 'running_swap'
            return

        if self.state == 'waiting_for_second_goal':
            if now >= self.second_goal_due:
                self._publish_goal_for('robot1')
                self.state = 'running_swap'
            return

        if self.state in ('running_swap', 'running_post_swap'):
            p0 = self.status['robot0'].pose.pose.position
            p1 = self.status['robot1'].pose.pose.position

            if (
                self.scenario == 'mid_swap_retarget'
                and self.state == 'running_swap'
                and not self.retargeted
                and p0.x >= 0.0
            ):
                self.retargeted = True
                new_target = (3.0, -1.0, 0.0)
                self.get_logger().info(
                    f'Injecting mid-swap retarget for robot0 at '
                    f'({p0.x:.2f}, {p0.y:.2f}) -> south corridor {new_target[:2]}'
                )
                self._publish_goal_for('robot0', target_override=new_target)

            d0 = math.hypot(
                p0.x - self.active_targets['robot0'][0],
                p0.y - self.active_targets['robot0'][1],
            )
            d1 = math.hypot(
                p1.x - self.active_targets['robot1'][0],
                p1.y - self.active_targets['robot1'][1],
            )
            self._log_progress(now, d0, d1)

            if d0 < self.goal_tolerance and d1 < self.goal_tolerance:
                elapsed = now - self.round_started_at
                if (
                    self.scenario == 'swap_then_new_goals'
                    and self.state == 'running_swap'
                ):
                    self.get_logger().info(
                        f'Initial corridor swap complete in {elapsed:.1f}s! '
                        'Dispatching post-swap goals...'
                    )
                    self.round_started_at = now
                    self._publish_goal_for(
                        'robot0',
                        target_override=(-1.0, 1.0, math.pi),
                    )
                    self._publish_goal_for(
                        'robot1',
                        target_override=(4.0, 0.0, 0.0),
                    )
                    self.state = 'running_post_swap'
                    return

                self.get_logger().info(
                    f'Scenario {self.scenario} round {self.swap_round + 1} '
                    f'complete in {elapsed:.1f}s!'
                )
                if self.loop:
                    self.swap_round += 1
                    self.ready_since = now
                    self.state = 'waiting_for_ready'
                else:
                    self.state = 'completed'


def main(args=None):
    """Entry point for corridor_swap_demo."""
    rclpy.init(args=args)
    node = CorridorSwapDemo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
