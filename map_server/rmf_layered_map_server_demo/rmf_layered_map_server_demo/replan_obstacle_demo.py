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

from math import atan2, cos, pi, sin
import subprocess
import time

from geometry_msgs.msg import Point
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rmf_prototype_msgs.msg import Plan
from visualization_msgs.msg import Marker, MarkerArray

from .replan_scenarios import get_scenario, ROBOT_COLORS


ROBOT_MARKER_Z = 0.40
GOAL_MARKER_Z = 0.30


def make_navigation_goal(x, y, yaw=pi / 2.0):
    """Create a map-frame Nav2 goal."""
    goal = NavigateToPose.Goal()
    goal.pose.header.frame_id = 'map'
    goal.pose.pose.position.x = float(x)
    goal.pose.pose.position.y = float(y)
    goal.pose.pose.orientation.z = sin(yaw / 2.0)
    goal.pose.pose.orientation.w = cos(yaw / 2.0)
    return goal


def plan_signature(plan):
    """Return a plan's waypoint coordinates."""
    return tuple((waypoint.position[0], waypoint.position[1]) for waypoint in plan.waypoints)


def _point(x, y, z=0.1):
    point = Point()
    point.x = float(x)
    point.y = float(y)
    point.z = float(z)
    return point


def _base_marker(marker_id, namespace, marker_type, color):
    marker = Marker()
    marker.header.frame_id = 'map'
    marker.ns = namespace
    marker.id = marker_id
    marker.type = marker_type
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0
    marker.scale.x = 1.0
    marker.scale.y = 1.0
    marker.scale.z = 1.0
    marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
    return marker


def triangle_marker(marker_id, namespace, x, y, yaw, color):
    """Create a robot marker."""
    marker = _base_marker(marker_id, namespace, Marker.TRIANGLE_LIST, color)
    local_points = ((0.65, 0.0), (-0.45, 0.4), (-0.45, -0.4))
    for local_x, local_y in local_points:
        world_x = x + cos(yaw) * local_x - sin(yaw) * local_y
        world_y = y + sin(yaw) * local_x + cos(yaw) * local_y
        marker.points.append(_point(world_x, world_y, ROBOT_MARKER_Z))
    return marker


def star_marker(
    marker_id,
    x,
    y,
    color=(0.25, 0.75, 0.12, 1.0),
    namespace='goal',
):
    """Create a star-shaped goal marker."""
    marker = _base_marker(
        marker_id,
        namespace,
        Marker.TRIANGLE_LIST,
        color,
    )
    ring = []
    for index in range(10):
        radius = 0.8 if index % 2 == 0 else 0.35
        angle = pi / 2.0 + index * pi / 5.0
        ring.append(
            _point(
                x + radius * cos(angle),
                y + radius * sin(angle),
                GOAL_MARKER_Z,
            )
        )
    center = _point(x, y, GOAL_MARKER_Z)
    for index, point in enumerate(ring):
        marker.points.extend((center, point, ring[(index + 1) % len(ring)]))
    return marker


def bar_marker(marker_id, scenario):
    """Create the obstacle marker."""
    marker = _base_marker(
        marker_id,
        'spawned_obstacle',
        Marker.CUBE,
        (0.95, 0.05, 0.05, 0.9),
    )
    marker.pose.position.x = scenario.bar_center[0]
    marker.pose.position.y = scenario.bar_center[1]
    marker.pose.position.z = 0.1
    marker.scale.x = scenario.bar_size[0]
    marker.scale.y = scenario.bar_size[1]
    marker.scale.z = 0.2
    return marker


def box_sdf(name, x, y, size_x, size_y, size_z, color):
    """Build SDF for a static box."""
    rgba = ' '.join(str(channel) for channel in color)
    return (
        "<sdf version='1.7'>"
        f"<model name='{name}'>"
        f'<pose>{x} {y} {size_z / 2.0} 0 0 0</pose>'
        '<static>true</static>'
        "<link name='link'>"
        "<visual name='visual'>"
        f'<geometry><box><size>{size_x} {size_y} {size_z}</size></box></geometry>'
        f'<material><ambient>{rgba}</ambient><diffuse>{rgba}</diffuse></material>'
        '</visual>'
        "<collision name='collision'>"
        f'<geometry><box><size>{size_x} {size_y} {size_z}</size></box></geometry>'
        '</collision>'
        '</link>'
        '</model>'
        '</sdf>'
    )


def yaw_from_odometry(odometry):
    """Return the planar yaw from odometry."""
    orientation = odometry.pose.pose.orientation
    return atan2(
        2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
        1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
    )


def spawn_bar(world_name, scenario):
    """Spawn the obstacle in Gazebo."""
    sdf = box_sdf(
        scenario.bar_name,
        scenario.bar_center[0],
        scenario.bar_center[1],
        *scenario.bar_size,
        color=(0.95, 0.05, 0.05, 1.0),
    )
    request = f'sdf: "{sdf}"'
    return subprocess.run(
        [
            'gz',
            'service',
            '-s',
            f'/world/{world_name}/create',
            '--reqtype',
            'gz.msgs.EntityFactory',
            '--reptype',
            'gz.msgs.Boolean',
            '--timeout',
            '1000',
            '--req',
            request,
        ],
        capture_output=True,
        check=False,
        text=True,
        timeout=3.0,
    )


def spawn_succeeded(result):
    """Check that Gazebo accepted the entity."""
    output = f'{result.stdout}\n{result.stderr}'.lower()
    return result.returncode == 0 and 'data: true' in output


class ReplanObstacleDemo(Node):
    """Run the two-robot replanning demo."""

    def __init__(self):
        super().__init__('replan_obstacle_demo')
        scenario_name = self.declare_parameter('scenario', 'simple').value
        self.scenario = get_scenario(scenario_name)
        self.robot_layout = {
            robot.name: robot.pose for robot in self.scenario.robots
        }
        self.robot_goals = {
            robot.name: robot.goal for robot in self.scenario.robots
        }
        self.world_name = self.declare_parameter(
            'world_name', self.scenario.world_name
        ).value
        self.spawn_delay_sec = self.declare_parameter(
            'spawn_delay_sec', 4.0
        ).value
        self.timeout_sec = self.declare_parameter(
            'scenario_timeout_sec', 180.0
        ).value
        self.started_at = time.monotonic()
        self.spawn_at = None
        self.next_spawn_attempt = None
        self.state = 'waiting_for_inputs'
        self.timeout_logged = False
        self.map_ready = False
        self.bar_spawned = False
        self.odometry = {}
        self.initial_versions = {}
        self.initial_paths = {}
        self.replan_versions = {}
        self.changed_paths = {}

        reliable_transient_qos = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.navigation_clients = {}
        self.inner_navigation_clients = {}
        self.nav2_lifecycle_clients = {}
        self.nav2_state_futures = {}
        self.nav2_active = set()
        self.goal_futures = {}
        self.goal_handles = {}
        self._demo_subscriptions = []
        for robot_name in self.robot_layout:
            self.navigation_clients[robot_name] = ActionClient(
                self,
                NavigateToPose,
                f'/{robot_name}/navigate_to_pose',
            )
            # Wait for Nav2 to activate before sending outer goals.
            self.inner_navigation_clients[robot_name] = ActionClient(
                self,
                NavigateToPose,
                f'/{robot_name}/inner/navigate_to_pose',
            )
            self.nav2_lifecycle_clients[robot_name] = self.create_client(
                GetState,
                f'/{robot_name}/inner/bt_navigator/get_state',
            )
            self._demo_subscriptions.append(
                self.create_subscription(
                    Plan,
                    f'/{robot_name}/plan',
                    lambda msg, name=robot_name: self.receive_plan(name, msg),
                    reliable_transient_qos,
                )
            )
            self._demo_subscriptions.append(
                self.create_subscription(
                    Odometry,
                    f'/{robot_name}/odom',
                    lambda msg, name=robot_name: self.receive_odometry(name, msg),
                    10,
                )
            )

        self._demo_subscriptions.append(
            self.create_subscription(
                OccupancyGrid,
                '/map',
                self.receive_map,
                reliable_transient_qos,
            )
        )
        self.marker_publisher = self.create_publisher(
            MarkerArray,
            '/replan_scenario/markers',
            reliable_transient_qos,
        )
        self.timer = self.create_timer(0.25, self.tick)
        self.marker_timer = self.create_timer(0.5, self.publish_markers)

    def receive_map(self, msg):
        self.map_ready = (
            msg.info.width > 0
            and msg.info.height > 0
            and msg.info.resolution > 0.0
        )

    def receive_odometry(self, robot_name, msg):
        self.odometry[robot_name] = msg

    def receive_plan(self, robot_name, msg):
        version = msg.plan_id.plan_version
        signature = plan_signature(msg)
        if robot_name not in self.initial_versions:
            self.initial_versions[robot_name] = version
            self.initial_paths[robot_name] = signature
            self.get_logger().info(
                f'Received initial {robot_name} plan v{version} '
                f'with {len(signature)} waypoints'
            )
        elif version > self.initial_versions[robot_name]:
            self.replan_versions[robot_name] = version
            self.changed_paths[robot_name] = (
                signature != self.initial_paths[robot_name]
            )
            self.get_logger().info(
                f'Received replanned {robot_name} plan v{version}; '
                f'route changed={self.changed_paths[robot_name]}'
            )

        if (
            self.state == 'waiting_for_initial_plans'
            and len(self.initial_versions) == len(self.robot_layout)
        ):
            self.spawn_at = time.monotonic() + self.spawn_delay_sec
            self.state = 'waiting_to_spawn'
            self.get_logger().info(
                f'Initial plans are active; spawning the blocking bar in '
                f'{self.spawn_delay_sec:.1f}s'
            )

        if (
            self.state == 'waiting_for_replan'
            and 'robot1' in self.replan_versions
        ):
            if self.changed_paths.get('robot1', False):
                versions = ', '.join(
                    f'{name}=v{self.replan_versions[name]}'
                    for name in sorted(self.replan_versions)
                )
                self.get_logger().info(
                    f'Replan succeeded ({versions}); '
                    'robot1 now avoids the spawned obstacle'
                )
                self.state = 'complete'
            else:
                self.get_logger().error(
                    'Plan version increased without a route change'
                )
                self.state = 'failed'

    def receive_goal_response(self, robot_name, future):
        try:
            goal_handle = future.result()
        except Exception as error:
            self.get_logger().error(
                f'Outer navigation request for {robot_name} failed: {error}'
            )
            self.state = 'failed'
            return
        if not goal_handle.accepted:
            self.get_logger().error(
                f'Outer navigation request for {robot_name} was rejected'
            )
            self.state = 'failed'
            return
        self.goal_handles[robot_name] = goal_handle
        self.get_logger().info(
            f'Outer navigation request for {robot_name} was accepted'
        )

    def nav2_servers_are_active(self):
        """Check whether both Nav2 servers are active."""
        for robot_name, client in self.nav2_lifecycle_clients.items():
            future = self.nav2_state_futures.get(robot_name)
            if future is not None and future.done():
                try:
                    response = future.result()
                except Exception as error:
                    self.get_logger().warning(
                        f'Cannot read {robot_name} Nav2 lifecycle state: {error}'
                    )
                else:
                    if response.current_state.id == State.PRIMARY_STATE_ACTIVE:
                        self.nav2_active.add(robot_name)
                del self.nav2_state_futures[robot_name]

            if (
                robot_name not in self.nav2_active
                and robot_name not in self.nav2_state_futures
                and client.service_is_ready()
            ):
                self.nav2_state_futures[robot_name] = client.call_async(
                    GetState.Request()
                )

        return len(self.nav2_active) == len(self.robot_layout)

    def tick(self):
        now = time.monotonic()
        if (
            not self.timeout_logged
            and self.state not in ('complete', 'failed')
            and now - self.started_at > self.timeout_sec
        ):
            timed_out_state = self.state
            self.timeout_logged = True
            self.state = 'failed'
            self.get_logger().error(
                f'Demo timed out in state {timed_out_state}; '
                f'initial plans={sorted(self.initial_versions)}, '
                f'replans={sorted(self.replan_versions)}'
            )
            return

        if self.state == 'waiting_for_inputs':
            action_servers_ready = all(
                client.server_is_ready()
                for client in self.navigation_clients.values()
            ) and all(
                client.server_is_ready()
                for client in self.inner_navigation_clients.values()
            )
            nav2_active = self.nav2_servers_are_active()
            if (
                self.map_ready
                and len(self.odometry) == len(self.robot_layout)
                and action_servers_ready
                and nav2_active
            ):
                for robot_name, (goal_x, goal_y) in self.robot_goals.items():
                    goal = make_navigation_goal(goal_x, goal_y)
                    future = self.navigation_clients[robot_name].send_goal_async(goal)
                    future.add_done_callback(
                        lambda result, name=robot_name: self.receive_goal_response(
                            name, result
                        )
                    )
                    self.goal_futures[robot_name] = future
                self.state = 'waiting_for_initial_plans'
                self.get_logger().info('Sent navigation requests')
            return

        if self.state == 'waiting_to_spawn' and now >= self.spawn_at:
            if self.next_spawn_attempt is not None and now < self.next_spawn_attempt:
                return
            try:
                result = spawn_bar(self.world_name, self.scenario)
            except (OSError, subprocess.TimeoutExpired) as error:
                self.get_logger().warning(f'Waiting to spawn obstacle: {error}')
                self.next_spawn_attempt = now + 2.0
                return

            if not spawn_succeeded(result):
                detail = result.stderr.strip() or result.stdout.strip()
                if not detail:
                    detail = 'Gazebo did not confirm entity creation'
                self.get_logger().warning(f'Waiting to spawn obstacle: {detail}')
                self.next_spawn_attempt = now + 2.0
                return

            self.bar_spawned = True
            self.state = 'waiting_for_replan'
            self.get_logger().info(
                f'Spawned bar at {self.scenario.bar_center}; '
                'waiting for CODE_PATH_BLOCKED'
            )

    def publish_markers(self):
        markers = [
            star_marker(
                100 + index,
                *robot.goal,
                color=ROBOT_COLORS[robot.name],
                namespace=f'{robot.name}_goal',
            )
            for index, robot in enumerate(self.scenario.robots)
        ]
        for index, (robot_name, initial_pose) in enumerate(
            self.robot_layout.items()
        ):
            if robot_name in self.odometry:
                pose = self.odometry[robot_name].pose.pose
                x = pose.position.x
                y = pose.position.y
                yaw = yaw_from_odometry(self.odometry[robot_name])
            else:
                x, y, yaw = initial_pose
            color = ROBOT_COLORS[robot_name]
            markers.append(
                triangle_marker(
                    index,
                    f'{robot_name}_pose',
                    x,
                    y,
                    yaw,
                    color,
                )
            )
        if self.bar_spawned:
            markers.append(bar_marker(200, self.scenario))
        self.marker_publisher.publish(MarkerArray(markers=markers))


def main():
    rclpy.init()
    node = ReplanObstacleDemo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
