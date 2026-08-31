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

from math import cos, sin

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


def parse_waypoints(values):
    """Parse a flat sequence of x, y, yaw waypoint triples."""
    if not values or len(values) % 3 != 0:
        raise ValueError('waypoints must contain one or more x, y, yaw triples')

    return [tuple(values[index:index + 3]) for index in range(0, len(values), 3)]


def goal_pose(x, y, yaw, stamp):
    """Create a map-frame pose for a Nav2 goal."""
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = stamp
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.orientation.z = sin(yaw / 2.0)
    pose.pose.orientation.w = cos(yaw / 2.0)
    return pose


def advance_waypoint(current, waypoint_count, status):
    """Advance after a successful goal and otherwise retry the same waypoint."""
    if status == GoalStatus.STATUS_SUCCEEDED:
        return (current + 1) % waypoint_count
    return current


class Nav2GoalPublisher(Node):
    """Cycle through demo waypoints whenever Nav2 completes a goal."""

    def __init__(self):
        super().__init__('nav2_goal_publisher')
        self.waypoints = parse_waypoints(
            self.declare_parameter('waypoints', [0.0, 0.0, 0.0]).value
        )
        self.next_waypoint = 0
        self.goal_in_progress = False
        self.waiting_logged = False
        self.localized = False
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        pose_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.pose_subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            'amcl_pose',
            self.receive_pose,
            pose_qos,
        )
        self.timer = self.create_timer(1.0, self.send_next_goal)

    def receive_pose(self, _):
        self.localized = True

    def send_next_goal(self):
        if self.goal_in_progress:
            return

        if not self.localized or not self.action_client.server_is_ready():
            if not self.waiting_logged:
                self.get_logger().info('Waiting for Nav2 localization and navigation')
                self.waiting_logged = True
            return

        self.waiting_logged = False
        x, y, yaw = self.waypoints[self.next_waypoint]
        goal = NavigateToPose.Goal()
        goal.pose = goal_pose(x, y, yaw, self.get_clock().now().to_msg())
        self.goal_in_progress = True
        future = self.action_client.send_goal_async(goal)
        future.add_done_callback(self.goal_response)
        self.get_logger().info(f'Sending demo goal ({x:.1f}, {y:.1f})')

    def goal_response(self, future):
        try:
            goal_handle = future.result()
        except Exception as error:
            self.get_logger().error(f'Failed to send demo goal: {error}')
            self.goal_in_progress = False
            return

        if not goal_handle.accepted:
            self.get_logger().warning('Nav2 rejected the demo goal; retrying')
            self.goal_in_progress = False
            return

        result = goal_handle.get_result_async()
        result.add_done_callback(self.goal_finished)

    def goal_finished(self, future):
        try:
            status = future.result().status
        except Exception as error:
            self.get_logger().error(f'Demo goal failed: {error}')
        else:
            if status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info('Demo goal reached')
            else:
                self.get_logger().warning(
                    f'Demo goal finished with status {status}; retrying'
                )
            self.next_waypoint = advance_waypoint(
                self.next_waypoint,
                len(self.waypoints),
                status,
            )

        self.goal_in_progress = False


def main():
    rclpy.init()
    node = Nav2GoalPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
