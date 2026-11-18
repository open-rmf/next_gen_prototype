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

from geometry_msgs.msg import Point, Quaternion
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rmf_prototype_msgs.msg import Participant, ParticipantList, Plan, PlanRelease, Progress


class MockRobotSimNode(Node):
    def __init__(self):
        super().__init__('rmf_mock_robot_sim')

        # 1. Declare and retrieve parameters
        self.declare_parameter('robot_name', 'robot_1')
        self.declare_parameter('speed', 1.0)
        self.declare_parameter('update_rate', 20.0)
        self.declare_parameter('initial_x', 0.0)
        self.declare_parameter('initial_y', 0.0)
        self.declare_parameter('initial_yaw', 0.0)
        self.declare_parameter('publish_discovery', True)

        self.robot_name = self.get_parameter(
            'robot_name'
        ).get_parameter_value().string_value

        self.speed = self.get_parameter(
            'speed'
        ).get_parameter_value().double_value

        self.update_rate = self.get_parameter(
            'update_rate'
        ).get_parameter_value().double_value

        self.x = self.get_parameter(
            'initial_x'
        ).get_parameter_value().double_value

        self.y = self.get_parameter(
            'initial_y'
        ).get_parameter_value().double_value

        self.yaw = self.get_parameter(
            'initial_yaw'
        ).get_parameter_value().double_value

        self.publish_discovery_param = self.get_parameter(
            'publish_discovery'
        ).get_parameter_value().bool_value

        self.get_logger().info(
            f"Initializing mock robot '{self.robot_name}' "
            f'at ({self.x:.2f}, {self.y:.2f}, {self.yaw:.2f}) '
            f'with speed {self.speed:.2f} m/s'
        )

        # 2. Initialize path following state
        self.current_path = []
        self.current_waypoint_idx = 0
        self.current_plan_id = None
        self.plan_start_pos = (self.x, self.y)
        self.last_update_time = self.get_clock().now()
        self.wait_time_remaining = 0.0
        self.released_waypoint_idx = 0
        self.latest_release = None
        self.was_blocked = False

        # 3. Configure QoS for Discovery & Reliable Data
        discovery_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        reliable_transient_qos = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        # 4. Create Publishers
        self.odom_pub = self.create_publisher(
            Odometry,
            f'{self.robot_name}/odom',
            qos_profile=10
        )

        self.progress_pub = self.create_publisher(
            Progress,
            f'{self.robot_name}/plan/progress',
            qos_profile=reliable_transient_qos
        )

        self.dest_discovery_pub = self.create_publisher(
            ParticipantList,
            '/destination/discovery',
            qos_profile=discovery_qos
        )

        self.plan_discovery_pub = self.create_publisher(
            ParticipantList,
            '/plan/discovery',
            qos_profile=discovery_qos
        )

        # 5. Create Subscriptions
        # Subscribe to plan topic
        self.plan_sub = self.create_subscription(
            Plan,
            f'{self.robot_name}/plan',
            self.handle_plan,
            qos_profile=reliable_transient_qos
        )

        # Also subscribe to path topic as mentioned in the README
        self.path_sub = self.create_subscription(
            Plan,
            f'{self.robot_name}/path',
            self.handle_plan,
            qos_profile=reliable_transient_qos
        )

        # Subscribe to plan release topic
        self.release_sub = self.create_subscription(
            PlanRelease,
            f'{self.robot_name}/plan/release',
            self.handle_release,
            qos_profile=reliable_transient_qos
        )

        # 6. Timers
        # Simulation update timer
        dt = 1.0 / self.update_rate
        self.sim_timer = self.create_timer(dt, self.sim_step)

        # Periodic discovery publication timer
        if self.publish_discovery_param:
            self.discovery_timer = self.create_timer(1.0, self.publish_discovery)

            # Publish initial discovery message immediately
            self.publish_discovery()

    def yaw_to_quaternion(self, yaw):
        return Quaternion(
            x=0.0,
            y=0.0,
            z=math.sin(yaw / 2.0),
            w=math.cos(yaw / 2.0)
        )

    def publish_discovery(self):
        msg = ParticipantList()
        p = Participant()
        p.name = self.robot_name
        p.components = []
        msg.participants.append(p)

        self.dest_discovery_pub.publish(msg)
        self.plan_discovery_pub.publish(msg)

    def _plan_id_matches(self, plan_id_a, plan_id_b):
        if plan_id_a is None or plan_id_b is None:
            return False
        return (
            list(plan_id_a.destination_session.uuid)
            == list(plan_id_b.destination_session.uuid)
            and plan_id_a.plan_version == plan_id_b.plan_version
        )

    def handle_plan(self, msg: Plan):
        self.get_logger().info(
            f'Received new plan with {len(msg.waypoints)} waypoints. '
            f'Plan version: {msg.plan_id.plan_version}'
        )
        self.current_path = msg.waypoints
        self.current_plan_id = msg.plan_id
        self.current_waypoint_idx = 0
        self.plan_start_pos = (self.x, self.y)
        self.wait_time_remaining = 0.0
        if self.latest_release and self._plan_id_matches(
            self.latest_release.plan_id, self.current_plan_id
        ):
            self.released_waypoint_idx = self.latest_release.waypoint_id
        else:
            self.released_waypoint_idx = 0
        self.was_blocked = False

        # If the path is not empty and we are far from the start,
        # we could either teleport or travel there.
        # The standard practice is to just start moving from current
        # position to waypoints[0].

    def handle_release(self, msg: PlanRelease):
        self.get_logger().info(
            f'Received PlanRelease: waypoint_id={msg.waypoint_id}, '
            f'plan_version={msg.plan_id.plan_version}'
        )
        self.latest_release = msg
        if not self.current_plan_id:
            self.get_logger().info('Cannot update release: current_plan_id is None')
            return

        msg_uuid = list(msg.plan_id.destination_session.uuid)
        curr_uuid = list(self.current_plan_id.destination_session.uuid)
        uuid_matches = msg_uuid == curr_uuid
        version_matches = msg.plan_id.plan_version == self.current_plan_id.plan_version
        self.get_logger().info(
            f'PlanRelease check: uuid_matches={uuid_matches}, version_matches={version_matches}'
        )
        if uuid_matches and version_matches:
            self.released_waypoint_idx = msg.waypoint_id
            self.get_logger().info(
                f'Successfully updated released_waypoint_idx to {self.released_waypoint_idx}'
            )

    def is_blocked(self):
        if not self.current_path or self.current_waypoint_idx >= len(self.current_path):
            return False
        if self.current_waypoint_idx > self.released_waypoint_idx:
            return True
        return False

    def sim_step(self):
        now = self.get_clock().now()
        # Calculate actual dt in seconds
        dt = (now - self.last_update_time).nanoseconds / 1e9
        self.last_update_time = now

        # Prevent large jumps if clock or simulation behaves unexpectedly
        if dt <= 0.0 or dt > 1.0:
            dt = 1.0 / self.update_rate

        # 1. Update position based on current path
        if self.current_path and self.current_waypoint_idx < len(self.current_path):
            if self.is_blocked():
                if not self.was_blocked:
                    self.get_logger().info(
                        f'Blocked! current_waypoint_idx={self.current_waypoint_idx}, '
                        f'released_waypoint_idx={self.released_waypoint_idx}'
                    )
                    self.was_blocked = True
                # Blocked by traffic dependencies! Do not move.
                pass
            else:
                if self.was_blocked:
                    self.get_logger().info(
                        f'Unblocked! current_waypoint_idx={self.current_waypoint_idx}, '
                        f'released_waypoint_idx={self.released_waypoint_idx}'
                    )
                    self.was_blocked = False
                if self.wait_time_remaining > 0.0:
                    self.wait_time_remaining -= dt
                    if self.wait_time_remaining < 0.0:
                        self.wait_time_remaining = 0.0
                else:
                    target_wp = self.current_path[self.current_waypoint_idx]
                    tx, ty = float(target_wp.position[0]), float(target_wp.position[1])
                    dx = tx - self.x
                    dy = ty - self.y
                    dist = math.hypot(dx, dy)

                    if dist > 1e-3:
                        self.yaw = math.atan2(dy, dx)
                        step = self.speed * dt
                        if step >= dist:
                            # Reached the target waypoint
                            self.x = tx
                            self.y = ty
                            self.current_waypoint_idx += 1
                            self.wait_time_remaining = 1.0
                        else:
                            self.x += (dx / dist) * step
                            self.y += (dy / dist) * step
                    else:
                        # Already at or extremely close to target waypoint, advance to next
                        self.x = tx
                        self.y = ty
                        self.current_waypoint_idx += 1
                        self.wait_time_remaining = 1.0

        # 2. Publish plan progress
        if self.current_path:
            self.publish_progress()

        # 3. Publish Odometry
        self.publish_odometry()

    def publish_progress(self):
        if not self.current_plan_id or not self.current_path:
            return

        progress_msg = Progress()
        progress_msg.plan_id = self.current_plan_id

        num_waypoints = len(self.current_path)

        if self.current_waypoint_idx >= num_waypoints:
            last_idx = num_waypoints - 1
            progress_msg.reached_waypoint = int(last_idx)
            progress_msg.target_waypoint = int(last_idx)
            progress_msg.progress = float(self.current_path[last_idx].progress)
        elif self.current_waypoint_idx == 0:
            first_wp = self.current_path[0]
            progress_msg.reached_waypoint = 0
            progress_msg.target_waypoint = 0
            seg_dist = math.hypot(
                first_wp.position[0] - self.plan_start_pos[0],
                first_wp.position[1] - self.plan_start_pos[1],
            )
            if seg_dist > 1e-3:
                dist_from_start = math.hypot(
                    self.x - self.plan_start_pos[0],
                    self.y - self.plan_start_pos[1],
                )
                fraction = min(1.0, max(0.0, dist_from_start / seg_dist))
                progress_msg.progress = float(fraction * first_wp.progress)
            else:
                progress_msg.progress = 0.0
        elif self.is_blocked():
            reached_idx = self.current_waypoint_idx - 1
            progress_msg.reached_waypoint = int(reached_idx)
            progress_msg.target_waypoint = int(reached_idx)
            progress_msg.progress = float(self.current_path[reached_idx].progress)
        else:
            reached_idx = self.current_waypoint_idx - 1
            target_idx = self.current_waypoint_idx
            progress_msg.reached_waypoint = int(reached_idx)
            progress_msg.target_waypoint = int(target_idx)

            prev_wp = self.current_path[reached_idx]
            next_wp = self.current_path[target_idx]

            prev_pos = prev_wp.position
            next_pos = next_wp.position
            seg_dist = math.hypot(
                next_pos[0] - prev_pos[0],
                next_pos[1] - prev_pos[1],
            )

            if seg_dist > 1e-3:
                dist_from_prev = math.hypot(
                    self.x - prev_pos[0],
                    self.y - prev_pos[1],
                )
                fraction = min(1.0, max(0.0, dist_from_prev / seg_dist))
                diff = next_wp.progress - prev_wp.progress
                progress_msg.progress = float(
                    prev_wp.progress + fraction * diff
                )
            else:
                fraction = min(1.0, max(0.0, 1.0 - self.wait_time_remaining))
                diff = next_wp.progress - prev_wp.progress
                progress_msg.progress = float(
                    prev_wp.progress + fraction * diff
                )

        progress_msg.reached_keys = []
        self.progress_pub.publish(progress_msg)

    def publish_odometry(self):
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'map'
        odom.child_frame_id = f'{self.robot_name}/base_link'

        # Set pose
        odom.pose.pose.position = Point(x=self.x, y=self.y, z=0.0)
        odom.pose.pose.orientation = self.yaw_to_quaternion(self.yaw)

        # Set twist (simple velocity estimate)
        if (
            self.current_path
            and self.current_waypoint_idx < len(self.current_path)
            and self.wait_time_remaining <= 0.0
            and not self.is_blocked()
        ):
            target_wp = self.current_path[self.current_waypoint_idx]
            tx, ty = target_wp.position[0], target_wp.position[1]
            dx = tx - self.x
            dy = ty - self.y
            dist = math.hypot(dx, dy)
            if dist > 1e-3:
                odom.twist.twist.linear.x = (dx / dist) * self.speed
                odom.twist.twist.linear.y = (dy / dist) * self.speed
                odom.twist.twist.angular.z = 0.0
        else:
            odom.twist.twist.linear.x = 0.0
            odom.twist.twist.linear.y = 0.0
            odom.twist.twist.angular.z = 0.0

        self.odom_pub.publish(odom)


def main(args=None):
    import rclpy
    rclpy.init(args=args)
    node = MockRobotSimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
