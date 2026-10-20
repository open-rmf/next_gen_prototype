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

"""Publish ground-truth map->odom TF and amcl_pose from Gazebo odometry."""

from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import StaticTransformBroadcaster


class GroundTruthLocalizer(Node):
    """Bridge Gazebo ground-truth world odometry to map->odom TF and amcl_pose."""

    def __init__(self):
        """Initialize the ground-truth localizer node."""
        super().__init__('ground_truth_localizer')
        self.tf_broadcaster = StaticTransformBroadcaster(self)

        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'map'
        transform.child_frame_id = 'odom'
        transform.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(transform)

        amcl_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.amcl_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, 'amcl_pose', amcl_qos
        )
        self.create_subscription(Odometry, 'odom', self._on_odom, 10)

    def _on_odom(self, msg: Odometry):
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = msg.header.stamp
        pose_msg.header.frame_id = 'map'
        pose_msg.pose = msg.pose
        self.amcl_pose_pub.publish(pose_msg)


def main(args=None):
    """Run the ground-truth localizer node."""
    rclpy.init(args=args)
    node = GroundTruthLocalizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
