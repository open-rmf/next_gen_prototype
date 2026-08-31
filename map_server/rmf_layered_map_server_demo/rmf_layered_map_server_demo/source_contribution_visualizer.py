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

from math import atan2, cos, sin

from geometry_msgs.msg import Point
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rmf_layered_map_msgs.msg import MapSourceSnapshot
from visualization_msgs.msg import Marker, MarkerArray

from .region_update_visualizer import OBSTACLE_MARKER_Z, SOURCE_COLORS
from .replan_scenarios import ROBOT_COLORS


def _yaw_from_quaternion(quaternion):
    return atan2(
        2.0 * (
            quaternion.w * quaternion.z
            + quaternion.x * quaternion.y
        ),
        1.0 - 2.0 * (
            quaternion.y * quaternion.y
            + quaternion.z * quaternion.z
        ),
    )


def _cell_point(info, cell_index):
    width = int(info.width)
    height = int(info.height)
    if width <= 0 or height <= 0 or cell_index >= width * height:
        return None

    cell_x = cell_index % width
    cell_y = cell_index // width
    local_x = (cell_x + 0.5) * info.resolution
    local_y = (cell_y + 0.5) * info.resolution
    yaw = _yaw_from_quaternion(info.origin.orientation)
    cosine = cos(yaw)
    sine = sin(yaw)

    point = Point()
    point.x = info.origin.position.x + cosine * local_x - sine * local_y
    point.y = info.origin.position.y + sine * local_x + cosine * local_y
    point.z = OBSTACLE_MARKER_Z
    return point


class SourceContributionMarkerState:
    """Convert backend source snapshots into colored marker actions."""

    def __init__(self):
        self.source_colors = {}

    def _source_color(self, source, key):
        color = self.source_colors.get(key)
        if color is not None:
            return color

        robot_color = ROBOT_COLORS.get(source.robot_name)
        color = (
            robot_color[:3]
            if robot_color is not None
            else SOURCE_COLORS[len(self.source_colors) % len(SOURCE_COLORS)]
        )
        self.source_colors[key] = color
        return color

    def _new_points_marker(self, snapshot, namespace, points, color):
        marker = Marker()
        marker.header = snapshot.header
        marker.ns = namespace
        marker.id = 0
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = max(0.03, snapshot.info.resolution * 0.8)
        marker.scale.y = marker.scale.x
        marker.points = points
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = 0.85
        return marker

    def apply_snapshot(self, snapshot):
        """Replace the displayed contributions with the latest snapshot."""
        marker_array = MarkerArray()
        reset = Marker()
        reset.action = Marker.DELETEALL
        marker_array.markers.append(reset)

        if (
            not snapshot.header.frame_id
            or snapshot.info.width == 0
            or snapshot.info.height == 0
            or snapshot.info.resolution <= 0.0
        ):
            return marker_array

        for contribution in snapshot.sources:
            if (
                not contribution.source.source_id
                or len(contribution.cell_indices)
                != len(contribution.occupancy_values)
            ):
                continue

            source_id = contribution.source.source_id
            key = (source_id, contribution.source.map_name)
            namespace = f'{source_id}/backend_contributions'
            color = self._source_color(contribution.source, key)
            points = []
            for cell_index, occupancy_value in zip(
                contribution.cell_indices,
                contribution.occupancy_values,
            ):
                if occupancy_value <= 0:
                    continue
                point = _cell_point(snapshot.info, cell_index)
                if point is not None:
                    points.append(point)

            if points:
                marker_array.markers.append(
                    self._new_points_marker(
                        snapshot,
                        namespace,
                        points,
                        color,
                    )
                )

        return marker_array


class SourceContributionVisualizer(Node):
    """Visualize map contributions by robot source."""

    def __init__(self):
        super().__init__('source_contribution_visualizer')
        input_topic = self.declare_parameter(
            'input_topic', '/map/source_contributions'
        ).value
        output_topic = self.declare_parameter(
            'output_topic', '/map/source_contribution_markers'
        ).value

        transient_qos = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.state = SourceContributionMarkerState()
        self.publisher = self.create_publisher(
            MarkerArray,
            output_topic,
            transient_qos,
        )
        self.subscription = self.create_subscription(
            MapSourceSnapshot,
            input_topic,
            self.visualize_snapshot,
            transient_qos,
        )
        self.get_logger().info(f'Visualizing {input_topic} as {output_topic}')

    def visualize_snapshot(self, snapshot):
        """Publish markers for a source snapshot."""
        self.publisher.publish(self.state.apply_snapshot(snapshot))


def main():
    rclpy.init()
    node = SourceContributionVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
