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

from math import isfinite

from geometry_msgs.msg import Point
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rmf_layered_map_msgs.msg import MapRegionPatch, MapRegionUpdate
from rmf_prototype_msgs.msg import Region
from visualization_msgs.msg import Marker, MarkerArray


SOURCE_COLORS = (
    (0.96, 0.26, 0.21),
    (0.18, 0.80, 0.44),
    (0.20, 0.60, 0.98),
    (1.00, 0.65, 0.15),
    (0.61, 0.35, 0.71),
    (0.10, 0.74, 0.80),
)


def _point(x, y):
    point = Point()
    point.x = float(x)
    point.y = float(y)
    point.z = 0.05
    return point


def _marker_lifetime(patch, update, default_ttl_sec):
    for ttl_sec in (
        patch.ttl_sec,
        update.source.default_ttl_sec,
        default_ttl_sec,
    ):
        if isfinite(ttl_sec) and ttl_sec > 0.0:
            return Duration(seconds=ttl_sec).to_msg()
    return Duration().to_msg()


def _new_marker(update, patch, marker_id, marker_type, color, default_ttl_sec):
    marker = Marker()
    marker.header.frame_id = update.source.header.frame_id
    marker.ns = update.source.source_id
    marker.id = marker_id
    marker.type = marker_type
    marker.action = Marker.ADD
    marker.pose = update.source.robot_pose
    marker.frame_locked = False
    marker.lifetime = _marker_lifetime(patch, update, default_ttl_sec)
    marker.color.r = color[0]
    marker.color.g = color[1]
    marker.color.b = color[2]
    marker.color.a = (
        0.85 if patch.update_type == MapRegionPatch.UPDATE_OBSTACLE else 0.35
    )
    return marker


def _markers_from_patch(update, patch, first_id, color, default_ttl_sec):
    point_regions = []
    rectangle_lines = []

    for region in patch.regions:
        if region.hint == Region.HINT_POINT and len(region.points) == 2:
            point_regions.append(_point(region.points[0], region.points[1]))
        elif (
            region.hint == Region.HINT_AXIS_ALIGNED_RECTANGLE
            and len(region.points) >= 4
            and len(region.points) % 2 == 0
        ):
            xs = region.points[0::2]
            ys = region.points[1::2]
            min_x, max_x = min(xs), max(xs)
            min_y, max_y = min(ys), max(ys)
            corners = (
                _point(min_x, min_y),
                _point(max_x, min_y),
                _point(max_x, max_y),
                _point(min_x, max_y),
            )
            rectangle_lines.extend((
                corners[0], corners[1],
                corners[1], corners[2],
                corners[2], corners[3],
                corners[3], corners[0],
            ))

    markers = []
    marker_id = first_id
    if point_regions:
        marker = _new_marker(
            update,
            patch,
            marker_id,
            Marker.POINTS,
            color,
            default_ttl_sec,
        )
        marker.scale.x = 0.12
        marker.scale.y = 0.12
        marker.points = point_regions
        markers.append(marker)
        marker_id += 1

    if rectangle_lines:
        marker = _new_marker(
            update,
            patch,
            marker_id,
            Marker.LINE_LIST,
            color,
            default_ttl_sec,
        )
        marker.scale.x = 0.08
        marker.points = rectangle_lines
        markers.append(marker)

    return markers


class RegionMarkerState:
    """Convert region updates into persistent per-source RViz marker actions."""

    def __init__(self, default_ttl_sec=30.0):
        self.default_ttl_sec = default_ttl_sec
        self.markers_by_source = {}
        self.next_ids = {}
        self.latest_stamps = {}
        self.source_colors = {}

    def apply_update(self, update):
        """Return marker actions that apply one region update to the RViz state."""
        stamp_nsec = (
            update.source.header.stamp.sec * 1_000_000_000
            + update.source.header.stamp.nanosec
        )
        if stamp_nsec == 0 or not update.source.header.frame_id:
            return MarkerArray()

        key = (update.source.source_id, update.source.map_name)
        latest_stamp = self.latest_stamps.get(key)
        if latest_stamp is not None and stamp_nsec < latest_stamp:
            return MarkerArray()

        color = self.source_colors.setdefault(
            key,
            SOURCE_COLORS[len(self.source_colors) % len(SOURCE_COLORS)],
        )
        marker_array = MarkerArray()
        self.markers_by_source[key] = [
            marker
            for marker in self.markers_by_source.get(key, ())
            if marker[3] is None or marker[3] > stamp_nsec
        ]

        if update.reset_source:
            for namespace, marker_id, frame_id, _ in self.markers_by_source[key]:
                marker = Marker()
                marker.header.frame_id = frame_id
                marker.ns = namespace
                marker.id = marker_id
                marker.action = Marker.DELETE
                marker_array.markers.append(marker)
            self.markers_by_source[key] = []
            self.next_ids[key] = 0

        next_id = self.next_ids.get(key, 0)
        new_markers = []
        for patch in update.patches:
            if patch.update_type not in (
                MapRegionPatch.UPDATE_CLEAR,
                MapRegionPatch.UPDATE_OBSTACLE,
            ):
                continue
            patch_markers = _markers_from_patch(
                update,
                patch,
                next_id,
                color,
                self.default_ttl_sec,
            )
            new_markers.extend(patch_markers)
            next_id += len(patch_markers)

        marker_array.markers.extend(new_markers)
        self.next_ids[key] = next_id
        self.markers_by_source[key].extend(
            (
                marker.ns,
                marker.id,
                marker.header.frame_id,
                stamp_nsec + marker.lifetime.sec * 1_000_000_000
                + marker.lifetime.nanosec
                if marker.lifetime.sec > 0 or marker.lifetime.nanosec > 0
                else None,
            )
            for marker in new_markers
        )

        if new_markers or update.reset_source:
            self.latest_stamps[key] = stamp_nsec

        return marker_array


class RegionUpdateVisualizer(Node):
    """Visualize active map-region contributions as colored RViz markers."""

    def __init__(self):
        super().__init__('region_update_visualizer')
        input_topic = self.declare_parameter(
            'input_topic', '/map/region_updates'
        ).value
        output_topic = self.declare_parameter(
            'output_topic', '/map/region_markers'
        ).value
        default_ttl_sec = self.declare_parameter(
            'default_ttl_sec', 30.0
        ).value

        reliable_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.state = RegionMarkerState(default_ttl_sec)
        self.publisher = self.create_publisher(
            MarkerArray,
            output_topic,
            reliable_qos,
        )
        self.subscription = self.create_subscription(
            MapRegionUpdate,
            input_topic,
            self.visualize_update,
            reliable_qos,
        )
        self.get_logger().info(
            f'Visualizing {input_topic} as {output_topic}'
        )

    def visualize_update(self, update):
        """Publish the marker actions for an incoming region update."""
        marker_array = self.state.apply_update(update)
        if marker_array.markers:
            self.publisher.publish(marker_array)


def main():
    rclpy.init()
    node = RegionUpdateVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
