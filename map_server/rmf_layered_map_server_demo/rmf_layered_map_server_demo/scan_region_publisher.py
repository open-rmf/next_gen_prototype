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

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    qos_profile_sensor_data,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.time import Time
from rmf_layered_map_msgs.msg import (
    MapObservationSource,
    MapRegionPatch,
    MapRegionUpdate,
)
from rmf_prototype_msgs.msg import Region
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener

from .scan_conversion import scan_regions


def make_scan_patches(clear_polygons, obstacle_points, ttl_sec):
    """Build clear and obstacle patches for one converted laser scan."""
    patches = []
    if clear_polygons:
        clear_patch = MapRegionPatch()
        clear_patch.update_type = MapRegionPatch.UPDATE_CLEAR
        clear_patch.occupancy_value = 0
        clear_patch.ttl_sec = ttl_sec
        for polygon in clear_polygons:
            region = Region()
            region.hint = Region.HINT_CONVEX_POLYGON
            region.points = list(polygon)
            clear_patch.regions.append(region)
        patches.append(clear_patch)

    if obstacle_points:
        obstacle_patch = MapRegionPatch()
        obstacle_patch.update_type = MapRegionPatch.UPDATE_OBSTACLE
        obstacle_patch.occupancy_value = 100
        obstacle_patch.ttl_sec = ttl_sec
        for x, y in obstacle_points:
            region = Region()
            region.hint = Region.HINT_POINT
            region.points = [x, y]
            obstacle_patch.regions.append(region)
        patches.append(obstacle_patch)

    return patches


class ScanRegionPublisher(Node):
    """Publish one robot's laser scan as clear and obstacle regions."""

    def __init__(self):
        super().__init__('scan_region_publisher')

        self.robot_name = self.declare_parameter('robot_name', '').value
        self.map_frame = self.declare_parameter('map_frame', 'map').value
        self.map_name = self.declare_parameter('map_name', 'warehouse').value
        self.scan_topic = self.declare_parameter('scan_topic', 'scan').value
        self.ttl_sec = self.declare_parameter('ttl_sec', 5.0).value
        self.reset_source = self.declare_parameter(
            'reset_source', False
        ).value
        self.publish_period_sec = self.declare_parameter(
            'publish_period_sec', 0.5
        ).value
        self.max_observation_range = self.declare_parameter(
            'max_observation_range', 2.5
        ).value
        self.beam_stride = self.declare_parameter('beam_stride', 1).value

        if not self.robot_name:
            raise ValueError('robot_name must not be empty')
        if self.ttl_sec <= 0.0:
            raise ValueError('ttl_sec must be positive')
        if self.publish_period_sec < 0.0:
            raise ValueError('publish_period_sec must not be negative')
        if self.beam_stride < 1:
            raise ValueError('beam_stride must be at least one')

        self.source_id = f'{self.robot_name}/scan'
        self.last_publish_stamp_sec = None
        self.pending_scan = None
        self.publish_count = 0
        self.transform_failure_count = 0
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        reliable_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.region_update_publisher = self.create_publisher(
            MapRegionUpdate,
            '/map/region_updates',
            reliable_qos,
        )
        self.scan_subscription = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.publish_scan,
            qos_profile_sensor_data,
        )
        self.scan_retry_timer = self.create_timer(0.05, self.publish_pending_scan)

        self.get_logger().info(
            f'Converting {self.scan_topic} into region updates for '
            f'{self.robot_name} (stride={self.beam_stride}, '
            f'period={self.publish_period_sec:.2f}s)'
        )

    def publish_scan(self, scan):
        self.pending_scan = scan
        self.publish_pending_scan()

    def publish_pending_scan(self):
        scan = self.pending_scan
        if scan is None:
            return

        stamp_sec = scan.header.stamp.sec + scan.header.stamp.nanosec / 1e9
        if scan.header.stamp.sec == 0 and scan.header.stamp.nanosec == 0:
            self.get_logger().warning('Ignoring scan with a zero timestamp')
            self.pending_scan = None
            return

        if self.last_publish_stamp_sec is not None:
            elapsed = stamp_sec - self.last_publish_stamp_sec
            if 0.0 <= elapsed < self.publish_period_sec:
                self.pending_scan = None
                return

        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                scan.header.frame_id,
                Time.from_msg(scan.header.stamp),
            )
        except TransformException as error:
            self.transform_failure_count += 1
            if self.transform_failure_count == 1 or (
                self.transform_failure_count % 20 == 0
            ):
                self.get_logger().warning(
                    f'Cannot transform {scan.header.frame_id} to '
                    f'{self.map_frame} at the scan timestamp: {error}'
                )
            return

        self.pending_scan = None
        clear_polygons, obstacle_points = scan_regions(
            scan.ranges,
            scan.angle_min,
            scan.angle_increment,
            scan.range_min,
            scan.range_max,
            self.max_observation_range,
            self.beam_stride,
        )
        update = self.make_update(transform, clear_polygons, obstacle_points)
        self.region_update_publisher.publish(update)
        self.last_publish_stamp_sec = stamp_sec
        self.publish_count += 1

        if self.publish_count == 1 or self.publish_count % 20 == 0:
            self.get_logger().info(
                f'Published {len(clear_polygons)} clear regions and '
                f'{len(obstacle_points)} obstacle regions from '
                f'{len(scan.ranges)} laser beams'
            )

    def make_update(self, transform, clear_polygons, obstacle_points):
        update = MapRegionUpdate()
        update.reset_source = self.reset_source
        update.source = MapObservationSource()
        # The Rust map server currently uses a system-time clock.
        # Keep TTL and update ordering in that clock while using the scan stamp for TF.
        update.source.header.stamp = self.get_clock().now().to_msg()
        update.source.header.frame_id = self.map_frame
        update.source.source_id = self.source_id
        update.source.robot_name = self.robot_name
        update.source.map_name = self.map_name
        update.source.default_ttl_sec = self.ttl_sec

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        update.source.robot_pose.position.x = translation.x
        update.source.robot_pose.position.y = translation.y
        update.source.robot_pose.position.z = translation.z
        update.source.robot_pose.orientation = rotation

        update.patches = make_scan_patches(
            clear_polygons,
            obstacle_points,
            self.ttl_sec,
        )
        return update


def main():
    rclpy.init()
    node = ScanRegionPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
