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

from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rmf_layered_map_msgs.msg import MapObservationSource, MapRegionUpdate
from rmf_prototype_msgs.msg import Region


class LayeredMapDemoPublisher(Node):
    def __init__(self):
        super().__init__('layered_map_demo_publisher')

        transient_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        reliable_qos = QoSProfile(
            depth=10,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        self.static_map_publisher = self.create_publisher(
            OccupancyGrid,
            '/map/static',
            transient_qos,
        )
        self.region_update_publisher = self.create_publisher(
            MapRegionUpdate,
            '/map/region_updates',
            reliable_qos,
        )
        self.timer = self.create_timer(8.0, self.publish_demo)
        self.publish_demo()

    def publish_demo(self):
        self.static_map_publisher.publish(static_map())
        self.region_update_publisher.publish(obstacle_update())
        self.get_logger().info('Published demo obstacle with a 5 second TTL')


def static_map():
    width = 10
    height = 10
    data = [0] * (width * height)

    for x in range(width):
        data[x] = 100
        data[(height - 1) * width + x] = 100

    for y in range(height):
        data[y * width] = 100
        data[y * width + width - 1] = 100

    msg = OccupancyGrid()
    msg.header.frame_id = 'map'
    msg.info.resolution = 1.0
    msg.info.width = width
    msg.info.height = height
    msg.info.origin.orientation.w = 1.0
    msg.data = data
    return msg


def obstacle_update():
    msg = MapRegionUpdate()
    msg.source = MapObservationSource()
    msg.source.source_id = 'demo/temporary_obstacle'
    msg.source.robot_name = 'demo_robot'
    msg.source.map_name = 'demo_map'
    msg.source.frame_id = 'map'
    msg.source.default_ttl.sec = 5
    msg.update_type = MapRegionUpdate.UPDATE_OBSTACLE
    msg.occupancy_value = 100
    msg.ttl.sec = 5

    region = Region()
    region.hint = Region.HINT_AXIS_ALIGNED_RECTANGLE
    region.points = [4.0, 4.0, 6.0, 6.0]
    msg.regions.append(region)
    return msg


def main():
    rclpy.init()
    node = LayeredMapDemoPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
