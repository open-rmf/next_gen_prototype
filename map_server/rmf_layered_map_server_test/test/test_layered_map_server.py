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

import time
import unittest

from launch import LaunchDescription
import launch_ros
import launch_testing
from nav_msgs.msg import OccupancyGrid
import pytest
import rclpy
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rmf_layered_map_msgs.msg import (
    MapObservationSource,
    MapRegionPatch,
    MapRegionUpdate,
    MapSourceSnapshot,
)
from rmf_prototype_msgs.msg import Region


@pytest.mark.launch_test
def generate_test_description():
    map_server = launch_ros.actions.Node(
        package='rmf_layered_map_server',
        executable='rmf_layered_map_server',
        output='screen',
    )

    return LaunchDescription([
        map_server,
        launch_testing.actions.ReadyToTest(),
    ]), {
        'map_server': map_server,
    }


class TestLayeredMapServer(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_layered_map_server_node')
        self.maps = []
        self.source_snapshots = []

        transient_qos = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        reliable_qos = QoSProfile(
            depth=10,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        self.static_map_publisher = self.node.create_publisher(
            OccupancyGrid,
            '/map/static',
            qos_profile=transient_qos,
        )
        self.region_update_publisher = self.node.create_publisher(
            MapRegionUpdate,
            '/map/region_updates',
            qos_profile=reliable_qos,
        )
        self.map_subscription = self.node.create_subscription(
            OccupancyGrid,
            '/map',
            self.maps.append,
            qos_profile=transient_qos,
        )
        self.source_subscription = self.node.create_subscription(
            MapSourceSnapshot,
            '/map/source_contributions',
            self.source_snapshots.append,
            qos_profile=transient_qos,
        )

    def tearDown(self):
        self.node.destroy_node()

    def wait_for(self, predicate, timeout=5.0):
        deadline = time.time() + timeout
        while time.time() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if predicate():
                return True
        return False

    def publish_until(self, publisher, msg, predicate, timeout=12.0):
        deadline = time.time() + timeout
        while time.time() < deadline:
            publisher.publish(msg)
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if predicate():
                return True
        return False

    def last_cell(self, x, y):
        if not self.maps:
            return None
        latest = self.maps[-1]
        return latest.data[y * latest.info.width + x]

    def last_source_cell(self, source_id, cell_index):
        if not self.source_snapshots:
            return None
        for source in self.source_snapshots[-1].sources:
            if source.source.source_id != source_id:
                continue
            values = dict(zip(source.cell_indices, source.occupancy_values))
            return values.get(cell_index)
        return None

    def test_region_update_is_composed_reset_and_expires(self):
        time.sleep(2.0)

        static = static_map()
        self.assertTrue(
            self.publish_until(
                self.static_map_publisher,
                static,
                lambda: len(self.maps) > 0,
            ),
            'layered map server did not publish a composed map',
        )
        self.assertEqual(self.maps[-1].info.width, 5)
        self.assertEqual(self.maps[-1].info.height, 5)
        self.assertEqual(self.last_cell(2, 2), 0)

        update = obstacle_update(ttl_sec=30)
        update.source.header.stamp = self.node.get_clock().now().to_msg()
        self.assertTrue(
            self.publish_until(
                self.region_update_publisher,
                update,
                lambda: (
                    self.last_cell(2, 2) == 100
                    and self.last_source_cell('test/obstacle', 12) == 100
                ),
            ),
            'layered map server did not publish the obstacle contribution',
        )

        reset = reset_update()
        reset.source.header.stamp = self.node.get_clock().now().to_msg()
        self.assertTrue(
            self.publish_until(
                self.region_update_publisher,
                reset,
                lambda: (
                    self.last_cell(2, 2) == 0
                    and self.last_source_cell('test/obstacle', 12) is None
                ),
            ),
            'layered map server did not reset the obstacle source',
        )

        update = obstacle_update()
        update.source.header.stamp = self.node.get_clock().now().to_msg()
        self.assertTrue(
            self.publish_until(
                self.region_update_publisher,
                update,
                lambda: self.last_cell(2, 2) == 100,
            ),
            'layered map server did not compose the obstacle region',
        )
        self.assertTrue(
            self.wait_for(
                lambda: (
                    self.last_cell(2, 2) == 0
                    and self.last_source_cell('test/obstacle', 12) is None
                ),
                timeout=4.0,
            ),
            'layered map server did not prune the expired obstacle source',
        )


def static_map():
    msg = OccupancyGrid()
    msg.header.frame_id = 'map'
    msg.info.resolution = 1.0
    msg.info.width = 5
    msg.info.height = 5
    msg.info.origin.orientation.w = 1.0
    msg.data = [0] * 25
    return msg


def map_source():
    msg = MapObservationSource()
    msg.header.frame_id = 'map'
    msg.robot_pose.orientation.w = 1.0
    msg.source_id = 'test/obstacle'
    msg.robot_name = 'test_robot'
    msg.map_name = 'test_map'
    msg.default_ttl_sec = 1.0
    return msg


def obstacle_update(ttl_sec=1):
    msg = MapRegionUpdate()
    msg.source = map_source()
    msg.source.default_ttl_sec = float(ttl_sec)

    patch = MapRegionPatch()
    patch.update_type = MapRegionPatch.UPDATE_OBSTACLE
    patch.occupancy_value = 100
    patch.ttl_sec = float(ttl_sec)

    region = Region()
    region.hint = Region.HINT_AXIS_ALIGNED_RECTANGLE
    region.points = [2.0, 2.0, 3.0, 3.0]
    patch.regions.append(region)
    msg.patches.append(patch)
    return msg


def reset_update():
    msg = MapRegionUpdate()
    msg.source = map_source()
    msg.reset_source = True
    return msg
