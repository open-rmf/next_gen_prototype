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

from geometry_msgs.msg import Transform
from rmf_layered_map_msgs.msg import MapRegionPatch
from rmf_layered_map_server_demo.scan_region_publisher import (
    make_scan_patches,
    ObstacleMemory,
)
from rmf_prototype_msgs.msg import Region


def _transform(x=0.0, y=0.0):
    transform = Transform()
    transform.translation.x = x
    transform.translation.y = y
    transform.rotation.w = 1.0
    return transform


def test_scan_patches_clear_rays_before_marking_obstacle_endpoints():
    patches = make_scan_patches(
        [(0.0, 0.0, 1.0, -0.1, 1.0, 0.1)],
        [(1.0, 0.0)],
        ttl_sec=5.0,
    )

    assert [patch.update_type for patch in patches] == [
        MapRegionPatch.UPDATE_CLEAR,
        MapRegionPatch.UPDATE_OBSTACLE,
    ]
    assert patches[0].occupancy_value == 0
    assert patches[0].regions[0].hint == Region.HINT_CONVEX_POLYGON
    assert patches[1].occupancy_value == 100
    assert patches[1].regions[0].hint == Region.HINT_POINT


def test_obstacle_memory_filters_points_inside_robot_body():
    memory = ObstacleMemory(
        retention_sec=0.0,
        self_filter_radius=0.22,
    )

    points = memory.remember(
        [(0.1, 0.0), (0.3, 0.0)],
        _transform(),
        now_sec=1.0,
    )

    assert points == [(0.3, 0.0)]


def test_obstacle_memory_prunes_points_the_robot_moves_over():
    memory = ObstacleMemory(
        retention_sec=-1.0,
        self_filter_radius=0.22,
    )
    assert memory.remember(
        [(1.0, 0.0)],
        _transform(),
        now_sec=1.0,
    ) == [(1.0, 0.0)]

    assert memory.remember(
        [],
        _transform(x=1.0),
        now_sec=2.0,
    ) == []


def test_obstacle_memory_keeps_points_when_the_scan_window_moves():
    memory = ObstacleMemory(retention_sec=-1.0, resolution=0.1)
    first = memory.remember(
        [(1.0, 0.0), (2.0, 0.0)],
        _transform(),
        now_sec=1.0,
    )
    moved = memory.remember(
        [(1.0, 0.0)],
        _transform(x=5.0),
        now_sec=2.0,
    )

    assert sorted(first) == [(1.0, 0.0), (2.0, 0.0)]
    assert sorted(moved) == [(-4.0, 0.0), (-3.0, 0.0), (1.0, 0.0)]


def test_finite_obstacle_memory_expires_old_scan_windows():
    memory = ObstacleMemory(retention_sec=5.0, resolution=0.1)
    memory.remember([(1.0, 0.0)], _transform(), now_sec=1.0)

    remembered = memory.remember(
        [(2.0, 0.0)],
        _transform(),
        now_sec=7.0,
    )

    assert remembered == [(2.0, 0.0)]
