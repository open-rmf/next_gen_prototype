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

import pytest

from rmf_layered_map_msgs.msg import MapRegionPatch, MapRegionUpdate
from rmf_layered_map_server_demo.region_update_visualizer import (
    RegionMarkerState,
    SOURCE_COLORS,
)
from rmf_prototype_msgs.msg import Region
from visualization_msgs.msg import Marker


def _update(stamp_sec=1, reset_source=True):
    update = MapRegionUpdate()
    update.source.header.stamp.sec = stamp_sec
    update.source.header.frame_id = 'map'
    update.source.source_id = 'robot0/scan'
    update.source.robot_name = 'robot0'
    update.source.map_name = 'warehouse'
    update.source.robot_pose.position.x = 2.0
    update.source.robot_pose.position.y = 3.0
    update.source.robot_pose.orientation.w = 1.0
    update.source.default_ttl_sec = 4.0
    update.reset_source = reset_source
    return update


def _region(hint, points):
    region = Region()
    region.hint = hint
    region.points = points
    return region


def test_visualizes_point_and_rectangle_regions_in_the_source_pose():
    update = _update()
    patch = MapRegionPatch()
    patch.update_type = MapRegionPatch.UPDATE_OBSTACLE
    patch.regions = [
        _region(Region.HINT_POINT, [1.0, 2.0]),
        _region(Region.HINT_AXIS_ALIGNED_RECTANGLE, [-1.0, -2.0, 1.0, 2.0]),
    ]
    update.patches = [patch]

    markers = RegionMarkerState().apply_update(update).markers

    assert len(markers) == 2
    assert markers[0].type == Marker.POINTS
    assert markers[0].pose.position.x == 2.0
    assert markers[0].pose.position.y == 3.0
    assert [(point.x, point.y) for point in markers[0].points] == [(1.0, 2.0)]
    assert markers[0].lifetime.sec == 4
    assert markers[1].type == Marker.LINE_LIST
    assert len(markers[1].points) == 8


def test_visualizes_clear_ray_sectors_as_polygon_outlines():
    update = _update()
    patch = MapRegionPatch()
    patch.update_type = MapRegionPatch.UPDATE_CLEAR
    patch.regions = [
        _region(Region.HINT_CONVEX_POLYGON, [0.0, 0.0, 1.0, -0.1, 1.0, 0.1]),
    ]
    update.patches = [patch]

    markers = RegionMarkerState().apply_update(update).markers

    assert len(markers) == 1
    assert markers[0].type == Marker.LINE_LIST
    assert markers[0].color.a == pytest.approx(0.35)
    assert len(markers[0].points) == 6


def test_visualizes_clear_regions_with_a_lighter_source_color():
    update = _update()
    clear_patch = MapRegionPatch()
    clear_patch.update_type = MapRegionPatch.UPDATE_CLEAR
    clear_patch.regions = [_region(Region.HINT_POINT, [1.0, 2.0])]
    obstacle_patch = MapRegionPatch()
    obstacle_patch.update_type = MapRegionPatch.UPDATE_OBSTACLE
    obstacle_patch.regions = [_region(Region.HINT_POINT, [1.0, 2.0])]
    update.patches = [clear_patch, obstacle_patch]

    clear_marker, obstacle_marker = RegionMarkerState().apply_update(
        update
    ).markers
    clear_color = (
        clear_marker.color.r,
        clear_marker.color.g,
        clear_marker.color.b,
    )
    obstacle_color = (
        obstacle_marker.color.r,
        obstacle_marker.color.g,
        obstacle_marker.color.b,
    )

    assert obstacle_color == pytest.approx(SOURCE_COLORS[0])
    assert clear_color == pytest.approx(
        tuple((channel + 1.0) / 2.0 for channel in SOURCE_COLORS[0])
    )
    assert clear_marker.points[0].z < obstacle_marker.points[0].z


def test_reset_deletes_the_previous_source_markers_before_replacing_them():
    state = RegionMarkerState()
    first = _update(stamp_sec=1)
    first_patch = MapRegionPatch()
    first_patch.update_type = MapRegionPatch.UPDATE_OBSTACLE
    first_patch.regions = [_region(Region.HINT_POINT, [1.0, 2.0])]
    first.patches = [first_patch]
    state.apply_update(first)

    replacement = _update(stamp_sec=2)
    replacement_patch = MapRegionPatch()
    replacement_patch.update_type = MapRegionPatch.UPDATE_OBSTACLE
    replacement_patch.regions = [_region(Region.HINT_POINT, [3.0, 4.0])]
    replacement.patches = [replacement_patch]

    markers = state.apply_update(replacement).markers

    assert [marker.action for marker in markers] == [Marker.DELETE, Marker.ADD]
    assert markers[0].ns == markers[1].ns == 'robot0/scan'
    assert markers[0].id == markers[1].id == 0
    assert [(point.x, point.y) for point in markers[1].points] == [(3.0, 4.0)]


def test_older_update_does_not_replace_newer_visualization():
    state = RegionMarkerState()
    state.apply_update(_update(stamp_sec=2))

    markers = state.apply_update(_update(stamp_sec=1)).markers

    assert markers == []


def test_non_reset_updates_retain_only_unexpired_marker_bookkeeping():
    state = RegionMarkerState()
    first = _update(stamp_sec=1, reset_source=False)
    first_patch = MapRegionPatch()
    first_patch.update_type = MapRegionPatch.UPDATE_OBSTACLE
    first_patch.regions = [_region(Region.HINT_POINT, [1.0, 2.0])]
    first.patches = [first_patch]
    state.apply_update(first)

    second = _update(stamp_sec=2, reset_source=False)
    second_patch = MapRegionPatch()
    second_patch.update_type = MapRegionPatch.UPDATE_OBSTACLE
    second_patch.regions = [_region(Region.HINT_POINT, [3.0, 4.0])]
    second.patches = [second_patch]
    markers = state.apply_update(second).markers

    assert [marker.action for marker in markers] == [Marker.ADD]
    assert markers[0].id == 1
    assert len(state.markers_by_source[('robot0/scan', 'warehouse')]) == 2

    state.apply_update(_update(stamp_sec=6, reset_source=False))

    assert state.markers_by_source[('robot0/scan', 'warehouse')] == []
