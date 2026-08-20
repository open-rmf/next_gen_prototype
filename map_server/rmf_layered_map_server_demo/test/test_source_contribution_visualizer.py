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

from math import sqrt

import pytest
from rmf_layered_map_msgs.msg import MapSourceContribution, MapSourceSnapshot
from rmf_layered_map_server_demo.replan_scenarios import ROBOT_COLORS
from rmf_layered_map_server_demo.source_contribution_visualizer import (
    _cell_point,
    SourceContributionMarkerState,
)
from visualization_msgs.msg import Marker


def _snapshot():
    snapshot = MapSourceSnapshot()
    snapshot.header.frame_id = 'map'
    snapshot.info.width = 3
    snapshot.info.height = 2
    snapshot.info.resolution = 1.0
    snapshot.info.origin.orientation.w = 1.0

    contribution = MapSourceContribution()
    contribution.source.source_id = 'robot0/scan'
    contribution.source.robot_name = 'robot0'
    contribution.source.map_name = 'warehouse'
    contribution.cell_indices = [1, 5]
    contribution.occupancy_values = [100, 0]
    snapshot.sources = [contribution]
    return snapshot


def test_visualizes_backend_obstacles_in_the_observing_robot_color():
    markers = SourceContributionMarkerState().apply_snapshot(_snapshot()).markers

    assert markers[0].action == Marker.DELETEALL
    marker = markers[1]
    assert marker.action == Marker.ADD
    assert marker.type == Marker.POINTS
    assert marker.ns == 'robot0/scan/backend_contributions'
    assert [(point.x, point.y) for point in marker.points] == [(1.5, 0.5)]
    assert (
        marker.color.r,
        marker.color.g,
        marker.color.b,
    ) == pytest.approx(ROBOT_COLORS['robot0'][:3])


def test_empty_snapshot_clears_markers():
    empty_snapshot = _snapshot()
    empty_snapshot.sources = []

    markers = SourceContributionMarkerState().apply_snapshot(
        empty_snapshot
    ).markers

    assert len(markers) == 1
    assert markers[0].action == Marker.DELETEALL


def test_cell_centers_respect_rotated_map_origins():
    snapshot = _snapshot()
    snapshot.info.origin.position.x = 2.0
    snapshot.info.origin.position.y = 3.0
    snapshot.info.origin.orientation.z = sqrt(0.5)
    snapshot.info.origin.orientation.w = sqrt(0.5)

    point = _cell_point(snapshot.info, 0)

    assert point.x == pytest.approx(1.5)
    assert point.y == pytest.approx(3.5)
