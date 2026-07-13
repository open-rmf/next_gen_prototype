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

from math import inf, nan, pi

import pytest

from rmf_layered_map_server_demo.scan_conversion import scan_obstacle_points


def test_scan_filters_invalid_and_out_of_range_returns():
    points = scan_obstacle_points(
        [nan, inf, 0.05, 1.0, 3.0],
        angle_min=0.0,
        angle_increment=pi / 2.0,
        range_min=0.1,
        range_max=10.0,
        max_observation_range=2.5,
        beam_stride=1,
    )

    assert len(points) == 1
    assert points[0][0] == pytest.approx(0.0, abs=1e-6)
    assert points[0][1] == pytest.approx(-1.0)


def test_scan_stride_preserves_original_beam_angles():
    points = scan_obstacle_points(
        [1.0, 1.0, 1.0, 1.0],
        angle_min=0.0,
        angle_increment=pi / 2.0,
        range_min=0.1,
        range_max=5.0,
        max_observation_range=0.0,
        beam_stride=2,
    )

    assert len(points) == 2
    assert points[0] == pytest.approx((1.0, 0.0), abs=1e-6)
    assert points[1] == pytest.approx((-1.0, 0.0), abs=1e-6)


def test_scan_rejects_invalid_stride():
    with pytest.raises(ValueError, match='beam_stride'):
        scan_obstacle_points(
            [],
            angle_min=0.0,
            angle_increment=0.1,
            range_min=0.1,
            range_max=5.0,
            max_observation_range=2.5,
            beam_stride=0,
        )
