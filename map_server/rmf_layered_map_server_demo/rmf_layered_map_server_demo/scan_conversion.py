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

from math import cos, isfinite, sin


def scan_obstacle_points(
    ranges,
    angle_min,
    angle_increment,
    range_min,
    range_max,
    max_observation_range,
    beam_stride,
):
    """Convert valid laser returns into points in the scan frame."""
    if beam_stride < 1:
        raise ValueError('beam_stride must be at least one')

    effective_max = range_max
    if max_observation_range > 0.0:
        effective_max = min(effective_max, max_observation_range)

    points = []
    for index in range(0, len(ranges), beam_stride):
        distance = ranges[index]
        if not isfinite(distance):
            continue
        if distance < range_min or distance > effective_max:
            continue

        angle = angle_min + index * angle_increment
        points.append((distance * cos(angle), distance * sin(angle)))

    return points
