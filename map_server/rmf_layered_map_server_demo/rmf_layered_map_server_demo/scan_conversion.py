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


def scan_regions(
    ranges,
    angle_min,
    angle_increment,
    range_min,
    range_max,
    max_observation_range,
    beam_stride,
):
    """Convert laser beams into clear sectors and occupied endpoints."""
    if beam_stride < 1:
        raise ValueError('beam_stride must be at least one')
    if not isfinite(angle_min) or not isfinite(angle_increment):
        raise ValueError('scan angles must be finite')
    if angle_increment == 0.0:
        raise ValueError('angle_increment must not be zero')

    effective_max = range_max
    if max_observation_range > 0.0:
        effective_max = min(effective_max, max_observation_range)

    clear_polygons = []
    obstacle_points = []
    half_width = abs(angle_increment) * beam_stride / 2.0
    for index in range(0, len(ranges), beam_stride):
        distance = ranges[index]
        has_obstacle = False
        if isfinite(distance):
            if distance < range_min or distance > range_max:
                continue
            clear_distance = min(distance, effective_max)
            has_obstacle = distance <= effective_max
        elif distance > 0.0 and isfinite(effective_max):
            clear_distance = effective_max
        else:
            continue
        if not isfinite(clear_distance) or clear_distance <= 0.0:
            continue

        angle = angle_min + index * angle_increment
        clear_polygons.append((
            0.0,
            0.0,
            clear_distance * cos(angle - half_width),
            clear_distance * sin(angle - half_width),
            clear_distance * cos(angle + half_width),
            clear_distance * sin(angle + half_width),
        ))
        if has_obstacle:
            obstacle_points.append((
                distance * cos(angle),
                distance * sin(angle),
            ))

    return clear_polygons, obstacle_points
