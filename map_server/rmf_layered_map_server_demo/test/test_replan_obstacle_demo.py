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

from math import hypot, pi
import subprocess

import pytest
from rmf_layered_map_msgs.msg import MapRegionPatch, MapRegionUpdate
from rmf_layered_map_server_demo.region_update_visualizer import (
    RegionMarkerState,
    SOURCE_COLORS,
)
from rmf_layered_map_server_demo.replan_obstacle_demo import (
    box_sdf,
    GOAL_MARKER_Z,
    make_navigation_goal,
    ROBOT_COLORS,
    ROBOT_MARKER_Z,
    spawn_succeeded,
    star_marker,
    triangle_marker,
)
from rmf_layered_map_server_demo.replan_scenarios import (
    get_scenario,
    SIMPLE_SCENARIO,
    WAREHOUSE_SCENARIO,
)
from rmf_prototype_msgs.msg import Region
from visualization_msgs.msg import Marker


def test_navigation_goal_uses_the_map_frame():
    goal = make_navigation_goal(*SIMPLE_SCENARIO.robots[1].goal)

    assert goal.pose.header.frame_id == 'map'
    assert goal.pose.pose.position.x == 6.0
    assert goal.pose.pose.position.y == 4.0
    assert goal.pose.pose.orientation.z == pytest.approx(
        goal.pose.pose.orientation.w
    )


def test_box_sdf():
    sdf = box_sdf(
        'bar',
        *SIMPLE_SCENARIO.bar_center,
        *SIMPLE_SCENARIO.bar_size,
        color=(0.95, 0.05, 0.05, 1.0),
    )

    assert "<model name='bar'>" in sdf
    assert '<size>15.5 0.5 1.0</size>' in sdf
    assert '<ambient>0.95 0.05 0.05 1.0</ambient>' in sdf


@pytest.mark.parametrize(
    ('returncode', 'stdout', 'expected'),
    (
        (0, 'data: true', True),
        (0, 'data: false', False),
        (1, 'data: true', False),
    ),
)
def test_spawn_succeeded_requires_a_positive_reply(
    returncode,
    stdout,
    expected,
):
    result = subprocess.CompletedProcess([], returncode, stdout, '')

    assert spawn_succeeded(result) is expected


def _region_update(robot_name):
    update = MapRegionUpdate()
    update.source.header.stamp.sec = 1
    update.source.header.frame_id = 'map'
    update.source.source_id = f'{robot_name}/scan'
    update.source.robot_name = robot_name
    update.source.map_name = 'single_room'
    update.source.robot_pose.orientation.w = 1.0
    patch = MapRegionPatch()
    patch.update_type = MapRegionPatch.UPDATE_OBSTACLE
    region = Region()
    region.hint = Region.HINT_POINT
    region.points = [1.0, 2.0]
    patch.regions = [region]
    update.patches = [patch]
    return update


def test_region_colors_match_robots_regardless_of_arrival_order():
    state = RegionMarkerState()

    robot1_marker = state.apply_update(_region_update('robot1')).markers[0]
    robot0_marker = state.apply_update(_region_update('robot0')).markers[0]
    other_marker = state.apply_update(_region_update('other_robot')).markers[0]

    for marker, color in (
        (robot0_marker, ROBOT_COLORS['robot0'][:3]),
        (robot1_marker, ROBOT_COLORS['robot1'][:3]),
        (other_marker, SOURCE_COLORS[2]),
    ):
        assert (
            marker.color.r,
            marker.color.g,
            marker.color.b,
        ) == pytest.approx(color)


def test_scenario_markers():
    triangle = triangle_marker(
        1,
        'robot',
        5.5,
        -4.5,
        pi / 2.0,
        (1.0, 0.4, 0.0, 0.8),
    )
    star = star_marker(
        2,
        *SIMPLE_SCENARIO.robots[1].goal,
        color=ROBOT_COLORS['robot1'],
    )

    assert triangle.type == Marker.TRIANGLE_LIST
    assert len(triangle.points) == 3
    assert triangle.points[0].y > -4.5
    assert all(point.z == ROBOT_MARKER_Z for point in triangle.points)
    assert triangle.scale.x == 1.0
    assert triangle.scale.y == 1.0
    assert triangle.scale.z == 1.0
    assert star.type == Marker.TRIANGLE_LIST
    assert len(star.points) == 30
    assert all(point.z == GOAL_MARKER_Z for point in star.points)
    assert (
        star.color.r,
        star.color.g,
        star.color.b,
        star.color.a,
    ) == ROBOT_COLORS['robot1']
    assert star.scale.x == 1.0


def test_simple_room_dead_end_layout():
    robot0, robot1 = SIMPLE_SCENARIO.robots
    center_distance = hypot(
        robot1.pose[0] - robot0.pose[0],
        robot1.pose[1] - robot0.pose[1],
    )
    assert center_distance > 2.0 * SIMPLE_SCENARIO.scan_radius

    bar_left = (
        SIMPLE_SCENARIO.bar_center[0] - SIMPLE_SCENARIO.bar_size[0] / 2.0
    )
    bar_right = (
        SIMPLE_SCENARIO.bar_center[0] + SIMPLE_SCENARIO.bar_size[0] / 2.0
    )
    assert bar_left == -6.0
    assert bar_right == 9.5

    for robot in SIMPLE_SCENARIO.robots:
        closest_x = min(max(robot.pose[0], bar_left), bar_right)
        distance = hypot(
            closest_x - robot.pose[0],
            SIMPLE_SCENARIO.bar_center[1] - robot.pose[1],
        )
        assert distance < SIMPLE_SCENARIO.scan_radius

    goal0, goal1 = (robot.goal for robot in SIMPLE_SCENARIO.robots)
    goal_distance = hypot(goal1[0] - goal0[0], goal1[1] - goal0[1])
    assert goal0 == (0.0, 6.0)
    assert goal1 == (6.0, 4.0)
    assert goal_distance > 6.0
    assert goal0[1] != goal1[1]


def test_warehouse_scenario_layout():
    assert get_scenario('warehouse') is WAREHOUSE_SCENARIO
    assert WAREHOUSE_SCENARIO.robots[0].pose[:2] == (-12.0, -21.0)
    assert WAREHOUSE_SCENARIO.robots[1].pose[:2] == (3.5, -21.0)
    assert WAREHOUSE_SCENARIO.bar_center == (-2.5, -18.5)
    assert WAREHOUSE_SCENARIO.bar_size == (15.0, 0.5, 1.0)
    assert WAREHOUSE_SCENARIO.ttl_sec == 210.0
