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

from math import pi

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
import pytest

from rmf_layered_map_server_demo.nav2_goal_publisher import advance_waypoint
from rmf_layered_map_server_demo.nav2_goal_publisher import goal_pose
from rmf_layered_map_server_demo.nav2_goal_publisher import parse_waypoints


def test_parse_waypoints_groups_xyz_triples():
    assert parse_waypoints([1.0, 2.0, 3.0, 4.0, 5.0, 6.0]) == [
        (1.0, 2.0, 3.0),
        (4.0, 5.0, 6.0),
    ]


@pytest.mark.parametrize('values', [[], [1.0], [1.0, 2.0]])
def test_parse_waypoints_rejects_incomplete_triples(values):
    with pytest.raises(ValueError, match='x, y, yaw'):
        parse_waypoints(values)


def test_goal_pose_uses_map_frame_and_yaw_orientation():
    stamp = PoseStamped().header.stamp
    stamp.sec = 1
    stamp.nanosec = 2

    pose = goal_pose(3.0, 4.0, pi, stamp)

    assert pose.header.frame_id == 'map'
    assert pose.header.stamp == stamp
    assert pose.pose.position.x == 3.0
    assert pose.pose.position.y == 4.0
    assert pose.pose.orientation.z == pytest.approx(1.0)
    assert pose.pose.orientation.w == pytest.approx(0.0, abs=1e-10)


def test_waypoint_advances_only_after_success():
    assert advance_waypoint(0, 2, GoalStatus.STATUS_SUCCEEDED) == 1
    assert advance_waypoint(1, 2, GoalStatus.STATUS_SUCCEEDED) == 0
    assert advance_waypoint(0, 2, GoalStatus.STATUS_ABORTED) == 0
    assert advance_waypoint(1, 2, GoalStatus.STATUS_CANCELED) == 1
