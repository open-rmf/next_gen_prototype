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

import math
import os
import unittest

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing
import launch_testing.actions
import pytest
from rmf_path_server_demo.corridor_e2e_base import (
    cleanup_gazebo_processes,
    CorridorNav2E2ETestBase,
)


@pytest.mark.launch_test
def generate_test_description():
    """Launch demo_corridor_nav2 headless with both robots spawned on the same lane."""
    demo_share = get_package_share_directory('rmf_path_server_demo')
    corridor_nav2_launch = os.path.join(
        demo_share, 'launch', 'demo_corridor_nav2.launch.py'
    )

    corridor_nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(corridor_nav2_launch),
        launch_arguments={
            'use_rviz': 'False',
            'scenario': 'none',
            'robots': (
                'robot0={x: -2.0, y: 0.0, yaw: 0.0}; '
                'robot1={x: 0.0, y: 0.0, yaw: 0.0}'
            ),
        }.items(),
    )

    return LaunchDescription([
        corridor_nav2,
        launch_testing.actions.ReadyToTest(),
    ]), {
        'corridor_nav2': corridor_nav2,
    }


class TestCorridorFollowE2E(CorridorNav2E2ETestBase):
    """Isolated end-to-end Nav2 test for same-direction robot following in a corridor."""

    def test_04_same_direction_following_in_corridor(self):
        """
        Verify same-direction following along the center corridor lane in Nav2.

        1. Spawn robot0 at (-2.0, 0.0, 0.0) (trailing) and robot1 at
           (0.0, 0.0, 0.0) (leading).
        2. Dispatch same-direction goals along y = 0.0:
           robot0 -> (4.0, 0.0, 0.0) and robot1 -> (5.0, 0.0, 0.0).
        3. Verify robot0 receives departure_blockers on robot1, stays safely
           behind robot1 throughout execution, and both reach their targets
           without any CODE_PATH_BLOCKED errors.
        """
        self._wait_for_nav2_ready()
        for r in ('robot0', 'robot1'):
            self.plan_errors[r].clear()

        self._publish_goal('robot0', 4.0, 0.0, 0.0)
        self._publish_goal('robot1', 5.0, 0.0, 0.0)

        min_dist = float('inf')
        max_x0_minus_x1 = float('-inf')

        def check_following_separation():
            nonlocal min_dist, max_x0_minus_x1
            if 'robot0' in self.statuses and 'robot1' in self.statuses:
                x0, y0 = self._robot_pos('robot0')
                x1, y1 = self._robot_pos('robot1')
                dist = math.hypot(x0 - x1, y0 - y1)
                if dist < min_dist:
                    min_dist = dist
                if x0 - x1 > max_x0_minus_x1:
                    max_x0_minus_x1 = x0 - x1

        self._wait_for_targets(
            {'robot0': (4.0, 0.0, 0.0), 'robot1': (5.0, 0.0, 0.0)},
            timeout_sec=80.0,
            step_hook=check_following_separation,
        )

        has_r1_blockers = any(
            any(b.name == 'robot1' for b in wp.departure_blockers)
            for plan in self.plans['robot0']
            for wp in plan.waypoints
        )
        self.assertTrue(
            has_r1_blockers,
            'Expected trailing robot0 plan to contain departure_blockers on leading robot1',
        )
        self.assertGreaterEqual(
            min_dist,
            0.70,
            f'Trailing robot0 got too close to leading robot1 (min_dist={min_dist:.3f}m)',
        )
        self.assertLessEqual(
            max_x0_minus_x1,
            -0.50,
            f'Trailing robot0 overtook or encroached on leading robot1 '
            f'(max x0 - x1 = {max_x0_minus_x1:.3f}m)',
        )

        self._assert_no_blocked_errors()


@launch_testing.post_shutdown_test()
class TestCorridorFollowShutdown(unittest.TestCase):
    """Clean up any residual simulator processes after launch shutdown."""

    def test_cleanup_gazebo(self):
        """Ensure gz sim and isolated Nav2 containers are terminated after the test."""
        cleanup_gazebo_processes()
