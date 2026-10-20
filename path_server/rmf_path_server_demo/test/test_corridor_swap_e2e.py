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
    """Launch demo_corridor_nav2 headless with scenario:=none for multi-scenario testing."""
    demo_share = get_package_share_directory('rmf_path_server_demo')
    corridor_nav2_launch = os.path.join(
        demo_share, 'launch', 'demo_corridor_nav2.launch.py'
    )

    corridor_nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(corridor_nav2_launch),
        launch_arguments={
            'use_rviz': 'False',
            'scenario': 'none',
        }.items(),
    )

    return LaunchDescription([
        corridor_nav2,
        launch_testing.actions.ReadyToTest(),
    ]), {
        'corridor_nav2': corridor_nav2,
    }


class TestCorridorSwapNav2(CorridorNav2E2ETestBase):
    """End-to-end Nav2 regression tests for docking, swapping, post-swap, and mid-swap goals."""

    def test_01_corridor_swap_and_new_goals_after_swap(self):
        """
        Verify a full corridor swap followed immediately by new post-swap goals.

        1. Swap robot0 from (-2, 0) -> (5, 0) and robot1 from (5, 0) -> (-2, 0).
        2. Once that swap finishes, immediately send new goals after the swap:
           robot0 -> (-1, 1) and robot1 -> (4, 0).
        """
        self._wait_for_nav2_ready()
        for r in ('robot0', 'robot1'):
            self.plan_errors[r].clear()

        # Leg A: Initial corridor swap
        self._publish_goal('robot0', 5.0, 0.0, 0.0)
        self._publish_goal('robot1', -2.0, 0.0, math.pi)
        self._wait_for_targets(
            {'robot0': (5.0, 0.0), 'robot1': (-2.0, 0.0)},
            timeout_sec=80.0,
        )

        for robot in ('robot0', 'robot1'):
            self.assertGreater(len(self.safe_zones[robot]), 0)
            self.assertGreater(len(self.costmap_seqs[robot]), 0)
            self.assertGreater(len(self.acked_seqs[robot]), 0)

        # Leg B: Send new unoccupied corridor goals after the swap completes
        self._publish_goal('robot0', -1.0, 1.0, math.pi)
        self._publish_goal('robot1', 4.0, 0.0, 0.0)
        self._wait_for_targets(
            {'robot0': (-1.0, 1.0), 'robot1': (4.0, 0.0)},
            timeout_sec=80.0,
        )

        self._assert_no_blocked_errors()

    def test_02_mid_swap_goal_injection(self):
        """
        Inject a new goal location for robot0 midway through a two-robot swap.

        1. Start with robot0 at (-1, 1) and robot1 at (4, 0); dispatch swap goals
           robot0 -> (5, 0) and robot1 -> (-2, 0).
        2. Once robot0 crosses x >= 0.5 midway through the swap, inject a new
           DestinationGoal redirecting robot0 to (3, -1) on the south corridor
           lane while robot1 continues to (-2, 0).
        """
        for r in ('robot0', 'robot1'):
            self.plan_errors[r].clear()

        self._publish_goal('robot0', 5.0, 0.0, 0.0)
        self._publish_goal('robot1', -2.0, 0.0, math.pi)

        retargeted = False

        def inject_mid_swap_goal():
            nonlocal retargeted
            if not retargeted and 'robot0' in self.statuses:
                x0, _ = self._robot_pos('robot0')
                if x0 >= 0.5:
                    retargeted = True
                    self._publish_goal('robot0', 3.0, -1.0, 0.0)

        self._wait_for_targets(
            {'robot0': (3.0, -1.0), 'robot1': (-2.0, 0.0)},
            timeout_sec=85.0,
            step_hook=inject_mid_swap_goal,
        )
        self.assertTrue(
            retargeted,
            'Mid-swap goal injection never triggered (robot0 did not cross x >= 0.5)',
        )
        self._assert_no_blocked_errors()

    def test_03_docking_and_passing_scenario(self):
        """
        Replicate the 'Dock + Pass' scenario in Nav2 with up-front dispatch.

        1. Stage the robots to the starting configuration:
           robot0 at (-1.0, 1.0, pi/2) (dock_north_1 staging) and
           robot1 right next to it at (-2.0, 1.0, 0.0).
        2. Dispatch both goals up-front: robot0 to dock into dock_north_1 at
           (-1.0, 2.0, pi/2) and robot1 to pass to (1.0, 1.0, 0.0), letting
           traffic management deconflict the robots and verify both reach
           their targets.
        """
        for r in ('robot0', 'robot1'):
            self.plan_errors[r].clear()

        # Stage both robots to the online web demo start poses
        self._publish_goal('robot0', -1.0, 1.0, math.pi / 2.0)
        self._publish_goal('robot1', -2.0, 1.0, 0.0)
        self._wait_for_targets(
            {'robot0': (-1.0, 1.0), 'robot1': (-2.0, 1.0)},
            timeout_sec=80.0,
        )

        # Dispatch both robots up-front: robot0 docks into (-1, 2) while
        # robot1 passes to (1, 1).
        self._publish_goal('robot0', -1.0, 2.0, math.pi / 2.0)
        self._publish_goal('robot1', 1.0, 1.0, 0.0)

        self._wait_for_targets(
            {'robot0': (-1.0, 2.0, math.pi / 2.0), 'robot1': (1.0, 1.0, 0.0)},
            timeout_sec=80.0,
        )

        self._assert_no_blocked_errors()

    def test_05_docking_and_passing(self):
        """
        Verify docking and passing with up-front dispatch when starting from another cell.

        1. Position robot0 at another cell (0.0, 1.0, 0.0) facing East (wrong
           orientation for docking North into dock_north_1) and robot1 at
           (-2.0, 1.0, 0.0).
        2. Dispatch both robots up-front: send robot0 to the dock staging cell
           (-1.0, 1.0, pi/2) (requiring it to translate West and rotate 90 deg
           in place to yaw = pi/2) and robot1 to (1.0, 1.0, 0.0), letting
           traffic management deconflict and route robot1 while ProgressTracker
           holds robot0's final progress until yaw = pi/2 is achieved.
        3. Once robot0 achieves (-1.0, 1.0, pi/2), dispatch robot0 into the dock
           at (-1.0, 2.0, pi/2) and verify both robots reach their targets.
        """
        for r in ('robot0', 'robot1'):
            self.plan_errors[r].clear()

        # First undock robot0 straight South out of dock_north_1 to (-1.0, 1.0)
        # before staging at another cell.
        self._publish_goal('robot0', -1.0, 1.0, -math.pi / 2.0)
        self._wait_for_targets(
            {'robot0': (-1.0, 1.0, -math.pi / 2.0)},
            timeout_sec=60.0,
        )

        # 1. Stage robot0 at another cell (0.0, 1.0) in the wrong orientation (0.0)
        #    and robot1 at (-2.0, 1.0, 0.0).
        self._publish_goal('robot0', 0.0, 1.0, 0.0)
        self._publish_goal('robot1', -2.0, 1.0, 0.0)
        self._wait_for_targets(
            {'robot0': (0.0, 1.0, 0.0), 'robot1': (-2.0, 1.0, 0.0)},
            timeout_sec=80.0,
        )

        # 2. Dispatch both robots up-front: robot0 to dock staging (-1.0, 1.0, pi/2)
        #    and robot1 to (1.0, 1.0, 0.0).
        stage_seed = self._publish_goal('robot0', -1.0, 1.0, math.pi / 2.0)
        self._publish_goal('robot1', 1.0, 1.0, 0.0)
        pos_reached_before_final_progress = False
        yaw_err_at_final_progress = None
        dispatched_dock = False

        def check_staging_orientation_gate():
            nonlocal pos_reached_before_final_progress
            nonlocal yaw_err_at_final_progress, dispatched_dock
            if 'robot0' not in self.statuses:
                return
            status0 = self.statuses['robot0']
            x0, y0 = self._robot_pos('robot0')
            yaw_err = self._angular_diff(
                self._robot_yaw('robot0'), math.pi / 2.0
            )
            if status0.progress and self.plans['robot0']:
                prog0 = status0.progress[0]
                if prog0.plan_id.destination_session.uuid[0] == (
                    stage_seed & 0xFF
                ):
                    matching_plans = [
                        p
                        for p in self.plans['robot0']
                        if p.plan_id.destination_session.uuid[0]
                        == (stage_seed & 0xFF)
                    ]
                    if matching_plans:
                        final_wp = len(matching_plans[-1].waypoints) - 1
                        if final_wp > 0:
                            if (
                                math.hypot(x0 - (-1.0), y0 - 1.0) <= 0.35
                                and yaw_err > 0.35
                                and prog0.reached_waypoint < final_wp
                            ):
                                pos_reached_before_final_progress = True
                            if (
                                prog0.reached_waypoint >= final_wp
                                and yaw_err_at_final_progress is None
                            ):
                                yaw_err_at_final_progress = yaw_err
            if (
                not dispatched_dock
                and math.hypot(x0 - (-1.0), y0 - 1.0) <= 0.35
                and yaw_err <= 0.30
            ):
                dispatched_dock = True
                self._publish_goal('robot0', -1.0, 2.0, math.pi / 2.0)

        self._wait_for_targets(
            {'robot0': (-1.0, 2.0, math.pi / 2.0), 'robot1': (1.0, 1.0, 0.0)},
            timeout_sec=90.0,
            step_hook=check_staging_orientation_gate,
        )
        self.assertTrue(
            dispatched_dock,
            'robot0 never completed staging rotation at (-1.0, 1.0, pi/2) to dispatch dock goal',
        )
        self.assertTrue(
            pos_reached_before_final_progress,
            'Expected robot0 to hold final progress at (-1.0, 1.0) while '
            'rotating from wrong orientation toward pi/2',
        )
        if yaw_err_at_final_progress is not None:
            self.assertLessEqual(
                yaw_err_at_final_progress,
                0.30,
                f'robot0 reported final waypoint reached before achieving target orientation pi/2 '
                f'(yaw_err={yaw_err_at_final_progress:.3f} rad)',
            )

        self._assert_no_blocked_errors()


@launch_testing.post_shutdown_test()
class TestCorridorSwapShutdown(unittest.TestCase):
    """Clean up any residual simulator processes after launch shutdown."""

    def test_cleanup_gazebo(self):
        """Ensure gz sim and isolated Nav2 containers are terminated after the test."""
        cleanup_gazebo_processes()
