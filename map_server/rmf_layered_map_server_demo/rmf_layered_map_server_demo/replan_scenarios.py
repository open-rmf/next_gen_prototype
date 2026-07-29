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

from dataclasses import dataclass
from math import pi


ROBOT_COLORS = {
    'robot0': (0.12, 0.55, 0.85, 1.0),
    'robot1': (1.0, 0.42, 0.08, 1.0),
}


@dataclass(frozen=True)
class RobotLayout:
    """Initial pose and destination for one demo robot."""

    name: str
    pose: tuple[float, float, float]
    goal: tuple[float, float]


@dataclass(frozen=True)
class ReplanScenario:
    """Geometry and resource names for a replanning scenario."""

    name: str
    map_name: str
    world_name: str
    robots: tuple[RobotLayout, ...]
    scan_radius: float
    ttl_sec: float
    bar_name: str
    bar_center: tuple[float, float]
    bar_size: tuple[float, float, float]


SIMPLE_SCENARIO = ReplanScenario(
    name='simple',
    map_name='single_room',
    world_name='single_room',
    robots=(
        RobotLayout('robot0', (-3.0, -4.5, pi / 2.0), (0.0, 6.0)),
        RobotLayout('robot1', (5.5, -4.5, pi / 2.0), (6.0, 4.0)),
    ),
    scan_radius=4.0,
    ttl_sec=60.0,
    bar_name='simple_replan_bar',
    bar_center=(1.75, -1.5),
    bar_size=(15.5, 0.5, 1.0),
)

WAREHOUSE_SCENARIO = ReplanScenario(
    name='warehouse',
    map_name='warehouse',
    world_name='warehouse',
    robots=(
        RobotLayout('robot0', (-12.0, -21.0, pi / 2.0), (2.5, -15.0)),
        RobotLayout('robot1', (3.5, -21.0, pi / 2.0), (3.5, -15.0)),
    ),
    scan_radius=3.5,
    ttl_sec=210.0,
    bar_name='warehouse_replan_bar',
    bar_center=(-2.5, -18.5),
    bar_size=(15.0, 0.5, 1.0),
)


SCENARIOS = {
    SIMPLE_SCENARIO.name: SIMPLE_SCENARIO,
    WAREHOUSE_SCENARIO.name: WAREHOUSE_SCENARIO,
}


def get_scenario(name):
    """Return a scenario by name."""
    try:
        return SCENARIOS[name]
    except KeyError as error:
        choices = ', '.join(sorted(SCENARIOS))
        raise ValueError(
            f'Unknown replanning scenario {name!r}; choose one of: {choices}'
        ) from error
