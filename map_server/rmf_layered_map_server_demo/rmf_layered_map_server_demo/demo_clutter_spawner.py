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

import subprocess

import rclpy
from rclpy.node import Node


DEMO_OBSTACLES = (
    ('layered_map_obstacle_0', -11.0, -20.0),
    ('layered_map_obstacle_1', 9.0, -20.0),
    ('layered_map_obstacle_2', -11.0, 20.0),
)


def box_sdf(name, x, y):
    """Return a small static box model for a deterministic laser target."""
    return (
        "<sdf version='1.7'>"
        f"<model name='{name}'>"
        f'<pose>{x} {y} 0.5 0 0 0</pose>'
        '<static>true</static>'
        "<link name='link'>"
        "<visual name='visual'>"
        '<geometry><box><size>0.3 0.3 1.0</size></box></geometry>'
        '</visual>'
        "<collision name='collision'>"
        '<geometry><box><size>0.3 0.3 1.0</size></box></geometry>'
        '</collision>'
        '</link>'
        '</model>'
        '</sdf>'
    )


class DemoClutterSpawner(Node):
    """Retry deterministic Gazebo box creation until the world is ready."""

    def __init__(self):
        super().__init__('layered_map_demo_clutter_spawner')
        self.world_name = self.declare_parameter('world_name', 'warehouse').value
        self.pending = list(DEMO_OBSTACLES)
        self.timer = self.create_timer(2.0, self.spawn_pending)

    def spawn_pending(self):
        for obstacle in list(self.pending):
            name, x, y = obstacle
            request = f'sdf: "{box_sdf(name, x, y)}"'
            try:
                result = subprocess.run(
                    [
                        'gz',
                        'service',
                        '-s',
                        f'/world/{self.world_name}/create',
                        '--reqtype',
                        'gz.msgs.EntityFactory',
                        '--reptype',
                        'gz.msgs.Boolean',
                        '--timeout',
                        '1000',
                        '--req',
                        request,
                    ],
                    capture_output=True,
                    check=False,
                    text=True,
                    timeout=3.0,
                )
            except (OSError, subprocess.TimeoutExpired) as error:
                self.get_logger().warning(f'Waiting for Gazebo: {error}')
                return

            if result.returncode != 0:
                detail = result.stderr.strip() or result.stdout.strip()
                self.get_logger().warning(f'Waiting for Gazebo: {detail}')
                return

            self.pending.remove(obstacle)
            self.get_logger().info(f'Spawned {name} at ({x}, {y})')

        if not self.pending:
            self.get_logger().info('All layered-map demo obstacles are ready')
            self.timer.cancel()
            rclpy.shutdown()


def main():
    rclpy.init()
    node = DemoClutterSpawner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
