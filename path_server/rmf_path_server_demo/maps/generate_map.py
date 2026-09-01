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

import os
from PIL import Image, ImageDraw


def generate_maps():
    maps_dir = os.path.dirname(os.path.abspath(__file__))

    # 1. Generate 1.0m resolution demo_grid (20x20 pixels)
    size_1m = 20
    img_1m = Image.new('L', (size_1m, size_1m), 255)
    draw_1m = ImageDraw.Draw(img_1m)
    # 4x4 obstacle in center
    draw_1m.rectangle([8, 8, 11, 11], fill=0)
    img_1m.save(os.path.join(maps_dir, 'demo_grid.png'))
    print("Generated demo_grid.png (20x20 @ 1.0m)")

    # 2. Generate 0.1m resolution demo_grid_0_1m (200x200 pixels)
    size_01m = 200
    img_01m = Image.new('L', (size_01m, size_01m), 255)
    draw_01m = ImageDraw.Draw(img_01m)

    # Center obstacle block (4m x 4m = 40x40 pixels: world [-2, 2])
    draw_01m.rectangle([80, 80, 119, 119], fill=0)

    # Left dividing wall at x=50 (world x = -5.0m) with a 1.0m gap (10 px, y: 95..105)
    draw_01m.rectangle([48, 20, 52, 94], fill=0)
    draw_01m.rectangle([48, 106, 52, 180], fill=0)

    # Right dividing wall at x=150 (world x = +5.0m) with a narrow 0.7m gap (7 px) and a wide 1.4m gap (14 px)
    draw_01m.rectangle([148, 20, 152, 50], fill=0)
    # narrow gap: 51..57 (0.7m)
    draw_01m.rectangle([148, 58, 152, 140], fill=0)
    # wide gap: 141..154 (1.4m)
    draw_01m.rectangle([148, 155, 152, 180], fill=0)

    img_01m.save(os.path.join(maps_dir, 'demo_grid_0_1m.png'))
    print("Generated demo_grid_0_1m.png (200x200 @ 0.1m)")


if __name__ == '__main__':
    generate_maps()
