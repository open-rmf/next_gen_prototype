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

from PIL import Image, ImageDraw


def generate_map():
    # 20x20 pixels
    size = 20
    # Create a white image (255 is white/free)
    img = Image.new('L', (size, size), 255)
    draw = ImageDraw.Draw(img)

    # Draw a black box in the center (0 is black/occupied)
    # Box from index 8 to 11 (4x4 cells in the center of 20x20)
    draw.rectangle([8, 8, 11, 11], fill=0)

    # Save the image
    img.save('demo_grid.png')
    print('Generated demo_grid.png')


def generate_corridor_map():
    # 14x9 pixels with origin [-5.0, -4.0, 0.0] and resolution 1.0
    # Central 3x8 corridor: wx in [-2..5] (ix in [3..10]), wy in [-1..1] (iy in [3..5])
    # Protruding 1x1 docks at wx in {-1, 1, 3} (ix in {4, 6, 8}) on wy in {-2, 2}
    width, height = 14, 9
    img = Image.new('L', (width, height), 0)
    draw = ImageDraw.Draw(img)

    # Carve 3x8 central corridor (free = 255)
    draw.rectangle([3, 3, 10, 5], fill=255)

    # Carve protruding dock alcoves (north: iy=2 in top-down / iy=6 in bottom-up; south: symmetric)
    for ix in (4, 6, 8):
        img.putpixel((ix, 2), 255)
        img.putpixel((ix, 6), 255)

    img.save('corridor_grid.png')
    print('Generated corridor_grid.png')


if __name__ == '__main__':
    generate_map()
    generate_corridor_map()
