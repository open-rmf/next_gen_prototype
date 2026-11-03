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


if __name__ == '__main__':
    generate_map()
