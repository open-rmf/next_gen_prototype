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

from rmf_layered_map_msgs.msg import MapRegionPatch
from rmf_layered_map_server_demo.scan_region_publisher import make_scan_patches
from rmf_prototype_msgs.msg import Region


def test_scan_patches_clear_rays_before_marking_obstacle_endpoints():
    patches = make_scan_patches(
        [(0.0, 0.0, 1.0, -0.1, 1.0, 0.1)],
        [(1.0, 0.0)],
        ttl_sec=5.0,
    )

    assert [patch.update_type for patch in patches] == [
        MapRegionPatch.UPDATE_CLEAR,
        MapRegionPatch.UPDATE_OBSTACLE,
    ]
    assert patches[0].occupancy_value == 0
    assert patches[0].regions[0].hint == Region.HINT_CONVEX_POLYGON
    assert patches[1].occupancy_value == 100
    assert patches[1].regions[0].hint == Region.HINT_POINT
