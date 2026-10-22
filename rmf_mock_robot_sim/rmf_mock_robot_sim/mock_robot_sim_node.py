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

import json
import math
import os
import random

from geometry_msgs.msg import Point, PoseStamped, Quaternion
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rmf_prototype_msgs.msg import (
    Participant,
    ParticipantList,
    Plan,
    PlanRelease,
    Progress,
    RobotStatus,
    SafeZone,
)


class MockRobotSimNode(Node):
    def __init__(self):
        super().__init__('rmf_mock_robot_sim')

        # 1. Declare and retrieve parameters
        self.declare_parameter('robot_name', 'robot_1')
        self.declare_parameter('speed', 1.0)
        self.declare_parameter('update_rate', 20.0)
        self.declare_parameter('initial_x', 0.0)
        self.declare_parameter('initial_y', 0.0)
        self.declare_parameter('initial_yaw', 0.0)
        self.declare_parameter('publish_discovery', True)
        self.declare_parameter('site_file', '')
        self.declare_parameter('default_action_duration', 3.0)
        self.declare_parameter('action_time_min_scale', 0.5)
        self.declare_parameter('action_time_max_scale', 2.0)

        self.robot_name = self.get_parameter(
            'robot_name'
        ).get_parameter_value().string_value

        self.speed = self.get_parameter(
            'speed'
        ).get_parameter_value().double_value

        self.update_rate = self.get_parameter(
            'update_rate'
        ).get_parameter_value().double_value

        self.x = self.get_parameter(
            'initial_x'
        ).get_parameter_value().double_value

        self.y = self.get_parameter(
            'initial_y'
        ).get_parameter_value().double_value

        self.yaw = self.get_parameter(
            'initial_yaw'
        ).get_parameter_value().double_value

        self.publish_discovery_param = self.get_parameter(
            'publish_discovery'
        ).get_parameter_value().bool_value

        self.site_file = self.get_parameter(
            'site_file'
        ).get_parameter_value().string_value

        self.default_action_duration = self.get_parameter(
            'default_action_duration'
        ).get_parameter_value().double_value

        self.action_time_min_scale = self.get_parameter(
            'action_time_min_scale'
        ).get_parameter_value().double_value

        self.action_time_max_scale = self.get_parameter(
            'action_time_max_scale'
        ).get_parameter_value().double_value

        self.get_logger().info(
            f"Initializing mock robot '{self.robot_name}' "
            f'at ({self.x:.2f}, {self.y:.2f}, {self.yaw:.2f}) '
            f'with speed {self.speed:.2f} m/s'
        )

        # 2. Initialize path following and site action state
        self.current_path = []
        self.current_waypoint_idx = 0
        self.current_plan_id = None
        self.plan_start_pos = (self.x, self.y)
        self.last_update_time = self.get_clock().now()
        self.wait_time_remaining = 0.0
        self.current_wait_duration = 1.0
        self.active_transition_speed = self.speed
        self.active_transition_action = None
        self.active_transition_duration = None
        self.sampled_action_durations = []
        self.released_waypoint_idx = 0
        self.latest_release = None
        self.latest_safe_zone_id = None
        self.was_blocked = False

        self.site_anchors = {}
        self.site_action_lanes = []
        self.site_dock_positions = []
        self.site_actions_by_name = {}
        if self.site_file:
            self.load_site_file(self.site_file)

        # 3. Configure QoS for Discovery & Reliable Data
        discovery_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        reliable_transient_qos = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        # 4. Create Publishers
        self.odom_pub = self.create_publisher(
            Odometry,
            f'{self.robot_name}/odom',
            qos_profile=10
        )

        self.progress_pub = self.create_publisher(
            Progress,
            f'{self.robot_name}/plan/progress',
            qos_profile=reliable_transient_qos
        )

        self.status_pub = self.create_publisher(
            RobotStatus,
            f'{self.robot_name}/status',
            qos_profile=reliable_transient_qos
        )

        self.dest_discovery_pub = self.create_publisher(
            ParticipantList,
            '/destination/discovery',
            qos_profile=discovery_qos
        )

        self.plan_discovery_pub = self.create_publisher(
            ParticipantList,
            '/plan/discovery',
            qos_profile=discovery_qos
        )

        # 5. Create Subscriptions
        # Subscribe to plan topic
        self.plan_sub = self.create_subscription(
            Plan,
            f'{self.robot_name}/plan',
            self.handle_plan,
            qos_profile=reliable_transient_qos
        )

        # Also subscribe to path topic as mentioned in the README
        self.path_sub = self.create_subscription(
            Plan,
            f'{self.robot_name}/path',
            self.handle_plan,
            qos_profile=reliable_transient_qos
        )

        # Subscribe to plan release topic
        self.release_sub = self.create_subscription(
            PlanRelease,
            f'{self.robot_name}/plan/release',
            self.handle_release,
            qos_profile=reliable_transient_qos
        )

        # Subscribe to safe zone topic
        self.safe_zone_sub = self.create_subscription(
            SafeZone,
            f'{self.robot_name}/plan/safe_zone',
            self.handle_safe_zone,
            qos_profile=reliable_transient_qos
        )

        # 6. Timers
        # Simulation update timer
        dt = 1.0 / self.update_rate
        self.sim_timer = self.create_timer(dt, self.sim_step)

        # Periodic discovery publication timer
        if self.publish_discovery_param:
            self.discovery_timer = self.create_timer(1.0, self.publish_discovery)

            # Publish initial discovery message immediately
            self.publish_discovery()

    def yaw_to_quaternion(self, yaw):
        return Quaternion(
            x=0.0,
            y=0.0,
            z=math.sin(yaw / 2.0),
            w=math.cos(yaw / 2.0)
        )

    def load_site_file(self, site_file_path):
        if not os.path.exists(site_file_path):
            self.get_logger().warning(
                f"Configured site_file '{site_file_path}' does not exist"
            )
            return

        try:
            with open(site_file_path, 'r', encoding='utf-8') as f:
                site_data = json.load(f)
        except Exception as exc:
            self.get_logger().error(
                f"Failed to load site_file '{site_file_path}': {exc}"
            )
            return

        anchors = {}
        for anchor_id, pose in site_data.get('anchors', {}).items():
            if isinstance(pose, list) and len(pose) >= 2:
                anchors[str(anchor_id)] = (float(pose[0]), float(pose[1]))

        for level_data in site_data.get('levels', {}).values():
            if not isinstance(level_data, dict):
                continue
            for anchor_id, pose in level_data.get('anchors', {}).items():
                if isinstance(pose, list) and len(pose) >= 2:
                    anchors[str(anchor_id)] = (float(pose[0]), float(pose[1]))

        self.site_anchors = anchors
        self.site_action_lanes = []
        self.site_dock_positions = []
        self.site_actions_by_name = {}

        guided = site_data.get('navigation', {}).get('guided', {})
        lanes = guided.get('lanes', {})
        for lane_id, lane_data in lanes.items():
            if not isinstance(lane_data, dict):
                continue
            left_id = str(lane_data.get('left', ''))
            right_id = str(lane_data.get('right', ''))
            if left_id not in anchors or right_id not in anchors:
                continue
            left_pos = anchors[left_id]
            right_pos = anchors[right_id]

            forward = lane_data.get('forward', {})
            fwd_action = self._extract_lane_action(forward, f'lane_{lane_id}')
            if fwd_action is not None:
                act_name, act_dur = fwd_action
                self.site_action_lanes.append({
                    'start': left_pos,
                    'end': right_pos,
                    'action_name': act_name,
                    'duration': act_dur,
                })
                self.site_dock_positions.append({
                    'position': right_pos,
                    'action_name': act_name,
                    'duration': act_dur,
                })
                self.site_actions_by_name[act_name] = act_dur

            reverse = lane_data.get('reverse')
            if reverse == 'Same' and fwd_action is not None:
                act_name, act_dur = fwd_action
                rev_name = f'undock:{act_name}'
                self.site_action_lanes.append({
                    'start': right_pos,
                    'end': left_pos,
                    'action_name': rev_name,
                    'duration': act_dur,
                })
                self.site_actions_by_name[rev_name] = act_dur
            elif isinstance(reverse, dict):
                rev_props = reverse.get('Different', reverse)
                rev_action = self._extract_lane_action(
                    rev_props, f'lane_{lane_id}_rev'
                )
                if rev_action is not None:
                    rev_name, rev_dur = rev_action
                    self.site_action_lanes.append({
                        'start': right_pos,
                        'end': left_pos,
                        'action_name': rev_name,
                        'duration': rev_dur,
                    })
                    self.site_dock_positions.append({
                        'position': left_pos,
                        'action_name': rev_name,
                        'duration': rev_dur,
                    })
                    self.site_actions_by_name[rev_name] = rev_dur

        locations = guided.get('locations', {})
        for loc_id, loc_data in locations.items():
            if not isinstance(loc_data, dict):
                continue
            anchor_id = str(loc_data.get('anchor', ''))
            if anchor_id not in anchors:
                continue
            loc_pos = anchors[anchor_id]
            loc_name = str(loc_data.get('name', f'location_{loc_id}'))
            tags = loc_data.get('tags', [])
            is_dock = bool(loc_data.get('is_dock', False))
            dock_dur = float(
                loc_data.get('dock_duration', self.default_action_duration)
            )
            if isinstance(tags, list):
                for tag in tags:
                    if tag == 'Dock' or (
                        isinstance(tag, dict) and 'Dock' in tag
                    ):
                        is_dock = True
                        if isinstance(tag, dict) and isinstance(
                            tag.get('Dock'), dict
                        ):
                            dock_info = tag['Dock']
                            loc_name = str(
                                dock_info.get('dock_name', loc_name)
                            )
                            if 'duration' in dock_info:
                                dock_dur = float(dock_info['duration'])
            if is_dock:
                if not any(
                    math.hypot(
                        d['position'][0] - loc_pos[0],
                        d['position'][1] - loc_pos[1],
                    )
                    < 1e-3
                    for d in self.site_dock_positions
                ):
                    self.site_dock_positions.append({
                        'position': loc_pos,
                        'action_name': loc_name,
                        'duration': dock_dur,
                    })
                self.site_actions_by_name.setdefault(loc_name, dock_dur)

        self.get_logger().info(
            f"Loaded site file '{site_file_path}': "
            f'{len(self.site_anchors)} anchors, '
            f'{len(self.site_action_lanes)} action lanes, '
            f'{len(self.site_dock_positions)} dock points'
        )

    def _extract_lane_action(self, lane_props, fallback_name):
        if not isinstance(lane_props, dict):
            return None
        dock = lane_props.get('dock')
        if isinstance(dock, dict):
            name = str(dock.get('dock_name', fallback_name))
            dur = float(dock.get('duration', self.default_action_duration))
            if dur <= 0.0:
                dur = self.default_action_duration
            return (name, dur)
        if isinstance(dock, str) and dock:
            return (dock, self.default_action_duration)
        action = lane_props.get('action')
        if isinstance(action, dict):
            name = str(action.get('name', fallback_name))
            dur = float(action.get('duration', self.default_action_duration))
            if dur <= 0.0:
                dur = self.default_action_duration
            return (name, dur)
        if isinstance(action, str) and action:
            return (action, self.default_action_duration)
        return None

    def sample_action_duration(self, base_duration=None):
        if base_duration is None or base_duration <= 0.0:
            base_duration = self.default_action_duration
        min_scale = min(self.action_time_min_scale, self.action_time_max_scale)
        max_scale = max(self.action_time_min_scale, self.action_time_max_scale)
        scale = random.uniform(min_scale, max_scale)
        return max(0.05, float(base_duration) * scale)

    def _get_transition_action(self, target_idx):
        if not self.current_path or target_idx < 0 or target_idx >= len(self.current_path):
            return None

        target_wp = self.current_path[target_idx]
        prev_wp = self.current_path[target_idx - 1] if target_idx > 0 else None

        dep_action = getattr(prev_wp, 'departure_action', '') if prev_wp is not None else ''
        arr_action = getattr(target_wp, 'arrival_action', '')
        if arr_action or dep_action:
            action_name = arr_action or dep_action
            base_duration = self.site_actions_by_name.get(
                action_name, self.default_action_duration
            )
            return (action_name, base_duration)

        if target_idx > 0:
            start_pos = (float(prev_wp.position[0]), float(prev_wp.position[1]))
        else:
            start_pos = (float(self.plan_start_pos[0]), float(self.plan_start_pos[1]))
        end_pos = (float(target_wp.position[0]), float(target_wp.position[1]))
        seg_dist = math.hypot(end_pos[0] - start_pos[0], end_pos[1] - start_pos[1])

        # Do not trigger implicit site actions on a zero-length initial waypoint (target_idx == 0)
        if target_idx == 0 and seg_dist <= 1e-3:
            return None

        tol = 0.55
        for lane in self.site_action_lanes:
            start_dist = math.hypot(
                start_pos[0] - lane['start'][0],
                start_pos[1] - lane['start'][1],
            )
            end_dist = math.hypot(
                end_pos[0] - lane['end'][0],
                end_pos[1] - lane['end'][1],
            )
            if start_dist <= tol and end_dist <= tol:
                return (lane['action_name'], lane['duration'])

        for dock in self.site_dock_positions:
            dpos = dock['position']
            end_dist = math.hypot(end_pos[0] - dpos[0], end_pos[1] - dpos[1])
            if end_dist <= tol:
                return (dock['action_name'], dock['duration'])
            start_dist = math.hypot(start_pos[0] - dpos[0], start_pos[1] - dpos[1])
            if target_idx > 0 and start_dist <= tol:
                return (f"undock:{dock['action_name']}", dock['duration'])

        return None

    def _configure_transition(self, target_idx, initial=False):
        self.current_waypoint_idx = target_idx
        if not self.current_path or target_idx >= len(self.current_path):
            self.wait_time_remaining = 0.0
            self.current_wait_duration = 1.0
            self.active_transition_speed = self.speed
            self.active_transition_action = None
            self.active_transition_duration = None
            return

        if target_idx > 0:
            prev_wp = self.current_path[target_idx - 1]
            start_pos = (float(prev_wp.position[0]), float(prev_wp.position[1]))
        else:
            start_pos = (float(self.plan_start_pos[0]), float(self.plan_start_pos[1]))
        target_wp = self.current_path[target_idx]
        end_pos = (float(target_wp.position[0]), float(target_wp.position[1]))
        seg_dist = math.hypot(end_pos[0] - start_pos[0], end_pos[1] - start_pos[1])

        action_info = self._get_transition_action(target_idx)
        if action_info is not None:
            action_name, base_duration = action_info
            sampled_duration = self.sample_action_duration(base_duration)
            self.active_transition_action = action_name
            self.active_transition_duration = sampled_duration
            self.sampled_action_durations.append(sampled_duration)
            if seg_dist > 1e-3:
                self.wait_time_remaining = 0.0
                self.current_wait_duration = sampled_duration
                self.active_transition_speed = seg_dist / sampled_duration
            else:
                self.current_wait_duration = sampled_duration
                self.wait_time_remaining = sampled_duration
                self.active_transition_speed = self.speed
            self.get_logger().info(
                f"Transition to waypoint {target_idx} has action '{action_name}': "
                f'sampled duration {sampled_duration:.2f}s (base {base_duration:.2f}s)'
            )
        else:
            self.active_transition_action = None
            self.active_transition_duration = None
            self.active_transition_speed = self.speed
            self.current_wait_duration = 1.0
            self.wait_time_remaining = 0.0 if initial else 1.0

    def publish_discovery(self):
        msg = ParticipantList()
        p = Participant()
        p.name = self.robot_name
        p.components = []
        msg.participants.append(p)

        self.dest_discovery_pub.publish(msg)
        self.plan_discovery_pub.publish(msg)

    def _plan_id_matches(self, plan_id_a, plan_id_b):
        if plan_id_a is None or plan_id_b is None:
            return False
        return (
            list(plan_id_a.destination_session.uuid)
            == list(plan_id_b.destination_session.uuid)
            and plan_id_a.plan_version == plan_id_b.plan_version
        )

    def handle_plan(self, msg: Plan):
        self.get_logger().info(
            f'Received new plan with {len(msg.waypoints)} waypoints. '
            f'Plan version: {msg.plan_id.plan_version}'
        )
        self.current_path = msg.waypoints
        self.current_plan_id = msg.plan_id
        self.plan_start_pos = (self.x, self.y)
        self._configure_transition(0, initial=True)
        if self.latest_release and self._plan_id_matches(
            self.latest_release.plan_id, self.current_plan_id
        ):
            self.released_waypoint_idx = self.latest_release.waypoint_id
        else:
            self.released_waypoint_idx = 0
        if self.latest_safe_zone_id and not self._plan_id_matches(
            self.latest_safe_zone_id.plan_id, self.current_plan_id
        ):
            self.latest_safe_zone_id = None
        self.was_blocked = False

        # If the path is not empty and we are far from the start,
        # we could either teleport or travel there.
        # The standard practice is to just start moving from current
        # position to waypoints[0].

    def handle_safe_zone(self, msg: SafeZone):
        if not self.current_plan_id or self._plan_id_matches(
            msg.id.plan_id, self.current_plan_id
        ):
            self.latest_safe_zone_id = msg.id

    def handle_release(self, msg: PlanRelease):
        self.get_logger().info(
            f'Received PlanRelease: waypoint_id={msg.waypoint_id}, '
            f'plan_version={msg.plan_id.plan_version}'
        )
        self.latest_release = msg
        if not self.current_plan_id:
            self.get_logger().info('Cannot update release: current_plan_id is None')
            return

        msg_uuid = list(msg.plan_id.destination_session.uuid)
        curr_uuid = list(self.current_plan_id.destination_session.uuid)
        uuid_matches = msg_uuid == curr_uuid
        version_matches = msg.plan_id.plan_version == self.current_plan_id.plan_version
        self.get_logger().info(
            f'PlanRelease check: uuid_matches={uuid_matches}, version_matches={version_matches}'
        )
        if uuid_matches and version_matches:
            self.released_waypoint_idx = msg.waypoint_id
            self.get_logger().info(
                f'Successfully updated released_waypoint_idx to {self.released_waypoint_idx}'
            )

    def is_blocked(self):
        if not self.current_path or self.current_waypoint_idx >= len(self.current_path):
            return False
        if self.current_waypoint_idx > self.released_waypoint_idx:
            return True
        return False

    def sim_step(self):
        now = self.get_clock().now()
        # Calculate actual dt in seconds
        dt = (now - self.last_update_time).nanoseconds / 1e9
        self.last_update_time = now

        # Prevent large jumps if clock or simulation behaves unexpectedly
        if dt <= 0.0 or dt > 1.0:
            dt = 1.0 / self.update_rate

        # 1. Update position based on current path
        if self.current_path and self.current_waypoint_idx < len(self.current_path):
            if self.is_blocked():
                if not self.was_blocked:
                    self.get_logger().info(
                        f'Blocked! current_waypoint_idx={self.current_waypoint_idx}, '
                        f'released_waypoint_idx={self.released_waypoint_idx}'
                    )
                    self.was_blocked = True
                # Blocked by traffic dependencies! Do not move.
                pass
            else:
                if self.was_blocked:
                    self.get_logger().info(
                        f'Unblocked! current_waypoint_idx={self.current_waypoint_idx}, '
                        f'released_waypoint_idx={self.released_waypoint_idx}'
                    )
                    self.was_blocked = False
                if self.wait_time_remaining > 0.0:
                    self.wait_time_remaining -= dt
                    if self.wait_time_remaining < 0.0:
                        self.wait_time_remaining = 0.0
                else:
                    target_wp = self.current_path[self.current_waypoint_idx]
                    tx, ty = float(target_wp.position[0]), float(target_wp.position[1])
                    dx = tx - self.x
                    dy = ty - self.y
                    dist = math.hypot(dx, dy)

                    if dist > 1e-3:
                        self.yaw = math.atan2(dy, dx)
                        step = self.active_transition_speed * dt
                        if step >= dist:
                            # Reached the target waypoint
                            self.x = tx
                            self.y = ty
                            self._configure_transition(self.current_waypoint_idx + 1)
                        else:
                            self.x += (dx / dist) * step
                            self.y += (dy / dist) * step
                    else:
                        # Already at or extremely close to target waypoint, advance to next
                        self.x = tx
                        self.y = ty
                        self._configure_transition(self.current_waypoint_idx + 1)

        # 2. Publish plan progress
        progress_msg = None
        if self.current_path:
            progress_msg = self.publish_progress()

        # 3. Publish Odometry
        self.publish_odometry()

        # 4. Publish unified RobotStatus
        self.publish_status(progress_msg)

    def build_progress_msg(self):
        if not self.current_plan_id or not self.current_path:
            return None

        progress_msg = Progress()
        progress_msg.plan_id = self.current_plan_id

        num_waypoints = len(self.current_path)

        if self.current_waypoint_idx >= num_waypoints:
            last_idx = num_waypoints - 1
            progress_msg.reached_waypoint = int(last_idx)
            progress_msg.target_waypoint = int(last_idx)
            progress_msg.progress = float(self.current_path[last_idx].progress)
        elif self.current_waypoint_idx == 0:
            first_wp = self.current_path[0]
            progress_msg.reached_waypoint = 0
            progress_msg.target_waypoint = 0
            seg_dist = math.hypot(
                first_wp.position[0] - self.plan_start_pos[0],
                first_wp.position[1] - self.plan_start_pos[1],
            )
            if seg_dist > 1e-3:
                dist_from_start = math.hypot(
                    self.x - self.plan_start_pos[0],
                    self.y - self.plan_start_pos[1],
                )
                fraction = min(1.0, max(0.0, dist_from_start / seg_dist))
                progress_msg.progress = float(fraction * first_wp.progress)
            else:
                progress_msg.progress = 0.0
        elif self.is_blocked():
            reached_idx = self.current_waypoint_idx - 1
            progress_msg.reached_waypoint = int(reached_idx)
            progress_msg.target_waypoint = int(reached_idx)
            progress_msg.progress = float(self.current_path[reached_idx].progress)
        else:
            reached_idx = self.current_waypoint_idx - 1
            target_idx = self.current_waypoint_idx
            progress_msg.reached_waypoint = int(reached_idx)
            progress_msg.target_waypoint = int(target_idx)

            prev_wp = self.current_path[reached_idx]
            next_wp = self.current_path[target_idx]

            prev_pos = prev_wp.position
            next_pos = next_wp.position
            seg_dist = math.hypot(
                next_pos[0] - prev_pos[0],
                next_pos[1] - prev_pos[1],
            )

            if seg_dist > 1e-3:
                dist_from_prev = math.hypot(
                    self.x - prev_pos[0],
                    self.y - prev_pos[1],
                )
                fraction = min(1.0, max(0.0, dist_from_prev / seg_dist))
                diff = next_wp.progress - prev_wp.progress
                progress_msg.progress = float(
                    prev_wp.progress + fraction * diff
                )
            else:
                denom = (
                    self.current_wait_duration
                    if self.current_wait_duration > 1e-6
                    else 1.0
                )
                fraction = min(
                    1.0, max(0.0, 1.0 - (self.wait_time_remaining / denom))
                )
                diff = next_wp.progress - prev_wp.progress
                progress_msg.progress = float(
                    prev_wp.progress + fraction * diff
                )

        progress_msg.reached_keys = []
        return progress_msg

    def publish_progress(self):
        progress_msg = self.build_progress_msg()
        if progress_msg is None:
            return None
        self.progress_pub.publish(progress_msg)
        return progress_msg

    def determine_mode(self):
        if (
            not self.current_plan_id
            or not self.current_path
            or self.current_waypoint_idx >= len(self.current_path)
        ):
            return RobotStatus.MODE_IDLE
        if self.is_blocked():
            return RobotStatus.MODE_WAITING
        if self.active_transition_action is not None:
            return RobotStatus.MODE_ACTION
        if self.wait_time_remaining > 0.0:
            return RobotStatus.MODE_WAITING
        return RobotStatus.MODE_NAVIGATING

    def publish_status(self, progress_msg=None):
        status = RobotStatus()
        status.pose = PoseStamped()
        status.pose.header.stamp = self.get_clock().now().to_msg()
        status.pose.header.frame_id = 'map'
        status.pose.pose.position = Point(x=self.x, y=self.y, z=0.0)
        status.pose.pose.orientation = self.yaw_to_quaternion(self.yaw)
        status.mode = self.determine_mode()

        if progress_msg is None:
            progress_msg = self.build_progress_msg()
        if progress_msg is not None:
            status.progress = [progress_msg]
        else:
            status.progress = []

        if self.latest_safe_zone_id is not None and (
            not self.current_plan_id
            or self._plan_id_matches(
                self.latest_safe_zone_id.plan_id, self.current_plan_id
            )
        ):
            status.safe_zone_id = [self.latest_safe_zone_id]
        else:
            status.safe_zone_id = []

        self.status_pub.publish(status)

    def publish_odometry(self):
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'map'
        odom.child_frame_id = f'{self.robot_name}/base_link'

        # Set pose
        odom.pose.pose.position = Point(x=self.x, y=self.y, z=0.0)
        odom.pose.pose.orientation = self.yaw_to_quaternion(self.yaw)

        # Set twist (simple velocity estimate)
        if (
            self.current_path
            and self.current_waypoint_idx < len(self.current_path)
            and self.wait_time_remaining <= 0.0
            and not self.is_blocked()
        ):
            target_wp = self.current_path[self.current_waypoint_idx]
            tx, ty = target_wp.position[0], target_wp.position[1]
            dx = tx - self.x
            dy = ty - self.y
            dist = math.hypot(dx, dy)
            if dist > 1e-3:
                odom.twist.twist.linear.x = (dx / dist) * self.active_transition_speed
                odom.twist.twist.linear.y = (dy / dist) * self.active_transition_speed
                odom.twist.twist.angular.z = 0.0
        else:
            odom.twist.twist.linear.x = 0.0
            odom.twist.twist.linear.y = 0.0
            odom.twist.twist.angular.z = 0.0

        self.odom_pub.publish(odom)


def main(args=None):
    import rclpy
    rclpy.init(args=args)
    node = MockRobotSimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
