"""Domain logic for SafeZone, LocalPlanRegistry, CostmapHandshake, and ProgressTracker."""

from collections import deque
from dataclasses import dataclass, field
from enum import Enum
import math
import time
from typing import Deque, List, Optional, Tuple
import uuid

from builtin_interfaces.msg import Time as RosTime
from geometry_msgs.msg import PoseStamped
from nav2_msgs.msg import Costmap
from rmf_prototype_msgs.msg import (
    DestinationConstraints,
    DestinationGoal,
    Plan,
    PlanId,
    Progress,
    Region,
    SafeZone,
    SafeZoneId,
    TargetOrientation,
    TargetRegion,
)
from unique_identifier_msgs.msg import UUID as RosUuid


class InnerActionKind(Enum):
    NAVIGATE_TO_POSE = 0
    DOCK_ROBOT = 1
    UNDOCK_ROBOT = 2


@dataclass(frozen=True)
class InnerNavigationAction:
    kind: InnerActionKind = InnerActionKind.NAVIGATE_TO_POSE
    use_dock_id: bool = False
    dock_id: str = ''
    dock_type: str = ''
    navigate_to_staging_pose: bool = False

    @staticmethod
    def navigate_to_pose() -> 'InnerNavigationAction':
        return InnerNavigationAction(kind=InnerActionKind.NAVIGATE_TO_POSE)

    @staticmethod
    def dock_robot(
        use_dock_id: bool,
        dock_id: str,
        dock_type: str,
        navigate_to_staging_pose: bool,
    ) -> 'InnerNavigationAction':
        return InnerNavigationAction(
            kind=InnerActionKind.DOCK_ROBOT,
            use_dock_id=use_dock_id,
            dock_id=dock_id,
            dock_type=dock_type,
            navigate_to_staging_pose=navigate_to_staging_pose,
        )

    @staticmethod
    def undock_robot(dock_type: str) -> 'InnerNavigationAction':
        return InnerNavigationAction(
            kind=InnerActionKind.UNDOCK_ROBOT,
            dock_type=dock_type,
        )


@dataclass
class InnerNavigationTarget:
    agent: int
    safe_zone_id: SafeZoneId
    x: float
    y: float
    yaw: float
    action: InnerNavigationAction = field(
        default_factory=InnerNavigationAction.navigate_to_pose
    )


@dataclass(frozen=True)
class KnownDock:
    id: str
    dock_type: str
    x: float
    y: float
    yaw: float
    staging_x: float
    staging_y: float

    @staticmethod
    def corridor_docks() -> List['KnownDock']:
        half_pi = math.pi / 2.0
        return [
            KnownDock('dock_north_1', 'simple_charging_dock', -1.0, 2.0, half_pi, -1.0, 1.0),
            KnownDock('dock_north_2', 'simple_charging_dock', 1.0, 2.0, half_pi, 1.0, 1.0),
            KnownDock('dock_north_3', 'simple_charging_dock', 3.0, 2.0, half_pi, 3.0, 1.0),
            KnownDock('dock_south_1', 'simple_charging_dock', -1.0, -2.0, -half_pi, -1.0, -1.0),
            KnownDock('dock_south_2', 'simple_charging_dock', 1.0, -2.0, -half_pi, 1.0, -1.0),
            KnownDock('dock_south_3', 'simple_charging_dock', 3.0, -2.0, -half_pi, 3.0, -1.0),
        ]

    @staticmethod
    def match_by_id(dock_id: str) -> Optional['KnownDock']:
        lower = dock_id.strip().lower()
        for dock in KnownDock.corridor_docks():
            if dock.id.lower() == lower:
                return dock
        return None

    @staticmethod
    def match_by_pose(x: float, y: float, tolerance: float) -> Optional['KnownDock']:
        for dock in KnownDock.corridor_docks():
            if math.hypot(dock.x - x, dock.y - y) <= tolerance:
                return dock
        return None


class LocalPlanRegistry:
    MAX_CACHED_PLANS = 16

    def __init__(self) -> None:
        self._entries: Deque[Plan] = deque(maxlen=self.MAX_CACHED_PLANS)
        self.latest_plan: Optional[Plan] = None

    def insert(self, plan: Plan) -> None:
        for i, existing in enumerate(self._entries):
            if existing.plan_id == plan.plan_id:
                self._entries[i] = plan
                self._update_latest(plan)
                return
        self._entries.append(plan)
        self._update_latest(plan)

    def get(self, plan_id: PlanId) -> Optional[Plan]:
        for entry in self._entries:
            if entry.plan_id == plan_id:
                return entry
        return None

    def _update_latest(self, plan: Plan) -> None:
        if (
            self.latest_plan is None
            or list(self.latest_plan.plan_id.destination_session.uuid)
            != list(plan.plan_id.destination_session.uuid)
            or plan.plan_id.plan_version >= self.latest_plan.plan_id.plan_version
        ):
            self.latest_plan = plan


@dataclass
class PendingNavigationTarget:
    required_seq: int
    target: InnerNavigationTarget
    target_xy_yaw: Tuple[float, float, float]
    costmap: Costmap
    last_published: float = field(default_factory=time.monotonic)


@dataclass
class CostmapHandshakeState:
    next_seq: int = 0
    acked_seq: int = 0
    pending_target: Optional[PendingNavigationTarget] = None
    last_published_costmap: Optional[Costmap] = None


def costmap_content_eq(a: Costmap, b: Costmap) -> bool:
    return (
        a.header.frame_id == b.header.frame_id
        and a.metadata.layer == b.metadata.layer
        and a.metadata.resolution == b.metadata.resolution
        and a.metadata.size_x == b.metadata.size_x
        and a.metadata.size_y == b.metadata.size_y
        and a.metadata.origin == b.metadata.origin
        and list(a.data) == list(b.data)
    )


def seq_to_stamp(seq: int) -> RosTime:
    t = RosTime()
    t.sec = int((seq >> 32) & 0x7FFFFFFF)
    t.nanosec = int(seq & 0xFFFFFFFF)
    return t


def stamp_to_seq(stamp: RosTime) -> int:
    return ((int(stamp.sec) & 0xFFFFFFFF) << 32) | (int(stamp.nanosec) & 0xFFFFFFFF)


def shortest_angular_distance(a: float, b: float) -> float:
    two_pi = 2.0 * math.pi
    diff = math.fmod(abs(a - b), two_pi)
    if diff < 0.0:
        diff += two_pi
    return min(diff, two_pi - diff)


def is_valid_safe_zone(safe_zone: SafeZone) -> bool:
    if len(safe_zone.target_waypoint) == 0:
        return False
    if (
        len(safe_zone.incremental_target.regions) == 0
        and len(safe_zone.incremental_target.nodes) == 0
    ):
        return False
    return True


def get_safe_zone_point(safe_zone: SafeZone) -> Optional[Tuple[float, float]]:
    if len(safe_zone.incremental_target.regions) == 0:
        return None
    region = safe_zone.incremental_target.regions[0].region
    if region.hint == Region.HINT_POINT and len(region.points) == 2:
        return float(region.points[0]), float(region.points[1])
    return None


def get_safe_zone_orientation(safe_zone: SafeZone) -> Optional[float]:
    if (
        len(safe_zone.incremental_target.regions) > 0
        and len(safe_zone.incremental_target.regions[0].orientations) > 0
    ):
        return float(
            safe_zone.incremental_target.regions[0].orientations[0].orientation_radians
        )
    if (
        len(safe_zone.incremental_target.nodes) > 0
        and len(safe_zone.incremental_target.nodes[0].orientations) > 0
    ):
        return float(
            safe_zone.incremental_target.nodes[0].orientations[0].orientation_radians
        )
    return None


def next_target(safe_zone: SafeZone) -> Optional[Tuple[float, float, float]]:
    xy: Optional[Tuple[float, float]] = None
    yaw: Optional[float] = None
    for target_region in safe_zone.incremental_target.regions:
        region = target_region.region
        if region.hint == Region.HINT_POINT and len(region.points) == 2:
            xy = (float(region.points[0]), float(region.points[1]))
        for ori in target_region.orientations:
            yaw = float(ori.orientation_radians)
    for target_node in safe_zone.incremental_target.nodes:
        for ori in target_node.orientations:
            yaw = float(ori.orientation_radians)
    if xy is not None and yaw is not None:
        return (xy[0], xy[1], yaw)
    return None


class CurrentSafeZone:

    def __init__(self) -> None:
        self.value: Optional[SafeZone] = None

    def update(self, sz: SafeZone) -> None:
        self.value = sz

    def clear(self) -> None:
        self.value = None

    def matches(self, other: SafeZone) -> bool:
        return self.value is not None and self.value.id == other.id

    def distancesq_to_target(self, other: SafeZone) -> float:
        if self.value is None:
            return float('inf')
        s_pt = get_safe_zone_point(self.value)
        d_pt = get_safe_zone_point(other)
        if s_pt is None or d_pt is None:
            return float('inf')
        dx = s_pt[0] - d_pt[0]
        dy = s_pt[1] - d_pt[1]
        return dx * dx + dy * dy

    def should_update_target(self, other: SafeZone) -> bool:
        if self.value is None:
            return True
        current = self.value
        if current.id.plan_id != other.id.plan_id:
            return True
        if len(other.target_waypoint) > 0 and list(current.target_waypoint) != list(
            other.target_waypoint
        ):
            return True
        cur_yaw = get_safe_zone_orientation(current)
        other_yaw = get_safe_zone_orientation(other)
        if cur_yaw is not None and other_yaw is not None:
            if shortest_angular_distance(cur_yaw, other_yaw) > 0.05:
                return True
        return self.distancesq_to_target(other) >= 0.5


class ProgressTracker:

    def __init__(self) -> None:
        self.plan_id: Optional[PlanId] = None
        self.plan_waypoints: List[Tuple[Tuple[float, float], float]] = []
        self.start_waypoint: int = 0
        self.target_waypoint: int = 0
        self.target_progress: float = 0.0
        self.start_xy: Tuple[float, float] = (0.0, 0.0)
        self.target_xy: Tuple[float, float] = (0.0, 0.0)
        self.target_tolerance: float = 0.25
        self.target_yaw: Optional[float] = None
        self.target_yaw_tolerance: float = math.pi
        self.active_action: InnerNavigationAction = (
            InnerNavigationAction.navigate_to_pose()
        )
        self.action_completed: bool = False
        self.gate_active: bool = False

    @staticmethod
    def extract_plan_waypoints(
        plan: Plan,
    ) -> List[Tuple[Tuple[float, float], float]]:
        out: List[Tuple[Tuple[float, float], float]] = []
        for i, wp in enumerate(plan.waypoints):
            prog = float(i) if (i > 0 and wp.progress <= 0.0) else float(wp.progress)
            out.append(((float(wp.position[0]), float(wp.position[1])), prog))
        return out

    def set_plan(self, plan: Plan, current_safe_zone: Optional[SafeZone]) -> None:
        if self.plan_id is not None and self.plan_id == plan.plan_id:
            self.plan_waypoints = self.extract_plan_waypoints(plan)
            if (
                current_safe_zone is not None
                and current_safe_zone.id.plan_id == plan.plan_id
            ):
                action, has_gate = self.resolve_target_action(
                    current_safe_zone, plan, self.start_xy, self.target_xy
                )
                if has_gate and not self.action_completed:
                    self.gate_active = True
                if (
                    self.active_action.kind == InnerActionKind.NAVIGATE_TO_POSE
                    and action.kind != InnerActionKind.NAVIGATE_TO_POSE
                ):
                    self.active_action = action

    @staticmethod
    def parse_action_string(
        action_str: str, target_xy: Tuple[float, float]
    ) -> Optional[Tuple[InnerNavigationAction, bool]]:
        trimmed = action_str.strip()
        if not trimmed:
            return None
        lower = trimmed.lower()
        has_gate = (
            'gate' in lower
            or 'load' in lower
            or 'offload' in lower
            or 'off-load' in lower
        )

        if lower.startswith('undock'):
            dock_type = 'simple_charging_dock'
            if ':' in trimmed:
                rest = trimmed.split(':', 1)[1].strip()
                known = KnownDock.match_by_id(rest)
                if known is not None:
                    dock_type = known.dock_type
            return InnerNavigationAction.undock_robot(dock_type), has_gate

        if lower.startswith('dock'):
            if ':' in trimmed:
                dock_id = trimmed.split(':', 1)[1].strip()
                if dock_id:
                    known = KnownDock.match_by_id(dock_id)
                    dock_type = (
                        known.dock_type
                        if known is not None
                        else 'simple_charging_dock'
                    )
                    return (
                        InnerNavigationAction.dock_robot(
                            known is not None, dock_id, dock_type, False
                        ),
                        has_gate,
                    )
            dock = KnownDock.match_by_pose(target_xy[0], target_xy[1], 0.45)
            if dock is not None:
                return (
                    InnerNavigationAction.dock_robot(
                        True, dock.id, dock.dock_type, False
                    ),
                    has_gate,
                )
            return (
                InnerNavigationAction.dock_robot(
                    False, '', 'simple_charging_dock', False
                ),
                has_gate,
            )

        if has_gate:
            return InnerNavigationAction.navigate_to_pose(), True
        return None

    @classmethod
    def resolve_target_action(
        cls,
        safe_zone: SafeZone,
        plan: Optional[Plan],
        start_xy: Tuple[float, float],
        target_xy: Tuple[float, float],
    ) -> Tuple[InnerNavigationAction, bool]:
        target_wp_idx = (
            int(safe_zone.target_waypoint[0])
            if len(safe_zone.target_waypoint) > 0
            else 0
        )
        last_wp_idx = int(safe_zone.last_waypoint)

        if plan is not None and plan.plan_id == safe_zone.id.plan_id:
            if target_wp_idx < len(plan.waypoints):
                wp = plan.waypoints[target_wp_idx]
                res = cls.parse_action_string(wp.arrival_action, target_xy)
                if res is not None:
                    return res
                for node in wp.arrival_constraints.nodes:
                    if len(node.key.name) > 0:
                        dock = KnownDock.match_by_id(node.key.name[0])
                        if dock is not None:
                            return (
                                InnerNavigationAction.dock_robot(
                                    True, dock.id, dock.dock_type, False
                                ),
                                False,
                            )
            if last_wp_idx < len(plan.waypoints):
                prev_wp = plan.waypoints[last_wp_idx]
                res = cls.parse_action_string(prev_wp.departure_action, target_xy)
                if res is not None:
                    return res
            if target_wp_idx > 0 and (target_wp_idx - 1) < len(plan.waypoints):
                prev_wp = plan.waypoints[target_wp_idx - 1]
                res = cls.parse_action_string(prev_wp.departure_action, target_xy)
                if res is not None:
                    return res

        for node in safe_zone.incremental_target.nodes:
            if len(node.key.name) > 0:
                dock = KnownDock.match_by_id(node.key.name[0])
                if dock is not None:
                    return (
                        InnerNavigationAction.dock_robot(
                            True, dock.id, dock.dock_type, False
                        ),
                        False,
                    )

        dock = KnownDock.match_by_pose(target_xy[0], target_xy[1], 0.35)
        if dock is not None:
            dist_to_dock = math.hypot(start_xy[0] - dock.x, start_xy[1] - dock.y)
            if dist_to_dock > 0.35:
                dist_to_staging = math.hypot(
                    start_xy[0] - dock.staging_x, start_xy[1] - dock.staging_y
                )
                nav_to_staging = dist_to_staging > 0.45
                return (
                    InnerNavigationAction.dock_robot(
                        True, dock.id, dock.dock_type, nav_to_staging
                    ),
                    False,
                )

        return InnerNavigationAction.navigate_to_pose(), False

    def is_holding_at_pre_waypoint(self) -> bool:
        waiting_for_dock = (
            self.active_action.kind
            in (InnerActionKind.DOCK_ROBOT, InnerActionKind.UNDOCK_ROBOT)
            and not self.action_completed
        )
        return waiting_for_dock or self.gate_active

    def release_gate(self) -> None:
        self.gate_active = False

    def mark_action_completed(self) -> None:
        self.action_completed = True

    @staticmethod
    def extract_orientation_constraint(
        safe_zone: SafeZone,
    ) -> Tuple[Optional[float], float]:
        ori = None
        if (
            len(safe_zone.incremental_target.regions) > 0
            and len(safe_zone.incremental_target.regions[0].orientations) > 0
        ):
            ori = safe_zone.incremental_target.regions[0].orientations[0]
        elif (
            len(safe_zone.incremental_target.nodes) > 0
            and len(safe_zone.incremental_target.nodes[0].orientations) > 0
        ):
            ori = safe_zone.incremental_target.nodes[0].orientations[0]
        if ori is None:
            return None, math.pi
        effective_tol = 0.25
        if ori.tolerance_radians > 0.0 or ori.spread_radians > 0.0:
            effective_tol = 0.5 * max(0.0, float(ori.spread_radians)) + max(
                0.0, float(ori.tolerance_radians)
            )
        if effective_tol < math.pi - 1e-3:
            return float(ori.orientation_radians), effective_tol
        return None, math.pi

    def is_orientation_satisfied(self, current_yaw: float) -> bool:
        if self.target_yaw is not None and self.target_yaw_tolerance < math.pi - 1e-3:
            return (
                shortest_angular_distance(current_yaw, self.target_yaw)
                <= self.target_yaw_tolerance
            )
        return True

    def is_target_pose_satisfied(
        self, current_pose: Optional[Tuple[float, float, float]]
    ) -> bool:
        if self.active_action.kind in (
            InnerActionKind.DOCK_ROBOT,
            InnerActionKind.UNDOCK_ROBOT,
        ):
            return self.action_completed
        if current_pose is None:
            return True
        rx, ry, ryaw = current_pose
        dist = math.hypot(rx - self.target_xy[0], ry - self.target_xy[1])
        return dist <= self.target_tolerance and self.is_orientation_satisfied(ryaw)

    def on_safe_zone_with_plan(
        self,
        safe_zone: SafeZone,
        plan: Optional[Plan],
        current_pose: Optional[Tuple[float, float, float]],
        previous: Optional[Progress],
    ) -> Progress:
        incoming_plan_id = safe_zone.id.plan_id
        target_wp = (
            int(safe_zone.target_waypoint[0])
            if len(safe_zone.target_waypoint) > 0
            else 0
        )
        pt = get_safe_zone_point(safe_zone)
        new_target_xy = (float(pt[0]), float(pt[1])) if pt is not None else (0.0, 0.0)
        new_tolerance = 0.25
        if (
            len(safe_zone.incremental_target.regions) > 0
            and safe_zone.incremental_target.regions[0].tolerance > 0.0
        ):
            new_tolerance = float(safe_zone.incremental_target.regions[0].tolerance)
        new_yaw, new_yaw_tol = self.extract_orientation_constraint(safe_zone)

        same_plan = self.plan_id is not None and self.plan_id == incoming_plan_id
        current_xy = (
            (current_pose[0], current_pose[1]) if current_pose is not None else None
        )
        fallback_start = current_xy if current_xy is not None else new_target_xy

        if not same_plan:
            action, has_gate = self.resolve_target_action(
                safe_zone, plan, fallback_start, new_target_xy
            )
            if plan is not None and plan.plan_id == incoming_plan_id:
                self.plan_waypoints = self.extract_plan_waypoints(plan)
            else:
                self.plan_waypoints = []
            self.plan_id = incoming_plan_id
            self.start_waypoint = int(safe_zone.last_waypoint)
            self.target_waypoint = target_wp
            self.target_progress = float(safe_zone.target_progress)
            self.start_xy = fallback_start
            self.target_xy = new_target_xy
            self.target_tolerance = new_tolerance
            self.target_yaw = new_yaw
            self.target_yaw_tolerance = new_yaw_tol
            self.active_action = action
            self.action_completed = False
            self.gate_active = has_gate
        else:
            if (
                not self.plan_waypoints
                and plan is not None
                and plan.plan_id == incoming_plan_id
            ):
                self.plan_waypoints = self.extract_plan_waypoints(plan)
            yaw_changed = False
            if self.target_yaw is not None and new_yaw is not None:
                yaw_changed = shortest_angular_distance(self.target_yaw, new_yaw) > 0.05
            elif (self.target_yaw is None) != (new_yaw is None):
                yaw_changed = True
            target_changed = (
                target_wp != self.target_waypoint
                or math.hypot(
                    new_target_xy[0] - self.target_xy[0],
                    new_target_xy[1] - self.target_xy[1],
                )
                > 1e-3
                or yaw_changed
            )
            if target_changed:
                new_start_xy = current_xy if current_xy is not None else self.target_xy
                action, has_gate = self.resolve_target_action(
                    safe_zone, plan, new_start_xy, new_target_xy
                )
                self.start_xy = new_start_xy
                self.start_waypoint = max(
                    self.start_waypoint, int(safe_zone.last_waypoint)
                )
                self.active_action = action
                self.action_completed = False
                self.gate_active = has_gate
            self.target_waypoint = target_wp
            self.target_progress = float(safe_zone.target_progress)
            self.target_xy = new_target_xy
            self.target_tolerance = new_tolerance
            self.target_yaw = new_yaw
            self.target_yaw_tolerance = new_yaw_tol

        return self.compute_progress(current_pose, previous)

    def compute_progress(
        self,
        current_pose: Optional[Tuple[float, float, float]],
        previous: Optional[Progress],
    ) -> Progress:
        out = Progress()
        if self.plan_id is None:
            return out
        out.plan_id = self.plan_id

        prev_same = previous is not None and previous.plan_id == self.plan_id
        prev_reached = int(previous.reached_waypoint) if prev_same else 0
        prev_progress = float(previous.progress) if prev_same else 0.0

        reached_wp = max(self.start_waypoint, prev_reached)
        progress = max(float(reached_wp), prev_progress)

        if self.is_holding_at_pre_waypoint():
            out.progress = float(progress)
            out.reached_waypoint = int(reached_wp)
            out.target_waypoint = int(self.target_waypoint)
            return out

        if (
            self.active_action.kind
            in (InnerActionKind.DOCK_ROBOT, InnerActionKind.UNDOCK_ROBOT)
            and self.action_completed
        ):
            out.progress = float(max(progress, self.target_progress))
            out.reached_waypoint = int(max(reached_wp, self.target_waypoint))
            out.target_waypoint = int(self.target_waypoint)
            return out

        if current_pose is not None:
            rx, ry, ryaw = current_pose
            dist_to_target = math.hypot(rx - self.target_xy[0], ry - self.target_xy[1])
            pos_reached = dist_to_target <= self.target_tolerance
            ori_reached = self.is_orientation_satisfied(ryaw)

            if pos_reached and ori_reached:
                reached_wp = max(reached_wp, self.target_waypoint)
                progress = max(progress, self.target_progress)
            elif reached_wp < self.target_waypoint:
                sx, sy = self.start_xy
                tx, ty = self.target_xy
                dx = tx - sx
                dy = ty - sy
                seg_len_sq = dx * dx + dy * dy
                start_prog = float(self.start_waypoint)
                pre_final_cap = (
                    max(self.target_progress - 1e-3, start_prog)
                    if self.target_yaw is not None
                    else self.target_progress
                )

                if pos_reached and not ori_reached:
                    pre_rot_wp = max(
                        self.start_waypoint,
                        self.target_waypoint - 1 if self.target_waypoint > 0 else 0,
                    )
                    reached_wp = max(reached_wp, pre_rot_wp)
                    seg_len = math.sqrt(seg_len_sq)
                    interp = min(
                        max(start_prog + seg_len, float(pre_rot_wp)), pre_final_cap
                    )
                    progress = min(max(progress, interp), self.target_progress)
                else:
                    target_idx = (
                        min(self.target_waypoint, len(self.plan_waypoints) - 1)
                        if self.plan_waypoints
                        else 0
                    )
                    search_start = max(reached_wp, self.start_waypoint)
                    best_seg_prog: Optional[Tuple[float, float]] = None

                    if self.plan_waypoints and search_start < target_idx:
                        for i in range(search_start, target_idx):
                            p0, prog0 = self.plan_waypoints[i]
                            p1, prog1 = self.plan_waypoints[i + 1]
                            sdx = p1[0] - p0[0]
                            sdy = p1[1] - p0[1]
                            slen_sq = sdx * sdx + sdy * sdy
                            if slen_sq <= 1e-6:
                                continue
                            t = max(
                                0.0,
                                min(
                                    1.0,
                                    ((rx - p0[0]) * sdx + (ry - p0[1]) * sdy) / slen_sq,
                                ),
                            )
                            proj_x = p0[0] + t * sdx
                            proj_y = p0[1] + t * sdy
                            dist_sq = (rx - proj_x) ** 2 + (ry - proj_y) ** 2
                            seg_prog = prog0 + t * (prog1 - prog0)
                            if (
                                best_seg_prog is None
                                or dist_sq + 1e-3 < best_seg_prog[0]
                            ):
                                best_seg_prog = (dist_sq, seg_prog)

                    if best_seg_prog is not None:
                        interp = max(
                            start_prog, min(pre_final_cap, best_seg_prog[1])
                        )
                        progress = min(max(progress, interp), self.target_progress)
                        if self.target_waypoint > self.start_waypoint + 1:
                            intermediate_wp = min(
                                int(math.floor(interp)), self.target_waypoint - 1
                            )
                            reached_wp = max(reached_wp, intermediate_wp)
                    elif seg_len_sq > 1e-6:
                        seg_len = math.sqrt(seg_len_sq)
                        t = max(
                            0.0,
                            min(1.0, ((rx - sx) * dx + (ry - sy) * dy) / seg_len_sq),
                        )
                        dist_along_seg = t * seg_len
                        wp_span = max(0.0, self.target_progress - start_prog)
                        interp = min(
                            start_prog + min(dist_along_seg, wp_span), pre_final_cap
                        )
                        progress = min(max(progress, interp), self.target_progress)
                        if self.target_waypoint > self.start_waypoint + 1:
                            intermediate_wp = min(
                                int(math.floor(interp)), self.target_waypoint - 1
                            )
                            reached_wp = max(reached_wp, intermediate_wp)

        out.progress = float(progress)
        out.reached_waypoint = int(reached_wp)
        out.target_waypoint = int(self.target_waypoint)
        return out

    def mark_target_reached(self, previous: Optional[Progress]) -> Optional[Progress]:
        self.action_completed = True
        if self.plan_id is None:
            return None
        prev_same = previous is not None and previous.plan_id == self.plan_id
        prev_reached = int(previous.reached_waypoint) if prev_same else 0
        prev_progress = float(previous.progress) if prev_same else 0.0

        out = Progress()
        out.plan_id = self.plan_id
        out.target_waypoint = int(self.target_waypoint)

        if self.gate_active:
            out.reached_waypoint = int(max(self.start_waypoint, prev_reached))
            out.progress = float(max(float(out.reached_waypoint), prev_progress))
            return out

        out.reached_waypoint = int(max(prev_reached, self.target_waypoint))
        out.progress = float(max(prev_progress, self.target_progress))
        return out


def build_progress_status(
    pose: PoseStamped,
    mode: int,
    progress: Optional[Progress],
    safe_zone_id: Optional[SafeZoneId],
) -> Progress:
    msg = Progress()
    if progress is not None:
        msg.progress = float(progress.progress)
        msg.reached_waypoint = int(progress.reached_waypoint)
        msg.target_waypoint = int(progress.target_waypoint)
        msg.reached_keys = list(progress.reached_keys)
        msg.plan_id = progress.plan_id
    msg.pose = pose
    msg.mode = int(mode)
    msg.safe_zone_id = [safe_zone_id] if safe_zone_id is not None else []
    return msg


def is_safe_zone_newer(candidate: SafeZoneId, current: SafeZoneId) -> bool:
    if list(candidate.plan_id.destination_session.uuid) != list(
        current.plan_id.destination_session.uuid
    ):
        return True
    if candidate.plan_id.plan_version != current.plan_id.plan_version:
        return candidate.plan_id.plan_version > current.plan_id.plan_version
    return candidate.safe_zone_version > current.safe_zone_version


def should_preempt_goal(
    candidate_id: SafeZoneId,
    candidate_action: InnerNavigationAction,
    current_id: SafeZoneId,
    current_action: InnerNavigationAction,
) -> bool:
    return is_safe_zone_newer(candidate_id, current_id) or (
        candidate_id == current_id and candidate_action != current_action
    )


def generate_random_uuid() -> RosUuid:
    u = RosUuid()
    u.uuid = list(uuid.uuid4().bytes)
    return u


def pose_stamped_to_destination_goal(
    pose: PoseStamped, session: RosUuid
) -> DestinationGoal:
    x = float(pose.pose.position.x)
    y = float(pose.pose.position.y)
    q = pose.pose.orientation
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw = float(math.atan2(siny_cosp, cosy_cosp))

    region = Region()
    region.points = [x, y]
    region.hint = Region.HINT_POINT

    target_ori = TargetOrientation()
    target_ori.orientation_radians = yaw
    target_ori.spread_radians = 0.0
    target_ori.tolerance_radians = 0.0

    target_region = TargetRegion()
    target_region.tolerance = 0.5
    target_region.region = region
    target_region.orientations = [target_ori]

    constraints = DestinationConstraints()
    constraints.regions = [target_region]

    goal = DestinationGoal()
    goal.one_of = [constraints]
    goal.session = session
    return goal
