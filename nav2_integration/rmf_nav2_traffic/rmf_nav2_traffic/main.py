"""Main entry point for Python nav2_simple_commander implementation of rmf_nav2_traffic."""

from collections import deque
from dataclasses import dataclass, field
import math
import os
import sys
import threading
import time
from typing import Deque, List, Optional, Tuple

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TwistWithCovariance
from nav2_msgs.action import DockRobot, NavigateToPose, UndockRobot
from nav2_msgs.msg import Costmap
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import Odometry
import rclpy
from rclpy._rclpy_pybind11 import GoalEvent
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rmf_prototype_msgs.msg import (
    DestinationGoal,
    Participant,
    ParticipantList,
    Plan,
    PlanError,
    Progress,
    SafeZone,
    SafeZoneId,
)
from std_msgs.msg import Empty as EmptyMsg, Header as HeaderMsg
from unique_identifier_msgs.msg import UUID as RosUuid

from rmf_nav2_traffic.safe_zone_tracker import (
    build_progress_status,
    costmap_content_eq,
    CostmapHandshakeState,
    CurrentSafeZone,
    generate_random_uuid,
    InnerActionKind,
    InnerNavigationAction,
    InnerNavigationTarget,
    is_valid_safe_zone,
    LocalPlanRegistry,
    next_target,
    PendingNavigationTarget,
    pose_stamped_to_destination_goal,
    ProgressTracker,
    seq_to_stamp,
    should_preempt_goal,
    stamp_to_seq,
)


class InnerSimpleNavigator(BasicNavigator):
    """Subclass of BasicNavigator operating in `/<robot>/inner` with non-blocking async hooks."""

    def __init__(self, agent_name: str, feedback_cb) -> None:
        super().__init__(
            node_name=f'{agent_name}_inner_navigator',
            namespace=f'{agent_name}/inner',
        )
        self._external_feedback_cb = feedback_cb
        self._active_seq: int = 0
        self._cancelled_up_to_seq: int = 0
        self._terminal_status: Optional[int] = None

    def start_navigate_to_pose(self, seq: int, pose: PoseStamped) -> bool:
        if not self.nav_to_pose_client.server_is_ready():
            return False
        self._active_seq = seq
        self._terminal_status = None
        self.goal_handle = None
        self.result_future = None

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        send_future = self.nav_to_pose_client.send_goal_async(
            goal_msg,
            feedback_callback=lambda fb: self._on_feedback(seq, fb),
        )
        send_future.add_done_callback(lambda fut: self._on_goal_response(seq, fut))
        return True

    def start_dock_robot(
        self,
        seq: int,
        action: InnerNavigationAction,
        dock_pose: PoseStamped,
    ) -> bool:
        if not self.docking_client.server_is_ready():
            return False
        self._active_seq = seq
        self._terminal_status = None
        self.goal_handle = None
        self.result_future = None

        goal_msg = DockRobot.Goal()
        goal_msg.use_dock_id = action.use_dock_id
        goal_msg.dock_id = action.dock_id
        goal_msg.dock_type = action.dock_type
        goal_msg.navigate_to_staging_pose = action.navigate_to_staging_pose
        goal_msg.max_staging_time = 1000.0
        if not action.use_dock_id:
            goal_msg.dock_pose = dock_pose

        send_future = self.docking_client.send_goal_async(goal_msg)
        send_future.add_done_callback(lambda fut: self._on_goal_response(seq, fut))
        return True

    def start_undock_robot(self, seq: int, dock_type: str) -> bool:
        if not self.undocking_client.server_is_ready():
            return False
        self._active_seq = seq
        self._terminal_status = None
        self.goal_handle = None
        self.result_future = None

        goal_msg = UndockRobot.Goal()
        goal_msg.dock_type = dock_type
        goal_msg.max_undocking_time = 30.0

        send_future = self.undocking_client.send_goal_async(goal_msg)
        send_future.add_done_callback(lambda fut: self._on_goal_response(seq, fut))
        return True

    def cancel_active_task(self) -> None:
        self._active_seq += 1
        self._cancelled_up_to_seq = self._active_seq
        self._terminal_status = None
        if self.goal_handle is not None:
            try:
                self.goal_handle.cancel_goal_async()
            except Exception:
                pass
        self.goal_handle = None
        self.result_future = None

    def poll_task_result(self) -> Optional[TaskResult]:
        if self._terminal_status is None:
            return None
        status = self._terminal_status
        self._terminal_status = None
        self.status = status
        return self.getResult()

    def _on_feedback(self, seq: int, fb_msg) -> None:
        if seq != self._active_seq:
            return
        self.feedback = fb_msg.feedback
        if self._external_feedback_cb is not None:
            self._external_feedback_cb(fb_msg.feedback)

    def _on_goal_response(self, seq: int, future) -> None:
        try:
            handle = future.result()
        except Exception:
            if seq == self._active_seq:
                self._terminal_status = GoalStatus.STATUS_ABORTED
            return

        if handle is None or not handle.accepted:
            if seq == self._active_seq:
                self._terminal_status = GoalStatus.STATUS_ABORTED
            return

        if seq != self._active_seq:
            if seq <= self._cancelled_up_to_seq:
                try:
                    handle.cancel_goal_async()
                except Exception:
                    pass
            return

        self.goal_handle = handle
        self.result_future = handle.get_result_async()
        self.result_future.add_done_callback(lambda fut: self._on_result(seq, fut))

    def _on_result(self, seq: int, future) -> None:
        if seq != self._active_seq:
            return
        try:
            res = future.result()
            status = res.status
            if status == GoalStatus.STATUS_SUCCEEDED and hasattr(
                res.result, 'success'
            ):
                if not res.result.success:
                    status = GoalStatus.STATUS_ABORTED
            self._terminal_status = status
        except Exception:
            self._terminal_status = GoalStatus.STATUS_ABORTED


@dataclass
class ActiveInnerGoal:
    seq: int
    safe_zone_id: SafeZoneId
    action: InnerNavigationAction
    target: InnerNavigationTarget
    superseded: bool = False


@dataclass
class ActiveOuterSession:
    session: RosUuid
    target_pose: PoseStamped
    threshold: float = 0.5
    superseded_sessions: List[RosUuid] = field(default_factory=list)
    seen_matching_safe_zone: bool = False
    feedback_queue: Deque[NavigateToPose.Feedback] = field(
        default_factory=lambda: deque(maxlen=32)
    )
    completed: bool = False
    cancelled: bool = False


class AgentTrafficController:
    """Manages one robot's RMF SafeZone/Plan execution and outer NavigateToPose server."""

    def __init__(
        self,
        index: int,
        name: str,
        node: Node,
        cb_group: ReentrantCallbackGroup,
    ) -> None:
        self.index = index
        self.name = name
        self.node = node
        self.cb_group = cb_group
        self.lock = threading.Lock()

        self.navigator = InnerSimpleNavigator(name, self._on_inner_feedback)

        self.localized = False
        self.latest_amcl: Optional[PoseWithCovarianceStamped] = None
        self.latest_pose: Optional[PoseStamped] = None
        self.current_pose_xy_yaw: Optional[Tuple[float, float, float]] = None

        self.current_safe_zone = CurrentSafeZone()
        self.plan_registry = LocalPlanRegistry()
        self.costmap_handshake = CostmapHandshakeState()
        self.progress_tracker = ProgressTracker()
        self.last_progress: Optional[Progress] = None
        self.last_published_status: Optional[Progress] = None

        self.cancelled_sessions: List[List[int]] = []

        self.incoming_amcl_poses: Deque[PoseWithCovarianceStamped] = deque(maxlen=32)
        self.incoming_plans: Deque[Plan] = deque(maxlen=32)
        self.incoming_safe_zones: Deque[SafeZone] = deque(maxlen=32)
        self.gate_release_pending = False
        self.ready_inner_targets: Deque[InnerNavigationTarget] = deque()

        self.next_inner_seq = 1
        self.active_inner_goal: Optional[ActiveInnerGoal] = None
        self.last_inner_action_kind: Optional[InnerActionKind] = None
        self.active_outer_session: Optional[ActiveOuterSession] = None

        default_qos = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        transient_qos = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        self.pub_odom = node.create_publisher(
            Odometry, f'/{name}/odom', qos_profile=default_qos
        )
        self.pub_costmap = node.create_publisher(
            Costmap,
            f'/{name}/inner/global_costmap/plan/costmap',
            qos_profile=transient_qos,
        )
        self.pub_progress = node.create_publisher(
            Progress, f'/{name}/plan/progress', qos_profile=transient_qos
        )
        self.pub_error = node.create_publisher(
            PlanError, f'/{name}/plan/error', qos_profile=transient_qos
        )
        self.pub_destination_goal = node.create_publisher(
            DestinationGoal, f'/{name}/destination/goal', qos_profile=transient_qos
        )
        self.pub_feedback_pose = node.create_publisher(
            PoseStamped, f'/{name}/inner/feedback_pose', qos_profile=default_qos
        )

        self.sub_amcl = node.create_subscription(
            PoseWithCovarianceStamped,
            f'/{name}/inner/amcl_pose',
            self._on_amcl,
            qos_profile=transient_qos,
            callback_group=cb_group,
        )
        self.sub_plan = node.create_subscription(
            Plan,
            f'/{name}/plan',
            self._on_plan,
            qos_profile=transient_qos,
            callback_group=cb_group,
        )
        self.sub_safe_zone = node.create_subscription(
            SafeZone,
            f'/{name}/plan/safe_zone',
            self._on_safe_zone,
            qos_profile=transient_qos,
            callback_group=cb_group,
        )
        self.sub_gate_release = node.create_subscription(
            EmptyMsg,
            f'/{name}/plan/gate_release',
            self._on_gate_release,
            qos_profile=transient_qos,
            callback_group=cb_group,
        )
        self.sub_costmap_ack = node.create_subscription(
            HeaderMsg,
            f'/{name}/inner/global_costmap/plan/costmap_updated',
            self._on_costmap_ack,
            qos_profile=transient_qos,
            callback_group=cb_group,
        )

        self.outer_server = ActionServer(
            node,
            NavigateToPose,
            f'/{name}/navigate_to_pose',
            execute_callback=self._execute_outer_goal,
            goal_callback=lambda _: GoalResponse.ACCEPT,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=cb_group,
        )

    @staticmethod
    def _extract_xy_yaw(ps: PoseStamped) -> Tuple[float, float, float]:
        x = float(ps.pose.position.x)
        y = float(ps.pose.position.y)
        q = ps.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = float(math.atan2(siny_cosp, cosy_cosp))
        return (x, y, yaw)

    def _on_amcl(self, msg: PoseWithCovarianceStamped) -> None:
        with self.lock:
            self.incoming_amcl_poses.append(msg)

    def _on_plan(self, msg: Plan) -> None:
        with self.lock:
            self.incoming_plans.append(msg)

    def _on_safe_zone(self, msg: SafeZone) -> None:
        with self.lock:
            self.incoming_safe_zones.append(msg)

    def _on_gate_release(self, _msg: EmptyMsg) -> None:
        with self.lock:
            self.gate_release_pending = True

    def _on_costmap_ack(self, msg: HeaderMsg) -> None:
        with self.lock:
            seq = stamp_to_seq(msg.stamp)
            if seq > self.costmap_handshake.acked_seq:
                self.costmap_handshake.acked_seq = seq

    def _on_inner_feedback(self, feedback: NavigateToPose.Feedback) -> None:
        with self.lock:
            self.latest_pose = feedback.current_pose
            if not self.localized:
                self.current_pose_xy_yaw = self._extract_xy_yaw(feedback.current_pose)
            self.pub_feedback_pose.publish(feedback.current_pose)
            if self.active_outer_session is not None:
                self.active_outer_session.feedback_queue.append(feedback)

    def _is_session_cancelled_locked(self, session: RosUuid) -> bool:
        u = list(session.uuid)
        return u in self.cancelled_sessions

    def _mark_session_cancelled_locked(self, session: RosUuid) -> None:
        u = list(session.uuid)
        if u not in self.cancelled_sessions:
            self.cancelled_sessions.append(u)

    def _cancel_active_inner_goal_locked(self) -> None:
        self.costmap_handshake.pending_target = None
        self.ready_inner_targets.clear()
        if self.active_inner_goal is not None:
            self.active_inner_goal.superseded = True
            self.active_inner_goal = None
        self.navigator.cancel_active_task()

    def _current_mode_locked(self) -> int:
        if self.progress_tracker.is_holding_at_pre_waypoint():
            return Progress.MODE_ACTION
        if self.active_outer_session is not None or self.active_inner_goal is not None:
            return Progress.MODE_NAVIGATING
        return Progress.MODE_IDLE

    def _publish_progress_status_locked(
        self, override_mode: Optional[int] = None
    ) -> None:
        pose = self.latest_pose if self.latest_pose is not None else PoseStamped()
        if self.localized and self.latest_amcl is not None:
            pose = PoseStamped()
            pose.header = self.latest_amcl.header
            pose.pose = self.latest_amcl.pose.pose
        mode = (
            override_mode
            if override_mode is not None
            else self._current_mode_locked()
        )
        sz_id = (
            self.current_safe_zone.value.id
            if self.current_safe_zone.value is not None
            else None
        )
        status_msg = build_progress_status(pose, mode, self.last_progress, sz_id)
        self.last_progress = status_msg
        if self.last_published_status != status_msg:
            self.pub_progress.publish(status_msg)
            self.last_published_status = status_msg

    @staticmethod
    def _terminate_outer_handle_canceled(goal_handle) -> None:
        if goal_handle.status == GoalStatus.STATUS_EXECUTING:
            try:
                goal_handle._update_state(GoalEvent.CANCEL_GOAL)
            except Exception:
                pass
        if goal_handle.status == GoalStatus.STATUS_CANCELING:
            goal_handle.canceled()

    def _execute_outer_goal(self, goal_handle) -> NavigateToPose.Result:
        session_uuid = generate_random_uuid()
        goal_pose = goal_handle.request.pose
        dest_goal = pose_stamped_to_destination_goal(goal_pose, session_uuid)

        my_session = ActiveOuterSession(
            session=session_uuid,
            target_pose=goal_pose,
            threshold=0.5,
        )

        with self.lock:
            self.pub_destination_goal.publish(dest_goal)

            if goal_handle.is_cancel_requested:
                self._mark_session_cancelled_locked(session_uuid)
                self._cancel_active_inner_goal_locked()
                self._terminate_outer_handle_canceled(goal_handle)
                return NavigateToPose.Result()

            should_cancel_inner = False
            if self.active_outer_session is not None:
                prev = self.active_outer_session
                prev.cancelled = True
                for s in prev.superseded_sessions:
                    my_session.superseded_sessions.append(s)
                    self._mark_session_cancelled_locked(s)
                my_session.superseded_sessions.append(prev.session)
                self._mark_session_cancelled_locked(prev.session)
                should_cancel_inner = True

            if self.current_safe_zone.value is not None:
                cur_sz_session = (
                    self.current_safe_zone.value.id.plan_id.destination_session
                )
                if list(cur_sz_session.uuid) != list(session_uuid.uuid):
                    my_session.superseded_sessions.append(cur_sz_session)
                    self._mark_session_cancelled_locked(cur_sz_session)
                    should_cancel_inner = True

            if should_cancel_inner:
                self._cancel_active_inner_goal_locked()

            self.active_outer_session = my_session

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                with self.lock:
                    for s in my_session.superseded_sessions:
                        self._mark_session_cancelled_locked(s)
                    self._mark_session_cancelled_locked(my_session.session)
                    if self.active_outer_session is my_session:
                        self.active_outer_session = None
                        self._cancel_active_inner_goal_locked()
                self._terminate_outer_handle_canceled(goal_handle)
                return NavigateToPose.Result()

            with self.lock:
                if my_session.cancelled:
                    self._terminate_outer_handle_canceled(goal_handle)
                    return NavigateToPose.Result()
                if my_session.completed:
                    goal_handle.succeed()
                    return NavigateToPose.Result()
                feedbacks = list(my_session.feedback_queue)
                my_session.feedback_queue.clear()

            for fb in feedbacks:
                goal_handle.publish_feedback(fb)

            time.sleep(0.02)

        self._terminate_outer_handle_canceled(goal_handle)
        return NavigateToPose.Result()

    def tick(self) -> None:
        with self.lock:
            self._tick_locked()

    def _tick_locked(self) -> None:
        # 1. Drain incoming AMCL poses -> publish Odometry + Progress
        received_amcl = False
        while self.incoming_amcl_poses:
            self.latest_amcl = self.incoming_amcl_poses.popleft()
            self.localized = True
            received_amcl = True

        if self.localized and received_amcl and self.latest_amcl is not None:
            ps = PoseStamped()
            ps.header = self.latest_amcl.header
            ps.pose = self.latest_amcl.pose.pose
            self.latest_pose = ps
            self.current_pose_xy_yaw = self._extract_xy_yaw(ps)

            odom = Odometry()
            odom.header = self.latest_amcl.header
            odom.child_frame_id = 'odom'
            odom.pose = self.latest_amcl.pose
            odom.twist = TwistWithCovariance()
            self.pub_odom.publish(odom)

            if self.progress_tracker.plan_id is not None:
                self.last_progress = self.progress_tracker.compute_progress(
                    self.current_pose_xy_yaw, self.last_progress
                )
            self._publish_progress_status_locked()

        # 2. Drain incoming plans
        while self.incoming_plans:
            plan = self.incoming_plans.popleft()
            prev_action = self.progress_tracker.active_action
            cur_sz = self.current_safe_zone.value
            self.progress_tracker.set_plan(plan, cur_sz)
            new_action = self.progress_tracker.active_action

            if (
                prev_action != new_action
                and cur_sz is not None
                and cur_sz.id.plan_id == plan.plan_id
            ):
                if (
                    self.costmap_handshake.pending_target is not None
                    and self.costmap_handshake.pending_target.target.safe_zone_id.plan_id
                    == plan.plan_id
                ):
                    self.costmap_handshake.pending_target.target.action = new_action
                else:
                    target_pose = next_target(cur_sz)
                    if target_pose is not None:
                        tx, ty, tyaw = target_pose
                        self.ready_inner_targets.append(
                            InnerNavigationTarget(
                                agent=self.index,
                                safe_zone_id=cur_sz.id,
                                x=tx,
                                y=ty,
                                yaw=tyaw,
                                action=new_action,
                            )
                        )
            self.plan_registry.insert(plan)

        # 3. Gate release
        if self.gate_release_pending:
            self.gate_release_pending = False
            self.progress_tracker.release_gate()
            if self.progress_tracker.plan_id is not None:
                self.last_progress = self.progress_tracker.compute_progress(
                    self.current_pose_xy_yaw, self.last_progress
                )
                self._publish_progress_status_locked(Progress.MODE_NAVIGATING)

        # 4. Drain incoming SafeZones (take latest)
        latest_sz: Optional[SafeZone] = None
        while self.incoming_safe_zones:
            latest_sz = self.incoming_safe_zones.popleft()

        if latest_sz is not None:
            if not self.current_safe_zone.matches(latest_sz) and is_valid_safe_zone(
                latest_sz
            ):
                target_pose = next_target(latest_sz)
                if target_pose is not None:
                    target_x, target_y, target_yaw = target_pose
                    incoming_session = latest_sz.id.plan_id.destination_session

                    skip_safe_zone = False
                    if self._is_session_cancelled_locked(incoming_session):
                        skip_safe_zone = True
                    elif self.active_outer_session is not None:
                        outer = self.active_outer_session
                        if list(outer.session.uuid) == list(incoming_session.uuid):
                            outer.superseded_sessions.clear()
                            outer.seen_matching_safe_zone = True
                        else:
                            in_superseded = any(
                                list(s.uuid) == list(incoming_session.uuid)
                                for s in outer.superseded_sessions
                            )
                            if in_superseded:
                                skip_safe_zone = True
                            else:
                                for s in outer.superseded_sessions:
                                    self._mark_session_cancelled_locked(s)
                                self._mark_session_cancelled_locked(outer.session)
                                outer.cancelled = True
                                self.active_outer_session = None

                    if not skip_safe_zone:
                        should_update = self.current_safe_zone.should_update_target(
                            latest_sz
                        )
                        costmap_changed = (
                            self.costmap_handshake.last_published_costmap is None
                            or not costmap_content_eq(
                                self.costmap_handshake.last_published_costmap,
                                latest_sz.costmap,
                            )
                        )

                        published_costmap: Optional[Tuple[int, Costmap]] = None
                        if should_update or costmap_changed:
                            self.costmap_handshake.next_seq += 1
                            seq = self.costmap_handshake.next_seq
                            costmap = Costmap()
                            costmap.header = latest_sz.costmap.header
                            costmap.header.stamp = seq_to_stamp(seq)
                            costmap.metadata = latest_sz.costmap.metadata
                            costmap.data = list(latest_sz.costmap.data)
                            self.pub_costmap.publish(costmap)
                            self.costmap_handshake.last_published_costmap = costmap
                            published_costmap = (seq, costmap)

                        current_pose = (
                            self.current_pose_xy_yaw if self.localized else None
                        )
                        matching_plan = self.plan_registry.get(latest_sz.id.plan_id)
                        base_progress = self.progress_tracker.on_safe_zone_with_plan(
                            latest_sz,
                            matching_plan,
                            current_pose,
                            self.last_progress,
                        )
                        self.last_progress = base_progress
                        mode = (
                            Progress.MODE_ACTION
                            if self.progress_tracker.is_holding_at_pre_waypoint()
                            else Progress.MODE_NAVIGATING
                        )

                        if not should_update:
                            if (
                                self.costmap_handshake.pending_target is not None
                                and published_costmap is not None
                            ):
                                self.costmap_handshake.pending_target.costmap = (
                                    published_costmap[1]
                                )
                            self._publish_progress_status_locked(mode)
                        else:
                            if published_costmap is not None:
                                seq, costmap = published_costmap
                            else:
                                seq = self.costmap_handshake.next_seq
                                costmap = Costmap()
                                costmap.header = latest_sz.costmap.header
                                costmap.header.stamp = seq_to_stamp(seq)
                                costmap.metadata = latest_sz.costmap.metadata
                                costmap.data = list(latest_sz.costmap.data)

                            action = self.progress_tracker.active_action
                            if self.active_inner_goal is not None:
                                self.active_inner_goal.superseded = True
                            self.current_safe_zone.update(latest_sz)
                            self._publish_progress_status_locked(mode)

                            nav_target = InnerNavigationTarget(
                                agent=self.index,
                                safe_zone_id=latest_sz.id,
                                x=target_x,
                                y=target_y,
                                yaw=target_yaw,
                                action=action,
                            )
                            self.costmap_handshake.pending_target = (
                                PendingNavigationTarget(
                                    required_seq=seq,
                                    target=nav_target,
                                    target_xy_yaw=(target_x, target_y, target_yaw),
                                    costmap=costmap,
                                    last_published=time.monotonic(),
                                )
                            )

        # 5. Costmap handshake check
        if self.costmap_handshake.pending_target is not None:
            pending = self.costmap_handshake.pending_target
            if self._is_session_cancelled_locked(
                pending.target.safe_zone_id.plan_id.destination_session
            ):
                self.costmap_handshake.pending_target = None
            elif self.costmap_handshake.acked_seq >= pending.required_seq:
                self.ready_inner_targets.append(pending.target)
                self.costmap_handshake.pending_target = None
            else:
                now = time.monotonic()
                if now - pending.last_published >= 0.5:
                    self.pub_costmap.publish(pending.costmap)
                    pending.last_published = now

        # 6. Poll inner SimpleCommander navigator completion
        task_result = self.navigator.poll_task_result()
        if task_result is not None and self.active_inner_goal is not None:
            finished = self.active_inner_goal
            self.active_inner_goal = None

            if task_result == TaskResult.SUCCEEDED and not finished.superseded:
                matches_sz = (
                    self.current_safe_zone.value is None
                    or self.current_safe_zone.value.id == finished.safe_zone_id
                )
                if (
                    matches_sz
                    and self.progress_tracker.plan_id is not None
                    and self.progress_tracker.plan_id == finished.safe_zone_id.plan_id
                ):
                    if finished.action.kind in (
                        InnerActionKind.DOCK_ROBOT,
                        InnerActionKind.UNDOCK_ROBOT,
                    ):
                        self.progress_tracker.mark_action_completed()
                    if not self.progress_tracker.is_target_pose_satisfied(
                        self.current_pose_xy_yaw
                    ):
                        self.ready_inner_targets.appendleft(finished.target)
                    else:
                        prog = self.progress_tracker.mark_target_reached(
                            self.last_progress
                        )
                        if prog is not None:
                            self.last_progress = prog
                            mode = (
                                Progress.MODE_ACTION
                                if self.progress_tracker.is_holding_at_pre_waypoint()
                                else Progress.MODE_NAVIGATING
                            )
                            self._publish_progress_status_locked(mode)
            elif task_result == TaskResult.FAILED and not finished.superseded:
                if (
                    self.current_safe_zone.value is None
                    or self.current_safe_zone.value.id == finished.safe_zone_id
                ):
                    err = PlanError()
                    err.plan_id = finished.safe_zone_id.plan_id
                    err.error.code = PlanError.CODE_PATH_BLOCKED
                    err.error.message = (
                        'Nav2 goal aborted: path blocked within safe zone'
                    )
                    self.pub_error.publish(err)

        # 7. Dispatch ready inner targets via SimpleCommander navigator
        while self.ready_inner_targets:
            target = self.ready_inner_targets[0]
            if self._is_session_cancelled_locked(
                target.safe_zone_id.plan_id.destination_session
            ):
                self.ready_inner_targets.popleft()
                continue

            if self.active_inner_goal is not None:
                if not should_preempt_goal(
                    target.safe_zone_id,
                    target.action,
                    self.active_inner_goal.safe_zone_id,
                    self.active_inner_goal.action,
                ):
                    self.ready_inner_targets.popleft()
                    continue
                existing_is_nav = (
                    self.active_inner_goal.action.kind
                    == InnerActionKind.NAVIGATE_TO_POSE
                )
                next_is_nav = (
                    target.action.kind == InnerActionKind.NAVIGATE_TO_POSE
                )
                self.active_inner_goal.superseded = True
                self.active_inner_goal = None
                if not (existing_is_nav and next_is_nav):
                    self.navigator.cancel_active_task()
            elif (
                self.last_inner_action_kind is not None
                and self.last_inner_action_kind != target.action.kind
            ):
                self.navigator.cancel_active_task()

            if self._dispatch_inner_target_locked(target):
                self.last_inner_action_kind = target.action.kind
                self.ready_inner_targets.popleft()
            else:
                break

        # 8. Check outer goal completion
        if self.active_outer_session is not None:
            outer = self.active_outer_session
            inner_idle = (
                self.active_inner_goal is None
                and self.costmap_handshake.pending_target is None
                and not self.ready_inner_targets
            )
            if inner_idle and self.latest_pose is not None:
                cx = float(self.latest_pose.pose.position.x)
                cy = float(self.latest_pose.pose.position.y)
                tx = float(outer.target_pose.pose.position.x)
                ty = float(outer.target_pose.pose.position.y)
                if math.hypot(cx - tx, cy - ty) < outer.threshold:
                    outer.completed = True
                    self.active_outer_session = None

                    if self.current_safe_zone.value is not None:
                        sz = self.current_safe_zone.value
                        target_wp = (
                            int(sz.target_waypoint[0])
                            if len(sz.target_waypoint) > 0
                            else 0
                        )
                        base_prog = Progress()
                        base_prog.progress = float(sz.target_progress)
                        base_prog.reached_waypoint = target_wp
                        base_prog.target_waypoint = target_wp
                        base_prog.plan_id = sz.id.plan_id
                        self.last_progress = base_prog
                        self._publish_progress_status_locked(Progress.MODE_IDLE)

    def _dispatch_inner_target_locked(self, target: InnerNavigationTarget) -> bool:
        seq = self.next_inner_seq
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x = float(target.x)
        pose.pose.position.y = float(target.y)
        pose.pose.orientation.z = math.sin(target.yaw * 0.5)
        pose.pose.orientation.w = math.cos(target.yaw * 0.5)

        if target.action.kind == InnerActionKind.NAVIGATE_TO_POSE:
            if not self.navigator.start_navigate_to_pose(seq, pose):
                return False
        elif target.action.kind == InnerActionKind.DOCK_ROBOT:
            if not self.navigator.start_dock_robot(seq, target.action, pose):
                return False
        else:
            if not self.navigator.start_undock_robot(seq, target.action.dock_type):
                return False

        self.next_inner_seq += 1
        self.active_inner_goal = ActiveInnerGoal(
            seq=seq,
            safe_zone_id=target.safe_zone_id,
            action=target.action,
            target=target,
        )
        return True


def parse_agent_names(argv: List[str]) -> List[str]:
    env_agents = os.environ.get('RMF_NAV2_AGENTS', '').strip()
    if env_agents:
        names = [s.strip() for s in env_agents.split(',') if s.strip()]
        if names:
            return names

    for i, arg in enumerate(argv):
        if arg == '--agents' and i + 1 < len(argv):
            names = [s.strip() for s in argv[i + 1].split(',') if s.strip()]
            if names:
                return names
        elif arg.startswith('--agents='):
            names = [s.strip() for s in arg[len('--agents='):].split(',') if s.strip()]
            if names:
                return names

    return ['robot0', 'robot1']


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    raw_argv = sys.argv[1:] if args is None else args
    agent_names = parse_agent_names(raw_argv)

    node = Node('nav_traffic_node')
    cb_group = ReentrantCallbackGroup()

    transient_qos = QoSProfile(
        depth=10,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
        reliability=ReliabilityPolicy.RELIABLE,
    )
    pub_discovery = node.create_publisher(
        ParticipantList, '/destination/discovery', qos_profile=transient_qos
    )
    discovery_msg = ParticipantList()
    for name in agent_names:
        p = Participant()
        p.name = name
        discovery_msg.participants.append(p)
    pub_discovery.publish(discovery_msg)

    controllers = [
        AgentTrafficController(idx, name, node, cb_group)
        for idx, name in enumerate(agent_names)
    ]

    def tick_all() -> None:
        for ctrl in controllers:
            ctrl.tick()

    node.create_timer(0.02, tick_all, callback_group=cb_group)

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    for ctrl in controllers:
        executor.add_node(ctrl.navigator)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        for ctrl in controllers:
            ctrl.navigator.destroy_node()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
