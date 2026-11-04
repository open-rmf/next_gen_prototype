#ifndef RMF_NAV2_TRAFFIC__BT_NODES_HPP_
#define RMF_NAV2_TRAFFIC__BT_NODES_HPP_

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/condition_node.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav2_msgs/action/dock_robot.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav2_msgs/action/undock_robot.hpp>
#include <nav2_msgs/msg/costmap.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rmf_prototype_msgs/msg/plan.hpp>
#include <rmf_prototype_msgs/msg/plan_error.hpp>
#include <rmf_prototype_msgs/msg/progress.hpp>
#include <rmf_prototype_msgs/msg/safe_zone.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/header.hpp>

#include "rmf_nav2_traffic/safe_zone_tracker.hpp"

namespace rmf_nav2_traffic
{

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using DockRobot = nav2_msgs::action::DockRobot;
using UndockRobot = nav2_msgs::action::UndockRobot;
using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;
using GoalHandleDock = rclcpp_action::ClientGoalHandle<DockRobot>;
using GoalHandleUndock = rclcpp_action::ClientGoalHandle<UndockRobot>;
using PoseWithCovarianceStamped = geometry_msgs::msg::PoseWithCovarianceStamped;
using EmptyMsg = std_msgs::msg::Empty;
using HeaderMsg = std_msgs::msg::Header;

enum class GoalState
{
  Idle,
  WaitingAcceptance,
  Executing,
};

struct ActiveNav2Goal
{
  std::uint64_t seq = 0;
  SafeZoneId safe_zone_id;
  InnerNavigationAction action;
  InnerNavigationTarget target;
  bool superseded = false;
  GoalState state = GoalState::Idle;
  GoalHandleNav::SharedPtr nav_handle;
  GoalHandleDock::SharedPtr dock_handle;
  GoalHandleUndock::SharedPtr undock_handle;
  std::optional<rclcpp_action::ResultCode> terminal_code;
};

struct AgentContext
{
  std::size_t index = 0;
  std::string name;
  rclcpp::Node::SharedPtr node;

  mutable std::mutex mutex;

  // Latest pose from /<robot>/amcl_pose or Nav2 action feedback
  bool localized = false;
  PoseWithCovarianceStamped latest_amcl;
  std::optional<PoseStamped> latest_pose;
  std::optional<std::tuple<float, float, float>> current_pose_xy_yaw;

  // Domain trackers
  CurrentSafeZone current_safe_zone;
  LocalPlanRegistry plan_registry;
  CostmapHandshakeState costmap_handshake;
  ProgressTracker progress_tracker;
  std::optional<Progress> last_progress;
  std::optional<std::uint8_t> pending_mode_override;
  bool progress_dirty = false;

  // Incoming message queues
  std::deque<PoseWithCovarianceStamped> incoming_amcl_poses;
  std::deque<Plan> incoming_plans;
  std::deque<SafeZone> incoming_safe_zones;
  bool gate_release_pending = false;
  std::deque<InnerNavigationTarget> ready_targets;

  // Nav2 action state
  std::uint64_t next_goal_seq = 1;
  std::uint64_t cancelled_up_to_seq = 0;
  std::optional<InnerActionKind> last_action_kind;
  std::optional<ActiveNav2Goal> active_goal;

  // ROS Publishers
  rclcpp::Publisher<Costmap>::SharedPtr pub_costmap;
  rclcpp::Publisher<Progress>::SharedPtr pub_progress;
  rclcpp::Publisher<PlanError>::SharedPtr pub_error;

  // ROS Subscriptions
  rclcpp::Subscription<PoseWithCovarianceStamped>::SharedPtr sub_amcl;
  rclcpp::Subscription<Plan>::SharedPtr sub_plan;
  rclcpp::Subscription<SafeZone>::SharedPtr sub_safe_zone;
  rclcpp::Subscription<EmptyMsg>::SharedPtr sub_gate_release;
  rclcpp::Subscription<HeaderMsg>::SharedPtr sub_costmap_ack;

  // ROS Action Clients
  rclcpp_action::Client<NavigateToPose>::SharedPtr client_nav;
  rclcpp_action::Client<DockRobot>::SharedPtr client_dock;
  rclcpp_action::Client<UndockRobot>::SharedPtr client_undock;

  AgentContext(std::size_t idx, std::string agent_name, rclcpp::Node::SharedPtr ros_node)
  : index(idx), name(std::move(agent_name)), node(std::move(ros_node))
  {
    const auto transient_qos = rclcpp::QoS(10).transient_local().reliable();

    pub_costmap = node->create_publisher<Costmap>(
      "/" + name + "/global_costmap/plan/costmap", transient_qos);
    pub_progress = node->create_publisher<Progress>("/" + name + "/plan/progress", transient_qos);
    pub_error = node->create_publisher<PlanError>("/" + name + "/plan/error", transient_qos);

    sub_amcl = node->create_subscription<PoseWithCovarianceStamped>(
      "/" + name + "/amcl_pose", transient_qos,
      [this](const PoseWithCovarianceStamped::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex);
        if (incoming_amcl_poses.size() >= 32) {
          incoming_amcl_poses.pop_front();
        }
        incoming_amcl_poses.push_back(*msg);
      });

    sub_plan = node->create_subscription<Plan>(
      "/" + name + "/plan", transient_qos, [this](const Plan::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex);
        if (incoming_plans.size() >= 32) {
          incoming_plans.pop_front();
        }
        incoming_plans.push_back(*msg);
      });

    sub_safe_zone = node->create_subscription<SafeZone>(
      "/" + name + "/plan/safe_zone", transient_qos, [this](const SafeZone::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex);
        if (incoming_safe_zones.size() >= 32) {
          incoming_safe_zones.pop_front();
        }
        incoming_safe_zones.push_back(*msg);
      });

    sub_gate_release = node->create_subscription<EmptyMsg>(
      "/" + name + "/plan/gate_release", transient_qos, [this](const EmptyMsg::SharedPtr) {
        std::lock_guard<std::mutex> lock(mutex);
        gate_release_pending = true;
      });

    sub_costmap_ack = node->create_subscription<HeaderMsg>(
      "/" + name + "/global_costmap/plan/costmap_updated", transient_qos,
      [this](const HeaderMsg::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex);
        const auto seq = stamp_to_seq(msg->stamp);
        if (seq > costmap_handshake.acked_seq) {
          costmap_handshake.acked_seq = seq;
        }
      });

    client_nav =
      rclcpp_action::create_client<NavigateToPose>(node, "/" + name + "/navigate_to_pose");
    client_dock = rclcpp_action::create_client<DockRobot>(node, "/" + name + "/dock_robot");
    client_undock =
      rclcpp_action::create_client<UndockRobot>(node, "/" + name + "/undock_robot");
  }

  static std::tuple<float, float, float> extract_xy_yaw(const PoseStamped & ps)
  {
    const float x = static_cast<float>(ps.pose.position.x);
    const float y = static_cast<float>(ps.pose.position.y);
    const auto & q = ps.pose.orientation;
    const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    const float yaw = static_cast<float>(std::atan2(siny_cosp, cosy_cosp));
    return std::make_tuple(x, y, yaw);
  }

  void update_pose_from_feedback_locked(const PoseStamped & ps)
  {
    latest_pose = ps;
    if (!localized) {
      current_pose_xy_yaw = extract_xy_yaw(ps);
    }
  }

  std::uint8_t current_mode_locked() const
  {
    if (progress_tracker.is_holding_at_pre_waypoint()) {
      return Progress::MODE_ACTION;
    }
    if (
      active_goal.has_value() || costmap_handshake.pending_target.has_value() ||
      !ready_targets.empty())
    {
      return Progress::MODE_NAVIGATING;
    }
    return Progress::MODE_IDLE;
  }

  void flush_progress_status_locked()
  {
    if (!progress_dirty) {
      return;
    }
    if (!localized && !latest_pose.has_value() && !last_progress.has_value()) {
      return;
    }
    progress_dirty = false;
    PoseStamped pose = latest_pose.value_or(PoseStamped{});
    if (localized) {
      pose.header = latest_amcl.header;
      pose.pose = latest_amcl.pose.pose;
    }
    const std::uint8_t mode = pending_mode_override.value_or(current_mode_locked());
    pending_mode_override.reset();
    const Progress * prog_ptr = last_progress.has_value() ? &(*last_progress) : nullptr;
    const SafeZoneId * sz_id_ptr =
      current_safe_zone.value.has_value() ? &(current_safe_zone.value->id) : nullptr;
    const Progress status_msg = build_progress_status(pose, mode, prog_ptr, sz_id_ptr);
    last_progress = status_msg;
    pub_progress->publish(status_msg);
  }

  void cancel_goal_handles_locked(ActiveNav2Goal & goal)
  {
    if (goal.nav_handle) {
      try {
        client_nav->async_cancel_goal(goal.nav_handle);
      } catch (const std::exception &) {
      }
    }
    if (goal.dock_handle) {
      try {
        client_dock->async_cancel_goal(goal.dock_handle);
      } catch (const std::exception &) {
      }
    }
    if (goal.undock_handle) {
      try {
        client_undock->async_cancel_goal(goal.undock_handle);
      } catch (const std::exception &) {
      }
    }
  }
};

/**
 * BT Node 1: SyncSafeZoneAndCostmap
 * Drains incoming `/<robot>/amcl_pose`, `/<robot>/plan`, `/<robot>/plan/gate_release`,
 * and `/<robot>/plan/safe_zone`. Deduplicates and publishes `/<robot>/global_costmap/plan/costmap`
 * and stages `PendingNavigationTarget` for the costmap handshake.
 */
class SyncSafeZoneAndCostmap : public BT::SyncActionNode
{
public:
  SyncSafeZoneAndCostmap(
    const std::string & name,
    const BT::NodeConfig & config,
    std::shared_ptr<AgentContext> ctx)
  : BT::SyncActionNode(name, config), ctx_(std::move(ctx))
  {
  }

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override
  {
    std::lock_guard<std::mutex> lock(ctx_->mutex);

    // 1. Drain incoming AMCL poses and update progress interpolation
    bool received_amcl = false;
    while (!ctx_->incoming_amcl_poses.empty()) {
      ctx_->latest_amcl = std::move(ctx_->incoming_amcl_poses.front());
      ctx_->incoming_amcl_poses.pop_front();
      ctx_->localized = true;
      received_amcl = true;
    }
    if (ctx_->localized && received_amcl) {
      PoseStamped ps;
      ps.header = ctx_->latest_amcl.header;
      ps.pose = ctx_->latest_amcl.pose.pose;
      ctx_->latest_pose = ps;
      ctx_->current_pose_xy_yaw = AgentContext::extract_xy_yaw(ps);

      if (ctx_->progress_tracker.plan_id.has_value()) {
        const Progress * prev =
          ctx_->last_progress.has_value() ? &(*ctx_->last_progress) : nullptr;
        ctx_->last_progress =
          ctx_->progress_tracker.compute_progress(ctx_->current_pose_xy_yaw, prev);
      }
      ctx_->progress_dirty = true;
    }

    // 2. Drain incoming plans into LocalPlanRegistry & ProgressTracker
    while (!ctx_->incoming_plans.empty()) {
      const Plan plan = std::move(ctx_->incoming_plans.front());
      ctx_->incoming_plans.pop_front();
      ctx_->progress_dirty = true;

      const InnerNavigationAction prev_action = ctx_->progress_tracker.active_action;
      const SafeZone * cur_sz =
        ctx_->current_safe_zone.value.has_value() ? &(*ctx_->current_safe_zone.value) : nullptr;
      ctx_->progress_tracker.set_plan(plan, cur_sz);
      const InnerNavigationAction new_action = ctx_->progress_tracker.active_action;

      if (prev_action != new_action && cur_sz != nullptr && cur_sz->id.plan_id == plan.plan_id) {
        if (
          ctx_->costmap_handshake.pending_target.has_value() &&
          ctx_->costmap_handshake.pending_target->target.safe_zone_id.plan_id == plan.plan_id)
        {
          ctx_->costmap_handshake.pending_target->target.action = new_action;
        } else if (const auto target_pose = next_target(*cur_sz)) {
          const auto [tx, ty, tyaw] = *target_pose;
          InnerNavigationTarget nav_target;
          nav_target.agent = ctx_->index;
          nav_target.safe_zone_id = cur_sz->id;
          nav_target.x = static_cast<double>(tx);
          nav_target.y = static_cast<double>(ty);
          nav_target.yaw = static_cast<double>(tyaw);
          nav_target.action = new_action;
          ctx_->ready_targets.push_back(std::move(nav_target));
        }
      }

      ctx_->plan_registry.insert(plan);
    }

    // 3. Handle gate release
    if (ctx_->gate_release_pending) {
      ctx_->gate_release_pending = false;
      ctx_->progress_tracker.release_gate();
      if (ctx_->progress_tracker.plan_id.has_value()) {
        const Progress * prev =
          ctx_->last_progress.has_value() ? &(*ctx_->last_progress) : nullptr;
        ctx_->last_progress =
          ctx_->progress_tracker.compute_progress(ctx_->current_pose_xy_yaw, prev);
        ctx_->pending_mode_override = Progress::MODE_NAVIGATING;
        ctx_->progress_dirty = true;
      }
    }

    // 4. Drain incoming SafeZones (only the latest SafeZone in the batch matters)
    std::optional<SafeZone> latest_sz;
    while (!ctx_->incoming_safe_zones.empty()) {
      latest_sz = std::move(ctx_->incoming_safe_zones.front());
      ctx_->incoming_safe_zones.pop_front();
    }

    if (latest_sz.has_value()) {
      const SafeZone & safe_zone = *latest_sz;
      if (!ctx_->current_safe_zone.matches(safe_zone) && is_valid_safe_zone(safe_zone)) {
        if (const auto target_pose = next_target(safe_zone)) {
          const auto [target_x, target_y, target_yaw] = *target_pose;

          const bool should_update = ctx_->current_safe_zone.should_update_target(safe_zone);
          const bool costmap_changed =
            !ctx_->costmap_handshake.last_published_costmap.has_value() ||
            !costmap_content_eq(
              *ctx_->costmap_handshake.last_published_costmap, safe_zone.costmap);

          std::optional<std::pair<std::uint64_t, Costmap>> published_costmap;
          if (should_update || costmap_changed) {
            ctx_->costmap_handshake.next_seq += 1;
            const std::uint64_t seq = ctx_->costmap_handshake.next_seq;
            Costmap costmap = safe_zone.costmap;
            costmap.header.stamp = seq_to_stamp(seq);
            ctx_->pub_costmap->publish(costmap);
            ctx_->costmap_handshake.last_published_costmap = costmap;
            published_costmap = std::make_pair(seq, std::move(costmap));
          }

          const std::optional<std::tuple<float, float, float>> current_pose =
            ctx_->localized ? ctx_->current_pose_xy_yaw : std::nullopt;
          const Plan * matching_plan = ctx_->plan_registry.get(safe_zone.id.plan_id);
          const Progress * prev_prog =
            ctx_->last_progress.has_value() ? &(*ctx_->last_progress) : nullptr;
          const Progress base_progress = ctx_->progress_tracker.on_safe_zone_with_plan(
            safe_zone, matching_plan, current_pose, prev_prog);

          ctx_->last_progress = base_progress;
          ctx_->pending_mode_override = ctx_->progress_tracker.is_holding_at_pre_waypoint()
                                          ? Progress::MODE_ACTION
                                          : Progress::MODE_NAVIGATING;
          ctx_->progress_dirty = true;

          if (!should_update) {
            if (
              ctx_->costmap_handshake.pending_target.has_value() &&
              published_costmap.has_value())
            {
              ctx_->costmap_handshake.pending_target->costmap = published_costmap->second;
            }
          } else {
            std::uint64_t seq = ctx_->costmap_handshake.next_seq;
            Costmap costmap = safe_zone.costmap;
            if (published_costmap.has_value()) {
              seq = published_costmap->first;
              costmap = std::move(published_costmap->second);
            } else {
              costmap.header.stamp = seq_to_stamp(seq);
            }

            const InnerNavigationAction action = ctx_->progress_tracker.active_action;
            if (ctx_->active_goal.has_value()) {
              ctx_->active_goal->superseded = true;
            }
            ctx_->current_safe_zone.update(safe_zone);

            InnerNavigationTarget nav_target;
            nav_target.agent = ctx_->index;
            nav_target.safe_zone_id = safe_zone.id;
            nav_target.x = static_cast<double>(target_x);
            nav_target.y = static_cast<double>(target_y);
            nav_target.yaw = static_cast<double>(target_yaw);
            nav_target.action = action;

            PendingNavigationTarget pending;
            pending.required_seq = seq;
            pending.target = std::move(nav_target);
            pending.target_xy_yaw = std::make_tuple(target_x, target_y, target_yaw);
            pending.costmap = std::move(costmap);
            pending.last_published = std::chrono::steady_clock::now();
            ctx_->costmap_handshake.pending_target = std::move(pending);
          }
        }
      }
    }

    return BT::NodeStatus::SUCCESS;
  }

private:
  std::shared_ptr<AgentContext> ctx_;
};

/**
 * BT Node 2: WaitForCostmapAck
 * Promotes `PendingNavigationTarget` once `/<robot>/global_costmap/plan/costmap_updated`
 * acknowledges its sequence number (retrying `/costmap` every 500ms while waiting),
 * and reconciles completed or preempted Nav2 goals before action branching.
 */
class WaitForCostmapAck : public BT::SyncActionNode
{
public:
  WaitForCostmapAck(
    const std::string & name,
    const BT::NodeConfig & config,
    std::shared_ptr<AgentContext> ctx)
  : BT::SyncActionNode(name, config), ctx_(std::move(ctx))
  {
  }

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override
  {
    std::lock_guard<std::mutex> lock(ctx_->mutex);

    // 1. Check costmap handshake acknowledgement
    if (ctx_->costmap_handshake.pending_target.has_value()) {
      auto & pending = *ctx_->costmap_handshake.pending_target;
      if (ctx_->costmap_handshake.acked_seq >= pending.required_seq) {
        ctx_->ready_targets.push_back(std::move(pending.target));
        ctx_->costmap_handshake.pending_target.reset();
      } else {
        const auto now = std::chrono::steady_clock::now();
        if (now - pending.last_published >= std::chrono::milliseconds(500)) {
          ctx_->pub_costmap->publish(pending.costmap);
          pending.last_published = now;
        }
      }
    }

    // 2. Check if active Nav2 goal finished
    if (ctx_->active_goal.has_value() && ctx_->active_goal->terminal_code.has_value()) {
      const auto code = *ctx_->active_goal->terminal_code;
      const SafeZoneId finished_sz_id = ctx_->active_goal->safe_zone_id;
      const InnerNavigationAction finished_act = ctx_->active_goal->action;
      const InnerNavigationTarget finished_target = ctx_->active_goal->target;
      const bool was_superseded = ctx_->active_goal->superseded;
      ctx_->active_goal.reset();

      if (code == rclcpp_action::ResultCode::SUCCEEDED && !was_superseded) {
        const bool matches_sz =
          !ctx_->current_safe_zone.value.has_value() ||
          ctx_->current_safe_zone.value->id == finished_sz_id;
        if (
          matches_sz && ctx_->progress_tracker.plan_id.has_value() &&
          *ctx_->progress_tracker.plan_id == finished_sz_id.plan_id)
        {
          if (
            finished_act.kind == InnerActionKind::DockRobot ||
            finished_act.kind == InnerActionKind::UndockRobot)
          {
            ctx_->progress_tracker.mark_action_completed();
          }
          if (!ctx_->progress_tracker.is_target_pose_satisfied(ctx_->current_pose_xy_yaw)) {
            ctx_->ready_targets.push_front(finished_target);
          } else {
            const Progress * prev =
              ctx_->last_progress.has_value() ? &(*ctx_->last_progress) : nullptr;
            if (const auto prog = ctx_->progress_tracker.mark_target_reached(prev)) {
              ctx_->last_progress = *prog;
              ctx_->progress_dirty = true;
            }
          }
        }
      } else if (code == rclcpp_action::ResultCode::ABORTED && !was_superseded) {
        if (
          !ctx_->current_safe_zone.value.has_value() ||
          ctx_->current_safe_zone.value->id == finished_sz_id)
        {
          PlanError err;
          err.plan_id = finished_sz_id.plan_id;
          err.error.code = PlanError::CODE_PATH_BLOCKED;
          err.error.message = "Nav2 goal aborted: path blocked within safe zone";
          ctx_->pub_error->publish(err);
        }
      }
    }

    // 3. Reconcile queued ready_targets against any active_goal (preempting if newer)
    while (!ctx_->ready_targets.empty()) {
      const InnerNavigationTarget & target = ctx_->ready_targets.front();
      if (ctx_->active_goal.has_value()) {
        if (!should_preempt_goal(
              target.safe_zone_id, target.action, ctx_->active_goal->safe_zone_id,
              ctx_->active_goal->action))
        {
          ctx_->ready_targets.pop_front();
          continue;
        }
        const bool existing_is_nav =
          ctx_->active_goal->action.kind == InnerActionKind::NavigateToPose;
        const bool next_is_nav = target.action.kind == InnerActionKind::NavigateToPose;
        ctx_->active_goal->superseded = true;
        if (!(existing_is_nav && next_is_nav)) {
          ctx_->cancelled_up_to_seq = ctx_->next_goal_seq;
          ctx_->cancel_goal_handles_locked(*ctx_->active_goal);
        }
        ctx_->active_goal.reset();
      } else if (
        ctx_->last_action_kind.has_value() && *ctx_->last_action_kind != target.action.kind)
      {
        ctx_->cancelled_up_to_seq = ctx_->next_goal_seq;
      }

      if (ctx_->ready_targets.size() > 1) {
        ctx_->ready_targets.pop_front();
        continue;
      }
      break;
    }

    return BT::NodeStatus::SUCCESS;
  }

private:
  std::shared_ptr<AgentContext> ctx_;
};

/**
 * BT Condition Node: IsNextAction
 * Checks whether the staged ready target matches the requested `kind` port
 * ("dock", "undock", or "navigate").
 */
class IsNextAction : public BT::ConditionNode
{
public:
  IsNextAction(
    const std::string & name,
    const BT::NodeConfig & config,
    std::shared_ptr<AgentContext> ctx)
  : BT::ConditionNode(name, config), ctx_(std::move(ctx))
  {
  }

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::string>("kind")};
  }

  BT::NodeStatus tick() override
  {
    std::lock_guard<std::mutex> lock(ctx_->mutex);
    if (ctx_->ready_targets.empty()) {
      return BT::NodeStatus::FAILURE;
    }
    const auto kind_str = getInput<std::string>("kind").value_or("navigate");
    const auto actual_kind = ctx_->ready_targets.front().action.kind;
    if (kind_str == "dock" && actual_kind == InnerActionKind::DockRobot) {
      return BT::NodeStatus::SUCCESS;
    }
    if (kind_str == "undock" && actual_kind == InnerActionKind::UndockRobot) {
      return BT::NodeStatus::SUCCESS;
    }
    if (kind_str == "navigate" && actual_kind == InnerActionKind::NavigateToPose) {
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
  }

private:
  std::shared_ptr<AgentContext> ctx_;
};

/**
 * BT Action Node: ExecuteDockRobot
 * Dispatches a `DockRobot` goal to `/<robot>/dock_robot`.
 */
class ExecuteDockRobot : public BT::SyncActionNode
{
public:
  ExecuteDockRobot(
    const std::string & name,
    const BT::NodeConfig & config,
    std::shared_ptr<AgentContext> ctx)
  : BT::SyncActionNode(name, config), ctx_(std::move(ctx))
  {
  }

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override
  {
    std::lock_guard<std::mutex> lock(ctx_->mutex);
    if (
      ctx_->ready_targets.empty() ||
      ctx_->ready_targets.front().action.kind != InnerActionKind::DockRobot)
    {
      return BT::NodeStatus::FAILURE;
    }
    if (!ctx_->client_dock->action_server_is_ready()) {
      return BT::NodeStatus::SUCCESS;
    }

    const InnerNavigationTarget target = ctx_->ready_targets.front();
    ctx_->ready_targets.pop_front();
    const std::uint64_t seq = ctx_->next_goal_seq++;

    ActiveNav2Goal active;
    active.seq = seq;
    active.safe_zone_id = target.safe_zone_id;
    active.action = target.action;
    active.target = target;
    active.state = GoalState::WaitingAcceptance;

    DockRobot::Goal goal;
    goal.use_dock_id = target.action.use_dock_id;
    goal.dock_id = target.action.dock_id;
    goal.dock_type = target.action.dock_type;
    goal.navigate_to_staging_pose = target.action.navigate_to_staging_pose;
    goal.max_staging_time = 1000.0f;
    if (!target.action.use_dock_id) {
      goal.dock_pose.header.frame_id = "map";
      goal.dock_pose.pose.position.x = target.x;
      goal.dock_pose.pose.position.y = target.y;
      goal.dock_pose.pose.orientation.z = std::sin(target.yaw * 0.5);
      goal.dock_pose.pose.orientation.w = std::cos(target.yaw * 0.5);
    }

    auto weak_ctx = std::weak_ptr<AgentContext>(ctx_);
    rclcpp_action::Client<DockRobot>::SendGoalOptions opts;
    opts.goal_response_callback = [weak_ctx, seq](const GoalHandleDock::SharedPtr & handle) {
      if (auto c = weak_ctx.lock()) {
        std::lock_guard<std::mutex> lock(c->mutex);
        if (c->active_goal.has_value() && c->active_goal->seq == seq) {
          if (!handle) {
            c->active_goal->terminal_code = rclcpp_action::ResultCode::ABORTED;
          } else {
            c->active_goal->dock_handle = handle;
            c->active_goal->state = GoalState::Executing;
          }
        } else if (handle) {
          try {
            c->client_dock->async_cancel_goal(handle);
          } catch (const std::exception &) {
          }
        }
      }
    };
    opts.result_callback = [weak_ctx, seq](const GoalHandleDock::WrappedResult & result) {
      if (auto c = weak_ctx.lock()) {
        std::lock_guard<std::mutex> lock(c->mutex);
        if (c->active_goal.has_value() && c->active_goal->seq == seq) {
          auto code = result.code;
          if (
            code == rclcpp_action::ResultCode::SUCCEEDED && result.result &&
            !result.result->success)
          {
            code = rclcpp_action::ResultCode::ABORTED;
          }
          c->active_goal->terminal_code = code;
        }
      }
    };

    ctx_->last_action_kind = InnerActionKind::DockRobot;
    ctx_->active_goal = std::move(active);
    ctx_->client_dock->async_send_goal(goal, opts);
    return BT::NodeStatus::SUCCESS;
  }

private:
  std::shared_ptr<AgentContext> ctx_;
};

/**
 * BT Action Node: ExecuteUndockRobot
 * Dispatches an `UndockRobot` goal to `/<robot>/undock_robot`.
 */
class ExecuteUndockRobot : public BT::SyncActionNode
{
public:
  ExecuteUndockRobot(
    const std::string & name,
    const BT::NodeConfig & config,
    std::shared_ptr<AgentContext> ctx)
  : BT::SyncActionNode(name, config), ctx_(std::move(ctx))
  {
  }

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override
  {
    std::lock_guard<std::mutex> lock(ctx_->mutex);
    if (
      ctx_->ready_targets.empty() ||
      ctx_->ready_targets.front().action.kind != InnerActionKind::UndockRobot)
    {
      return BT::NodeStatus::FAILURE;
    }
    if (!ctx_->client_undock->action_server_is_ready()) {
      return BT::NodeStatus::SUCCESS;
    }

    const InnerNavigationTarget target = ctx_->ready_targets.front();
    ctx_->ready_targets.pop_front();
    const std::uint64_t seq = ctx_->next_goal_seq++;

    ActiveNav2Goal active;
    active.seq = seq;
    active.safe_zone_id = target.safe_zone_id;
    active.action = target.action;
    active.target = target;
    active.state = GoalState::WaitingAcceptance;

    UndockRobot::Goal goal;
    goal.dock_type = target.action.dock_type;
    goal.max_undocking_time = 30.0f;

    auto weak_ctx = std::weak_ptr<AgentContext>(ctx_);
    rclcpp_action::Client<UndockRobot>::SendGoalOptions opts;
    opts.goal_response_callback = [weak_ctx, seq](const GoalHandleUndock::SharedPtr & handle) {
      if (auto c = weak_ctx.lock()) {
        std::lock_guard<std::mutex> lock(c->mutex);
        if (c->active_goal.has_value() && c->active_goal->seq == seq) {
          if (!handle) {
            c->active_goal->terminal_code = rclcpp_action::ResultCode::ABORTED;
          } else {
            c->active_goal->undock_handle = handle;
            c->active_goal->state = GoalState::Executing;
          }
        } else if (handle) {
          try {
            c->client_undock->async_cancel_goal(handle);
          } catch (const std::exception &) {
          }
        }
      }
    };
    opts.result_callback = [weak_ctx, seq](const GoalHandleUndock::WrappedResult & result) {
      if (auto c = weak_ctx.lock()) {
        std::lock_guard<std::mutex> lock(c->mutex);
        if (c->active_goal.has_value() && c->active_goal->seq == seq) {
          auto code = result.code;
          if (
            code == rclcpp_action::ResultCode::SUCCEEDED && result.result &&
            !result.result->success)
          {
            code = rclcpp_action::ResultCode::ABORTED;
          }
          c->active_goal->terminal_code = code;
        }
      }
    };

    ctx_->last_action_kind = InnerActionKind::UndockRobot;
    ctx_->active_goal = std::move(active);
    ctx_->client_undock->async_send_goal(goal, opts);
    return BT::NodeStatus::SUCCESS;
  }

private:
  std::shared_ptr<AgentContext> ctx_;
};

/**
 * BT Action Node: ExecuteNavigateToPose
 * Dispatches a `NavigateToPose` goal to `/<robot>/navigate_to_pose` if one is staged.
 */
class ExecuteNavigateToPose : public BT::SyncActionNode
{
public:
  ExecuteNavigateToPose(
    const std::string & name,
    const BT::NodeConfig & config,
    std::shared_ptr<AgentContext> ctx)
  : BT::SyncActionNode(name, config), ctx_(std::move(ctx))
  {
  }

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override
  {
    std::lock_guard<std::mutex> lock(ctx_->mutex);
    if (
      ctx_->ready_targets.empty() ||
      ctx_->ready_targets.front().action.kind != InnerActionKind::NavigateToPose)
    {
      return BT::NodeStatus::SUCCESS;
    }
    if (!ctx_->client_nav->action_server_is_ready()) {
      return BT::NodeStatus::SUCCESS;
    }

    const InnerNavigationTarget target = ctx_->ready_targets.front();
    ctx_->ready_targets.pop_front();
    const std::uint64_t seq = ctx_->next_goal_seq++;

    ActiveNav2Goal active;
    active.seq = seq;
    active.safe_zone_id = target.safe_zone_id;
    active.action = target.action;
    active.target = target;
    active.state = GoalState::WaitingAcceptance;

    NavigateToPose::Goal goal;
    goal.pose.header.frame_id = "map";
    goal.pose.pose.position.x = target.x;
    goal.pose.pose.position.y = target.y;
    goal.pose.pose.orientation.z = std::sin(target.yaw * 0.5);
    goal.pose.pose.orientation.w = std::cos(target.yaw * 0.5);

    auto weak_ctx = std::weak_ptr<AgentContext>(ctx_);
    rclcpp_action::Client<NavigateToPose>::SendGoalOptions opts;
    opts.goal_response_callback = [weak_ctx, seq](const GoalHandleNav::SharedPtr & handle) {
      if (auto c = weak_ctx.lock()) {
        std::lock_guard<std::mutex> lock(c->mutex);
        if (c->active_goal.has_value() && c->active_goal->seq == seq) {
          if (!handle) {
            c->active_goal->terminal_code = rclcpp_action::ResultCode::ABORTED;
          } else {
            c->active_goal->nav_handle = handle;
            c->active_goal->state = GoalState::Executing;
          }
        } else if (handle && seq <= c->cancelled_up_to_seq) {
          try {
            c->client_nav->async_cancel_goal(handle);
          } catch (const std::exception &) {
          }
        }
      }
    };
    opts.feedback_callback =
      [weak_ctx, seq](
        GoalHandleNav::SharedPtr,
        const std::shared_ptr<const NavigateToPose::Feedback> feedback) {
        if (auto c = weak_ctx.lock()) {
          std::lock_guard<std::mutex> lock(c->mutex);
          if (c->active_goal.has_value() && c->active_goal->seq == seq) {
            c->update_pose_from_feedback_locked(feedback->current_pose);
          }
        }
      };
    opts.result_callback = [weak_ctx, seq](const GoalHandleNav::WrappedResult & result) {
      if (auto c = weak_ctx.lock()) {
        std::lock_guard<std::mutex> lock(c->mutex);
        if (c->active_goal.has_value() && c->active_goal->seq == seq) {
          c->active_goal->terminal_code = result.code;
        }
      }
    };

    ctx_->last_action_kind = InnerActionKind::NavigateToPose;
    ctx_->active_goal = std::move(active);
    ctx_->client_nav->async_send_goal(goal, opts);
    return BT::NodeStatus::SUCCESS;
  }

private:
  std::shared_ptr<AgentContext> ctx_;
};

/**
 * BT Action Node: PublishRobotProgress
 * Flushes deduplicated `/<robot>/plan/progress` (`Progress`) updates at the end of the tick.
 */
class PublishRobotProgress : public BT::SyncActionNode
{
public:
  PublishRobotProgress(
    const std::string & name,
    const BT::NodeConfig & config,
    std::shared_ptr<AgentContext> ctx)
  : BT::SyncActionNode(name, config), ctx_(std::move(ctx))
  {
  }

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override
  {
    std::lock_guard<std::mutex> lock(ctx_->mutex);
    ctx_->flush_progress_status_locked();
    return BT::NodeStatus::SUCCESS;
  }

private:
  std::shared_ptr<AgentContext> ctx_;
};

inline constexpr const char * kAgentBehaviorTreeXml = R"(
<root BTCPP_format="4">
  <BehaviorTree ID="AgentTrafficTree">
    <Sequence name="AgentTickSequence">
      <SyncSafeZoneAndCostmap/>
      <WaitForCostmapAck/>
      <Fallback name="DispatchOrContinue">
        <Sequence name="DockBranch">
          <IsNextAction kind="dock"/>
          <ExecuteDockRobot/>
        </Sequence>
        <Sequence name="UndockBranch">
          <IsNextAction kind="undock"/>
          <ExecuteUndockRobot/>
        </Sequence>
        <ExecuteNavigateToPose/>
      </Fallback>
      <PublishRobotProgress/>
    </Sequence>
  </BehaviorTree>
</root>
)";

}  // namespace rmf_nav2_traffic

#endif  // RMF_NAV2_TRAFFIC__BT_NODES_HPP_
