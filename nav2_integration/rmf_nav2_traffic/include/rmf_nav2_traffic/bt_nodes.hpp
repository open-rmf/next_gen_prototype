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
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance.hpp>
#include <nav2_msgs/action/dock_robot.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav2_msgs/action/undock_robot.hpp>
#include <nav2_msgs/msg/costmap.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rmf_prototype_msgs/msg/destination_goal.hpp>
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
using ServerGoalHandleNav = rclcpp_action::ServerGoalHandle<NavigateToPose>;
using PoseWithCovarianceStamped = geometry_msgs::msg::PoseWithCovarianceStamped;
using TwistWithCovariance = geometry_msgs::msg::TwistWithCovariance;
using Odometry = nav_msgs::msg::Odometry;
using EmptyMsg = std_msgs::msg::Empty;
using HeaderMsg = std_msgs::msg::Header;

enum class InnerGoalState
{
  Idle,
  WaitingAcceptance,
  Executing,
};

struct ActiveInnerGoal
{
  std::uint64_t seq = 0;
  SafeZoneId safe_zone_id;
  InnerNavigationAction action;
  InnerNavigationTarget target;
  bool superseded = false;
  InnerGoalState state = InnerGoalState::Idle;
  GoalHandleNav::SharedPtr nav_handle;
  GoalHandleDock::SharedPtr dock_handle;
  GoalHandleUndock::SharedPtr undock_handle;
  std::optional<rclcpp_action::ResultCode> terminal_code;
};

struct ActiveOuterSession
{
  RosUuid session;
  std::shared_ptr<ServerGoalHandleNav> handle;
  PoseStamped target_pose;
  double threshold = 0.5;
  std::vector<RosUuid> superseded_sessions;
  bool seen_matching_safe_zone = false;
};

struct AgentContext
{
  std::size_t index = 0;
  std::string name;
  rclcpp::Node::SharedPtr node;

  mutable std::mutex mutex;

  // Latest pose from inner/amcl_pose or inner feedback
  bool localized = false;
  PoseWithCovarianceStamped latest_amcl;
  std::optional<PoseStamped> latest_pose;
  std::optional<std::tuple<float, float, float>> current_pose_xy_yaw;
  std::deque<NavigateToPose::Feedback> outer_feedback_queue;

  // Domain trackers
  CurrentSafeZone current_safe_zone;
  LocalPlanRegistry plan_registry;
  CostmapHandshakeState costmap_handshake;
  ProgressTracker progress_tracker;
  std::optional<Progress> last_progress;
  std::optional<Progress> last_published_status;

  // Cancelled & superseded sessions
  std::vector<RosUuid> cancelled_sessions;

  // Incoming message queues
  std::deque<PoseWithCovarianceStamped> incoming_amcl_poses;
  std::deque<Plan> incoming_plans;
  std::deque<SafeZone> incoming_safe_zones;
  bool gate_release_pending = false;
  std::deque<InnerNavigationTarget> ready_inner_targets;

  // Inner action state
  std::uint64_t next_inner_seq = 1;
  std::uint64_t cancelled_up_to_seq = 0;
  std::optional<InnerActionKind> last_inner_action_kind;
  std::optional<ActiveInnerGoal> active_inner_goal;

  // Outer mock server state
  std::deque<std::shared_ptr<ServerGoalHandleNav>> incoming_outer_goals;
  std::optional<ActiveOuterSession> active_outer_session;

  // ROS Publishers
  rclcpp::Publisher<Odometry>::SharedPtr pub_odom;
  rclcpp::Publisher<Costmap>::SharedPtr pub_costmap;
  rclcpp::Publisher<Progress>::SharedPtr pub_progress;
  rclcpp::Publisher<PlanError>::SharedPtr pub_error;
  rclcpp::Publisher<DestinationGoal>::SharedPtr pub_destination_goal;
  rclcpp::Publisher<PoseStamped>::SharedPtr pub_feedback_pose;

  // ROS Subscriptions
  rclcpp::Subscription<PoseWithCovarianceStamped>::SharedPtr sub_amcl;
  rclcpp::Subscription<Plan>::SharedPtr sub_plan;
  rclcpp::Subscription<SafeZone>::SharedPtr sub_safe_zone;
  rclcpp::Subscription<EmptyMsg>::SharedPtr sub_gate_release;
  rclcpp::Subscription<HeaderMsg>::SharedPtr sub_costmap_ack;

  // ROS Action Clients & Server
  rclcpp_action::Client<NavigateToPose>::SharedPtr client_nav;
  rclcpp_action::Client<DockRobot>::SharedPtr client_dock;
  rclcpp_action::Client<UndockRobot>::SharedPtr client_undock;
  rclcpp_action::Server<NavigateToPose>::SharedPtr server_outer_nav;

  AgentContext(std::size_t idx, std::string agent_name, rclcpp::Node::SharedPtr ros_node)
  : index(idx), name(std::move(agent_name)), node(std::move(ros_node))
  {
    const auto default_qos = rclcpp::QoS(10);
    const auto transient_qos = rclcpp::QoS(10).transient_local().reliable();

    pub_odom = node->create_publisher<Odometry>("/" + name + "/odom", default_qos);
    pub_costmap = node->create_publisher<Costmap>(
      "/" + name + "/inner/global_costmap/plan/costmap", transient_qos);
    pub_progress = node->create_publisher<Progress>("/" + name + "/plan/progress", transient_qos);
    pub_error = node->create_publisher<PlanError>("/" + name + "/plan/error", transient_qos);
    pub_destination_goal =
      node->create_publisher<DestinationGoal>("/" + name + "/destination/goal", transient_qos);
    pub_feedback_pose =
      node->create_publisher<PoseStamped>("/" + name + "/inner/feedback_pose", default_qos);

    sub_amcl = node->create_subscription<PoseWithCovarianceStamped>(
      "/" + name + "/inner/amcl_pose", transient_qos,
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
      "/" + name + "/inner/global_costmap/plan/costmap_updated", transient_qos,
      [this](const HeaderMsg::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex);
        const auto seq = stamp_to_seq(msg->stamp);
        if (seq > costmap_handshake.acked_seq) {
          costmap_handshake.acked_seq = seq;
        }
      });

    client_nav =
      rclcpp_action::create_client<NavigateToPose>(node, "/" + name + "/inner/navigate_to_pose");
    client_dock = rclcpp_action::create_client<DockRobot>(node, "/" + name + "/inner/dock_robot");
    client_undock =
      rclcpp_action::create_client<UndockRobot>(node, "/" + name + "/inner/undock_robot");

    server_outer_nav = rclcpp_action::create_server<NavigateToPose>(
      node, "/" + name + "/navigate_to_pose",
      [](const rclcpp_action::GoalUUID &, std::shared_ptr<const NavigateToPose::Goal>) {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [](const std::shared_ptr<ServerGoalHandleNav>) {
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](const std::shared_ptr<ServerGoalHandleNav> goal_handle) {
        std::lock_guard<std::mutex> lock(mutex);
        incoming_outer_goals.push_back(goal_handle);
      });
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

  bool is_session_cancelled_locked(const RosUuid & session) const
  {
    for (const auto & s : cancelled_sessions) {
      if (s.uuid == session.uuid) {
        return true;
      }
    }
    return false;
  }

  void mark_session_cancelled_locked(const RosUuid & session)
  {
    if (!is_session_cancelled_locked(session)) {
      cancelled_sessions.push_back(session);
    }
  }

  std::uint8_t current_mode_locked() const
  {
    if (progress_tracker.is_holding_at_pre_waypoint()) {
      return Progress::MODE_ACTION;
    }
    if (active_outer_session.has_value() || active_inner_goal.has_value()) {
      return Progress::MODE_NAVIGATING;
    }
    return Progress::MODE_IDLE;
  }

  void publish_progress_status_locked(std::optional<std::uint8_t> override_mode = std::nullopt)
  {
    PoseStamped pose = latest_pose.value_or(PoseStamped{});
    if (localized) {
      pose.header = latest_amcl.header;
      pose.pose = latest_amcl.pose.pose;
    }
    const std::uint8_t mode = override_mode.value_or(current_mode_locked());
    const Progress * prog_ptr = last_progress.has_value() ? &(*last_progress) : nullptr;
    const SafeZoneId * sz_id_ptr =
      current_safe_zone.value.has_value() ? &(current_safe_zone.value->id) : nullptr;
    const Progress status_msg = build_progress_status(pose, mode, prog_ptr, sz_id_ptr);
    last_progress = status_msg;
    if (!last_published_status.has_value() || *last_published_status != status_msg) {
      pub_progress->publish(status_msg);
      last_published_status = status_msg;
    }
  }

  static void cancel_outer_handle(std::shared_ptr<ServerGoalHandleNav> & handle)
  {
    if (!handle) {
      return;
    }
    if (handle->is_active() && handle->is_canceling()) {
      try {
        handle->canceled(std::make_shared<NavigateToPose::Result>());
      } catch (const std::exception &) {
      }
    }
    handle.reset();
  }

  void cancel_active_inner_goal_locked()
  {
    costmap_handshake.pending_target.reset();
    ready_inner_targets.clear();
    cancelled_up_to_seq = next_inner_seq;
    if (!active_inner_goal.has_value()) {
      return;
    }
    active_inner_goal->superseded = true;
    if (active_inner_goal->nav_handle) {
      try {
        client_nav->async_cancel_goal(active_inner_goal->nav_handle);
      } catch (const std::exception &) {
      }
    }
    if (active_inner_goal->dock_handle) {
      try {
        client_dock->async_cancel_goal(active_inner_goal->dock_handle);
      } catch (const std::exception &) {
      }
    }
    if (active_inner_goal->undock_handle) {
      try {
        client_undock->async_cancel_goal(active_inner_goal->undock_handle);
      } catch (const std::exception &) {
      }
    }
    active_inner_goal.reset();
  }
};

/**
 * BT Node 1: HandleOuterGoalRequests
 * Manages incoming outer `/navigate_to_pose` action requests and client cancellations,
 * translating accepted goals into `/destination/goal` publications with a fresh session UUID.
 */
class HandleOuterGoalRequests : public BT::SyncActionNode
{
public:
  HandleOuterGoalRequests(
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

    // 1. Check if active outer goal was cancelled by the client
    if (ctx_->active_outer_session.has_value()) {
      auto & session = *ctx_->active_outer_session;
      if (session.handle && session.handle->is_canceling()) {
        for (const auto & sup : session.superseded_sessions) {
          ctx_->mark_session_cancelled_locked(sup);
        }
        ctx_->mark_session_cancelled_locked(session.session);
        AgentContext::cancel_outer_handle(session.handle);
        ctx_->active_outer_session.reset();
        ctx_->cancel_active_inner_goal_locked();
      }
    }

    // 2. Drain incoming outer goals; only the latest survives if multiple arrived in one tick
    while (!ctx_->incoming_outer_goals.empty()) {
      auto new_handle = ctx_->incoming_outer_goals.front();
      ctx_->incoming_outer_goals.pop_front();

      if (!new_handle || !new_handle->is_active()) {
        continue;
      }

      const RosUuid session_uuid = generate_random_uuid();
      const auto goal_msg =
        pose_stamped_to_destination_goal(new_handle->get_goal()->pose, session_uuid);
      ctx_->pub_destination_goal->publish(goal_msg);

      if (new_handle->is_canceling()) {
        ctx_->mark_session_cancelled_locked(session_uuid);
        AgentContext::cancel_outer_handle(new_handle);
        ctx_->cancel_active_inner_goal_locked();
        continue;
      }

      std::vector<RosUuid> superseded;
      bool should_cancel_inner = false;

      // Preempt previous outer goal if any
      if (ctx_->active_outer_session.has_value()) {
        auto & existing = *ctx_->active_outer_session;
        for (const auto & s : existing.superseded_sessions) {
          superseded.push_back(s);
          ctx_->mark_session_cancelled_locked(s);
        }
        superseded.push_back(existing.session);
        ctx_->mark_session_cancelled_locked(existing.session);
        AgentContext::cancel_outer_handle(existing.handle);
        ctx_->active_outer_session.reset();
        should_cancel_inner = true;
      }

      if (ctx_->current_safe_zone.value.has_value()) {
        const auto & cur_sz_session = ctx_->current_safe_zone.value->id.plan_id.destination_session;
        if (cur_sz_session.uuid != session_uuid.uuid) {
          superseded.push_back(cur_sz_session);
          ctx_->mark_session_cancelled_locked(cur_sz_session);
          should_cancel_inner = true;
        }
      }

      if (should_cancel_inner) {
        ctx_->cancel_active_inner_goal_locked();
      }

      ActiveOuterSession new_session;
      new_session.session = session_uuid;
      new_session.handle = new_handle;
      new_session.target_pose = new_handle->get_goal()->pose;
      new_session.threshold = 0.5;
      new_session.superseded_sessions = std::move(superseded);
      new_session.seen_matching_safe_zone = false;
      ctx_->active_outer_session = std::move(new_session);
    }

    return BT::NodeStatus::SUCCESS;
  }

private:
  std::shared_ptr<AgentContext> ctx_;
};

/**
 * BT Node 2: ProcessSafeZoneAndCostmap
 * Processes incoming `/inner/amcl_pose`, `/plan`, `/plan/safe_zone`, `/plan/gate_release`.
 * Publishes `/odom`, deduplicates and publishes `/inner/global_costmap/plan/costmap`,
 * and stages `PendingNavigationTarget` for the costmap handshake.
 */
class ProcessSafeZoneAndCostmap : public BT::SyncActionNode
{
public:
  ProcessSafeZoneAndCostmap(
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

    // 1. Drain incoming AMCL poses and publish Odometry + updated Progress
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

      Odometry odom;
      odom.header = ctx_->latest_amcl.header;
      odom.child_frame_id = "odom";
      odom.pose = ctx_->latest_amcl.pose;
      odom.twist = TwistWithCovariance{};
      ctx_->pub_odom->publish(odom);

      if (ctx_->progress_tracker.plan_id.has_value()) {
        const Progress * prev =
          ctx_->last_progress.has_value() ? &(*ctx_->last_progress) : nullptr;
        ctx_->last_progress =
          ctx_->progress_tracker.compute_progress(ctx_->current_pose_xy_yaw, prev);
      }
      ctx_->publish_progress_status_locked();
    }

    // 2. Drain incoming plans into LocalPlanRegistry & ProgressTracker
    while (!ctx_->incoming_plans.empty()) {
      const Plan plan = std::move(ctx_->incoming_plans.front());
      ctx_->incoming_plans.pop_front();

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
          ctx_->ready_inner_targets.push_back(std::move(nav_target));
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
        ctx_->publish_progress_status_locked(Progress::MODE_NAVIGATING);
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
          const auto & incoming_session = safe_zone.id.plan_id.destination_session;

          bool skip_safe_zone = false;
          if (ctx_->is_session_cancelled_locked(incoming_session)) {
            skip_safe_zone = true;
          } else if (ctx_->active_outer_session.has_value()) {
            auto & outer = *ctx_->active_outer_session;
            if (outer.session.uuid == incoming_session.uuid) {
              outer.superseded_sessions.clear();
              outer.seen_matching_safe_zone = true;
            } else {
              bool in_superseded = false;
              for (const auto & s : outer.superseded_sessions) {
                if (s.uuid == incoming_session.uuid) {
                  in_superseded = true;
                  break;
                }
              }
              if (in_superseded) {
                skip_safe_zone = true;
              } else {
                for (const auto & s : outer.superseded_sessions) {
                  ctx_->mark_session_cancelled_locked(s);
                }
                ctx_->mark_session_cancelled_locked(outer.session);
                AgentContext::cancel_outer_handle(outer.handle);
                ctx_->active_outer_session.reset();
              }
            }
          }

          if (!skip_safe_zone) {
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
            const std::uint8_t mode = ctx_->progress_tracker.is_holding_at_pre_waypoint()
                                        ? Progress::MODE_ACTION
                                        : Progress::MODE_NAVIGATING;

            if (!should_update) {
              if (
                ctx_->costmap_handshake.pending_target.has_value() &&
                published_costmap.has_value())
              {
                ctx_->costmap_handshake.pending_target->costmap = published_costmap->second;
              }
              ctx_->publish_progress_status_locked(mode);
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
              if (ctx_->active_inner_goal.has_value()) {
                ctx_->active_inner_goal->superseded = true;
              }
              ctx_->current_safe_zone.update(safe_zone);
              ctx_->publish_progress_status_locked(mode);

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
    }

    return BT::NodeStatus::SUCCESS;
  }

private:
  std::shared_ptr<AgentContext> ctx_;
};

/**
 * BT Node 3: WaitForCostmapAck
 * Checks if a PendingNavigationTarget is waiting for `/costmap_updated` acknowledgement,
 * republishes `/costmap` every 500ms while waiting, and promotes the target once acknowledged.
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

    if (!ctx_->costmap_handshake.pending_target.has_value()) {
      return BT::NodeStatus::SUCCESS;
    }

    auto & pending = *ctx_->costmap_handshake.pending_target;
    if (ctx_->is_session_cancelled_locked(pending.target.safe_zone_id.plan_id.destination_session))
    {
      ctx_->costmap_handshake.pending_target.reset();
      return BT::NodeStatus::SUCCESS;
    }

    if (ctx_->costmap_handshake.acked_seq >= pending.required_seq) {
      ctx_->ready_inner_targets.push_back(std::move(pending.target));
      ctx_->costmap_handshake.pending_target.reset();
      return BT::NodeStatus::SUCCESS;
    }

    const auto now = std::chrono::steady_clock::now();
    if (now - pending.last_published >= std::chrono::milliseconds(500)) {
      ctx_->pub_costmap->publish(pending.costmap);
      pending.last_published = now;
    }

    return BT::NodeStatus::SUCCESS;
  }

private:
  std::shared_ptr<AgentContext> ctx_;
};

/**
 * BT Node 4: ExecuteInnerNavigation
 * Drives the inner Nav2 action clients (`navigate_to_pose`, `dock_robot`, `undock_robot`),
 * handles goal preemption, and updates `ProgressTracker` or publishes `PlanError` on completion.
 */
class ExecuteInnerNavigation : public BT::SyncActionNode
{
public:
  ExecuteInnerNavigation(
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

    // 1. Check if active inner goal finished
    if (ctx_->active_inner_goal.has_value() && ctx_->active_inner_goal->terminal_code.has_value()) {
      const auto code = *ctx_->active_inner_goal->terminal_code;
      const SafeZoneId finished_sz_id = ctx_->active_inner_goal->safe_zone_id;
      const InnerNavigationAction finished_act = ctx_->active_inner_goal->action;
      const InnerNavigationTarget finished_target = ctx_->active_inner_goal->target;
      const bool was_superseded = ctx_->active_inner_goal->superseded;
      ctx_->active_inner_goal.reset();

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
            ctx_->ready_inner_targets.push_front(finished_target);
          } else {
            const Progress * prev =
              ctx_->last_progress.has_value() ? &(*ctx_->last_progress) : nullptr;
            if (const auto prog = ctx_->progress_tracker.mark_target_reached(prev)) {
              ctx_->last_progress = *prog;
              const std::uint8_t mode = ctx_->progress_tracker.is_holding_at_pre_waypoint()
                                          ? Progress::MODE_ACTION
                                          : Progress::MODE_NAVIGATING;
              ctx_->publish_progress_status_locked(mode);
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

    // 2. Dispatch ready_inner_targets if present
    while (!ctx_->ready_inner_targets.empty()) {
      const InnerNavigationTarget target = ctx_->ready_inner_targets.front();
      if (ctx_->is_session_cancelled_locked(target.safe_zone_id.plan_id.destination_session)) {
        ctx_->ready_inner_targets.pop_front();
        continue;
      }

      if (ctx_->active_inner_goal.has_value()) {
        if (!should_preempt_goal(
              target.safe_zone_id, target.action, ctx_->active_inner_goal->safe_zone_id,
              ctx_->active_inner_goal->action))
        {
          ctx_->ready_inner_targets.pop_front();
          continue;
        }
        const bool existing_is_nav =
          ctx_->active_inner_goal->action.kind == InnerActionKind::NavigateToPose;
        const bool next_is_nav = target.action.kind == InnerActionKind::NavigateToPose;
        ctx_->active_inner_goal->superseded = true;
        if (!(existing_is_nav && next_is_nav)) {
          ctx_->cancelled_up_to_seq = ctx_->next_inner_seq;
          if (ctx_->active_inner_goal->nav_handle) {
            try {
              ctx_->client_nav->async_cancel_goal(ctx_->active_inner_goal->nav_handle);
            } catch (const std::exception &) {
            }
          }
          if (ctx_->active_inner_goal->dock_handle) {
            try {
              ctx_->client_dock->async_cancel_goal(ctx_->active_inner_goal->dock_handle);
            } catch (const std::exception &) {
            }
          }
          if (ctx_->active_inner_goal->undock_handle) {
            try {
              ctx_->client_undock->async_cancel_goal(ctx_->active_inner_goal->undock_handle);
            } catch (const std::exception &) {
            }
          }
        }
        ctx_->active_inner_goal.reset();
      } else if (
        ctx_->last_inner_action_kind.has_value() &&
        *ctx_->last_inner_action_kind != target.action.kind)
      {
        ctx_->cancelled_up_to_seq = ctx_->next_inner_seq;
      }

      if (dispatch_inner_goal_locked(target)) {
        ctx_->last_inner_action_kind = target.action.kind;
        ctx_->ready_inner_targets.pop_front();
      } else {
        // Server not ready yet; retry on next tick
        break;
      }
    }

    return BT::NodeStatus::SUCCESS;
  }

private:
  bool dispatch_inner_goal_locked(const InnerNavigationTarget & target)
  {
    const std::uint64_t seq = ctx_->next_inner_seq++;

    ActiveInnerGoal active;
    active.seq = seq;
    active.safe_zone_id = target.safe_zone_id;
    active.action = target.action;
    active.target = target;
    active.state = InnerGoalState::WaitingAcceptance;

    auto weak_ctx = std::weak_ptr<AgentContext>(ctx_);

    if (target.action.kind == InnerActionKind::NavigateToPose) {
      if (!ctx_->client_nav->action_server_is_ready()) {
        return false;
      }
      NavigateToPose::Goal goal;
      goal.pose.header.frame_id = "map";
      goal.pose.pose.position.x = target.x;
      goal.pose.pose.position.y = target.y;
      goal.pose.pose.orientation.z = std::sin(target.yaw * 0.5);
      goal.pose.pose.orientation.w = std::cos(target.yaw * 0.5);

      rclcpp_action::Client<NavigateToPose>::SendGoalOptions opts;
      opts.goal_response_callback = [weak_ctx, seq](const GoalHandleNav::SharedPtr & handle) {
        if (auto c = weak_ctx.lock()) {
          std::lock_guard<std::mutex> lock(c->mutex);
          if (c->active_inner_goal.has_value() && c->active_inner_goal->seq == seq) {
            if (!handle) {
              c->active_inner_goal->terminal_code = rclcpp_action::ResultCode::ABORTED;
            } else {
              c->active_inner_goal->nav_handle = handle;
              c->active_inner_goal->state = InnerGoalState::Executing;
            }
          } else if (handle && seq <= c->cancelled_up_to_seq) {
            // Goal was explicitly cancelled before acceptance arrived!
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
            if (c->active_inner_goal.has_value() && c->active_inner_goal->seq == seq) {
              c->update_pose_from_feedback_locked(feedback->current_pose);
              c->pub_feedback_pose->publish(feedback->current_pose);
              if (c->outer_feedback_queue.size() >= 32) {
                c->outer_feedback_queue.pop_front();
              }
              c->outer_feedback_queue.push_back(*feedback);
            }
          }
        };
      opts.result_callback = [weak_ctx, seq](const GoalHandleNav::WrappedResult & result) {
        if (auto c = weak_ctx.lock()) {
          std::lock_guard<std::mutex> lock(c->mutex);
          if (c->active_inner_goal.has_value() && c->active_inner_goal->seq == seq) {
            c->active_inner_goal->terminal_code = result.code;
          }
        }
      };

      ctx_->active_inner_goal = std::move(active);
      ctx_->client_nav->async_send_goal(goal, opts);
      return true;
    }

    if (target.action.kind == InnerActionKind::DockRobot) {
      if (!ctx_->client_dock->action_server_is_ready()) {
        return false;
      }
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

      rclcpp_action::Client<DockRobot>::SendGoalOptions opts;
      opts.goal_response_callback = [weak_ctx, seq](const GoalHandleDock::SharedPtr & handle) {
        if (auto c = weak_ctx.lock()) {
          std::lock_guard<std::mutex> lock(c->mutex);
          if (c->active_inner_goal.has_value() && c->active_inner_goal->seq == seq) {
            if (!handle) {
              c->active_inner_goal->terminal_code = rclcpp_action::ResultCode::ABORTED;
            } else {
              c->active_inner_goal->dock_handle = handle;
              c->active_inner_goal->state = InnerGoalState::Executing;
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
          if (c->active_inner_goal.has_value() && c->active_inner_goal->seq == seq) {
            auto code = result.code;
            if (
              code == rclcpp_action::ResultCode::SUCCEEDED && result.result &&
              !result.result->success)
            {
              code = rclcpp_action::ResultCode::ABORTED;
            }
            c->active_inner_goal->terminal_code = code;
          }
        }
      };

      ctx_->active_inner_goal = std::move(active);
      ctx_->client_dock->async_send_goal(goal, opts);
      return true;
    }

    // UndockRobot
    if (!ctx_->client_undock->action_server_is_ready()) {
      return false;
    }
    UndockRobot::Goal goal;
    goal.dock_type = target.action.dock_type;
    goal.max_undocking_time = 30.0f;

    rclcpp_action::Client<UndockRobot>::SendGoalOptions opts;
    opts.goal_response_callback = [weak_ctx, seq](const GoalHandleUndock::SharedPtr & handle) {
      if (auto c = weak_ctx.lock()) {
        std::lock_guard<std::mutex> lock(c->mutex);
        if (c->active_inner_goal.has_value() && c->active_inner_goal->seq == seq) {
          if (!handle) {
            c->active_inner_goal->terminal_code = rclcpp_action::ResultCode::ABORTED;
          } else {
            c->active_inner_goal->undock_handle = handle;
            c->active_inner_goal->state = InnerGoalState::Executing;
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
        if (c->active_inner_goal.has_value() && c->active_inner_goal->seq == seq) {
          auto code = result.code;
          if (
            code == rclcpp_action::ResultCode::SUCCEEDED && result.result &&
            !result.result->success)
          {
            code = rclcpp_action::ResultCode::ABORTED;
          }
          c->active_inner_goal->terminal_code = code;
        }
      }
    };

    ctx_->active_inner_goal = std::move(active);
    ctx_->client_undock->async_send_goal(goal, opts);
    return true;
  }

  std::shared_ptr<AgentContext> ctx_;
};

/**
 * BT Node 5: MonitorOuterGoalCompletion
 * Forwards inner feedback to the outer `/navigate_to_pose` action server and checks if the
 * outer destination has been reached while the inner navigation client is idle.
 */
class MonitorOuterGoalCompletion : public BT::SyncActionNode
{
public:
  MonitorOuterGoalCompletion(
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

    if (!ctx_->active_outer_session.has_value()) {
      ctx_->outer_feedback_queue.clear();
      return BT::NodeStatus::SUCCESS;
    }

    auto & outer = *ctx_->active_outer_session;
    if (!outer.handle || !outer.handle->is_active()) {
      ctx_->active_outer_session.reset();
      ctx_->outer_feedback_queue.clear();
      return BT::NodeStatus::SUCCESS;
    }

    if (outer.handle->is_canceling()) {
      for (const auto & s : outer.superseded_sessions) {
        ctx_->mark_session_cancelled_locked(s);
      }
      ctx_->mark_session_cancelled_locked(outer.session);
      AgentContext::cancel_outer_handle(outer.handle);
      ctx_->active_outer_session.reset();
      ctx_->cancel_active_inner_goal_locked();
      ctx_->outer_feedback_queue.clear();
      return BT::NodeStatus::SUCCESS;
    }

    // Forward any queued feedback to outer action server
    while (!ctx_->outer_feedback_queue.empty()) {
      auto fb = std::make_shared<NavigateToPose::Feedback>(
        std::move(ctx_->outer_feedback_queue.front()));
      ctx_->outer_feedback_queue.pop_front();
      try {
        outer.handle->publish_feedback(fb);
      } catch (const std::exception &) {
      }
    }

    // Check if inner navigation is idle and robot pose is within threshold of outer target
    const bool inner_idle =
      !ctx_->active_inner_goal.has_value() &&
      !ctx_->costmap_handshake.pending_target.has_value() &&
      ctx_->ready_inner_targets.empty();

    if (inner_idle && ctx_->latest_pose.has_value()) {
      const double cx = ctx_->latest_pose->pose.position.x;
      const double cy = ctx_->latest_pose->pose.position.y;
      const double tx = outer.target_pose.pose.position.x;
      const double ty = outer.target_pose.pose.position.y;
      if (std::hypot(cx - tx, cy - ty) < outer.threshold) {
        try {
          outer.handle->succeed(std::make_shared<NavigateToPose::Result>());
        } catch (const std::exception &) {
        }
        ctx_->active_outer_session.reset();

        // Publish final MODE_IDLE progress status
        if (ctx_->current_safe_zone.value.has_value()) {
          const auto & sz = *ctx_->current_safe_zone.value;
          const std::uint64_t target_wp =
            sz.target_waypoint.empty() ? 0 : sz.target_waypoint.front();
          Progress base_prog;
          base_prog.progress = sz.target_progress;
          base_prog.reached_waypoint = target_wp;
          base_prog.target_waypoint = target_wp;
          base_prog.plan_id = sz.id.plan_id;
          ctx_->last_progress = base_prog;
          ctx_->publish_progress_status_locked(Progress::MODE_IDLE);
        }
      }
    }

    return BT::NodeStatus::SUCCESS;
  }

private:
  std::shared_ptr<AgentContext> ctx_;
};

inline constexpr const char * kAgentBehaviorTreeXml = R"(
<root BTCPP_format="4">
  <BehaviorTree ID="AgentTrafficTree">
    <Sequence name="AgentTickSequence">
      <HandleOuterGoalRequests/>
      <ProcessSafeZoneAndCostmap/>
      <WaitForCostmapAck/>
      <ExecuteInnerNavigation/>
      <MonitorOuterGoalCompletion/>
    </Sequence>
  </BehaviorTree>
</root>
)";

}  // namespace rmf_nav2_traffic

#endif  // RMF_NAV2_TRAFFIC__BT_NODES_HPP_
