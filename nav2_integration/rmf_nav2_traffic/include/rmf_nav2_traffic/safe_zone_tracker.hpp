#ifndef RMF_NAV2_TRAFFIC__SAFE_ZONE_TRACKER_HPP_
#define RMF_NAV2_TRAFFIC__SAFE_ZONE_TRACKER_HPP_

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>
#include <optional>
#include <random>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/msg/costmap.hpp>
#include <rmf_prototype_msgs/msg/destination_constraints.hpp>
#include <rmf_prototype_msgs/msg/destination_goal.hpp>
#include <rmf_prototype_msgs/msg/plan.hpp>
#include <rmf_prototype_msgs/msg/plan_error.hpp>
#include <rmf_prototype_msgs/msg/plan_id.hpp>
#include <rmf_prototype_msgs/msg/progress.hpp>
#include <rmf_prototype_msgs/msg/region.hpp>
#include <rmf_prototype_msgs/msg/safe_zone.hpp>
#include <rmf_prototype_msgs/msg/safe_zone_id.hpp>
#include <rmf_prototype_msgs/msg/target_orientation.hpp>
#include <rmf_prototype_msgs/msg/target_region.hpp>
#include <unique_identifier_msgs/msg/uuid.hpp>

namespace rmf_nav2_traffic
{

using RosTime = builtin_interfaces::msg::Time;
using PoseStamped = geometry_msgs::msg::PoseStamped;
using Costmap = nav2_msgs::msg::Costmap;
using DestinationConstraints = rmf_prototype_msgs::msg::DestinationConstraints;
using DestinationGoal = rmf_prototype_msgs::msg::DestinationGoal;
using Plan = rmf_prototype_msgs::msg::Plan;
using PlanError = rmf_prototype_msgs::msg::PlanError;
using PlanId = rmf_prototype_msgs::msg::PlanId;
using Progress = rmf_prototype_msgs::msg::Progress;
using Region = rmf_prototype_msgs::msg::Region;
using SafeZone = rmf_prototype_msgs::msg::SafeZone;
using SafeZoneId = rmf_prototype_msgs::msg::SafeZoneId;
using TargetOrientation = rmf_prototype_msgs::msg::TargetOrientation;
using TargetRegion = rmf_prototype_msgs::msg::TargetRegion;
using RosUuid = unique_identifier_msgs::msg::UUID;

enum class InnerActionKind
{
  NavigateToPose,
  DockRobot,
  UndockRobot,
};

struct InnerNavigationAction
{
  InnerActionKind kind = InnerActionKind::NavigateToPose;
  bool use_dock_id = false;
  std::string dock_id;
  std::string dock_type;
  bool navigate_to_staging_pose = false;

  static InnerNavigationAction navigate_to_pose()
  {
    return InnerNavigationAction{};
  }

  static InnerNavigationAction dock_robot(
    bool use_id, std::string id, std::string type, bool nav_to_staging)
  {
    InnerNavigationAction act;
    act.kind = InnerActionKind::DockRobot;
    act.use_dock_id = use_id;
    act.dock_id = std::move(id);
    act.dock_type = std::move(type);
    act.navigate_to_staging_pose = nav_to_staging;
    return act;
  }

  static InnerNavigationAction undock_robot(std::string type)
  {
    InnerNavigationAction act;
    act.kind = InnerActionKind::UndockRobot;
    act.dock_type = std::move(type);
    return act;
  }

  bool operator==(const InnerNavigationAction & other) const
  {
    if (kind != other.kind) {
      return false;
    }
    if (kind == InnerActionKind::NavigateToPose) {
      return true;
    }
    if (kind == InnerActionKind::DockRobot) {
      return use_dock_id == other.use_dock_id && dock_id == other.dock_id &&
             dock_type == other.dock_type &&
             navigate_to_staging_pose == other.navigate_to_staging_pose;
    }
    return dock_type == other.dock_type;
  }

  bool operator!=(const InnerNavigationAction & other) const
  {
    return !(*this == other);
  }
};

struct InnerNavigationTarget
{
  std::size_t agent = 0;
  SafeZoneId safe_zone_id;
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
  InnerNavigationAction action;
};

struct KnownDock
{
  std::string id;
  std::string dock_type;
  float x = 0.0f;
  float y = 0.0f;
  float yaw = 0.0f;
  float staging_x = 0.0f;
  float staging_y = 0.0f;

  static std::vector<KnownDock> corridor_docks()
  {
    constexpr float kHalfPi = static_cast<float>(M_PI_2);
    return {
      {"dock_north_1", "simple_charging_dock", -1.0f, 2.0f, kHalfPi, -1.0f, 1.0f},
      {"dock_north_2", "simple_charging_dock", 1.0f, 2.0f, kHalfPi, 1.0f, 1.0f},
      {"dock_north_3", "simple_charging_dock", 3.0f, 2.0f, kHalfPi, 3.0f, 1.0f},
      {"dock_south_1", "simple_charging_dock", -1.0f, -2.0f, -kHalfPi, -1.0f, -1.0f},
      {"dock_south_2", "simple_charging_dock", 1.0f, -2.0f, -kHalfPi, 1.0f, -1.0f},
      {"dock_south_3", "simple_charging_dock", 3.0f, -2.0f, -kHalfPi, 3.0f, -1.0f},
    };
  }

  static std::string to_lower_ascii(std::string s)
  {
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) {
      return static_cast<char>(std::tolower(c));
    });
    return s;
  }

  static std::optional<KnownDock> match_by_id(const std::string & id)
  {
    const auto lower_id = to_lower_ascii(id);
    for (const auto & d : corridor_docks()) {
      if (to_lower_ascii(d.id) == lower_id) {
        return d;
      }
    }
    return std::nullopt;
  }

  static std::optional<KnownDock> match_by_pose(float x, float y, float tolerance)
  {
    for (const auto & d : corridor_docks()) {
      if (std::hypot(d.x - x, d.y - y) <= tolerance) {
        return d;
      }
    }
    return std::nullopt;
  }
};

class LocalPlanRegistry
{
public:
  static constexpr std::size_t kMaxCachedPlans = 16;

  void insert(const Plan & plan)
  {
    for (auto & entry : entries_) {
      if (entry.plan_id == plan.plan_id) {
        entry = plan;
        update_latest(plan);
        return;
      }
    }
    if (entries_.size() >= kMaxCachedPlans) {
      entries_.pop_front();
    }
    entries_.push_back(plan);
    update_latest(plan);
  }

  const Plan * get(const PlanId & plan_id) const
  {
    for (const auto & entry : entries_) {
      if (entry.plan_id == plan_id) {
        return &entry;
      }
    }
    return nullptr;
  }

  const std::optional<Plan> & latest() const
  {
    return latest_plan_;
  }

private:
  void update_latest(const Plan & plan)
  {
    if (
      !latest_plan_.has_value() ||
      latest_plan_->plan_id.destination_session.uuid != plan.plan_id.destination_session.uuid ||
      plan.plan_id.plan_version >= latest_plan_->plan_id.plan_version)
    {
      latest_plan_ = plan;
    }
  }

  std::deque<Plan> entries_;
  std::optional<Plan> latest_plan_;
};

struct PendingNavigationTarget
{
  std::uint64_t required_seq = 0;
  InnerNavigationTarget target;
  std::tuple<float, float, float> target_xy_yaw{0.0f, 0.0f, 0.0f};
  Costmap costmap;
  std::chrono::steady_clock::time_point last_published;
};

struct CostmapHandshakeState
{
  std::uint64_t next_seq = 0;
  std::uint64_t acked_seq = 0;
  std::optional<PendingNavigationTarget> pending_target;
  std::optional<Costmap> last_published_costmap;
};

inline bool costmap_content_eq(const Costmap & a, const Costmap & b)
{
  return a.header.frame_id == b.header.frame_id && a.metadata.layer == b.metadata.layer &&
         a.metadata.resolution == b.metadata.resolution &&
         a.metadata.size_x == b.metadata.size_x && a.metadata.size_y == b.metadata.size_y &&
         a.metadata.origin == b.metadata.origin && a.data == b.data;
}

inline RosTime seq_to_stamp(std::uint64_t seq)
{
  RosTime t;
  t.sec = static_cast<std::int32_t>(seq >> 32);
  t.nanosec = static_cast<std::uint32_t>(seq & 0xFFFFFFFFULL);
  return t;
}

inline std::uint64_t stamp_to_seq(const RosTime & stamp)
{
  return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(stamp.sec)) << 32) |
         static_cast<std::uint64_t>(stamp.nanosec);
}

inline float shortest_angular_distance(float a, float b)
{
  constexpr float kTwoPi = static_cast<float>(2.0 * M_PI);
  float diff = std::fmod(std::fabs(a - b), kTwoPi);
  if (diff < 0.0f) {
    diff += kTwoPi;
  }
  return std::min(diff, kTwoPi - diff);
}

inline bool is_valid_safe_zone(const SafeZone & safe_zone)
{
  if (safe_zone.target_waypoint.empty()) {
    return false;
  }
  if (
    safe_zone.incremental_target.regions.empty() &&
    safe_zone.incremental_target.nodes.empty())
  {
    return false;
  }
  return true;
}

inline std::optional<std::pair<double, double>> get_safe_zone_point(const SafeZone & safe_zone)
{
  if (safe_zone.incremental_target.regions.empty()) {
    return std::nullopt;
  }
  const auto & region = safe_zone.incremental_target.regions.front().region;
  if (region.hint == Region::HINT_POINT && region.points.size() == 2) {
    return std::make_pair(
      static_cast<double>(region.points[0]), static_cast<double>(region.points[1]));
  }
  return std::nullopt;
}

inline std::optional<float> get_safe_zone_orientation(const SafeZone & safe_zone)
{
  if (
    !safe_zone.incremental_target.regions.empty() &&
    !safe_zone.incremental_target.regions.front().orientations.empty())
  {
    return safe_zone.incremental_target.regions.front().orientations.front().orientation_radians;
  }
  if (
    !safe_zone.incremental_target.nodes.empty() &&
    !safe_zone.incremental_target.nodes.front().orientations.empty())
  {
    return safe_zone.incremental_target.nodes.front().orientations.front().orientation_radians;
  }
  return std::nullopt;
}

inline std::optional<std::tuple<float, float, float>> next_target(const SafeZone & safe_zone)
{
  std::optional<std::pair<float, float>> xy;
  std::optional<float> yaw;

  for (const auto & target_region : safe_zone.incremental_target.regions) {
    const auto & region = target_region.region;
    if (region.hint == Region::HINT_POINT && region.points.size() == 2) {
      xy = std::make_pair(region.points[0], region.points[1]);
    }
    for (const auto & ori : target_region.orientations) {
      yaw = ori.orientation_radians;
    }
  }
  for (const auto & target_node : safe_zone.incremental_target.nodes) {
    for (const auto & ori : target_node.orientations) {
      yaw = ori.orientation_radians;
    }
  }
  if (xy.has_value() && yaw.has_value()) {
    return std::make_tuple(xy->first, xy->second, *yaw);
  }
  return std::nullopt;
}

struct CurrentSafeZone
{
  std::optional<SafeZone> value;

  void update(const SafeZone & sz)
  {
    value = sz;
  }

  void clear()
  {
    value.reset();
  }

  bool matches(const SafeZone & other) const
  {
    return value.has_value() && value->id == other.id;
  }

  double distancesq_to_target(const SafeZone & other) const
  {
    if (!value.has_value()) {
      return std::numeric_limits<double>::infinity();
    }
    const auto s_pt = get_safe_zone_point(*value);
    const auto d_pt = get_safe_zone_point(other);
    if (!s_pt.has_value() || !d_pt.has_value()) {
      return std::numeric_limits<double>::infinity();
    }
    const double dx = s_pt->first - d_pt->first;
    const double dy = s_pt->second - d_pt->second;
    return dx * dx + dy * dy;
  }

  bool should_update_target(const SafeZone & other) const
  {
    if (!value.has_value()) {
      return true;
    }
    const auto & current = *value;
    if (current.id.plan_id != other.id.plan_id) {
      return true;
    }
    if (!other.target_waypoint.empty() && current.target_waypoint != other.target_waypoint) {
      return true;
    }
    const auto cur_yaw = get_safe_zone_orientation(current);
    const auto other_yaw = get_safe_zone_orientation(other);
    if (cur_yaw.has_value() && other_yaw.has_value()) {
      if (shortest_angular_distance(*cur_yaw, *other_yaw) > 0.05f) {
        return true;
      }
    }
    return distancesq_to_target(other) >= 0.5;
  }
};

class ProgressTracker
{
public:
  std::optional<PlanId> plan_id;
  std::vector<std::pair<std::array<float, 2>, float>> plan_waypoints;
  std::uint64_t start_waypoint = 0;
  std::uint64_t target_waypoint = 0;
  float target_progress = 0.0f;
  std::pair<float, float> start_xy{0.0f, 0.0f};
  std::pair<float, float> target_xy{0.0f, 0.0f};
  float target_tolerance = 0.25f;
  std::optional<float> target_yaw;
  float target_yaw_tolerance = static_cast<float>(M_PI);
  InnerNavigationAction active_action;
  bool action_completed = false;
  bool gate_active = false;

  static std::string trim_ascii(const std::string & s)
  {
    const auto first = s.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) {
      return "";
    }
    const auto last = s.find_last_not_of(" \t\r\n");
    return s.substr(first, last - first + 1);
  }

  static std::vector<std::pair<std::array<float, 2>, float>> extract_plan_waypoints(
    const Plan & plan)
  {
    std::vector<std::pair<std::array<float, 2>, float>> out;
    out.reserve(plan.waypoints.size());
    for (std::size_t i = 0; i < plan.waypoints.size(); ++i) {
      const auto & wp = plan.waypoints[i];
      const float prog = (i > 0 && wp.progress <= 0.0f) ? static_cast<float>(i) : wp.progress;
      out.push_back({wp.position, prog});
    }
    return out;
  }

  void set_plan(const Plan & plan, const SafeZone * current_safe_zone)
  {
    if (plan_id.has_value() && *plan_id == plan.plan_id) {
      plan_waypoints = extract_plan_waypoints(plan);
      if (current_safe_zone != nullptr && current_safe_zone->id.plan_id == plan.plan_id) {
        const auto [action, has_gate] =
          resolve_target_action(*current_safe_zone, &plan, start_xy, target_xy);
        if (has_gate && !action_completed) {
          gate_active = true;
        }
        if (
          active_action.kind == InnerActionKind::NavigateToPose &&
          action.kind != InnerActionKind::NavigateToPose)
        {
          active_action = action;
        }
      }
    }
  }

  static std::optional<std::pair<InnerNavigationAction, bool>> parse_action_string(
    const std::string & action_str, std::pair<float, float> target_xy)
  {
    const auto trimmed = trim_ascii(action_str);
    if (trimmed.empty()) {
      return std::nullopt;
    }
    const auto lower = KnownDock::to_lower_ascii(trimmed);
    const bool has_gate =
      lower.find("gate") != std::string::npos || lower.find("load") != std::string::npos ||
      lower.find("offload") != std::string::npos || lower.find("off-load") != std::string::npos;

    if (lower.rfind("undock", 0) == 0) {
      std::string dock_type = "simple_charging_dock";
      const auto colon = trimmed.find(':');
      if (colon != std::string::npos) {
        const auto rest = trim_ascii(trimmed.substr(colon + 1));
        if (const auto d = KnownDock::match_by_id(rest)) {
          dock_type = d->dock_type;
        }
      }
      return std::make_pair(InnerNavigationAction::undock_robot(dock_type), has_gate);
    }

    if (lower.rfind("dock", 0) == 0) {
      const auto colon = trimmed.find(':');
      if (colon != std::string::npos) {
        const auto dock_id = trim_ascii(trimmed.substr(colon + 1));
        if (!dock_id.empty()) {
          const auto known = KnownDock::match_by_id(dock_id);
          const std::string dock_type =
            known.has_value() ? known->dock_type : "simple_charging_dock";
          return std::make_pair(
            InnerNavigationAction::dock_robot(known.has_value(), dock_id, dock_type, false),
            has_gate);
        }
      }
      if (const auto dock = KnownDock::match_by_pose(target_xy.first, target_xy.second, 0.45f)) {
        return std::make_pair(
          InnerNavigationAction::dock_robot(true, dock->id, dock->dock_type, false), has_gate);
      }
      return std::make_pair(
        InnerNavigationAction::dock_robot(false, "", "simple_charging_dock", false), has_gate);
    }

    if (has_gate) {
      return std::make_pair(InnerNavigationAction::navigate_to_pose(), true);
    }
    return std::nullopt;
  }

  static std::pair<InnerNavigationAction, bool> resolve_target_action(
    const SafeZone & safe_zone,
    const Plan * plan,
    std::pair<float, float> start_xy,
    std::pair<float, float> target_xy)
  {
    const std::size_t target_wp_idx =
      safe_zone.target_waypoint.empty() ? 0 : static_cast<std::size_t>(safe_zone.target_waypoint[0]);
    const std::size_t last_wp_idx = static_cast<std::size_t>(safe_zone.last_waypoint);

    if (plan != nullptr && plan->plan_id == safe_zone.id.plan_id) {
      if (target_wp_idx < plan->waypoints.size()) {
        const auto & wp = plan->waypoints[target_wp_idx];
        if (const auto res = parse_action_string(wp.arrival_action, target_xy)) {
          return *res;
        }
        for (const auto & node : wp.arrival_constraints.nodes) {
          if (!node.key.name.empty()) {
            if (const auto dock = KnownDock::match_by_id(node.key.name.front())) {
              return {
                InnerNavigationAction::dock_robot(true, dock->id, dock->dock_type, false), false};
            }
          }
        }
      }
      if (last_wp_idx < plan->waypoints.size()) {
        const auto & prev_wp = plan->waypoints[last_wp_idx];
        if (const auto res = parse_action_string(prev_wp.departure_action, target_xy)) {
          return *res;
        }
      }
      if (target_wp_idx > 0 && (target_wp_idx - 1) < plan->waypoints.size()) {
        const auto & prev_wp = plan->waypoints[target_wp_idx - 1];
        if (const auto res = parse_action_string(prev_wp.departure_action, target_xy)) {
          return *res;
        }
      }
    }

    for (const auto & node : safe_zone.incremental_target.nodes) {
      if (!node.key.name.empty()) {
        if (const auto dock = KnownDock::match_by_id(node.key.name.front())) {
          return {
            InnerNavigationAction::dock_robot(true, dock->id, dock->dock_type, false), false};
        }
      }
    }

    if (const auto dock = KnownDock::match_by_pose(target_xy.first, target_xy.second, 0.35f)) {
      const float dist_to_dock = std::hypot(start_xy.first - dock->x, start_xy.second - dock->y);
      if (dist_to_dock > 0.35f) {
        const float dist_to_staging =
          std::hypot(start_xy.first - dock->staging_x, start_xy.second - dock->staging_y);
        const bool nav_to_staging = dist_to_staging > 0.45f;
        return {
          InnerNavigationAction::dock_robot(true, dock->id, dock->dock_type, nav_to_staging),
          false};
      }
    }

    return {InnerNavigationAction::navigate_to_pose(), false};
  }

  bool is_holding_at_pre_waypoint() const
  {
    const bool waiting_for_dock =
      (active_action.kind == InnerActionKind::DockRobot ||
       active_action.kind == InnerActionKind::UndockRobot) &&
      !action_completed;
    return waiting_for_dock || gate_active;
  }

  void release_gate()
  {
    gate_active = false;
  }

  void mark_action_completed()
  {
    action_completed = true;
  }

  static std::pair<std::optional<float>, float> extract_orientation_constraint(
    const SafeZone & safe_zone)
  {
    constexpr float kPi = static_cast<float>(M_PI);
    const TargetOrientation * ori = nullptr;
    if (
      !safe_zone.incremental_target.regions.empty() &&
      !safe_zone.incremental_target.regions.front().orientations.empty())
    {
      ori = &safe_zone.incremental_target.regions.front().orientations.front();
    } else if (
      !safe_zone.incremental_target.nodes.empty() &&
      !safe_zone.incremental_target.nodes.front().orientations.empty())
    {
      ori = &safe_zone.incremental_target.nodes.front().orientations.front();
    }
    if (ori == nullptr) {
      return {std::nullopt, kPi};
    }
    float effective_tol = 0.25f;
    if (ori->tolerance_radians > 0.0f || ori->spread_radians > 0.0f) {
      effective_tol =
        0.5f * std::max(0.0f, ori->spread_radians) + std::max(0.0f, ori->tolerance_radians);
    }
    if (effective_tol < kPi - 1e-3f) {
      return {ori->orientation_radians, effective_tol};
    }
    return {std::nullopt, kPi};
  }

  bool is_orientation_satisfied(float current_yaw) const
  {
    constexpr float kPi = static_cast<float>(M_PI);
    if (target_yaw.has_value() && target_yaw_tolerance < kPi - 1e-3f) {
      return shortest_angular_distance(current_yaw, *target_yaw) <= target_yaw_tolerance;
    }
    return true;
  }

  bool is_target_pose_satisfied(
    const std::optional<std::tuple<float, float, float>> & current_pose) const
  {
    if (
      active_action.kind == InnerActionKind::DockRobot ||
      active_action.kind == InnerActionKind::UndockRobot)
    {
      return action_completed;
    }
    if (!current_pose.has_value()) {
      return true;
    }
    const auto [rx, ry, ryaw] = *current_pose;
    const float dist = std::hypot(rx - target_xy.first, ry - target_xy.second);
    return dist <= target_tolerance && is_orientation_satisfied(ryaw);
  }

  Progress on_safe_zone_with_plan(
    const SafeZone & safe_zone,
    const Plan * plan,
    const std::optional<std::tuple<float, float, float>> & current_pose,
    const Progress * previous)
  {
    const PlanId incoming_plan_id = safe_zone.id.plan_id;
    const std::uint64_t target_wp =
      safe_zone.target_waypoint.empty() ? 0 : safe_zone.target_waypoint.front();
    const auto pt = get_safe_zone_point(safe_zone);
    const std::pair<float, float> new_target_xy =
      pt.has_value()
        ? std::make_pair(static_cast<float>(pt->first), static_cast<float>(pt->second))
        : std::make_pair(0.0f, 0.0f);
    float new_tolerance = 0.25f;
    if (
      !safe_zone.incremental_target.regions.empty() &&
      safe_zone.incremental_target.regions.front().tolerance > 0.0f)
    {
      new_tolerance = safe_zone.incremental_target.regions.front().tolerance;
    }
    const auto [new_yaw, new_yaw_tol] = extract_orientation_constraint(safe_zone);

    const bool same_plan = plan_id.has_value() && *plan_id == incoming_plan_id;
    const std::optional<std::pair<float, float>> current_xy =
      current_pose.has_value()
        ? std::make_optional(
            std::make_pair(std::get<0>(*current_pose), std::get<1>(*current_pose)))
        : std::nullopt;
    const auto fallback_start = current_xy.value_or(new_target_xy);

    if (!same_plan) {
      const auto [action, has_gate] =
        resolve_target_action(safe_zone, plan, fallback_start, new_target_xy);
      if (plan != nullptr && plan->plan_id == incoming_plan_id) {
        plan_waypoints = extract_plan_waypoints(*plan);
      } else {
        plan_waypoints.clear();
      }
      plan_id = incoming_plan_id;
      start_waypoint = safe_zone.last_waypoint;
      target_waypoint = target_wp;
      target_progress = safe_zone.target_progress;
      start_xy = fallback_start;
      target_xy = new_target_xy;
      target_tolerance = new_tolerance;
      target_yaw = new_yaw;
      target_yaw_tolerance = new_yaw_tol;
      active_action = action;
      action_completed = false;
      gate_active = has_gate;
    } else {
      if (plan_waypoints.empty() && plan != nullptr && plan->plan_id == incoming_plan_id) {
        plan_waypoints = extract_plan_waypoints(*plan);
      }
      bool yaw_changed = false;
      if (target_yaw.has_value() && new_yaw.has_value()) {
        yaw_changed = shortest_angular_distance(*target_yaw, *new_yaw) > 0.05f;
      } else if (target_yaw.has_value() != new_yaw.has_value()) {
        yaw_changed = true;
      }
      const bool target_changed =
        target_wp != target_waypoint ||
        std::hypot(new_target_xy.first - target_xy.first, new_target_xy.second - target_xy.second) >
          1e-3f ||
        yaw_changed;
      if (target_changed) {
        const auto new_start_xy = current_xy.value_or(target_xy);
        const auto [action, has_gate] =
          resolve_target_action(safe_zone, plan, new_start_xy, new_target_xy);
        start_xy = new_start_xy;
        start_waypoint = std::max(start_waypoint, safe_zone.last_waypoint);
        active_action = action;
        action_completed = false;
        gate_active = has_gate;
      }
      target_waypoint = target_wp;
      target_progress = safe_zone.target_progress;
      target_xy = new_target_xy;
      target_tolerance = new_tolerance;
      target_yaw = new_yaw;
      target_yaw_tolerance = new_yaw_tol;
    }

    return compute_progress(current_pose, previous);
  }

  Progress compute_progress(
    const std::optional<std::tuple<float, float, float>> & current_pose,
    const Progress * previous) const
  {
    Progress out;
    if (!plan_id.has_value()) {
      return out;
    }
    out.plan_id = *plan_id;

    const bool prev_same = (previous != nullptr && previous->plan_id == *plan_id);
    const std::uint64_t prev_reached = prev_same ? previous->reached_waypoint : 0;
    const float prev_progress = prev_same ? previous->progress : 0.0f;

    std::uint64_t reached_wp = std::max(start_waypoint, prev_reached);
    float progress = std::max(static_cast<float>(reached_wp), prev_progress);

    if (is_holding_at_pre_waypoint()) {
      out.progress = progress;
      out.reached_waypoint = reached_wp;
      out.target_waypoint = target_waypoint;
      return out;
    }

    if (
      (active_action.kind == InnerActionKind::DockRobot ||
       active_action.kind == InnerActionKind::UndockRobot) &&
      action_completed)
    {
      out.progress = std::max(progress, target_progress);
      out.reached_waypoint = std::max(reached_wp, target_waypoint);
      out.target_waypoint = target_waypoint;
      return out;
    }

    if (current_pose.has_value()) {
      const auto [rx, ry, ryaw] = *current_pose;
      const float dist_to_target = std::hypot(rx - target_xy.first, ry - target_xy.second);
      const bool pos_reached = dist_to_target <= target_tolerance;
      const bool ori_reached = is_orientation_satisfied(ryaw);

      if (pos_reached && ori_reached) {
        reached_wp = std::max(reached_wp, target_waypoint);
        progress = std::max(progress, target_progress);
      } else if (reached_wp < target_waypoint) {
        const float sx = start_xy.first;
        const float sy = start_xy.second;
        const float tx = target_xy.first;
        const float ty = target_xy.second;
        const float dx = tx - sx;
        const float dy = ty - sy;
        const float seg_len_sq = dx * dx + dy * dy;
        const float start_prog = static_cast<float>(start_waypoint);
        const float pre_final_cap =
          target_yaw.has_value() ? std::max(target_progress - 1e-3f, start_prog) : target_progress;

        if (pos_reached && !ori_reached) {
          const std::uint64_t pre_rot_wp =
            std::max(start_waypoint, target_waypoint > 0 ? target_waypoint - 1 : 0);
          reached_wp = std::max(reached_wp, pre_rot_wp);
          const float seg_len = std::sqrt(seg_len_sq);
          const float interp = std::min(
            std::max(start_prog + seg_len, static_cast<float>(pre_rot_wp)), pre_final_cap);
          progress = std::min(std::max(progress, interp), target_progress);
        } else {
          const std::size_t target_idx =
            plan_waypoints.empty()
              ? 0
              : std::min(
                  static_cast<std::size_t>(target_waypoint), plan_waypoints.size() - 1);
          const std::size_t search_start =
            std::max(static_cast<std::size_t>(reached_wp), static_cast<std::size_t>(start_waypoint));
          std::optional<std::pair<float, float>> best_seg_prog;

          if (!plan_waypoints.empty() && search_start < target_idx) {
            for (std::size_t i = search_start; i < target_idx; ++i) {
              const auto & [p0, prog0] = plan_waypoints[i];
              const auto & [p1, prog1] = plan_waypoints[i + 1];
              const float sdx = p1[0] - p0[0];
              const float sdy = p1[1] - p0[1];
              const float slen_sq = sdx * sdx + sdy * sdy;
              if (slen_sq <= 1e-6f) {
                continue;
              }
              const float t =
                std::clamp(((rx - p0[0]) * sdx + (ry - p0[1]) * sdy) / slen_sq, 0.0f, 1.0f);
              const float proj_x = p0[0] + t * sdx;
              const float proj_y = p0[1] + t * sdy;
              const float dist_sq = (rx - proj_x) * (rx - proj_x) + (ry - proj_y) * (ry - proj_y);
              const float seg_prog = prog0 + t * (prog1 - prog0);
              if (!best_seg_prog.has_value() || dist_sq + 1e-3f < best_seg_prog->first) {
                best_seg_prog = std::make_pair(dist_sq, seg_prog);
              }
            }
          }

          if (best_seg_prog.has_value()) {
            const float interp = std::clamp(best_seg_prog->second, start_prog, pre_final_cap);
            progress = std::min(std::max(progress, interp), target_progress);
            if (target_waypoint > start_waypoint + 1) {
              const std::uint64_t intermediate_wp = std::min(
                static_cast<std::uint64_t>(std::floor(interp)), target_waypoint - 1);
              reached_wp = std::max(reached_wp, intermediate_wp);
            }
          } else if (seg_len_sq > 1e-6f) {
            const float seg_len = std::sqrt(seg_len_sq);
            const float t = std::clamp(((rx - sx) * dx + (ry - sy) * dy) / seg_len_sq, 0.0f, 1.0f);
            const float dist_along_seg = t * seg_len;
            const float wp_span = std::max(0.0f, target_progress - start_prog);
            const float interp = std::min(start_prog + std::min(dist_along_seg, wp_span), pre_final_cap);
            progress = std::min(std::max(progress, interp), target_progress);
            if (target_waypoint > start_waypoint + 1) {
              const std::uint64_t intermediate_wp = std::min(
                static_cast<std::uint64_t>(std::floor(interp)), target_waypoint - 1);
              reached_wp = std::max(reached_wp, intermediate_wp);
            }
          }
        }
      }
    }

    out.progress = progress;
    out.reached_waypoint = reached_wp;
    out.target_waypoint = target_waypoint;
    return out;
  }

  std::optional<Progress> mark_target_reached(const Progress * previous)
  {
    action_completed = true;
    if (!plan_id.has_value()) {
      return std::nullopt;
    }
    const bool prev_same = (previous != nullptr && previous->plan_id == *plan_id);
    const std::uint64_t prev_reached = prev_same ? previous->reached_waypoint : 0;
    const float prev_progress = prev_same ? previous->progress : 0.0f;

    Progress out;
    out.plan_id = *plan_id;
    out.target_waypoint = target_waypoint;

    if (gate_active) {
      out.reached_waypoint = std::max(start_waypoint, prev_reached);
      out.progress = std::max(static_cast<float>(out.reached_waypoint), prev_progress);
      return out;
    }

    out.reached_waypoint = std::max(prev_reached, target_waypoint);
    out.progress = std::max(prev_progress, target_progress);
    return out;
  }
};

inline Progress build_progress_status(
  const PoseStamped & pose,
  std::uint8_t mode,
  const Progress * progress,
  const SafeZoneId * safe_zone_id)
{
  Progress msg = (progress != nullptr) ? *progress : Progress{};
  msg.pose = pose;
  msg.mode = mode;
  msg.safe_zone_id.clear();
  if (safe_zone_id != nullptr) {
    msg.safe_zone_id.push_back(*safe_zone_id);
  }
  return msg;
}

inline bool is_safe_zone_newer(const SafeZoneId & candidate, const SafeZoneId & current)
{
  if (candidate.plan_id.destination_session.uuid != current.plan_id.destination_session.uuid) {
    return true;
  }
  if (candidate.plan_id.plan_version != current.plan_id.plan_version) {
    return candidate.plan_id.plan_version > current.plan_id.plan_version;
  }
  return candidate.safe_zone_version > current.safe_zone_version;
}

inline bool should_preempt_goal(
  const SafeZoneId & candidate_id,
  const InnerNavigationAction & candidate_action,
  const SafeZoneId & current_id,
  const InnerNavigationAction & current_action)
{
  return is_safe_zone_newer(candidate_id, current_id) ||
         (candidate_id == current_id && candidate_action != current_action);
}

inline RosUuid generate_random_uuid()
{
  static thread_local std::mt19937_64 rng{std::random_device{}()};
  std::uniform_int_distribution<int> dist(0, 255);
  RosUuid u;
  for (auto & b : u.uuid) {
    b = static_cast<std::uint8_t>(dist(rng));
  }
  return u;
}

inline DestinationGoal pose_stamped_to_destination_goal(
  const PoseStamped & pose, const RosUuid & session)
{
  const float x = static_cast<float>(pose.pose.position.x);
  const float y = static_cast<float>(pose.pose.position.y);
  const auto & q = pose.pose.orientation;
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  const float yaw = static_cast<float>(std::atan2(siny_cosp, cosy_cosp));

  Region region;
  region.points = {x, y};
  region.hint = Region::HINT_POINT;

  TargetOrientation target_ori;
  target_ori.orientation_radians = yaw;
  target_ori.spread_radians = 0.0f;
  target_ori.tolerance_radians = 0.0f;

  TargetRegion target_region;
  target_region.tolerance = 0.5f;
  target_region.region = region;
  target_region.orientations = {target_ori};

  DestinationConstraints constraints;
  constraints.regions = {target_region};

  DestinationGoal goal;
  goal.one_of = {constraints};
  goal.session = session;
  return goal;
}

}  // namespace rmf_nav2_traffic

#endif  // RMF_NAV2_TRAFFIC__SAFE_ZONE_TRACKER_HPP_
