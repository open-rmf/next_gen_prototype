/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  Copyright (c) 2020, Samsung R&D Institute Russia
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Eitan Marder-Eppstein
 *         David V. Lu!!
 *         Alexey Merzlyakov
 *
 * Reference tutorial:
 * https://navigation.ros.org/tutorials/docs/writing_new_costmap2d_plugin.html
 *********************************************************************/
#include "geometry_msgs/msg/pose_array.hpp"
#include "spatio_temporal_partition_layer/spatio_temporal_partition_layer.hpp"

#include "nav2_costmap_2d/costmap_math.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "rclcpp/parameter_events_filter.hpp"
#include "rclcpp/logging.hpp"
#include <cstdint>
#include <exception>
#include <nav2_costmap_2d/cost_values.hpp>
#include <curl/curl.h>
#include <nlohmann/json.hpp>
#include <optional>

using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;
using nav2_costmap_2d::FREE_SPACE;

using json = nlohmann::json;

namespace rmf
{

void strip_trailing_slash(std::string& str) {
    if (!str.empty() && str.back() == '/') {
        str.pop_back();
    }
}

int extract_robot_id(std::string& name) {
  int robot_id;
  if (sscanf(name.c_str(), "/robot%d/", &robot_id) == 1) {
    // Successfully extracted robot_id
  }
  else {
    throw std::exception();
  }
  return robot_id;
}

SpatioTemporalPartitionLayer::SpatioTemporalPartitionLayer()
: last_min_x_(-std::numeric_limits<float>::max()),
  last_min_y_(-std::numeric_limits<float>::max()),
  last_max_x_(std::numeric_limits<float>::max()),
  last_max_y_(std::numeric_limits<float>::max())
{
}

// This method is called at the end of plugin initialization.
// It contains ROS parameter(s) declaration and initialization
// of need_recalculation_ variable.
void
SpatioTemporalPartitionLayer::onInitialize()
{
  auto node = node_.lock();
  declareParameter("enabled", rclcpp::ParameterValue(true));
  node->get_parameter(name_ + "." + "enabled", enabled_);
  costmap_sub_ = node->create_subscription<nav2_msgs::msg::Costmap>("plan/costmap", 10, 
    std::bind(&SpatioTemporalPartitionLayer::costmap_cb, this, std::placeholders::_1));
  
  need_recalculation_ = false;
  current_ = true;
  RCLCPP_ERROR(logger_, "Initializing grid");
  current_robot_ = node->get_namespace();
  strip_trailing_slash(current_robot_);
}

// The method is called to ask the plugin: which area of costmap it needs to update.
// Inside this method window bounds are re-calculated if need_recalculation_ is true
// and updated independently on its value.
void
SpatioTemporalPartitionLayer::updateBounds(
  double robot_x, double robot_y, double /*robot_yaw*/, double * min_x,
  double * min_y, double * max_x, double * max_y)
{
  robot_x_ = robot_x;
  robot_y_ = robot_y;
  pose_recvd_ = true;
  if (need_recalculation_) {
    last_min_x_ = *min_x;
    last_min_y_ = *min_y;
    last_max_x_ = *max_x;
    last_max_y_ = *max_y;
    // For some reason when I make these -<double>::max() it does not
    // work with Costmap2D::worldToMapEnforceBounds(), so I'm using
    // -<float>::max() instead.
    *min_x = -std::numeric_limits<float>::max();
    *min_y = -std::numeric_limits<float>::max();
    *max_x = std::numeric_limits<float>::max();
    *max_y = std::numeric_limits<float>::max();
    need_recalculation_ = false;
  } else {
    double tmp_min_x = last_min_x_;
    double tmp_min_y = last_min_y_;
    double tmp_max_x = last_max_x_;
    double tmp_max_y = last_max_y_;
    last_min_x_ = *min_x;
    last_min_y_ = *min_y;
    last_max_x_ = *max_x;
    last_max_y_ = *max_y;
    *min_x = std::min(tmp_min_x, *min_x);
    *min_y = std::min(tmp_min_y, *min_y);
    *max_x = std::max(tmp_max_x, *max_x);
    *max_y = std::max(tmp_max_y, *max_y);
  }
}

void SpatioTemporalPartitionLayer::costmap_cb(nav2_msgs::msg::Costmap::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(mutex_);
  costmap_msg_ = msg;
}

// Callback function to write curl response to a string
size_t WriteCallback(void* contents, size_t size, size_t nmemb, std::string* userp) {
    userp->append((char*)contents, size * nmemb);
    return size * nmemb;
}

// The method is called when footprint was changed.
// Here it just resets need_recalculation_ variable.
void
SpatioTemporalPartitionLayer::onFootprintChanged()
{
  need_recalculation_ = true;

  RCLCPP_DEBUG(rclcpp::get_logger(
      "nav2_costmap_2d"), "GradientLayer::onFootprintChanged(): num footprint points: %lu",
    layered_costmap_->getFootprint().size());
}

// The method is called when costmap recalculation is required.
// It updates the costmap within its window bounds.
// Inside this method the costmap gradient is generated and is writing directly
// to the resulting costmap master_grid without any merging with previous layers.
void
SpatioTemporalPartitionLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j,
  int max_i,
  int max_j)
{
  if (!enabled_) {
    return;
  }
  if (!pose_recvd_) {
    RCLCPP_ERROR(logger_, "No pose received");
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (!costmap_msg_) {
    RCLCPP_WARN(logger_, "No costmap message received yet");
    return;
  }

  if (costmap_msg_->data.empty()) {
    RCLCPP_WARN(logger_, "Costmap data is empty");
    return;
  }

  // NOTE(@xiyuoh) may need transformation if robot TF is different from server
  const auto & costmap = costmap_msg_->metadata;
  for (int j = min_j; j < max_j; ++j) {
    for (int i = min_i; i < max_i; ++i) {

      double world_x, world_y;
      master_grid.mapToWorld(i, j, world_x, world_y);
      int costmap_i = std::floor((world_x - costmap.origin.position.x) / costmap.resolution);
      int costmap_j = std::floor((world_y - costmap.origin.position.y) / costmap.resolution);

      if (costmap_i >= 0 && costmap_i < static_cast<int>(costmap.size_x) &&
          costmap_j >= 0 && costmap_j < static_cast<int>(costmap.size_y)) 
      {
        unsigned int costmap_idx = costmap_j * costmap.size_x + costmap_i;
        if (costmap_idx >= costmap_msg_->data.size()) {
          RCLCPP_WARN(logger_, "Costmap index %u out of bounds (data size: %zu)", 
                     costmap_idx, costmap_msg_->data.size());
          continue;
        }

        uint8_t costmap_cost = costmap_msg_->data[costmap_idx];
        if (costmap_cost == nav2_costmap_2d::NO_INFORMATION) {
          continue;
        }
        uint8_t old_cost = master_grid.getCost(i, j);
        if (old_cost == nav2_costmap_2d::NO_INFORMATION || old_cost < costmap_cost) {
          master_grid.setCost(i, j, costmap_cost);
        }
      }
    }
  }
}

}  // namespace rmf

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(rmf::SpatioTemporalPartitionLayer, nav2_costmap_2d::Layer)
