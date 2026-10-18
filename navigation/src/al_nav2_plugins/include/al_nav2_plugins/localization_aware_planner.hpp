// Copyright 2026 Matvey Razgulov
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

// Yaw lever (spec T-009 section 4): a global planner plugin that wraps an inner planner
// (Theta*) and rewrites the headings of the returned path with the Viterbi DP of
// yaw_dp.hpp. The BT keeps calling ComputePathToPose with planner_id "GridBased"; only
// the plugin behind that id changes in the active params file.
#pragma once

#include <memory>
#include <string>

#include "al_nav2_plugins/ros_params.hpp"
#include "al_nav2_plugins/yaw_dp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"

namespace al_nav2_plugins
{

class LocalizationAwarePlanner : public nav2_core::GlobalPlanner
{
public:
  LocalizationAwarePlanner() = default;
  ~LocalizationAwarePlanner() override;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  void cleanup() override;
  void activate() override;
  void deactivate() override;

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

private:
  // Headings for `path`; throws on any failure (the caller falls back to the inner path).
  nav_msgs::msg::Path orient(
    const nav_msgs::msg::Path & path, const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) const;

  rclcpp::Logger logger_{rclcpp::get_logger("LocalizationAwarePlanner")};
  std::string name_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;

  // The loader must outlive the instance it created (members are destroyed in reverse).
  std::unique_ptr<pluginlib::ClassLoader<nav2_core::GlobalPlanner>> loader_;
  pluginlib::UniquePtr<nav2_core::GlobalPlanner> inner_;

  ScorerSetup setup_;
  YawDpParams dp_;
};

}  // namespace al_nav2_plugins
