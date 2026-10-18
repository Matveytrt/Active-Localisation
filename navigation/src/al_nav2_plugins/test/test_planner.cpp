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

// LocalizationAwarePlanner on a synthetic costmap (spec T-009 section 4): wiring of the inner
// Theta* planner, the heading DP, the start / goal pins, the fallbacks and the costmap-based
// occlusion. Needs ROS (rclcpp, nav2_costmap_2d, nav2_theta_star_planner).
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "al_nav2_plugins/geometry.hpp"
#include "al_nav2_plugins/landmarks_io.hpp"
#include "al_nav2_plugins/localization_aware_planner.hpp"
#include "al_nav2_plugins/pose_utils.hpp"
#include "al_nav2_plugins/scorer.hpp"
#include "al_nav2_plugins/yaw_dp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace
{
constexpr char kLandmarks[] = AL_FIXTURE_DIR "/landmarks_sample.yaml";
constexpr char kName[] = "GridBased";
constexpr double kRes = 0.05;
constexpr double kOriginX = -3.0;   // map: x in [-3, 5], y in [-1.5, 4.5]
constexpr double kOriginY = -1.5;
constexpr double kPathY = -0.2;     // below the shelf row with id 1 and 2 (y = 0.9)

geometry_msgs::msg::PoseStamped makePose(double x, double y, double yaw)
{
  geometry_msgs::msg::PoseStamped p;
  p.header.frame_id = "map";
  p.pose.position.x = x;
  p.pose.position.y = y;
  al_nav2_plugins::quaternionFromYaw(yaw, p.pose.orientation.z, p.pose.orientation.w);
  return p;
}

double yawOf(const geometry_msgs::msg::Pose & p)
{
  const auto & q = p.orientation;
  return al_nav2_plugins::yawFromQuaternion(q.x, q.y, q.z, q.w);
}
}  // namespace

class PlannerTest : public ::testing::Test
{
public:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

protected:
  void SetUp() override
  {
    costmap_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
      "global_costmap", "/", "global_costmap");
    costmap_ros_->set_parameter(rclcpp::Parameter("plugins", std::vector<std::string>{}));
    costmap_ros_->set_parameter(rclcpp::Parameter("global_frame", std::string("map")));
    costmap_ros_->set_parameter(rclcpp::Parameter("robot_base_frame", std::string("base_link")));
    costmap_ros_->set_parameter(rclcpp::Parameter("resolution", kRes));
    costmap_ros_->set_parameter(rclcpp::Parameter("width", 8));
    costmap_ros_->set_parameter(rclcpp::Parameter("height", 6));
    costmap_ros_->set_parameter(rclcpp::Parameter("origin_x", kOriginX));
    costmap_ros_->set_parameter(rclcpp::Parameter("origin_y", kOriginY));
    ASSERT_EQ(costmap_ros_->configure().label(), "inactive");
  }

  void TearDown() override
  {
    if (planner_) {
      planner_->cleanup();
      planner_.reset();
    }
    node_.reset();
    costmap_ros_->cleanup();
    costmap_ros_.reset();
  }

  // Creates the planner node with `scorer` ("marker_soft_count" or "null") and configures it.
  void makePlanner(const std::string & scorer)
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides(
      {rclcpp::Parameter(std::string(kName) + ".scorer", scorer),
        rclcpp::Parameter(std::string(kName) + ".landmarks_file", std::string(kLandmarks)),
        rclcpp::Parameter(
          std::string(kName) + ".marker_ids", std::vector<int64_t>{1, 2, 3, 4, 5})});
    node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>("al_planner_test", options);
    planner_ = std::make_unique<al_nav2_plugins::LocalizationAwarePlanner>();
    planner_->configure(node_, kName, nullptr, costmap_ros_);
    planner_->activate();
  }

  // Lethal row across the whole map at world y (blocks every view through it).
  void addLethalRow(double y)
  {
    nav2_costmap_2d::Costmap2D * costmap = costmap_ros_->getCostmap();
    unsigned int mx = 0;
    unsigned int my = 0;
    ASSERT_TRUE(costmap->worldToMap(0.0, y, mx, my));
    for (unsigned int i = 0; i < costmap->getSizeInCellsX(); ++i) {
      costmap->setCost(i, my, nav2_costmap_2d::LETHAL_OBSTACLE);
    }
  }

  static std::vector<al_nav2_plugins::Point2> xyOf(const nav_msgs::msg::Path & path)
  {
    std::vector<al_nav2_plugins::Point2> xy;
    for (const auto & ps : path.poses) {
      xy.push_back({ps.pose.position.x, ps.pose.position.y});
    }
    return xy;
  }

  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
  std::unique_ptr<al_nav2_plugins::LocalizationAwarePlanner> planner_;
};

// Control run: scorer "null" -> the DP only has the forward prior and the pins, so every
// heading is the direction of travel (+x here).
TEST_F(PlannerTest, NullScorerFollowsTravelDirection) {
  makePlanner("null");
  const auto start = makePose(-1.0, kPathY, 0.0);
  const auto goal = makePose(2.0, kPathY, 0.0);
  const auto path = planner_->createPlan(start, goal);

  ASSERT_GE(path.poses.size(), 3u);
  EXPECT_EQ(path.header.frame_id, "map");
  for (std::size_t k = 0; k < path.poses.size(); ++k) {
    EXPECT_NEAR(yawOf(path.poses[k].pose), 0.0, 1e-6) << k;
  }
}

// The goal pose is copied exactly; the start heading is pinned to the start pose.
TEST_F(PlannerTest, GoalPoseIsExactAndStartIsPinned) {
  makePlanner("marker_soft_count");
  const auto start = makePose(-1.0, kPathY, 0.5);
  const auto goal = makePose(2.0, kPathY, -0.7);
  const auto path = planner_->createPlan(start, goal);

  ASSERT_GE(path.poses.size(), 3u);
  EXPECT_DOUBLE_EQ(path.poses.back().pose.position.x, goal.pose.position.x);
  EXPECT_DOUBLE_EQ(path.poses.back().pose.position.y, goal.pose.position.y);
  EXPECT_NEAR(yawOf(path.poses.back().pose), -0.7, 1e-9);
  EXPECT_NEAR(yawOf(path.poses.front().pose), 0.5, 1e-9);
  EXPECT_LT(
    std::hypot(
      path.poses.front().pose.position.x - start.pose.position.x,
      path.poses.front().pose.position.y - start.pose.position.y), 0.1);
}

// Plumbing check: the planner's headings equal the header-only DP run directly on the same
// points, and the lever actually turns the camera towards the markers.
TEST_F(PlannerTest, MatchesDirectDpAndTurnsTowardsMarkers) {
  makePlanner("marker_soft_count");
  const auto start = makePose(-1.0, kPathY, 0.0);
  const auto goal = makePose(2.0, kPathY, 0.0);
  const auto path = planner_->createPlan(start, goal);
  ASSERT_GE(path.poses.size(), 5u);

  const al_nav2_plugins::ScorerParams sp;   // defaults == planner defaults (spec v2)
  al_nav2_plugins::MarkerScorer scorer(
    al_nav2_plugins::loadLandmarks(kLandmarks, {1, 2, 3, 4, 5}, 0.0, -1.0), sp);
  al_nav2_plugins::YawDpParams dp;
  dp.step = 0.0;   // the planner already resampled; do not resample again
  const auto xy = xyOf(path);
  const auto expected = al_nav2_plugins::planYaws(
    xy, [&scorer](double x, double y, double yaw) {return scorer.score(x, y, yaw);}, dp,
    0.0, 0.0);
  ASSERT_EQ(expected.yaw.size(), path.poses.size());

  double max_yaw = 0.0;
  double score_sum = 0.0;
  for (std::size_t k = 0; k < path.poses.size(); ++k) {
    const double yaw = yawOf(path.poses[k].pose);
    EXPECT_NEAR(yaw, expected.yaw[k], 1e-9) << k;
    max_yaw = std::max(max_yaw, yaw);
    score_sum += scorer.score(xy[k].x, xy[k].y, yaw);
  }
  // Facing +y (towards the row of markers at y = 0.9) somewhere along the path ...
  EXPECT_GT(max_yaw, 1.0);
  // ... and seeing a marker most of the time (a forward-only path would score ~0 here).
  EXPECT_GT(score_sum / static_cast<double>(path.poses.size()), 0.3);
}

// The planner's occluders are the lethal cells of the live global costmap: with a wall between
// the path and every marker nothing is visible, so the headings fall back to the forward prior.
TEST_F(PlannerTest, LethalCellsOccludeMarkers) {
  addLethalRow(0.3);
  makePlanner("marker_soft_count");
  const auto start = makePose(-1.0, kPathY, 0.0);
  const auto goal = makePose(2.0, kPathY, 0.0);
  const auto path = planner_->createPlan(start, goal);

  ASSERT_GE(path.poses.size(), 3u);
  for (std::size_t k = 0; k < path.poses.size(); ++k) {
    EXPECT_NEAR(yawOf(path.poses[k].pose), 0.0, 1e-6) << k;
  }
}

// Spec 4.4: a path with fewer than 2 poses is returned as the inner planner produced it.
TEST_F(PlannerTest, DegeneratePathDoesNotThrow) {
  makePlanner("marker_soft_count");
  const auto pose = makePose(0.0, kPathY, 0.0);
  nav_msgs::msg::Path path;
  EXPECT_NO_THROW(path = planner_->createPlan(pose, pose));
  EXPECT_LE(path.poses.size(), 2u);
}

TEST_F(PlannerTest, UnknownScorerNameIsRejected) {
  rclcpp::NodeOptions options;
  options.parameter_overrides(
    {rclcpp::Parameter(
        std::string(kName) + ".scorer",
        std::string("bogus"))});
  node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>("al_planner_bad", options);
  al_nav2_plugins::LocalizationAwarePlanner planner;
  EXPECT_THROW(planner.configure(node_, kName, nullptr, costmap_ros_), std::runtime_error);
}
