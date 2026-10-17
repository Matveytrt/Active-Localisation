#include "al_nav2_plugins/localization_aware_planner.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <exception>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>

#include "al_nav2_plugins/occlusion.hpp"
#include "al_nav2_plugins/pose_utils.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace al_nav2_plugins
{

namespace
{
double poseYaw(const geometry_msgs::msg::Pose & p)
{
  const auto & q = p.orientation;
  return yawFromQuaternion(q.x, q.y, q.z, q.w);
}
}  // namespace

LocalizationAwarePlanner::~LocalizationAwarePlanner()
{
  inner_.reset();   // before loader_ (also guaranteed by member order)
}

void LocalizationAwarePlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  auto node = parent.lock();
  if (!node) {
    throw std::runtime_error("LocalizationAwarePlanner: failed to lock the lifecycle node");
  }
  logger_ = node->get_logger();
  name_ = std::move(name);
  costmap_ros_ = costmap_ros;

  // Scorer (same keys as the layer) and the heading DP.
  setup_ = loadScorerSetup(node, name_);
  const YawDpParams d;
  dp_.n_bins = paramInt(node, name_, "yaw_bins", d.n_bins);
  dp_.step = paramDouble(node, name_, "step", d.step);
  dp_.w_score = paramDouble(node, name_, "w_s", d.w_score);
  dp_.w_forward = paramDouble(node, name_, "w_f", d.w_forward);
  dp_.w_smooth = paramDouble(node, name_, "w_sm", d.w_smooth);
  dp_.max_dyaw_per_m = paramDouble(node, name_, "k_yaw_rate", d.max_dyaw_per_m);
  dp_.w_limit = paramDouble(node, name_, "w_lim", d.w_limit);
  if (dp_.n_bins < 2) {
    throw std::runtime_error(name_ + ".yaw_bins must be >= 2");
  }
  if (dp_.step <= 0.0) {
    throw std::runtime_error(name_ + ".step must be > 0");
  }

  // Inner planner, configured under "<name>.inner" so its parameters live in
  // GridBased.inner.* (values unchanged from the baseline Theta*).
  const std::string inner_plugin = paramString(
    node, name_, "inner_plugin", "nav2_theta_star_planner/ThetaStarPlanner");
  loader_ = std::make_unique<pluginlib::ClassLoader<nav2_core::GlobalPlanner>>(
    "nav2_core", "nav2_core::GlobalPlanner");
  inner_ = loader_->createUniqueInstance(inner_plugin);
  inner_->configure(parent, name_ + ".inner", tf, costmap_ros);

  RCLCPP_INFO(
    logger_,
    "%s: inner planner %s, scorer=%s (%zu markers), %d yaw bins, step %.2f m, "
    "w_s=%.2f w_f=%.2f w_sm=%.2f k=%.2f w_lim=%.0f",
    name_.c_str(), inner_plugin.c_str(), setup_.kind.c_str(), setup_.markers.size(), dp_.n_bins,
    dp_.step, dp_.w_score, dp_.w_forward, dp_.w_smooth, dp_.max_dyaw_per_m, dp_.w_limit);
}

void LocalizationAwarePlanner::cleanup()
{
  if (inner_) {
    inner_->cleanup();
  }
}

void LocalizationAwarePlanner::activate()
{
  if (inner_) {
    inner_->activate();
  }
}

void LocalizationAwarePlanner::deactivate()
{
  if (inner_) {
    inner_->deactivate();
  }
}

nav_msgs::msg::Path LocalizationAwarePlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start, const geometry_msgs::msg::PoseStamped & goal)
{
  nav_msgs::msg::Path path = inner_->createPlan(start, goal);
  if (path.poses.size() < 2) {
    RCLCPP_WARN(logger_, "%s: inner planner returned %zu poses, headings left untouched",
      name_.c_str(), path.poses.size());
    return path;
  }
  try {
    return orient(path, start, goal);
  } catch (const std::exception & e) {
    RCLCPP_WARN(logger_, "%s: heading DP failed (%s), returning the inner path unchanged",
      name_.c_str(), e.what());
    return path;
  }
}

nav_msgs::msg::Path LocalizationAwarePlanner::orient(
  const nav_msgs::msg::Path & path, const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal) const
{
  const auto t0 = std::chrono::steady_clock::now();

  std::vector<Point2> xy;
  xy.reserve(path.poses.size());
  for (const auto & ps : path.poses) {
    xy.push_back({ps.pose.position.x, ps.pose.position.y});
  }

  ScoreFn score_fn = [](double, double, double) {return 0.0;};   // "null": forward prior only
  std::vector<std::uint8_t> snapshot;   // occlusion source; must outlive the DP
  MarkerScorer scorer(setup_.markers, setup_.params);
  if (!setup_.isNull()) {
    // Occluders = lethal cells of the live global costmap (copied under its lock).
    nav2_costmap_2d::Costmap2D * costmap = costmap_ros_->getCostmap();
    GridView view;
    {
      std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap->getMutex()));
      const unsigned int sx = costmap->getSizeInCellsX();
      const unsigned int sy = costmap->getSizeInCellsY();
      const unsigned char * data = costmap->getCharMap();
      snapshot.assign(data, data + static_cast<std::size_t>(sx) * sy);
      view.origin_x = costmap->getOriginX();
      view.origin_y = costmap->getOriginY();
      view.resolution = costmap->getResolution();
      view.size_x = sx;
      view.size_y = sy;
    }
    view.data = snapshot.data();
    view.lethal_threshold = nav2_costmap_2d::LETHAL_OBSTACLE;
    scorer.setOcclusion(
      [view](std::size_t, double cx, double cy, double qx, double qy) {
        return view.segmentBlocked(cx, cy, qx, qy);
      });
    score_fn = [&scorer](double x, double y, double yaw) {return scorer.score(x, y, yaw);};
  }

  const double start_yaw = poseYaw(start.pose);
  const double goal_yaw = poseYaw(goal.pose);
  const YawPlan plan = planYaws(xy, score_fn, dp_, start_yaw, goal_yaw);
  if (plan.xy.size() < 2 || plan.yaw.size() != plan.xy.size()) {
    throw std::runtime_error("DP returned an empty plan");
  }

  nav_msgs::msg::Path out;
  out.header = path.header;
  out.poses.reserve(plan.xy.size());
  for (std::size_t k = 0; k < plan.xy.size(); ++k) {
    geometry_msgs::msg::PoseStamped ps;
    ps.header = path.header;
    ps.pose.position.x = plan.xy[k].x;
    ps.pose.position.y = plan.xy[k].y;
    ps.pose.position.z = 0.0;
    quaternionFromYaw(plan.yaw[k], ps.pose.orientation.z, ps.pose.orientation.w);
    out.poses.push_back(ps);
  }
  out.poses.back().pose = goal.pose;   // the goal pose stays exact

  RCLCPP_DEBUG(
    logger_, "%s: %zu -> %zu poses, DP cost %.3f, %.1f ms", name_.c_str(), path.poses.size(),
    out.poses.size(), plan.total_cost,
    std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count());
  return out;
}

}  // namespace al_nav2_plugins

PLUGINLIB_EXPORT_CLASS(al_nav2_plugins::LocalizationAwarePlanner, nav2_core::GlobalPlanner)
