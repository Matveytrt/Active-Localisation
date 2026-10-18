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

#include "al_nav2_plugins/localizability_layer.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>

#include "al_nav2_plugins/occlusion.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_costmap_2d/static_layer.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace al_nav2_plugins
{

bool LocalizabilityLayer::GridSignature::operator==(const GridSignature & o) const
{
  return size_x == o.size_x && size_y == o.size_y && std::fabs(resolution - o.resolution) < 1e-9 &&
         std::fabs(origin_x - o.origin_x) < 1e-9 && std::fabs(origin_y - o.origin_y) < 1e-9;
}

LocalizabilityLayer::GridSignature LocalizabilityLayer::signatureOf(
  const nav2_costmap_2d::Costmap2D & m)
{
  return {m.getSizeInCellsX(), m.getSizeInCellsY(), m.getResolution(), m.getOriginX(),
    m.getOriginY()};
}

void LocalizabilityLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("LocalizabilityLayer: failed to lock the lifecycle node");
  }
  logger_ = node->get_logger();

  enabled_ = paramBool(node, name_, "enabled", true);
  setup_ = loadScorerSetup(node, name_);
  const CostFieldParams d;
  cost_params_.yaw_bins = paramInt(node, name_, "yaw_bins", d.yaw_bins);
  // c_max is an integer cost value (of 252); the YAML writes it as `c_max: 40`.
  cost_params_.c_max =
    static_cast<double>(paramInt(node, name_, "c_max", static_cast<int>(d.c_max)));
  cost_params_.power = paramDouble(node, name_, "power", d.power);
  if (cost_params_.yaw_bins < 1) {
    throw std::runtime_error(name_ + ".yaw_bins must be >= 1");
  }
  if (cost_params_.c_max < 0.0 || cost_params_.c_max > kMaxLayerCost) {
    throw std::runtime_error(name_ + ".c_max must be in [0, 252]");
  }

  current_ = true;
  RCLCPP_INFO(
    logger_, "%s: scorer=%s, %zu markers, c_max=%.0f, yaw_bins=%d, power=%.2f%s", name_.c_str(),
    setup_.kind.c_str(), setup_.markers.size(), cost_params_.c_max, cost_params_.yaw_bins,
    cost_params_.power, enabled_ ? "" : " (disabled)");
}

void LocalizabilityLayer::updateBounds(
  double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/, double * min_x, double * min_y,
  double * max_x, double * max_y)
{
  if (!enabled_ || !bounds_pending_) {
    return;
  }
  // Like StaticLayer: touch the whole map until the costs have been written once.
  const nav2_costmap_2d::Costmap2D * master = layered_costmap_->getCostmap();
  *min_x = std::min(*min_x, master->getOriginX());
  *min_y = std::min(*min_y, master->getOriginY());
  *max_x = std::max(*max_x, master->getOriginX() + master->getSizeInMetersX());
  *max_y = std::max(*max_y, master->getOriginY() + master->getSizeInMetersY());
}

const nav2_costmap_2d::Costmap2D * LocalizabilityLayer::occlusionSource(
  const nav2_costmap_2d::Costmap2D & master)
{
  const GridSignature want = signatureOf(master);
  bool has_static_layer = false;
  for (const auto & layer : *layered_costmap_->getPlugins()) {
    const auto static_layer = std::dynamic_pointer_cast<nav2_costmap_2d::StaticLayer>(layer);
    if (!static_layer) {
      continue;
    }
    has_static_layer = true;
    if (!(signatureOf(*static_layer) == want)) {
      continue;
    }
    // The static map has arrived once it contains an obstacle.
    const unsigned char * data = static_layer->getCharMap();
    const std::size_t n = static_cast<std::size_t>(want.size_x) * want.size_y;
    if (std::any_of(
        data, data + n, [](unsigned char c) {return c >= nav2_costmap_2d::LETHAL_OBSTACLE;}))
    {
      return static_layer.get();
    }
  }
  if (!has_static_layer || ++wait_cycles_ >= kMaxWaitCycles) {
    if (has_static_layer) {
      RCLCPP_WARN(
        logger_, "%s: static map not available after %d cycles, using the master costmap for "
        "occlusion", name_.c_str(), kMaxWaitCycles);
    }
    return &master;
  }
  return nullptr;
}

void LocalizabilityLayer::ensureCosts(const nav2_costmap_2d::Costmap2D & master)
{
  const GridSignature sig = signatureOf(master);
  if (computed_ && sig == costs_grid_) {
    return;
  }
  const std::size_t n = static_cast<std::size_t>(sig.size_x) * sig.size_y;

  if (setup_.isNull()) {   // control run: the layer is inert
    costs_.assign(n, 0);
    costs_grid_ = sig;
    computed_ = true;
    return;
  }

  const nav2_costmap_2d::Costmap2D * source = occlusionSource(master);
  if (source == nullptr) {
    return;   // static map not here yet; try again next cycle
  }

  const auto t0 = std::chrono::steady_clock::now();
  const unsigned char * data = source->getCharMap();
  const std::vector<std::uint8_t> snapshot(data, data + n);
  GridView view;
  view.origin_x = sig.origin_x;
  view.origin_y = sig.origin_y;
  view.resolution = sig.resolution;
  view.size_x = sig.size_x;
  view.size_y = sig.size_y;
  view.data = snapshot.data();
  view.lethal_threshold = nav2_costmap_2d::LETHAL_OBSTACLE;

  MarkerScorer scorer(setup_.markers, setup_.params);
  scorer.setOcclusion(
    [&view](std::size_t, double cx, double cy, double qx, double qy) {
      return view.segmentBlocked(cx, cy, qx, qy);
    });
  const CellGrid grid{sig.origin_x, sig.origin_y, sig.resolution, sig.size_x, sig.size_y};
  costs_ = computeLayerCosts(scorer, grid, cost_params_);
  costs_grid_ = sig;
  computed_ = true;

  const double ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - t0).count();
  RCLCPP_INFO(
    logger_, "%s: layer costs computed for %ux%u cells at %.3f m in %.0f ms (occlusion from %s)",
    name_.c_str(), sig.size_x, sig.size_y, sig.resolution, ms,
    source == &master ? "master costmap" : "static layer");
}

void LocalizabilityLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_) {
    return;
  }
  ensureCosts(master_grid);
  if (!computed_) {
    return;
  }
  bounds_pending_ = false;

  unsigned char * master_array = master_grid.getCharMap();
  const unsigned int span = master_grid.getSizeInCellsX();
  const int size_x = static_cast<int>(master_grid.getSizeInCellsX());
  const int size_y = static_cast<int>(master_grid.getSizeInCellsY());
  min_i = std::max(min_i, 0);
  min_j = std::max(min_j, 0);
  max_i = std::min(max_i, size_x);
  max_j = std::min(max_j, size_y);
  for (int j = min_j; j < max_j; ++j) {
    for (int i = min_i; i < max_i; ++i) {
      const std::size_t idx = static_cast<std::size_t>(j) * span + static_cast<std::size_t>(i);
      // Additive rule of spec v2; unknown / inscribed / lethal cells are left alone.
      master_array[idx] = combineCost(master_array[idx], costs_[idx]);
    }
  }
}

void LocalizabilityLayer::matchSize()
{
  computed_ = false;
  bounds_pending_ = true;
}

void LocalizabilityLayer::reset()
{
  computed_ = false;
  bounds_pending_ = true;
  wait_cycles_ = 0;
}

}  // namespace al_nav2_plugins

PLUGINLIB_EXPORT_CLASS(al_nav2_plugins::LocalizabilityLayer, nav2_costmap_2d::Layer)
