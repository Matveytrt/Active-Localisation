// Route lever (spec T-009 section 3): a static costmap layer that makes cells where no
// marker can be detected (best over yaw) cost up to c_max, so Theta* prefers to pass in
// front of the markers when the detour is cheap. Global costmap only.
#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "al_nav2_plugins/cost_field.hpp"
#include "al_nav2_plugins/ros_params.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "rclcpp/logger.hpp"

namespace al_nav2_plugins
{

class LocalizabilityLayer : public nav2_costmap_2d::Layer
{
public:
  LocalizabilityLayer() = default;

  void onInitialize() override;
  void updateBounds(
    double robot_x, double robot_y, double robot_yaw, double * min_x, double * min_y,
    double * max_x, double * max_y) override;
  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j, int max_i, int max_j) override;
  void matchSize() override;
  void reset() override;
  bool isClearable() override {return false;}

private:
  struct GridSignature
  {
    unsigned int size_x{0};
    unsigned int size_y{0};
    double resolution{0.0};
    double origin_x{0.0};
    double origin_y{0.0};

    bool operator==(const GridSignature & o) const;
  };

  static GridSignature signatureOf(const nav2_costmap_2d::Costmap2D & m);

  // Picks the costmap that holds the *static* map (the StaticLayer, once its map has
  // arrived). Returns nullptr while we should keep waiting for it; the master costmap
  // is used when there is no StaticLayer or after kMaxWaitCycles.
  const nav2_costmap_2d::Costmap2D * occlusionSource(const nav2_costmap_2d::Costmap2D & master);

  // Computes costs_ for the master grid geometry (once; again only if it changes).
  void ensureCosts(const nav2_costmap_2d::Costmap2D & master);

  static constexpr int kMaxWaitCycles = 20;

  rclcpp::Logger logger_{rclcpp::get_logger("LocalizabilityLayer")};
  bool enabled_{true};
  ScorerSetup setup_;
  CostFieldParams cost_params_;
  std::vector<std::uint8_t> costs_;   // row-major, master grid geometry
  GridSignature costs_grid_;
  bool computed_{false};
  bool bounds_pending_{true};
  int wait_cycles_{0};
};

}  // namespace al_nav2_plugins
