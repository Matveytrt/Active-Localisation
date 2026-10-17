// Route lever core (spec T-009 section 3): best-over-yaw score per cell mapped
// to a costmap cost.
//
//   s*(x, y) = max_k s(x, y, psi_k),  psi_k = -pi + 2 pi k / yaw_bins
//   cost     = round(c_max * (1 - min(s* / s_ref, 1))^p),  round = floor(v + 0.5)
//
// Mirrors active_localization/cost_field.layer_cost_grid (the oracle). Header-only.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "al_nav2_plugins/scorer.hpp"

namespace al_nav2_plugins
{

constexpr int kMaxLayerCost = 252;   // 253+ = INSCRIBED / LETHAL / NO_INFORMATION

struct CostFieldParams
{
  int yaw_bins{36};
  double c_max{40.0};
  double power{1.0};
};

inline int layerCostFromScore(double s_star, const CostFieldParams & p)
{
  const double f = 1.0 - std::min(s_star / sRef(), 1.0);
  const double v = p.c_max * std::pow(std::max(f, 0.0), p.power);
  const int c = static_cast<int>(std::floor(v + 0.5));
  return std::clamp(c, 0, kMaxLayerCost);
}

// Best score over the yaw bins for a base position.
inline double bestScore(const MarkerScorer & scorer, double x, double y, int yaw_bins)
{
  double best = -1.0;
  for (int k = 0; k < yaw_bins; ++k) {
    best = std::max(best, scorer.score(x, y, -M_PI + 2.0 * M_PI * k / yaw_bins));
  }
  return best;
}

struct CellGrid
{
  double origin_x{0.0};
  double origin_y{0.0};
  double resolution{0.05};
  unsigned int size_x{0};
  unsigned int size_y{0};
};

// Every cell is evaluated (no distance cut-off; the oracle notes that a cut-off
// at r_max + 0.5 m already deviates by up to 2 cost units). Cell centres are
// origin + (i + 0.5) * res. Returns size_x * size_y values, row-major.
inline std::vector<std::uint8_t> computeLayerCosts(
  const MarkerScorer & scorer, const CellGrid & g, const CostFieldParams & p)
{
  std::vector<std::uint8_t> out(static_cast<std::size_t>(g.size_x) * g.size_y, 0);
  for (unsigned int j = 0; j < g.size_y; ++j) {
    const double y = g.origin_y + (j + 0.5) * g.resolution;
    for (unsigned int i = 0; i < g.size_x; ++i) {
      const double x = g.origin_x + (i + 0.5) * g.resolution;
      out[static_cast<std::size_t>(j) * g.size_x + i] = static_cast<std::uint8_t>(
        layerCostFromScore(bestScore(scorer, x, y, p.yaw_bins), p));
    }
  }
  return out;
}

}  // namespace al_nav2_plugins
