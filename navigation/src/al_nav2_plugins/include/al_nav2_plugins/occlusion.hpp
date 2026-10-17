// Occlusion helpers. The C++ plugins ray-cast the costmap's lethal cells (spec
// T-009 section 2); the oracle uses axis-aligned shelf rectangles. Both live
// here so the gtests can compare them. Header-only, no ROS.
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace al_nav2_plugins
{

// Read-only view of a row-major cost grid (cell (i, j) -> data[j * size_x + i]),
// e.g. nav2_costmap_2d::Costmap2D::getCharMap(). Cell (i, j) covers
// [origin + i*res, origin + (i+1)*res) in x (same for y).
struct GridView
{
  double origin_x{0.0};
  double origin_y{0.0};
  double resolution{0.05};
  unsigned int size_x{0};
  unsigned int size_y{0};
  const std::uint8_t * data{nullptr};
  std::uint8_t lethal_threshold{254};   // cost >= this blocks the view

  bool worldToCell(double x, double y, int & i, int & j) const
  {
    const double fx = std::floor((x - origin_x) / resolution);
    const double fy = std::floor((y - origin_y) / resolution);
    if (fx < 0.0 || fy < 0.0 || fx >= size_x || fy >= size_y) {
      return false;
    }
    i = static_cast<int>(fx);
    j = static_cast<int>(fy);
    return true;
  }

  // Cells outside the grid are free.
  bool blockedAt(double x, double y) const
  {
    int i = 0, j = 0;
    if (data == nullptr || !worldToCell(x, y, i, j)) {
      return false;
    }
    return data[static_cast<std::size_t>(j) * size_x + static_cast<std::size_t>(i)] >=
           lethal_threshold;
  }

  // Samples the segment every `sample_fraction * resolution` (both ends included).
  bool segmentBlocked(
    double ax, double ay, double bx, double by, double sample_fraction = 0.5) const
  {
    const double len = std::hypot(bx - ax, by - ay);
    const int n = std::max(1, static_cast<int>(std::ceil(len / (sample_fraction * resolution))));
    for (int k = 0; k <= n; ++k) {
      const double t = static_cast<double>(k) / n;
      if (blockedAt(ax + t * (bx - ax), ay + t * (by - ay))) {
        return true;
      }
    }
    return false;
  }
};

using Rect = std::array<double, 4>;   // xmin, xmax, ymin, ymax

// True if the segment a -> b touches the closed rectangle (Liang-Barsky slab
// test, same as the oracle's segments_hit_rects).
inline bool segmentHitsRect(double ax, double ay, double bx, double by, const Rect & r)
{
  const double a[2] = {ax, ay};
  const double d[2] = {bx - ax, by - ay};
  const double lo[2] = {r[0], r[2]};
  const double hi[2] = {r[1], r[3]};
  double t_in = 0.0;
  double t_out = 1.0;
  for (int k = 0; k < 2; ++k) {
    if (std::fabs(d[k]) < 1e-12) {
      if (a[k] < lo[k] || a[k] > hi[k]) {
        return false;
      }
    } else {
      const double t1 = (lo[k] - a[k]) / d[k];
      const double t2 = (hi[k] - a[k]) / d[k];
      t_in = std::max(t_in, std::min(t1, t2));
      t_out = std::min(t_out, std::max(t1, t2));
    }
  }
  return t_in <= t_out;
}

inline bool segmentHitsRects(
  double ax, double ay, double bx, double by, const std::vector<Rect> & rects)
{
  for (const auto & r : rects) {
    if (segmentHitsRect(ax, ay, bx, by, r)) {
      return true;
    }
  }
  return false;
}

}  // namespace al_nav2_plugins
