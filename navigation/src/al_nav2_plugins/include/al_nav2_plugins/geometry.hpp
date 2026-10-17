// Small SE(2) helpers shared by the scorer, the yaw DP and the Nav2 plugins.
// Pure C++17, no ROS. Mirrors active_localization/geometry.py (the oracle).
#pragma once

#include <algorithm>
#include <cmath>
#include <vector>

namespace al_nav2_plugins
{

struct Point2
{
  double x{0.0};
  double y{0.0};
};

// Wrap to [-pi, pi). Same convention as numpy's (a + pi) % (2 pi) - pi.
inline double wrapAngle(double a)
{
  const double two_pi = 2.0 * M_PI;
  double m = std::fmod(a + M_PI, two_pi);
  if (m < 0.0) {
    m += two_pi;
  }
  return m - M_PI;
}

// Resample a polyline at (approximately) uniform arc length `step`.
// Keeps the first and the last point exactly. Mirrors geometry.resample_polyline.
inline std::vector<Point2> resamplePolyline(const std::vector<Point2> & xy, double step)
{
  if (xy.size() < 2) {
    return xy;
  }
  std::vector<double> s(xy.size(), 0.0);
  for (std::size_t i = 1; i < xy.size(); ++i) {
    s[i] = s[i - 1] + std::hypot(xy[i].x - xy[i - 1].x, xy[i].y - xy[i - 1].y);
  }
  const double total = s.back();
  if (total <= 1e-9) {
    return {xy.front(), xy.back()};
  }
  const int n = std::max(static_cast<int>(std::ceil(total / step)), 1);
  const double h = total / n;
  std::vector<Point2> out;
  out.reserve(static_cast<std::size_t>(n) + 1);
  for (int k = 0; k <= n; ++k) {
    const double sk = (k == n) ? total : k * h;
    // np.interp: last j with s[j] <= sk, linear inside the segment.
    const auto it = std::upper_bound(s.begin(), s.end(), sk);
    std::size_t j = static_cast<std::size_t>(it - s.begin());
    j = (j == 0) ? 0 : j - 1;
    if (j + 1 >= xy.size()) {
      out.push_back(xy.back());
      continue;
    }
    const double ds = s[j + 1] - s[j];
    const double sx = (xy[j + 1].x - xy[j].x) / ds;
    const double sy = (xy[j + 1].y - xy[j].y) / ds;
    out.push_back({xy[j].x + sx * (sk - s[j]), xy[j].y + sy * (sk - s[j])});
  }
  return out;
}

// Direction of travel at every point (last point repeats). geometry.path_headings.
inline std::vector<double> pathHeadings(const std::vector<Point2> & xy)
{
  std::vector<double> h(xy.size(), 0.0);
  if (xy.size() < 2) {
    return h;
  }
  for (std::size_t i = 0; i + 1 < xy.size(); ++i) {
    h[i] = std::atan2(xy[i + 1].y - xy[i].y, xy[i + 1].x - xy[i].x);
  }
  h.back() = h[h.size() - 2];
  return h;
}

}  // namespace al_nav2_plugins
