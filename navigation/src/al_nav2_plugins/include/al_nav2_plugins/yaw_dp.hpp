// Yaw lever (spec T-009 section 4): choose a heading for every waypoint of a
// fixed (x, y) path by Viterbi dynamic programming over a discrete heading set.
//
//   node(j, b)   = - w_s * s(x_j, y_j, psi_b) / s_ref + w_f * (1 - cos(psi_b - phi_j))
//   edge(j, a->b) = w_sm * d^2 + w_lim * max(0, |d| - k * ds_{j-1})^2,  d = wrap(psi_b - psi_a)
//   pins: psi_0 = start yaw, psi_N = goal yaw (cost 1e6 * (1 - cos) on the pinned nodes)
//
// Same algorithm as active_localization/yaw_planner.py (the oracle); argmin
// takes the lowest index on ties, like numpy. Header-only, no ROS.
#pragma once

#include <cmath>
#include <cstddef>
#include <functional>
#include <limits>
#include <optional>
#include <vector>

#include "al_nav2_plugins/geometry.hpp"
#include "al_nav2_plugins/scorer.hpp"

namespace al_nav2_plugins
{

struct YawDpParams
{
  int n_bins{36};
  double step{0.20};            // resampling step along the path [m]
  double w_score{1.0};          // w_s
  double w_forward{0.3};        // w_f
  double w_smooth{0.5};         // w_sm
  double w_limit{100.0};        // w_lim
  double max_dyaw_per_m{1.0};   // k [rad/m]
  double pin_cost{1.0e6};       // weight of the start / goal pins
  double score_scale{-1.0};     // < 0: use sRef()
  bool pin_start{true};
  bool pin_goal{true};
};

struct YawPlan
{
  std::vector<Point2> xy;        // resampled path
  std::vector<double> yaw;       // chosen headings (pins exact)
  std::vector<double> score;     // score at the chosen bins
  std::vector<double> travel;    // direction of travel
  std::vector<int> bin_idx;      // chosen bin per waypoint
  double total_cost{0.0};        // minimum of the DP objective
};

using ScoreFn = std::function<double (double x, double y, double yaw)>;

inline YawPlan planYaws(
  const std::vector<Point2> & path, const ScoreFn & score, const YawDpParams & p,
  std::optional<double> start_yaw = std::nullopt,
  std::optional<double> goal_yaw = std::nullopt)
{
  YawPlan out;
  out.xy = (p.step > 0.0) ? resamplePolyline(path, p.step) : path;
  const std::size_t n = out.xy.size();
  if (n == 0) {
    return out;
  }
  const int nb = p.n_bins;
  out.travel = pathHeadings(out.xy);

  std::vector<double> bins(static_cast<std::size_t>(nb));
  for (int b = 0; b < nb; ++b) {
    bins[static_cast<std::size_t>(b)] = -M_PI + 2.0 * M_PI * b / nb;
  }
  const double scale = std::max(p.score_scale > 0.0 ? p.score_scale : sRef(), 1e-9);

  // node costs (N x B)
  std::vector<double> s_val(n * static_cast<std::size_t>(nb));
  std::vector<double> node(n * static_cast<std::size_t>(nb));
  for (std::size_t j = 0; j < n; ++j) {
    for (int b = 0; b < nb; ++b) {
      const std::size_t k = j * static_cast<std::size_t>(nb) + static_cast<std::size_t>(b);
      const double sv = score(out.xy[j].x, out.xy[j].y, bins[static_cast<std::size_t>(b)]);
      s_val[k] = sv;
      node[k] = -p.w_score * sv / scale +
        p.w_forward * (1.0 - std::cos(bins[static_cast<std::size_t>(b)] - out.travel[j]));
    }
  }
  if (p.pin_start && start_yaw) {
    for (int b = 0; b < nb; ++b) {
      node[static_cast<std::size_t>(b)] += p.pin_cost *
        (1.0 - std::cos(bins[static_cast<std::size_t>(b)] - *start_yaw));
    }
  }
  if (p.pin_goal && goal_yaw) {
    for (int b = 0; b < nb; ++b) {
      node[(n - 1) * static_cast<std::size_t>(nb) + static_cast<std::size_t>(b)] += p.pin_cost *
        (1.0 - std::cos(bins[static_cast<std::size_t>(b)] - *goal_yaw));
    }
  }

  // |wrap(psi_b - psi_a)|, rows = a (from), cols = b (to)
  std::vector<double> dyaw(static_cast<std::size_t>(nb) * static_cast<std::size_t>(nb));
  for (int a = 0; a < nb; ++a) {
    for (int b = 0; b < nb; ++b) {
      dyaw[static_cast<std::size_t>(a * nb + b)] = std::fabs(
        wrapAngle(bins[static_cast<std::size_t>(b)] - bins[static_cast<std::size_t>(a)]));
    }
  }

  std::vector<double> cost(node.begin(), node.begin() + nb);
  std::vector<int> back(n * static_cast<std::size_t>(nb), 0);
  std::vector<double> next(static_cast<std::size_t>(nb));
  for (std::size_t j = 1; j < n; ++j) {
    const double ds = std::hypot(out.xy[j].x - out.xy[j - 1].x, out.xy[j].y - out.xy[j - 1].y);
    const double allowed = p.max_dyaw_per_m * ds;
    for (int b = 0; b < nb; ++b) {
      double best = std::numeric_limits<double>::infinity();
      int arg = 0;
      for (int a = 0; a < nb; ++a) {
        const double d = dyaw[static_cast<std::size_t>(a * nb + b)];
        const double excess = std::max(0.0, d - allowed);
        const double total = cost[static_cast<std::size_t>(a)] + p.w_smooth * d * d +
          p.w_limit * excess * excess;
        if (total < best) {   // strict: lowest index wins ties
          best = total;
          arg = a;
        }
      }
      back[j * static_cast<std::size_t>(nb) + static_cast<std::size_t>(b)] = arg;
      next[static_cast<std::size_t>(b)] = best + node[j * static_cast<std::size_t>(nb) +
          static_cast<std::size_t>(b)];
    }
    cost = next;
  }

  out.bin_idx.assign(n, 0);
  int last = 0;
  for (int b = 1; b < nb; ++b) {
    if (cost[static_cast<std::size_t>(b)] < cost[static_cast<std::size_t>(last)]) {
      last = b;
    }
  }
  out.total_cost = cost[static_cast<std::size_t>(last)];
  out.bin_idx[n - 1] = last;
  for (std::size_t j = n - 1; j > 0; --j) {
    out.bin_idx[j - 1] = back[j * static_cast<std::size_t>(nb) +
        static_cast<std::size_t>(out.bin_idx[j])];
  }

  out.yaw.resize(n);
  out.score.resize(n);
  for (std::size_t j = 0; j < n; ++j) {
    const int b = out.bin_idx[j];
    out.yaw[j] = bins[static_cast<std::size_t>(b)];
    out.score[j] = s_val[j * static_cast<std::size_t>(nb) + static_cast<std::size_t>(b)];
  }
  if (p.pin_start && start_yaw) {
    out.yaw.front() = wrapAngle(*start_yaw);
  }
  if (p.pin_goal && goal_yaw) {
    out.yaw.back() = wrapAngle(*goal_yaw);
  }
  return out;
}

}  // namespace al_nav2_plugins
