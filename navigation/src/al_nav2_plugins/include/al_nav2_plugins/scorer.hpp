// S2* scorer (ADR-001, spec T-009 section 2): soft count of detectable markers.
//
//   w_i = sig(k_fov (HFOV/2 - |beta_i|)) * sig(k_a (A_i / a_min - 1))
//         * sig(k_r (r_i - r_min)) * occ_i
//   A_i = (f s / r_i)^2 * max(cos theta_i, 0)        [px^2]
//   n_eff = sum_i w_i,   s = 1 - exp(-n_eff / M)
//
// Identical to active_localization/scorers/marker_soft_count.py (the oracle);
// the gtests compare against its JSON fixtures. Header-only, no ROS.
#pragma once

#include <cmath>
#include <cstddef>
#include <functional>
#include <utility>
#include <vector>

#include "al_nav2_plugins/geometry.hpp"

namespace al_nav2_plugins
{

// Score at which the layer cost drops to 0 and the DP normalises its node cost.
inline double sRef() {return 1.0 - std::exp(-1.0);}

struct Marker
{
  int id{0};
  double x{0.0};
  double y{0.0};
  double nx{0.0};   // unit outward face normal (map frame)
  double ny{-1.0};
};

// Defaults = spec T-009 section 2 / FACTS 18.10. Every value is overridable
// from the plugin YAML.
struct ScorerParams
{
  double hfov{1.089};
  double k_fov{40.0};
  double a_min{500.0};
  double k_a{10.0};
  double r_min{0.70};
  double k_r{20.0};
  double focal_px{528.4};
  double marker_size{0.08};
  double m_sat{1.0};
  double cam_offset_x{0.07};
  double cam_offset_y{0.0};
  double cam_offset_yaw{0.0};
  double occ_stop{0.20};   // occlusion segment ends this far in front of the marker
};

// Returns true if the segment camera (cx, cy) -> target (qx, qy) towards marker
// `marker_idx` is blocked by the static map.
using OcclusionFn = std::function<bool (std::size_t marker_idx, double cx, double cy,
    double qx, double qy)>;

struct MarkerTerms
{
  double r{0}, beta{0}, cos_theta{0}, area{0};
  double g_fov{0}, g_area{0}, g_near{0}, occ{1}, w{0};
};

struct ScoreDetail
{
  std::vector<MarkerTerms> markers;
  double n_eff{0.0};
  double s{0.0};
};

inline double sigmoid(double z) {return 0.5 * (1.0 + std::tanh(0.5 * z));}

class MarkerScorer
{
public:
  // Occlusion is only evaluated for a marker whose other gates are above this
  // (the product is < 1e-14 otherwise, far below every tolerance).
  static constexpr double kOcclusionSkipBelow = 1e-14;
  static constexpr double kRangeGuard = 1e-9;

  MarkerScorer(std::vector<Marker> markers, const ScorerParams & params)
  : markers_(std::move(markers)), p_(params)
  {
    for (auto & m : markers_) {
      const double n = std::hypot(m.nx, m.ny);
      if (n < kRangeGuard) {
        m.nx = 0.0;
        m.ny = -1.0;
      } else {
        m.nx /= n;
        m.ny /= n;
      }
    }
  }

  void setOcclusion(OcclusionFn f) {occ_ = std::move(f);}
  const std::vector<Marker> & markers() const {return markers_;}
  const ScorerParams & params() const {return p_;}

  // s in [0, 1); occlusion evaluated lazily (fast path for the layer / DP).
  double score(double x, double y, double yaw) const
  {
    return eval(x, y, yaw, nullptr);
  }

  // All intermediate terms; occlusion always evaluated (used by the tests).
  ScoreDetail detail(double x, double y, double yaw) const
  {
    ScoreDetail d;
    d.markers.resize(markers_.size());
    d.s = eval(x, y, yaw, &d);
    return d;
  }

private:
  double eval(double x, double y, double yaw, ScoreDetail * d) const
  {
    const double c = std::cos(yaw);
    const double s = std::sin(yaw);
    const double cx = x + c * p_.cam_offset_x - s * p_.cam_offset_y;
    const double cy = y + s * p_.cam_offset_x + c * p_.cam_offset_y;
    const double cyaw = yaw + p_.cam_offset_yaw;
    const double fs = p_.focal_px * p_.marker_size;

    double n_eff = 0.0;
    for (std::size_t i = 0; i < markers_.size(); ++i) {
      const Marker & m = markers_[i];
      const double dx = m.x - cx;
      const double dy = m.y - cy;
      const double r = std::hypot(dx, dy);
      const double rg = std::max(r, kRangeGuard);
      const double beta = wrapAngle(std::atan2(dy, dx) - cyaw);
      const double cos_t = (-dx * m.nx - dy * m.ny) / rg;
      const double ratio = fs / rg;
      const double area = ratio * ratio * std::max(cos_t, 0.0);
      const double g_fov = sigmoid(p_.k_fov * (0.5 * p_.hfov - std::fabs(beta)));
      const double g_area = sigmoid(p_.k_a * (area / p_.a_min - 1.0));
      const double g_near = sigmoid(p_.k_r * (r - p_.r_min));
      const double w_other = g_fov * g_area * g_near;

      double occ = 1.0;
      if (occ_ && (d != nullptr || w_other > kOcclusionSkipBelow)) {
        const bool blocked = occ_(i, cx, cy, m.x + p_.occ_stop * m.nx, m.y + p_.occ_stop * m.ny);
        occ = blocked ? 0.0 : 1.0;
      }
      const double w = w_other * occ;
      n_eff += w;
      if (d != nullptr) {
        d->markers[i] = {r, beta, cos_t, area, g_fov, g_area, g_near, occ, w};
      }
    }
    const double sc = 1.0 - std::exp(-n_eff / p_.m_sat);
    if (d != nullptr) {
      d->n_eff = n_eff;
      d->s = sc;
    }
    return sc;
  }

  std::vector<Marker> markers_;
  ScorerParams p_;
  OcclusionFn occ_;
};

}  // namespace al_nav2_plugins
