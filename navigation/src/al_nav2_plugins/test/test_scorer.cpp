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

// S2* scorer vs the Python oracle's golden fixtures (scorer_cases.json) and the
// check values of spec T-009 section 2.
#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "al_nav2_plugins/scorer.hpp"
#include "fixture_common.hpp"

using al_nav2_plugins::MarkerScorer;

namespace
{
constexpr double kTolS = 1e-6;   // spec: s within 1e-6

double tol(double v) {return kTolS * std::max(1.0, std::fabs(v));}
}  // namespace

TEST(Scorer, MatchesOracleFixtures) {
  const auto root = fixture::load("scorer_cases.json");
  const auto & h = root["header"];
  MarkerScorer scorer(fixture::markersFromHeader(h), fixture::paramsFromHeader(h));

  // The fixtures carry occ per marker (oracle rectangle rule); feed it back so
  // the comparison does not depend on the occlusion implementation.
  const mini_json::Value * current = nullptr;
  scorer.setOcclusion(
    [&current](std::size_t i, double, double, double, double) {
      return (*current)["markers"][i]["occ"].d() < 0.5;
    });

  int n = 0;
  for (const auto & c : root["cases"].arr) {
    current = &c;
    const auto d = scorer.detail(c["x"].d(), c["y"].d(), c["yaw"].d());
    const std::string name = c["name"].s();
    EXPECT_NEAR(d.s, c["s"].d(), kTolS) << name;
    EXPECT_NEAR(d.n_eff, c["n_eff"].d(), kTolS) << name;
    ASSERT_EQ(d.markers.size(), c["markers"].size());
    for (std::size_t i = 0; i < d.markers.size(); ++i) {
      const auto & e = c["markers"][i];
      const auto & t = d.markers[i];
      EXPECT_NEAR(t.r, e["r"].d(), tol(e["r"].d())) << name << " marker " << i;
      EXPECT_NEAR(t.beta, e["beta"].d(), kTolS) << name << " marker " << i;
      EXPECT_NEAR(t.cos_theta, e["cos_theta"].d(), kTolS) << name << " marker " << i;
      EXPECT_NEAR(t.area, e["area"].d(), tol(e["area"].d())) << name << " marker " << i;
      EXPECT_NEAR(t.g_fov, e["g_fov"].d(), kTolS) << name << " marker " << i;
      EXPECT_NEAR(t.g_area, e["g_area"].d(), kTolS) << name << " marker " << i;
      EXPECT_NEAR(t.g_near, e["g_near"].d(), kTolS) << name << " marker " << i;
      EXPECT_NEAR(t.w, e["w"].d(), kTolS) << name << " marker " << i;
    }
    // lazy fast path must agree with the detailed evaluation
    EXPECT_NEAR(scorer.score(c["x"].d(), c["y"].d(), c["yaw"].d()), d.s, 1e-12) << name;
    ++n;
  }
  EXPECT_EQ(n, static_cast<int>(root["cases"].size()));
  EXPECT_GE(n, 50);
}

// Spec T-009 section 2 check values (no occlusion).
TEST(Scorer, SpecCheckValues) {
  al_nav2_plugins::ScorerParams p;   // defaults = spec values
  const al_nav2_plugins::Marker id1{1, 0.19940729439258575, 0.8983477354049683, 0.0, -1.0};
  MarkerScorer scorer({id1}, p);
  const double xm = id1.x;
  const double yaw = M_PI / 2.0;   // facing +y, towards the marker

  auto at_camera_range = [&](double r) {
      // camera is 0.07 m ahead of base_link along the heading
      return scorer.detail(xm, id1.y - r - p.cam_offset_x, yaw);
    };

  // v2 parameters (k_a 20, r_min 0.66, k_r 40): the defaults of ScorerParams.
  auto d1 = at_camera_range(1.0);
  EXPECT_NEAR(d1.markers[0].w, 1.0, 5e-3);
  EXPECT_NEAR(d1.s, 0.63, 5e-3);

  auto d2 = at_camera_range(1.88);   // area == A_min -> w ~ 0.5
  EXPECT_NEAR(d2.markers[0].area, 505.6, 0.5);
  EXPECT_NEAR(d2.markers[0].w, 0.556, 0.01);   // sigma(20 * (505.6 / 500 - 1))

  auto d3 = at_camera_range(0.6);
  EXPECT_NEAR(d3.markers[0].w, 0.083, 0.005);   // sigma(40 * (0.6 - 0.66)); spec v2: ~0.08

  // no detection at 0.61 m, detection at 0.71 m (ENG-10), centred by r_min = 0.66
  EXPECT_NEAR(at_camera_range(0.61).markers[0].g_near, 0.12, 0.01);
  EXPECT_NEAR(at_camera_range(0.71).markers[0].g_near, 0.88, 0.01);

  // turned 90 deg away from the marker
  EXPECT_LT(scorer.score(xm, id1.y - 1.0 - p.cam_offset_x, yaw + M_PI / 2.0), 1e-6);

  // seen from behind: soft floor sigma(-k_a), not clamped to 0 (oracle behaviour)
  const auto behind = scorer.detail(xm, id1.y + 1.0, -M_PI / 2.0);
  EXPECT_NEAR(behind.markers[0].g_area, al_nav2_plugins::sigmoid(-p.k_a), 1e-12);
}

TEST(Scorer, RangeGuardAtMarker) {
  al_nav2_plugins::ScorerParams p;
  const al_nav2_plugins::Marker m{1, 1.0, 1.0, 0.0, -1.0};
  MarkerScorer scorer({m}, p);
  // camera exactly on the marker: finite result, no NaN
  const double s = scorer.score(1.0 - p.cam_offset_x, 1.0, 0.0);
  EXPECT_TRUE(std::isfinite(s));
  EXPECT_GE(s, 0.0);
}

TEST(Scorer, WrapAngleConvention) {
  using al_nav2_plugins::wrapAngle;
  EXPECT_NEAR(wrapAngle(M_PI), -M_PI, 1e-12);    // [-pi, pi)
  EXPECT_NEAR(wrapAngle(-M_PI), -M_PI, 1e-12);
  EXPECT_NEAR(wrapAngle(7.0), 7.0 - 2.0 * M_PI, 1e-12);
  EXPECT_NEAR(wrapAngle(-7.0), -7.0 + 2.0 * M_PI, 1e-12);
}
