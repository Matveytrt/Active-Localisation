// Yaw DP vs the Python oracle (yaw_dp_case.json) plus behavioural checks.
#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "al_nav2_plugins/geometry.hpp"
#include "al_nav2_plugins/scorer.hpp"
#include "al_nav2_plugins/yaw_dp.hpp"
#include "fixture_common.hpp"

using al_nav2_plugins::MarkerScorer;
using al_nav2_plugins::Point2;
using al_nav2_plugins::YawDpParams;

namespace
{
YawDpParams dpFromHeader(const mini_json::Value & h)
{
  YawDpParams p;
  const auto & d = h["dp"];
  p.n_bins = d["n_bins"].i();
  p.step = d["step"].d();
  p.w_score = d["w_score"].d();
  p.w_forward = d["w_forward"].d();
  p.w_smooth = d["w_smooth"].d();
  p.w_limit = d["w_limit"].d();
  p.max_dyaw_per_m = d["max_dyaw_per_m"].d();
  p.pin_start = d["pin_start"].boolean;
  p.pin_goal = d["pin_goal"].boolean;
  return p;
}
}  // namespace

TEST(YawDp, ResamplingMatchesOracle) {
  const auto root = fixture::load("yaw_dp_case.json");
  std::vector<Point2> poly;
  for (const auto & q : root["polyline"].arr) {
    poly.push_back({q[0].d(), q[1].d()});
  }
  const auto pts = al_nav2_plugins::resamplePolyline(poly, root["header"]["dp"]["step"].d());
  ASSERT_EQ(pts.size(), root["points"].size());
  for (std::size_t i = 0; i < pts.size(); ++i) {
    EXPECT_NEAR(pts[i].x, root["points"][i][0].d(), 1e-9) << i;
    EXPECT_NEAR(pts[i].y, root["points"][i][1].d(), 1e-9) << i;
  }
  const auto trav = al_nav2_plugins::pathHeadings(pts);
  for (std::size_t i = 0; i < trav.size(); ++i) {
    EXPECT_NEAR(trav[i], root["travel"][i].d(), 1e-9) << i;
  }
}

TEST(YawDp, MatchesOracle) {
  const auto root = fixture::load("yaw_dp_case.json");
  const auto & h = root["header"];
  MarkerScorer scorer(fixture::markersFromHeader(h), fixture::paramsFromHeader(h));
  scorer.setOcclusion(fixture::rectOcclusion(fixture::rectsFromHeader(h)));

  std::vector<Point2> poly;
  for (const auto & q : root["polyline"].arr) {
    poly.push_back({q[0].d(), q[1].d()});
  }
  const auto plan = al_nav2_plugins::planYaws(
    poly, [&scorer](double x, double y, double yaw) {return scorer.score(x, y, yaw);},
    dpFromHeader(h), h["start_yaw"].d(), h["goal_yaw"].d());

  ASSERT_EQ(plan.xy.size(), root["points"].size());
  int idx_mismatch = 0;
  for (std::size_t i = 0; i < plan.xy.size(); ++i) {
    if (plan.bin_idx[i] != root["bin_idx"][i].i()) {
      ++idx_mismatch;
    }
    EXPECT_NEAR(plan.yaw[i], root["yaw"][i].d(), 1e-9) << "yaw " << i;
    EXPECT_NEAR(plan.score[i], root["score_at_chosen_bin"][i].d(), 1e-6) << "score " << i;
  }
  EXPECT_EQ(idx_mismatch, 0);
  EXPECT_NEAR(plan.total_cost, root["total_cost"].d(), 1e-6);
}

TEST(YawDp, NullScorerFollowsTravelDirection) {
  // Without a score the forward prior makes the heading follow the path
  // (control run = baseline-like), pins respected.
  std::vector<Point2> poly = {{0.0, 0.0}, {3.0, 0.0}};
  YawDpParams p;
  const auto plan = al_nav2_plugins::planYaws(
    poly, [](double, double, double) {return 0.0;}, p, 0.0, 0.0);
  for (std::size_t i = 0; i < plan.yaw.size(); ++i) {
    EXPECT_NEAR(plan.yaw[i], 0.0, 1e-12) << i;
  }
}

TEST(YawDp, YawRateLimitRespected) {
  // A score that rewards facing +y everywhere on a +x path: the DP may turn
  // towards it, but not faster than k * ds per step (soft limit, w_lim large).
  std::vector<Point2> poly = {{0.0, 0.0}, {4.0, 0.0}};
  YawDpParams p;
  p.w_forward = 0.0;
  const auto plan = al_nav2_plugins::planYaws(
    poly,
    [](double, double, double yaw) {return 0.5 * (1.0 + std::sin(yaw));}, p, 0.0, std::nullopt);
  ASSERT_GE(plan.yaw.size(), 3u);
  EXPECT_NEAR(plan.yaw.front(), 0.0, 1e-12);
  for (std::size_t i = 1; i < plan.yaw.size(); ++i) {
    const double ds = std::hypot(
      plan.xy[i].x - plan.xy[i - 1].x, plan.xy[i].y - plan.xy[i - 1].y);
    const double d = std::fabs(al_nav2_plugins::wrapAngle(plan.yaw[i] - plan.yaw[i - 1]));
    // one bin of discretisation slack
    EXPECT_LE(d, p.max_dyaw_per_m * ds + 2.0 * M_PI / p.n_bins + 1e-9) << i;
  }
}

TEST(YawDp, DegeneratePaths) {
  YawDpParams p;
  auto zero = [](double, double, double) {return 0.0;};
  EXPECT_TRUE(al_nav2_plugins::planYaws({}, zero, p).yaw.empty());
  const auto one = al_nav2_plugins::planYaws({{1.0, 2.0}}, zero, p, 0.3, 0.7);
  ASSERT_EQ(one.yaw.size(), 1u);
  // single point: goal pin is applied last (same as the oracle)
  EXPECT_NEAR(one.yaw[0], 0.7, 1e-12);
  const auto same = al_nav2_plugins::planYaws({{1.0, 2.0}, {1.0, 2.0}}, zero, p);
  EXPECT_EQ(same.yaw.size(), 2u);
}
