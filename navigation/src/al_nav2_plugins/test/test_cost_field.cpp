// LocalizabilityLayer core vs the Python oracle (layer_cost_cases.json).
#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <vector>

#include "al_nav2_plugins/cost_field.hpp"
#include "al_nav2_plugins/occlusion.hpp"
#include "al_nav2_plugins/scorer.hpp"
#include "fixture_common.hpp"

using al_nav2_plugins::CellGrid;
using al_nav2_plugins::CostFieldParams;
using al_nav2_plugins::MarkerScorer;

namespace
{
CostFieldParams costParams(const mini_json::Value & h)
{
  CostFieldParams p;
  p.c_max = h["c_max"].d();
  p.power = h["power"].d();
  p.yaw_bins = h["yaw_bins"].i();
  return p;
}

CellGrid gridFrom(const mini_json::Value & h)
{
  CellGrid g;
  g.origin_x = h["grid"]["origin_x"].d();
  g.origin_y = h["grid"]["origin_y"].d();
  g.resolution = h["grid"]["resolution"].d();
  g.size_x = static_cast<unsigned int>(h["grid"]["width"].i());
  g.size_y = static_cast<unsigned int>(h["grid"]["height"].i());
  return g;
}
}  // namespace

TEST(CostField, SpecFormula) {
  CostFieldParams p;   // c_max 40, p 1
  EXPECT_EQ(al_nav2_plugins::layerCostFromScore(0.0, p), 40);
  EXPECT_EQ(al_nav2_plugins::layerCostFromScore(al_nav2_plugins::sRef(), p), 0);
  EXPECT_EQ(al_nav2_plugins::layerCostFromScore(10.0, p), 0);          // saturates
  EXPECT_EQ(al_nav2_plugins::layerCostFromScore(0.5 * al_nav2_plugins::sRef(), p), 20);
  p.c_max = 1000.0;                                                    // never >= 253
  EXPECT_EQ(al_nav2_plugins::layerCostFromScore(0.0, p), al_nav2_plugins::kMaxLayerCost);
}

TEST(CostField, CellsMatchOracleWithRectOcclusion) {
  const auto root = fixture::load("layer_cost_cases.json");
  const auto & h = root["header"];
  MarkerScorer scorer(fixture::markersFromHeader(h), fixture::paramsFromHeader(h));
  scorer.setOcclusion(fixture::rectOcclusion(fixture::rectsFromHeader(h)));
  const auto cp = costParams(h);
  const auto g = gridFrom(h);

  // (a) cell by cell through bestScore / layerCostFromScore
  for (const auto & c : root["cells"].arr) {
    const double s_star = al_nav2_plugins::bestScore(scorer, c["x"].d(), c["y"].d(), cp.yaw_bins);
    EXPECT_NEAR(s_star, c["s_star"].d(), h["tolerance_s_star"].d())
      << "cell " << c["col"].i() << "," << c["row"].i();
    EXPECT_NEAR(
      al_nav2_plugins::layerCostFromScore(s_star, cp), c["cost"].d(), h["tolerance_cost"].d());
  }

  // (b) the whole-grid routine: indexing (row = y, col = x) and values
  const auto costs = al_nav2_plugins::computeLayerCosts(scorer, g, cp);
  ASSERT_EQ(costs.size(), static_cast<std::size_t>(g.size_x) * g.size_y);
  for (const auto & c : root["cells"].arr) {
    const std::size_t idx = static_cast<std::size_t>(c["row"].i()) * g.size_x +
      static_cast<std::size_t>(c["col"].i());
    EXPECT_NEAR(static_cast<double>(costs[idx]), c["cost"].d(), h["tolerance_cost"].d())
      << "cell " << c["col"].i() << "," << c["row"].i();
  }
}

// The layer ray-casts a costmap instead of the oracle rectangles. Rasterise the
// oracle rectangles into a grid and require the two to agree (+-1 cost unit) on
// the fixture cells; documents how close the C++ occlusion is to the oracle.
TEST(CostField, GridRayCastAgreesWithRectOcclusion) {
  const auto root = fixture::load("layer_cost_cases.json");
  const auto & h = root["header"];
  const auto g = gridFrom(h);
  const auto rects = fixture::rectsFromHeader(h);

  std::vector<std::uint8_t> grid(static_cast<std::size_t>(g.size_x) * g.size_y, 0);
  for (unsigned int j = 0; j < g.size_y; ++j) {
    for (unsigned int i = 0; i < g.size_x; ++i) {
      const double x = g.origin_x + (i + 0.5) * g.resolution;
      const double y = g.origin_y + (j + 0.5) * g.resolution;
      for (const auto & r : rects) {
        if (x >= r[0] && x <= r[1] && y >= r[2] && y <= r[3]) {
          grid[static_cast<std::size_t>(j) * g.size_x + i] = 254;
        }
      }
    }
  }
  al_nav2_plugins::GridView view;
  view.origin_x = g.origin_x;
  view.origin_y = g.origin_y;
  view.resolution = g.resolution;
  view.size_x = g.size_x;
  view.size_y = g.size_y;
  view.data = grid.data();

  MarkerScorer scorer(fixture::markersFromHeader(h), fixture::paramsFromHeader(h));
  scorer.setOcclusion(
    [&view](std::size_t, double cx, double cy, double qx, double qy) {
      return view.segmentBlocked(cx, cy, qx, qy);
    });
  const auto cp = costParams(h);
  int off = 0;
  for (const auto & c : root["cells"].arr) {
    const double s_star = al_nav2_plugins::bestScore(scorer, c["x"].d(), c["y"].d(), cp.yaw_bins);
    const int cost = al_nav2_plugins::layerCostFromScore(s_star, cp);
    if (std::abs(cost - c["cost"].i()) > h["tolerance_cost"].i()) {
      ++off;
      ADD_FAILURE() << "cell " << c["col"].i() << "," << c["row"].i() << " cost " << cost
                    << " vs oracle " << c["cost"].i();
    }
  }
  EXPECT_EQ(off, 0);
}

TEST(CostField, NullStyleGridIsZeroWhenNoMarkers) {
  // no markers -> s* = 0 -> c_max everywhere; the Layer handles scorer "null"
  // separately (cost 0); this documents the scorer-level behaviour.
  CostFieldParams p;
  MarkerScorer scorer({}, al_nav2_plugins::ScorerParams{});
  CellGrid g;
  g.size_x = 4;
  g.size_y = 3;
  const auto costs = al_nav2_plugins::computeLayerCosts(scorer, g, p);
  ASSERT_EQ(costs.size(), 12u);
  for (auto c : costs) {
    EXPECT_EQ(c, 40);
  }
}

TEST(Occlusion, GridViewBasics) {
  std::vector<std::uint8_t> data(10 * 10, 0);
  data[5 * 10 + 5] = 254;   // cell (5, 5)
  al_nav2_plugins::GridView v;
  v.resolution = 1.0;
  v.size_x = 10;
  v.size_y = 10;
  v.data = data.data();
  EXPECT_TRUE(v.blockedAt(5.5, 5.5));
  EXPECT_FALSE(v.blockedAt(4.5, 5.5));
  EXPECT_FALSE(v.blockedAt(-3.0, 2.0));    // outside = free
  EXPECT_TRUE(v.segmentBlocked(0.5, 5.5, 9.5, 5.5));
  EXPECT_FALSE(v.segmentBlocked(0.5, 4.5, 9.5, 4.5));
  data[5 * 10 + 5] = 253;                   // inscribed does not block
  EXPECT_FALSE(v.segmentBlocked(0.5, 5.5, 9.5, 5.5));
}

TEST(Occlusion, RectSegment) {
  using al_nav2_plugins::segmentHitsRect;
  const al_nav2_plugins::Rect r = {0.0, 1.0, 0.0, 1.0};
  EXPECT_TRUE(segmentHitsRect(-1.0, 0.5, 2.0, 0.5, r));
  EXPECT_FALSE(segmentHitsRect(-1.0, 2.0, 2.0, 2.0, r));
  EXPECT_TRUE(segmentHitsRect(0.5, 0.5, 0.6, 0.6, r));     // fully inside
  EXPECT_TRUE(segmentHitsRect(-1.0, 1.0, 0.0, 1.0, r));    // touches the boundary
  EXPECT_FALSE(segmentHitsRect(2.0, 0.0, 2.0, 1.0, r));    // parallel, outside
}
