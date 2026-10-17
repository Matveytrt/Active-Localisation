// loadLandmarks() against a copy of the RTAB-Map export (test/fixtures/landmarks_sample.yaml).
#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "al_nav2_plugins/landmarks_io.hpp"

namespace
{
const std::string kFile = std::string(AL_FIXTURE_DIR) + "/landmarks_sample.yaml";
}

TEST(LandmarksIo, LoadsAllMarkers) {
  std::string frame;
  const auto m = al_nav2_plugins::loadLandmarks(kFile, {}, 0.0, -1.0, &frame);
  ASSERT_EQ(m.size(), 5u);
  EXPECT_EQ(frame, "map");
  EXPECT_EQ(m[0].id, 1);
  EXPECT_NEAR(m[0].x, 0.2, 0.05);   // s2 in the world: (0.2, 0.9)
  EXPECT_NEAR(m[0].y, 0.9, 0.05);
  for (const auto & k : m) {
    EXPECT_DOUBLE_EQ(k.nx, 0.0);
    EXPECT_DOUBLE_EQ(k.ny, -1.0);
  }
}

TEST(LandmarksIo, FiltersByIds) {
  const auto m = al_nav2_plugins::loadLandmarks(kFile, {2, 5}, 0.0, -1.0);
  ASSERT_EQ(m.size(), 2u);
  EXPECT_EQ(m[0].id, 2);
  EXPECT_EQ(m[1].id, 5);
}

TEST(LandmarksIo, UnknownIdsGiveEmptyResult) {
  EXPECT_TRUE(al_nav2_plugins::loadLandmarks(kFile, {42}, 0.0, -1.0).empty());
}

TEST(LandmarksIo, MissingFileThrows) {
  EXPECT_THROW(al_nav2_plugins::loadLandmarks("/nonexistent/landmarks.yaml", {}, 0.0, -1.0),
    std::runtime_error);
}
