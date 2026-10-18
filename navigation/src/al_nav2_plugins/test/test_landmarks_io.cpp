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

// loadLandmarks() against a copy of the RTAB-Map export (test/fixtures/landmarks_sample.yaml).
#include <gtest/gtest.h>

#include <cmath>
#include <string>
#include <vector>

#include "al_nav2_plugins/landmarks_io.hpp"

namespace
{
const char kFile[] = AL_FIXTURE_DIR "/landmarks_sample.yaml";
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
  EXPECT_THROW(
    al_nav2_plugins::loadLandmarks("/nonexistent/landmarks.yaml", {}, 0.0, -1.0),
    std::runtime_error);
}

TEST(LandmarksIo, DerivesPerMarkerNormalsFromOrientation) {
  const auto m = al_nav2_plugins::loadLandmarks(kFile, {}, 0.0, -1.0, nullptr, "+x");
  ASSERT_EQ(m.size(), 5u);
  // The export is yaw-only: the local +x axis points along the face normal, about -y; the
  // markers differ by a few degrees (id 5 leans towards +x).
  for (const auto & k : m) {
    EXPECT_NEAR(std::hypot(k.nx, k.ny), 1.0, 1e-9);
    EXPECT_LT(std::fabs(k.nx), 0.06);
    EXPECT_LT(k.ny, -0.99);
  }
  EXPECT_NEAR(m[0].nx, -0.028, 0.002);   // id 1
  EXPECT_NEAR(m[4].nx, 0.044, 0.002);    // id 5
  EXPECT_NE(m[0].nx, m[4].nx);
  const auto flipped = al_nav2_plugins::loadLandmarks(kFile, {}, 0.0, -1.0, nullptr, "-x");
  EXPECT_NEAR(flipped[0].nx, -m[0].nx, 1e-12);
  EXPECT_NEAR(flipped[0].ny, -m[0].ny, 1e-12);
}

TEST(LandmarksIo, VerticalAxisFallsBackToTheUniformNormal) {
  // The markers' z axis is vertical in the export, so the facing direction is undefined.
  const auto m = al_nav2_plugins::loadLandmarks(kFile, {}, 0.0, -1.0, nullptr, "+z");
  ASSERT_EQ(m.size(), 5u);
  for (const auto & k : m) {
    EXPECT_DOUBLE_EQ(k.nx, 0.0);
    EXPECT_DOUBLE_EQ(k.ny, -1.0);
  }
}

TEST(LandmarksIo, InvalidNormalAxisThrows) {
  EXPECT_THROW(
    al_nav2_plugins::loadLandmarks(kFile, {}, 0.0, -1.0, nullptr, "up"), std::runtime_error);
}
