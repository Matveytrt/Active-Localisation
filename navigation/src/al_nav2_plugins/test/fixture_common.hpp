// Shared helpers for the fixture-driven gtests (header of the JSON files ->
// scorer parameters, markers and the oracle's rectangle occlusion rule).
#pragma once

#include <string>
#include <vector>

#include "al_nav2_plugins/occlusion.hpp"
#include "al_nav2_plugins/scorer.hpp"
#include "mini_json.hpp"

#ifndef AL_FIXTURE_DIR
#error "AL_FIXTURE_DIR must point to the golden fixtures"
#endif

namespace fixture
{

inline mini_json::Value load(const std::string & name)
{
  return mini_json::parseFile(std::string(AL_FIXTURE_DIR) + "/" + name);
}

inline al_nav2_plugins::ScorerParams paramsFromHeader(const mini_json::Value & h)
{
  al_nav2_plugins::ScorerParams p;
  p.hfov = h["camera"]["hfov"].d();
  p.cam_offset_x = h["camera"]["offset_x"].d();
  p.cam_offset_y = h["camera"]["offset_y"].d();
  p.cam_offset_yaw = h["camera"]["offset_yaw"].d();
  p.k_fov = h["k_fov"].d();
  p.a_min = h["a_min"].d();
  p.k_a = h["k_a"].d();
  p.r_min = h["r_min"].d();
  p.k_r = h["k_r"].d();
  p.focal_px = h["focal_px"].d();
  p.marker_size = h["marker_size"].d();
  p.m_sat = h["m_sat"].d();
  p.occ_stop = h["occlusion"]["occ_stop"].d();
  return p;
}

inline std::vector<al_nav2_plugins::Marker> markersFromHeader(const mini_json::Value & h)
{
  std::vector<al_nav2_plugins::Marker> out;
  for (const auto & m : h["markers"].arr) {
    al_nav2_plugins::Marker k;
    k.id = m["id"].i();
    k.x = m["x"].d();
    k.y = m["y"].d();
    k.nx = m["normal"][0].d();
    k.ny = m["normal"][1].d();
    out.push_back(k);
  }
  return out;
}

inline std::vector<al_nav2_plugins::Rect> rectsFromHeader(const mini_json::Value & h)
{
  std::vector<al_nav2_plugins::Rect> out;
  for (const auto & r : h["occlusion"]["rows"].arr) {
    out.push_back({r[0].d(), r[1].d(), r[2].d(), r[3].d()});
  }
  return out;
}

// The oracle's occlusion rule as an OcclusionFn.
inline al_nav2_plugins::OcclusionFn rectOcclusion(std::vector<al_nav2_plugins::Rect> rects)
{
  return [rects](std::size_t, double cx, double cy, double qx, double qy) {
           return al_nav2_plugins::segmentHitsRects(cx, cy, qx, qy, rects);
         };
}

}  // namespace fixture
