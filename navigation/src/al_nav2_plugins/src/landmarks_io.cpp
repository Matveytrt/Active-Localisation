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

#include "al_nav2_plugins/landmarks_io.hpp"

#include <algorithm>
#include <cmath>
#include <set>
#include <stdexcept>

#include "yaml-cpp/yaml.h"

namespace al_nav2_plugins
{

namespace
{
// A marker normal shorter than this in the map plane means the axis is vertical (the facing
// direction is undefined for a planar robot).
constexpr double kMinPlanarNormal = 0.1;

void parseAxis(const std::string & axis, int * index, double * sign)
{
  std::string a = axis;
  *sign = 1.0;
  if (!a.empty() && (a[0] == '+' || a[0] == '-')) {
    *sign = a[0] == '-' ? -1.0 : 1.0;
    a.erase(0, 1);
  }
  if (a == "x" || a == "X") {
    *index = 0;
  } else if (a == "y" || a == "Y") {
    *index = 1;
  } else if (a == "z" || a == "Z") {
    *index = 2;
  } else {
    throw std::runtime_error("normal_axis must be one of [+-]x/y/z, got '" + axis + "'");
  }
}

// Local axis `index` rotated by the quaternion, projected on the map plane and normalised.
bool planarNormal(const YAML::Node & o, int index, double sign, double * nx, double * ny)
{
  double x = o["x"].as<double>();
  double y = o["y"].as<double>();
  double z = o["z"].as<double>();
  double w = o["w"].as<double>();
  const double len = std::sqrt(x * x + y * y + z * z + w * w);
  if (len < 1e-9) {
    return false;
  }
  x /= len;
  y /= len;
  z /= len;
  w /= len;
  double vx = 0.0;
  double vy = 0.0;
  if (index == 0) {
    vx = 1.0 - 2.0 * (y * y + z * z);
    vy = 2.0 * (x * y + z * w);
  } else if (index == 1) {
    vx = 2.0 * (x * y - z * w);
    vy = 1.0 - 2.0 * (x * x + z * z);
  } else {
    vx = 2.0 * (x * z + y * w);
    vy = 2.0 * (y * z - x * w);
  }
  vx *= sign;
  vy *= sign;
  const double n = std::hypot(vx, vy);
  if (n < kMinPlanarNormal) {
    return false;
  }
  *nx = vx / n;
  *ny = vy / n;
  return true;
}
}  // namespace

std::vector<Marker> loadLandmarks(
  const std::string & path, const std::vector<int> & ids, double nx, double ny,
  std::string * frame_id, const std::string & normal_axis)
{
  int axis_index = 0;
  double axis_sign = 1.0;
  if (!normal_axis.empty()) {
    parseAxis(normal_axis, &axis_index, &axis_sign);
  }
  YAML::Node root;
  try {
    root = YAML::LoadFile(path);
  } catch (const YAML::Exception & e) {
    throw std::runtime_error("cannot read landmarks file '" + path + "': " + e.what());
  }
  const YAML::Node markers = root["big_markers"];
  if (!markers || !markers.IsSequence() || markers.size() == 0) {
    throw std::runtime_error("landmarks file '" + path + "' has no big_markers");
  }
  if (frame_id != nullptr) {
    *frame_id = root["frame_id"] ? root["frame_id"].as<std::string>() : std::string();
  }

  std::vector<Marker> out;
  std::set<int> seen;
  for (const auto & node : markers) {
    Marker m;
    try {
      m.id = node["id"].as<int>();
      m.x = node["position"]["x"].as<double>();
      m.y = node["position"]["y"].as<double>();
    } catch (const YAML::Exception & e) {
      throw std::runtime_error("bad marker entry in '" + path + "': " + e.what());
    }
    if (!seen.insert(m.id).second) {
      throw std::runtime_error(
              "duplicate marker id " + std::to_string(m.id) + " in '" + path +
              "'");
    }
    if (!ids.empty() && std::find(ids.begin(), ids.end(), m.id) == ids.end()) {
      continue;
    }
    m.nx = nx;
    m.ny = ny;
    if (!normal_axis.empty() && node["orientation"]) {
      try {
        planarNormal(node["orientation"], axis_index, axis_sign, &m.nx, &m.ny);
      } catch (const YAML::Exception & e) {
        throw std::runtime_error("bad orientation in '" + path + "': " + e.what());
      }
    }
    out.push_back(m);
  }
  return out;
}

}  // namespace al_nav2_plugins
