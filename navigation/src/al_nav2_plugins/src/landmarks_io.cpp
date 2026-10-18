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
#include <set>
#include <stdexcept>

#include "yaml-cpp/yaml.h"

namespace al_nav2_plugins
{

std::vector<Marker> loadLandmarks(
  const std::string & path, const std::vector<int> & ids, double nx, double ny,
  std::string * frame_id)
{
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
    out.push_back(m);
  }
  return out;
}

}  // namespace al_nav2_plugins
