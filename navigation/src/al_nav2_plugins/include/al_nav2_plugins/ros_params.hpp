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

// Parameter loading shared by both plugins. Both read the scorer under their own
// plugin name (`loc_layer.*` for the layer, `GridBased.*` for the planner), with the
// same keys as active_localization/config and tasks/T-009 section 2.
#pragma once

#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

#include "al_nav2_plugins/landmarks_io.hpp"
#include "al_nav2_plugins/scorer.hpp"
#include "nav2_util/node_utils.hpp"
#include "rclcpp/parameter_value.hpp"

namespace al_nav2_plugins
{

// Declares `<name>.<key>` (if still undeclared) and returns its value. NodePtr is any
// shared_ptr to a (lifecycle) node.
template<typename NodePtr>
double paramDouble(
  const NodePtr & node, const std::string & name, const std::string & key,
  double def)
{
  const std::string full = name + "." + key;
  nav2_util::declare_parameter_if_not_declared(node, full, rclcpp::ParameterValue(def));
  return node->get_parameter(full).as_double();
}

template<typename NodePtr>
int paramInt(const NodePtr & node, const std::string & name, const std::string & key, int def)
{
  const std::string full = name + "." + key;
  nav2_util::declare_parameter_if_not_declared(node, full, rclcpp::ParameterValue(def));
  return static_cast<int>(node->get_parameter(full).as_int());
}

template<typename NodePtr>
bool paramBool(const NodePtr & node, const std::string & name, const std::string & key, bool def)
{
  const std::string full = name + "." + key;
  nav2_util::declare_parameter_if_not_declared(node, full, rclcpp::ParameterValue(def));
  return node->get_parameter(full).as_bool();
}

template<typename NodePtr>
std::string paramString(
  const NodePtr & node, const std::string & name, const std::string & key,
  const std::string & def)
{
  const std::string full = name + "." + key;
  nav2_util::declare_parameter_if_not_declared(node, full, rclcpp::ParameterValue(def));
  return node->get_parameter(full).as_string();
}

struct ScorerSetup
{
  std::string kind{"marker_soft_count"};   // or "null" (control run: score == 0)
  std::vector<Marker> markers;
  ScorerParams params;

  bool isNull() const {return kind == "null";}
};

template<typename NodePtr>
ScorerSetup loadScorerSetup(const NodePtr & node, const std::string & name)
{
  ScorerSetup s;
  s.kind = paramString(node, name, "scorer", "marker_soft_count");
  if (s.kind != "marker_soft_count" && s.kind != "null") {
    throw std::runtime_error(
            name + ".scorer must be 'marker_soft_count' or 'null', got '" + s.kind + "'");
  }

  const ScorerParams d;   // spec defaults
  s.params.hfov = paramDouble(node, name, "hfov", d.hfov);
  s.params.k_fov = paramDouble(node, name, "k_fov", d.k_fov);
  s.params.focal_px = paramDouble(node, name, "focal_px", d.focal_px);
  s.params.marker_size = paramDouble(node, name, "marker_size", d.marker_size);
  s.params.a_min = paramDouble(node, name, "a_min", d.a_min);
  s.params.k_a = paramDouble(node, name, "k_a", d.k_a);
  s.params.r_min = paramDouble(node, name, "r_min", d.r_min);
  s.params.k_r = paramDouble(node, name, "k_r", d.k_r);
  s.params.m_sat = paramDouble(node, name, "m_sat", d.m_sat);
  s.params.cam_offset_x = paramDouble(node, name, "cam_offset_x", d.cam_offset_x);
  s.params.cam_offset_y = paramDouble(node, name, "cam_offset_y", d.cam_offset_y);
  s.params.cam_offset_yaw = paramDouble(node, name, "cam_offset_yaw", d.cam_offset_yaw);
  s.params.occ_stop = paramDouble(node, name, "occlusion_margin", d.occ_stop);

  const std::string file = paramString(node, name, "landmarks_file", "");
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".marker_ids", rclcpp::ParameterValue(std::vector<int64_t>{}));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".marker_normal", rclcpp::ParameterValue(std::vector<double>{0.0, -1.0}));
  const auto ids64 = node->get_parameter(name + ".marker_ids").as_integer_array();
  const auto normal = node->get_parameter(name + ".marker_normal").as_double_array();
  if (normal.size() != 2) {
    throw std::runtime_error(name + ".marker_normal must have exactly 2 elements");
  }

  if (!s.isNull()) {
    const std::vector<int> ids(ids64.begin(), ids64.end());
    s.markers = loadLandmarks(file, ids, normal[0], normal[1]);
    if (s.markers.empty()) {
      throw std::runtime_error(name + ": no markers left after filtering marker_ids");
    }
  }
  return s;
}

}  // namespace al_nav2_plugins
