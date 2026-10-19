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

// Reads the RTAB-Map landmark export (rtabmap_landmarks/config/landmarks.yaml,
// keys `frame_id`, `big_markers[]` with `id` and `position`) into scorer markers.
#pragma once

#include <string>
#include <vector>

#include "al_nav2_plugins/scorer.hpp"

namespace al_nav2_plugins
{

// By default every marker gets the same outward face normal (nx, ny): all HD17 markers in the
// world face -y (FACTS 17.10 / 18.10). With `normal_axis` ("+x", "-y", ... : a local axis of
// the marker frame) each marker's normal is derived from its `orientation` instead (rotate the
// axis, project on the map plane, normalise - same as the Python `normal_axis` option);
// (nx, ny) stays the fallback for a marker without `orientation` or with a (nearly) vertical
// axis. An invalid `normal_axis` throws.
// `ids` empty = keep all markers. Throws std::runtime_error when the file cannot be
// read, has no `big_markers`, or contains an id more than once.
// `frame_id` (optional) receives the file's `frame_id` ("" if absent).
std::vector<Marker> loadLandmarks(
  const std::string & path, const std::vector<int> & ids, double nx, double ny,
  std::string * frame_id = nullptr, const std::string & normal_axis = "");

}  // namespace al_nav2_plugins
