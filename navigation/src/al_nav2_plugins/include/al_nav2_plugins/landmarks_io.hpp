// Reads the RTAB-Map landmark export (rtabmap_landmarks/config/landmarks.yaml,
// keys `frame_id`, `big_markers[]` with `id` and `position`) into scorer markers.
#pragma once

#include <string>
#include <vector>

#include "al_nav2_plugins/scorer.hpp"

namespace al_nav2_plugins
{

// Every marker gets the same outward face normal (nx, ny): all HD17 markers in the
// world face -y (FACTS 17.10 / 18.10); a per-marker normal is not needed here.
// `ids` empty = keep all markers. Throws std::runtime_error when the file cannot be
// read, has no `big_markers`, or contains an id more than once.
// `frame_id` (optional) receives the file's `frame_id` ("" if absent).
std::vector<Marker> loadLandmarks(
  const std::string & path, const std::vector<int> & ids, double nx, double ny,
  std::string * frame_id = nullptr);

}  // namespace al_nav2_plugins
