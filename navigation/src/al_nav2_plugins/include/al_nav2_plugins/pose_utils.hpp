// Planar quaternion <-> yaw helpers (no tf2 dependency). Header-only, no ROS.
#pragma once

#include <cmath>

namespace al_nav2_plugins
{

// Yaw of a unit quaternion (x, y, z, w), rotation about +z.
inline double yawFromQuaternion(double x, double y, double z, double w)
{
  return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

// Quaternion components (z, w) of a pure yaw rotation; x = y = 0.
inline void quaternionFromYaw(double yaw, double & qz, double & qw)
{
  qz = std::sin(0.5 * yaw);
  qw = std::cos(0.5 * yaw);
}

}  // namespace al_nav2_plugins
