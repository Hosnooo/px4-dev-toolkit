#pragma once

#include <array>

#include <offboard_controllers/se3/types.hpp>

namespace offboard_controllers::px4_wrench
{

double normalized_projected_collective_thrust(
  const se3::Vector3 & force,
  const se3::RotationMatrix & attitude,
  double mass,
  double hover_thrust);

// PX4 attitude quaternions use Hamilton [w, x, y, z] and rotate FRD body
// vectors into the NED inertial frame.
std::array<double, 4> quaternion_from_rotation(
  const se3::RotationMatrix & rotation);

se3::RotationMatrix rotation_from_quaternion(
  const std::array<double, 4> & quaternion);

}  // namespace offboard_controllers::px4_wrench
