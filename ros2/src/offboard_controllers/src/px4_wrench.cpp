#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>

#include <offboard_controllers/px4_wrench.hpp>
#include <offboard_controllers/se3/constants.hpp>

namespace offboard_controllers::px4_wrench
{

double normalized_projected_collective_thrust(
  const se3::Vector3 & force,
  const se3::RotationMatrix & attitude,
  double mass,
  double hover_thrust)
{
  // Lee SE(3) collective thrust before actuator limits:
  //   f = -A . R e3
  // RotationMatrix::b3 is the current body z-axis R e3 in NED.
  const double physical_thrust =
    -(
      force.x * attitude.b3.x +
      force.y * attitude.b3.y +
      force.z * attitude.b3.z);

  const double normalized_thrust =
    hover_thrust *
    physical_thrust /
    (mass * se3::kStandardGravity);

  // PX4 VehicleThrustSetpoint is a normalized body-axis thrust setpoint.
  // Limit the multicopter collective magnitude here; the publisher applies
  // the negative FRD body-z sign.
  return std::clamp(
    normalized_thrust,
    0.0,
    1.0);
}


std::array<double, 4> quaternion_from_rotation(
  const se3::RotationMatrix & rotation)
{
  // RotationMatrix stores the body-to-NED rotation by columns.
  const double r00 = rotation.b1.x;
  const double r01 = rotation.b2.x;
  const double r02 = rotation.b3.x;
  const double r10 = rotation.b1.y;
  const double r11 = rotation.b2.y;
  const double r12 = rotation.b3.y;
  const double r20 = rotation.b1.z;
  const double r21 = rotation.b2.z;
  const double r22 = rotation.b3.z;

  if (
    !std::isfinite(r00) || !std::isfinite(r01) || !std::isfinite(r02) ||
    !std::isfinite(r10) || !std::isfinite(r11) || !std::isfinite(r12) ||
    !std::isfinite(r20) || !std::isfinite(r21) || !std::isfinite(r22))
  {
    throw std::invalid_argument(
            "PX4 attitude rotation must be finite.");
  }

  double w;
  double x;
  double y;
  double z;

  const double trace =
    r00 + r11 + r22;

  if (trace > 0.0) {
    const double s =
      2.0 * std::sqrt(trace + 1.0);

    w = 0.25 * s;
    x = (r21 - r12) / s;
    y = (r02 - r20) / s;
    z = (r10 - r01) / s;
  } else if (r00 > r11 && r00 > r22) {
    const double s =
      2.0 * std::sqrt(1.0 + r00 - r11 - r22);

    w = (r21 - r12) / s;
    x = 0.25 * s;
    y = (r01 + r10) / s;
    z = (r02 + r20) / s;
  } else if (r11 > r22) {
    const double s =
      2.0 * std::sqrt(1.0 + r11 - r00 - r22);

    w = (r02 - r20) / s;
    x = (r01 + r10) / s;
    y = 0.25 * s;
    z = (r12 + r21) / s;
  } else {
    const double s =
      2.0 * std::sqrt(1.0 + r22 - r00 - r11);

    w = (r10 - r01) / s;
    x = (r02 + r20) / s;
    y = (r12 + r21) / s;
    z = 0.25 * s;
  }

  const double quaternion_norm =
    std::sqrt(
      w * w +
      x * x +
      y * y +
      z * z);

  if (
    !std::isfinite(quaternion_norm) ||
    quaternion_norm <= 1.0e-12)
  {
    throw std::invalid_argument(
            "PX4 attitude rotation produced an invalid quaternion.");
  }

  w /= quaternion_norm;
  x /= quaternion_norm;
  y /= quaternion_norm;
  z /= quaternion_norm;

  // q and -q represent the same attitude. Keep a deterministic sign.
  if (w < 0.0) {
    w = -w;
    x = -x;
    y = -y;
    z = -z;
  }

  return {
    w,
    x,
    y,
    z,
  };
}


se3::RotationMatrix rotation_from_quaternion(
  const std::array<double, 4> & quaternion)
{
  double w = quaternion[0];
  double x = quaternion[1];
  double y = quaternion[2];
  double z = quaternion[3];

  const double quaternion_norm =
    std::sqrt(
      w * w +
      x * x +
      y * y +
      z * z);

  if (
    !std::isfinite(quaternion_norm) ||
    quaternion_norm <= 1.0e-12)
  {
    throw std::invalid_argument(
            "PX4 attitude quaternion must be finite and non-zero.");
  }

  w /= quaternion_norm;
  x /= quaternion_norm;
  y /= quaternion_norm;
  z /= quaternion_norm;

  const double r00 = 1.0 - 2.0 * (y * y + z * z);
  const double r01 = 2.0 * (x * y - w * z);
  const double r02 = 2.0 * (x * z + w * y);
  const double r10 = 2.0 * (x * y + w * z);
  const double r11 = 1.0 - 2.0 * (x * x + z * z);
  const double r12 = 2.0 * (y * z - w * x);
  const double r20 = 2.0 * (x * z - w * y);
  const double r21 = 2.0 * (y * z + w * x);
  const double r22 = 1.0 - 2.0 * (x * x + y * y);

  return {
    {r00, r10, r20},
    {r01, r11, r21},
    {r02, r12, r22},
  };
}

}  // namespace offboard_controllers::px4_wrench
