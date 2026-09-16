#pragma once

#include <cmath>
#include <stdexcept>
#include <string>

#include <rclcpp/rclcpp.hpp>


namespace offboard_controllers
{

struct InitialSetpoint
{
  float x;
  float y;
  float z;
  float yaw;
};


/*
 * Offboard setpoints are intentionally required parameters.
 *
 * No values are duplicated in C++: launch files load config/offboard.yaml and
 * startup fails if a required value is absent or invalid.
 */
inline InitialSetpoint load_initial_setpoint(
  rclcpp::Node & node)
{
  try {
    const double x =
      node.declare_parameter<double>("initial_x");

    const double y =
      node.declare_parameter<double>("initial_y");

    const double z =
      node.declare_parameter<double>("initial_z");

    const double yaw =
      node.declare_parameter<double>("initial_yaw");

    if (!std::isfinite(x) ||
      !std::isfinite(y) ||
      !std::isfinite(z) ||
      !std::isfinite(yaw))
    {
      throw std::runtime_error(
              "Offboard initial-setpoint parameters must be finite.");
    }

    return {
      static_cast<float>(x),
      static_cast<float>(y),
      static_cast<float>(z),
      static_cast<float>(yaw),
    };

  } catch (const std::exception & error) {
    throw std::runtime_error(
            "Could not load required Offboard initial setpoint: " +
            std::string(error.what()));
  }
}

}  // namespace offboard_controllers
