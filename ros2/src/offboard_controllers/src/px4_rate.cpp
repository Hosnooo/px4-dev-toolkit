/*
 * PX4 multicopter angular-rate controller reproduction.
 *
 * Source of truth:
 *   ANCL/PX4-Autopilot
 *   commit f5083ca2c5b919350880e8667e636ef70715db17
 *
 *   src/lib/rate_control/rate_control.cpp
 *   src/modules/mc_rate_control/MulticopterRateControl.cpp
 *   src/lib/mathlib/math/filter/AlphaFilter.hpp
 *
 * This class reproduces the rate-loop calculation, integrator anti-windup,
 * timestamp-based dt limiting, and yaw-torque low-pass filter. ROS 2 transport
 * and PX4 Offboard lifecycle remain outside this class.
 */

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>

#include <offboard_controllers/px4_rate.hpp>
#include <offboard_controllers/se3/math.hpp>

namespace offboard_controllers::px4_rate
{

namespace
{

constexpr double kMinimumDtS = 0.000125;
constexpr double kMaximumDtS = 0.02;
constexpr double kPi = 3.14159265358979323846;
constexpr double kIntegratorErrorScale =
  400.0 * kPi / 180.0;


double & component(
  se3::Vector3 & vector,
  std::size_t axis)
{
  if (axis == 0) {
    return vector.x;
  }

  if (axis == 1) {
    return vector.y;
  }

  return vector.z;
}


void require_nonnegative_finite(
  const se3::Vector3 & value,
  const char * message)
{
  if (
    !se3::is_finite(value) ||
    value.x < 0.0 ||
    value.y < 0.0 ||
    value.z < 0.0)
  {
    throw std::invalid_argument(message);
  }
}

}  // namespace


Controller::Controller(
  const Parameters & parameters)
: parameters_(parameters)
{
  require_nonnegative_finite(
    parameters_.p,
    "PX4 rate P gains must be finite and non-negative.");

  require_nonnegative_finite(
    parameters_.i,
    "PX4 rate I gains must be finite and non-negative.");

  require_nonnegative_finite(
    parameters_.d,
    "PX4 rate D gains must be finite and non-negative.");

  require_nonnegative_finite(
    parameters_.ff,
    "PX4 rate feed-forward gains must be finite and non-negative.");

  require_nonnegative_finite(
    parameters_.k,
    "PX4 rate K gains must be finite and non-negative.");

  require_nonnegative_finite(
    parameters_.integrator_limit,
    "PX4 rate integrator limits must be finite and non-negative.");

  if (
    !std::isfinite(parameters_.yaw_torque_cutoff_hz) ||
    parameters_.yaw_torque_cutoff_hz < 0.0)
  {
    throw std::invalid_argument(
            "PX4 yaw-torque cutoff must be finite and non-negative.");
  }

  gain_p_ =
    se3::component_product(
      parameters_.k,
      parameters_.p);

  gain_i_ =
    se3::component_product(
      parameters_.k,
      parameters_.i);

  gain_d_ =
    se3::component_product(
      parameters_.k,
      parameters_.d);

  if (
    parameters_.yaw_torque_cutoff_hz >
    std::numeric_limits<float>::epsilon())
  {
    yaw_filter_time_constant_s_ =
      1.0 /
      (
        2.0 *
        kPi *
        parameters_.yaw_torque_cutoff_hz
      );
  }
}


void Controller::set_saturation_status(
  const std::array<bool, 3> & positive,
  const std::array<bool, 3> & negative)
{
  saturation_positive_ = positive;
  saturation_negative_ = negative;
}


void Controller::reset_integral()
{
  integrator_ = {};
}


void Controller::observe_timestamp_sample(
  uint64_t timestamp_sample_us)
{
  last_run_us_ = timestamp_sample_us;
}


Output Controller::update(
  uint64_t timestamp_sample_us,
  const se3::Vector3 & rate,
  const se3::Vector3 & rate_setpoint,
  const se3::Vector3 & angular_acceleration,
  bool landed)
{
  const uint64_t elapsed_us =
    timestamp_sample_us - last_run_us_;

  const double dt_s =
    std::clamp(
      static_cast<double>(elapsed_us) * 1.0e-6,
      kMinimumDtS,
      kMaximumDtS);

  last_run_us_ = timestamp_sample_us;

  const se3::Vector3 rate_error =
    rate_setpoint - rate;

  const se3::Vector3 proportional_feedback =
    se3::component_product(
      gain_p_,
      rate_error);

  const se3::Vector3 integral_feedback =
    integrator_;

  const se3::Vector3 derivative_feedback =
    -se3::component_product(
      gain_d_,
      angular_acceleration);

  const se3::Vector3 feedforward =
    se3::component_product(
      parameters_.ff,
      rate_setpoint);

  const se3::Vector3 unfiltered_torque =
    proportional_feedback +
    integral_feedback +
    derivative_feedback +
    feedforward;

  if (!landed) {
    se3::Vector3 integration_error =
      rate_error;

    for (std::size_t axis = 0; axis < 3; ++axis) {
      double & error =
        component(
          integration_error,
          axis);

      if (saturation_positive_[axis]) {
        error =
          std::min(
            error,
            0.0);
      }

      if (saturation_negative_[axis]) {
        error =
          std::max(
            error,
            0.0);
      }

      double i_factor =
        error /
        kIntegratorErrorScale;

      i_factor =
        std::max(
          0.0,
          1.0 - i_factor * i_factor);

      const double candidate =
        component(integrator_, axis) +
        i_factor *
        component(gain_i_, axis) *
        error *
        dt_s;

      if (std::isfinite(candidate)) {
        component(integrator_, axis) =
          std::clamp(
            candidate,
            -component(parameters_.integrator_limit, axis),
            component(parameters_.integrator_limit, axis));
      }
    }
  }

  se3::Vector3 normalized_torque =
    unfiltered_torque;

  const double denominator =
    yaw_filter_time_constant_s_ +
    dt_s;

  double alpha = 0.0;

  if (
    denominator >
    std::numeric_limits<float>::epsilon())
  {
    alpha =
      dt_s /
      denominator;
  }

  yaw_filter_state_ +=
    alpha *
    (
      unfiltered_torque.z -
      yaw_filter_state_
    );

  normalized_torque.z =
    yaw_filter_state_;

  return {
    dt_s,
    rate_error,
    proportional_feedback,
    integral_feedback,
    derivative_feedback,
    feedforward,
    unfiltered_torque,
    normalized_torque,
    integrator_,
  };
}

}  // namespace offboard_controllers::px4_rate
