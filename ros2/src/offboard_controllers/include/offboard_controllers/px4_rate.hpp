#pragma once

#include <array>
#include <cstdint>

#include <offboard_controllers/se3/types.hpp>

namespace offboard_controllers::px4_rate
{

struct Parameters
{
  // Raw PX4 multicopter-rate parameters. Effective P/I/D gains are
  // K multiplied component-wise by the corresponding P/I/D values.
  se3::Vector3 p{};
  se3::Vector3 i{};
  se3::Vector3 d{};
  se3::Vector3 ff{};
  se3::Vector3 k{};
  se3::Vector3 integrator_limit{};

  double yaw_torque_cutoff_hz{0.0};
};


struct Output
{
  double dt_s{0.0};

  se3::Vector3 rate_error{};
  se3::Vector3 proportional_feedback{};
  se3::Vector3 integral_feedback{};
  se3::Vector3 derivative_feedback{};
  se3::Vector3 feedforward{};

  se3::Vector3 unfiltered_torque{};
  se3::Vector3 normalized_torque{};

  // Integrator state after this update. The torque above uses the integrator
  // state that existed at the beginning of the update, matching PX4.
  se3::Vector3 integrator_state{};
};


class Controller
{
public:
  explicit Controller(const Parameters & parameters);

  void set_saturation_status(
    const std::array<bool, 3> & positive,
    const std::array<bool, 3> & negative);

  void reset_integral();

  // MulticopterRateControl advances its timing state on every
  // VehicleAngularVelocity callback, even while rate control is inactive.
  // Use this for samples that do not run the controller so the next active
  // update sees the same sample-to-sample dt as PX4.
  void observe_timestamp_sample(
    uint64_t timestamp_sample_us);

  // Run one PX4-equivalent rate-controller update from a new
  // VehicleAngularVelocity sample. timestamp_sample_us drives dt exactly as
  // MulticopterRateControl does in the pinned PX4 implementation.
  Output update(
    uint64_t timestamp_sample_us,
    const se3::Vector3 & rate,
    const se3::Vector3 & rate_setpoint,
    const se3::Vector3 & angular_acceleration,
    bool landed);

private:
  Parameters parameters_{};

  se3::Vector3 gain_p_{};
  se3::Vector3 gain_i_{};
  se3::Vector3 gain_d_{};

  se3::Vector3 integrator_{};

  std::array<bool, 3> saturation_positive_{};
  std::array<bool, 3> saturation_negative_{};

  uint64_t last_run_us_{0};

  double yaw_filter_time_constant_s_{0.0};
  double yaw_filter_state_{0.0};
};

}  // namespace offboard_controllers::px4_rate
