#pragma once

#include <array>
#include <cstddef>

#include <offboard_controllers/se3/types.hpp>

namespace offboard_controllers::lee_px4_adapter
{

// The direct physical-wrench adapter currently targets a conventional
// four-rotor multicopter. All geometry supplied here is vehicle data;
// no airframe dimensions or propulsion constants are embedded in the
// adapter itself.
constexpr std::size_t kRotorCount = 4;


struct PhysicalRotor
{
  // Rotor position relative to the vehicle center of mass in FRD [m].
  se3::Vector3 position_frd_m{};

  // Signed reaction-moment / thrust ratio [m].
  //
  // Positive means positive FRD yaw moment for positive rotor thrust.
  double yaw_moment_ratio{0.0};
};


struct AllocatorRotor
{
  // PX4 control-allocation rotor position in FRD [m].
  se3::Vector3 position_frd_m{};

  // PX4 CA_ROTOR*_CT and CA_ROTOR*_KM values.
  double thrust_coefficient{0.0};
  double moment_ratio{0.0};
};


struct Parameters
{
  std::array<PhysicalRotor, kRotorCount> physical_rotors{};
  std::array<AllocatorRotor, kRotorCount> allocator_rotors{};

  // Gazebo multicopter-motor-model thrust relation:
  //
  //   T = motor_constant * omega^2
  //
  // with T [N] and omega [rad/s].
  double motor_constant{0.0};

  // PX4 Gazebo ESC output range. GZMixingInterfaceESC sends these output
  // values directly as Gazebo rotor-velocity commands.
  double output_min_rad_s{0.0};
  double output_max_rad_s{0.0};

  // PX4 THR_MDL_FAC:
  //
  //   normalized_thrust =
  //     factor * signal^2 + (1 - factor) * signal
  //
  // FunctionMotors performs the inverse before the ESC output mapping.
  double thrust_model_factor{0.0};
};


struct Output
{
  // Values to publish through PX4 VehicleTorqueSetpoint and
  // VehicleThrustSetpoint.
  se3::Vector3 normalized_torque{};
  double normalized_thrust_z{0.0};

  // Intermediate values retained for diagnostics and verification.
  std::array<double, kRotorCount> rotor_thrust_n{};
  std::array<double, kRotorCount> actuator_control{};
};


// Convert a feasible physical body wrench into the normalized control
// coordinates expected by the pinned PX4 control allocator.
//
// collective_thrust_n is the positive magnitude along body -Z.
// moment_nm is an FRD body moment.
//
// The conversion explicitly composes:
//   physical wrench
//     -> physical rotor thrust
//     -> rotor velocity
//     -> PX4 actuator_motors control
//     -> normalized PX4 torque/thrust setpoint
//
// The final step reproduces the pinned PX4 multirotor pseudo-inverse
// normalization. No guessed maximum-torque constant is used.
Output adapt(
  const Parameters & parameters,
  double collective_thrust_n,
  const se3::Vector3 & moment_nm);

}  // namespace offboard_controllers::lee_px4_adapter
