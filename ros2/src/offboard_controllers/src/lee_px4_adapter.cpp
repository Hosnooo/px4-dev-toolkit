/*
 * Physical wrench -> normalized PX4 multicopter control adapter.
 *
 * PX4 source of truth:
 *   ANCL/PX4-Autopilot
 *   commit f5083ca2c5b919350880e8667e636ef70715db17
 *
 * Relevant PX4 paths:
 *   src/modules/control_allocator/VehicleActuatorEffectiveness/
 *     ActuatorEffectivenessRotors.cpp
 *   src/lib/control_allocation/control_allocation/
 *     ControlAllocationPseudoInverse.cpp
 *   src/lib/mixer_module/functions/FunctionMotors.hpp
 *   src/lib/mixer_module/mixer_module.cpp
 *   src/modules/simulation/gz_bridge/GZMixingInterfaceESC.cpp
 *
 * This is a toolkit adapter, not part of Lee's controller. Lee's controller
 * ends with a physical collective thrust and physical body moment.
 */

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>

#include <offboard_controllers/lee_px4_adapter.hpp>
#include <offboard_controllers/se3/math.hpp>

namespace offboard_controllers::lee_px4_adapter
{

namespace
{

using Matrix4 =
  std::array<
    std::array<double, kRotorCount>,
    kRotorCount>;

using Vector4 =
  std::array<double, kRotorCount>;

constexpr double kMatrixTolerance = 1.0e-12;
constexpr double kFeasibilityTolerance = 1.0e-9;


void require_finite(
  double value,
  const char * message)
{
  if (!std::isfinite(value)) {
    throw std::invalid_argument(message);
  }
}


void require_positive(
  double value,
  const char * message)
{
  if (!std::isfinite(value) || value <= 0.0) {
    throw std::invalid_argument(message);
  }
}


Matrix4 inverse(
  Matrix4 matrix)
{
  Matrix4 result{};

  for (std::size_t row = 0; row < kRotorCount; ++row) {
    result[row][row] = 1.0;
  }

  for (std::size_t column = 0; column < kRotorCount; ++column) {
    std::size_t pivot_row = column;

    for (
      std::size_t row = column + 1;
      row < kRotorCount;
      ++row)
    {
      if (
        std::abs(matrix[row][column]) >
        std::abs(matrix[pivot_row][column]))
      {
        pivot_row = row;
      }
    }

    if (
      !std::isfinite(matrix[pivot_row][column]) ||
      std::abs(matrix[pivot_row][column]) <=
      kMatrixTolerance)
    {
      throw std::invalid_argument(
              "Lee/PX4 adapter matrix is singular.");
    }

    if (pivot_row != column) {
      std::swap(
        matrix[pivot_row],
        matrix[column]);

      std::swap(
        result[pivot_row],
        result[column]);
    }

    const double pivot =
      matrix[column][column];

    for (
      std::size_t entry = 0;
      entry < kRotorCount;
      ++entry)
    {
      matrix[column][entry] /= pivot;
      result[column][entry] /= pivot;
    }

    for (std::size_t row = 0; row < kRotorCount; ++row) {
      if (row == column) {
        continue;
      }

      const double factor =
        matrix[row][column];

      for (
        std::size_t entry = 0;
        entry < kRotorCount;
        ++entry)
      {
        matrix[row][entry] -=
          factor *
          matrix[column][entry];

        result[row][entry] -=
          factor *
          result[column][entry];
      }
    }
  }

  return result;
}


Vector4 multiply(
  const Matrix4 & matrix,
  const Vector4 & vector)
{
  Vector4 result{};

  for (std::size_t row = 0; row < kRotorCount; ++row) {
    for (
      std::size_t column = 0;
      column < kRotorCount;
      ++column)
    {
      result[row] +=
        matrix[row][column] *
        vector[column];
    }
  }

  return result;
}


Matrix4 physical_effectiveness(
  const Parameters & parameters)
{
  Matrix4 effectiveness{};

  for (std::size_t rotor = 0; rotor < kRotorCount; ++rotor) {
    const PhysicalRotor & data =
      parameters.physical_rotors[rotor];

    if (
      !se3::is_finite(data.position_frd_m) ||
      !std::isfinite(data.yaw_moment_ratio))
    {
      throw std::invalid_argument(
              "Lee/PX4 physical rotor data must be finite.");
    }

    // A rotor produces force [0, 0, -T] in FRD.
    //
    // r x F =
    //   [-y T, x T, 0]
    //
    // The signed reaction-moment ratio supplies the FRD yaw moment.
    effectiveness[0][rotor] =
      -data.position_frd_m.y;

    effectiveness[1][rotor] =
      data.position_frd_m.x;

    effectiveness[2][rotor] =
      data.yaw_moment_ratio;

    effectiveness[3][rotor] =
      1.0;
  }

  return effectiveness;
}


Matrix4 allocator_effectiveness(
  const Parameters & parameters)
{
  Matrix4 effectiveness{};

  for (std::size_t rotor = 0; rotor < kRotorCount; ++rotor) {
    const AllocatorRotor & data =
      parameters.allocator_rotors[rotor];

    if (
      !se3::is_finite(data.position_frd_m) ||
      !std::isfinite(data.moment_ratio) ||
      !std::isfinite(data.thrust_coefficient) ||
      data.thrust_coefficient <= 0.0)
    {
      throw std::invalid_argument(
              "Lee/PX4 allocator rotor data are invalid.");
    }

    // Pinned PX4 ActuatorEffectivenessRotors for an upward multicopter
    // rotor whose normalized axis is [0, 0, -1]:
    //
    //   thrust = CT * axis
    //   moment = CT * position x axis - CT * KM * axis
    //
    // Active rows are roll, pitch, yaw, and body-Z thrust.
    const double ct =
      data.thrust_coefficient;

    effectiveness[0][rotor] =
      -ct * data.position_frd_m.y;

    effectiveness[1][rotor] =
      ct * data.position_frd_m.x;

    effectiveness[2][rotor] =
      ct * data.moment_ratio;

    effectiveness[3][rotor] =
      -ct;
  }

  return effectiveness;
}


Vector4 allocator_normalization_scale(
  const Matrix4 & effectiveness)
{
  // For four independent multicopter controls, the Moore-Penrose
  // pseudo-inverse is the ordinary matrix inverse.
  const Matrix4 mix =
    inverse(effectiveness);

  int nonzero_roll = 0;
  int nonzero_pitch = 0;

  double roll_norm_squared = 0.0;
  double pitch_norm_squared = 0.0;

  for (std::size_t rotor = 0; rotor < kRotorCount; ++rotor) {
    const double roll =
      mix[rotor][0];

    const double pitch =
      mix[rotor][1];

    roll_norm_squared += roll * roll;
    pitch_norm_squared += pitch * pitch;

    if (std::abs(roll) > 1.0e-3) {
      ++nonzero_roll;
    }

    if (std::abs(pitch) > 1.0e-3) {
      ++nonzero_pitch;
    }
  }

  if (nonzero_roll == 0 || nonzero_pitch == 0) {
    throw std::invalid_argument(
            "Lee/PX4 allocator cannot control roll and pitch.");
  }

  const double roll_scale =
    std::sqrt(
      roll_norm_squared /
      (
        static_cast<double>(nonzero_roll) /
        2.0
      ));

  const double pitch_scale =
    std::sqrt(
      pitch_norm_squared /
      (
        static_cast<double>(nonzero_pitch) /
        2.0
      ));

  const double roll_pitch_scale =
    std::max(
      roll_scale,
      pitch_scale);

  double yaw_scale =
    mix[0][2];

  for (std::size_t rotor = 1; rotor < kRotorCount; ++rotor) {
    yaw_scale =
      std::max(
        yaw_scale,
        mix[rotor][2]);
  }

  constexpr double float_epsilon =
    std::numeric_limits<float>::epsilon();

  if (
    !std::isfinite(roll_pitch_scale) ||
    roll_pitch_scale <= float_epsilon ||
    !std::isfinite(yaw_scale) ||
    yaw_scale <= float_epsilon)
  {
    throw std::invalid_argument(
            "Lee/PX4 allocator normalization is invalid.");
  }

  int nonzero_thrust = 0;
  double thrust_norm_sum = 0.0;

  for (std::size_t rotor = 0; rotor < kRotorCount; ++rotor) {
    const double magnitude =
      std::abs(
        mix[rotor][3]);

    thrust_norm_sum += magnitude;

    if (magnitude > float_epsilon) {
      ++nonzero_thrust;
    }
  }

  if (nonzero_thrust == 0) {
    throw std::invalid_argument(
            "Lee/PX4 allocator cannot control collective thrust.");
  }

  const double thrust_scale =
    thrust_norm_sum /
    static_cast<double>(nonzero_thrust);

  return {
    roll_pitch_scale,
    roll_pitch_scale,
    yaw_scale,
    thrust_scale,
  };
}


void validate_parameters(
  const Parameters & parameters)
{
  require_positive(
    parameters.motor_constant,
    "Lee/PX4 motor constant must be finite and positive.");

  require_finite(
    parameters.output_min_rad_s,
    "Lee/PX4 minimum rotor output must be finite.");

  require_finite(
    parameters.output_max_rad_s,
    "Lee/PX4 maximum rotor output must be finite.");

  if (
    parameters.output_min_rad_s < 0.0 ||
    parameters.output_max_rad_s <=
    parameters.output_min_rad_s)
  {
    throw std::invalid_argument(
            "Lee/PX4 rotor-output range is invalid.");
  }

  if (
    !std::isfinite(parameters.thrust_model_factor) ||
    parameters.thrust_model_factor < 0.0 ||
    parameters.thrust_model_factor > 1.0)
  {
    throw std::invalid_argument(
            "Lee/PX4 thrust-model factor must be in [0, 1].");
  }

  // Construct both matrices here so invalid or singular vehicle data fail
  // deterministically before any conversion is attempted.
  (void)inverse(
    physical_effectiveness(
      parameters));

  (void)allocator_normalization_scale(
    allocator_effectiveness(
      parameters));
}

}  // namespace


Output adapt(
  const Parameters & parameters,
  double collective_thrust_n,
  const se3::Vector3 & moment_nm)
{
  validate_parameters(
    parameters);

  if (
    !std::isfinite(collective_thrust_n) ||
    collective_thrust_n < 0.0 ||
    !se3::is_finite(moment_nm))
  {
    throw std::invalid_argument(
            "Lee physical wrench must contain finite values and "
            "non-negative collective thrust.");
  }

  const Matrix4 physical_matrix =
    physical_effectiveness(
      parameters);

  const Vector4 physical_wrench{
    moment_nm.x,
    moment_nm.y,
    moment_nm.z,
    collective_thrust_n,
  };

  const Vector4 rotor_thrust =
    multiply(
      inverse(
        physical_matrix),
      physical_wrench);

  std::array<double, kRotorCount> actuator_control{};

  const double output_range =
    parameters.output_max_rad_s -
    parameters.output_min_rad_s;

  for (std::size_t rotor = 0; rotor < kRotorCount; ++rotor) {
    double thrust =
      rotor_thrust[rotor];

    if (thrust < -kFeasibilityTolerance) {
      throw std::domain_error(
              "Lee physical wrench requires negative rotor thrust.");
    }

    thrust =
      std::max(
        thrust,
        0.0);

    const double rotor_velocity =
      std::sqrt(
        thrust /
        parameters.motor_constant);

    if (
      rotor_velocity <
      parameters.output_min_rad_s -
      kFeasibilityTolerance ||
      rotor_velocity >
      parameters.output_max_rad_s +
      kFeasibilityTolerance)
    {
      throw std::domain_error(
              "Lee physical wrench exceeds the configured rotor-output "
              "range.");
    }

    const double signal =
      std::clamp(
        (
          rotor_velocity -
          parameters.output_min_rad_s
        ) / output_range,
        0.0,
        1.0);

    // FunctionMotors in PX4 performs the inverse of this equation before
    // mapping [0, 1] to the ESC output range. Supplying this value therefore
    // reconstructs the required rotor-velocity signal.
    actuator_control[rotor] =
      parameters.thrust_model_factor *
      signal *
      signal +
      (
        1.0 -
        parameters.thrust_model_factor
      ) *
      signal;
  }

  const Matrix4 allocator_matrix =
    allocator_effectiveness(
      parameters);

  const Vector4 allocator_control =
    multiply(
      allocator_matrix,
      actuator_control);

  const Vector4 scale =
    allocator_normalization_scale(
      allocator_matrix);

  // Pinned PX4 first computes the pseudo-inverse of the physical
  // effectiveness matrix and then divides each mixer column by these
  // normalization scales. Therefore the normalized control setpoint that
  // reconstructs a chosen actuator vector is:
  //
  //   c_normalized = scale .* (B * u)
  //
  // where B is the unnormalized PX4 effectiveness matrix.
  const se3::Vector3 normalized_torque{
    scale[0] * allocator_control[0],
    scale[1] * allocator_control[1],
    scale[2] * allocator_control[2],
  };

  const double normalized_thrust_z =
    scale[3] *
    allocator_control[3];

  if (
    !se3::is_finite(normalized_torque) ||
    !std::isfinite(normalized_thrust_z))
  {
    throw std::runtime_error(
            "Lee/PX4 adapter produced a non-finite normalized wrench.");
  }

  return {
    normalized_torque,
    normalized_thrust_z,
    rotor_thrust,
    actuator_control,
  };
}

}  // namespace offboard_controllers::lee_px4_adapter
