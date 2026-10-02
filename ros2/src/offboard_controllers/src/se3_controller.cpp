/*
 * Geometric controller based on:
 *
 * T. Lee, M. Leok, and N. H. McClamroch,
 * "Geometric Tracking Control of a Quadrotor UAV on SE(3)",
 * 49th IEEE Conference on Decision and Control, 2010.
 * DOI: 10.1109/CDC.2010.5717652
 *
 * The analytical desired-attitude derivative construction follows the open
 * FDCL reference implementation of the same geometric-control family:
 *
 *   fdcl-gwu/uav_geometric_control
 *   commit f5fcb51c3b962152a4895a9d2e86e4a184655741
 *   cpp/src/fdcl_control.cpp, control::position_control()
 *
 * Controller mathematics stay independent of ROS 2 and PX4 transport.
 */

#include <cmath>
#include <stdexcept>

#include <offboard_controllers/se3/constants.hpp>
#include <offboard_controllers/se3/controller.hpp>
#include <offboard_controllers/se3/math.hpp>

namespace offboard_controllers::se3
{

namespace
{

constexpr double kDirectionTolerance = 1.0e-12;

struct DirectionDynamics
{
  Vector3 value{};
  Vector3 derivative{};
  Vector3 second_derivative{};
};

struct BasisDynamics
{
  DirectionDynamics b1{};
  DirectionDynamics b2{};
  DirectionDynamics b3{};
};

void require_positive_finite(
  double value,
  const char * message)
{
  if (!std::isfinite(value) || value <= 0.0) {
    throw std::invalid_argument(message);
  }
}


void require_nonnegative_finite(
  const Vector3 & value,
  const char * message)
{
  if (
    !is_finite(value) ||
    value.x < 0.0 ||
    value.y < 0.0 ||
    value.z < 0.0)
  {
    throw std::invalid_argument(message);
  }
}


Vector3 normalized_direction(
  const Vector3 & vector,
  const char * error_message)
{
  const double magnitude = norm(vector);

  if (
    !std::isfinite(magnitude) ||
    magnitude <= kDirectionTolerance)
  {
    throw std::invalid_argument(error_message);
  }

  return vector / magnitude;
}


DirectionDynamics normalized_direction_dynamics(
  const Vector3 & vector,
  const Vector3 & derivative,
  const Vector3 & second_derivative,
  const char * error_message)
{
  const double magnitude = norm(vector);

  if (
    !std::isfinite(magnitude) ||
    magnitude <= kDirectionTolerance ||
    !is_finite(derivative) ||
    !is_finite(second_derivative))
  {
    throw std::invalid_argument(error_message);
  }

  const Vector3 direction =
    vector / magnitude;

  const double magnitude_derivative =
    dot(direction, derivative);

  const Vector3 direction_derivative =
    (
      derivative -
      magnitude_derivative * direction
    ) / magnitude;

  const double magnitude_second_derivative =
    (
      dot(derivative, derivative) +
      dot(vector, second_derivative) -
      magnitude_derivative * magnitude_derivative
    ) / magnitude;

  const Vector3 direction_second_derivative =
    (
      second_derivative -
      2.0 * magnitude_derivative * direction_derivative -
      magnitude_second_derivative * direction
    ) / magnitude;

  return {
    direction,
    direction_derivative,
    direction_second_derivative,
  };
}


BasisDynamics desired_basis_dynamics(
  const Vector3 & force,
  const Vector3 & force_derivative,
  const Vector3 & force_second_derivative,
  double yaw,
  double yaw_rate,
  double yaw_acceleration)
{
  if (
    !is_finite(force) ||
    !is_finite(force_derivative) ||
    !is_finite(force_second_derivative) ||
    !std::isfinite(yaw) ||
    !std::isfinite(yaw_rate) ||
    !std::isfinite(yaw_acceleration))
  {
    throw std::invalid_argument(
            "SE3 desired-attitude dynamics require finite inputs.");
  }

  // b3_d = -A / ||A||. Differentiating the normalized force analytically
  // keeps desired angular-rate feedforward tied to the same force used for Rd.
  const DirectionDynamics b3 =
    normalized_direction_dynamics(
      -force,
      -force_derivative,
      -force_second_derivative,
      "SE3 desired attitude requires a non-zero force vector.");

  const double cosine = std::cos(yaw);
  const double sine = std::sin(yaw);

  const Vector3 b1c{
    cosine,
    sine,
    0.0,
  };

  const Vector3 b1c_derivative{
    -sine * yaw_rate,
    cosine * yaw_rate,
    0.0,
  };

  const Vector3 b1c_second_derivative{
    -cosine * yaw_rate * yaw_rate - sine * yaw_acceleration,
    -sine * yaw_rate * yaw_rate + cosine * yaw_acceleration,
    0.0,
  };

  const Vector3 b2_raw =
    cross(b3.value, b1c);

  const Vector3 b2_raw_derivative =
    cross(b3.derivative, b1c) +
    cross(b3.value, b1c_derivative);

  const Vector3 b2_raw_second_derivative =
    cross(b3.second_derivative, b1c) +
    2.0 * cross(b3.derivative, b1c_derivative) +
    cross(b3.value, b1c_second_derivative);

  const DirectionDynamics b2 =
    normalized_direction_dynamics(
      b2_raw,
      b2_raw_derivative,
      b2_raw_second_derivative,
      "SE3 desired attitude heading is degenerate.");

  const DirectionDynamics b1{
    cross(b2.value, b3.value),
    cross(b2.derivative, b3.value) +
    cross(b2.value, b3.derivative),
    cross(b2.second_derivative, b3.value) +
    2.0 * cross(b2.derivative, b3.derivative) +
    cross(b2.value, b3.second_derivative),
  };

  return {
    b1,
    b2,
    b3,
  };
}


RotationMatrix rotation_from_basis(
  const BasisDynamics & basis)
{
  return {
    basis.b1.value,
    basis.b2.value,
    basis.b3.value,
  };
}


Vector3 desired_angular_velocity(
  const BasisDynamics & basis)
{
  // R_d^T Rdot_d = hat(Omega_d). Taking the skew part suppresses only
  // floating-point loss of orthogonality; analytically it is already skew.
  return {
    0.5 * (
      dot(basis.b3.value, basis.b2.derivative) -
      dot(basis.b2.value, basis.b3.derivative)),
    0.5 * (
      dot(basis.b1.value, basis.b3.derivative) -
      dot(basis.b3.value, basis.b1.derivative)),
    0.5 * (
      dot(basis.b2.value, basis.b1.derivative) -
      dot(basis.b1.value, basis.b2.derivative)),
  };
}


Vector3 desired_angular_acceleration(
  const BasisDynamics & basis)
{
  // d/dt(R_d^T Rdot_d) = hat(Omegadot_d). Terms quadratic in Rdot_d are
  // symmetric and disappear when the skew part is taken.
  return {
    0.5 * (
      dot(basis.b3.value, basis.b2.second_derivative) -
      dot(basis.b2.value, basis.b3.second_derivative)),
    0.5 * (
      dot(basis.b1.value, basis.b3.second_derivative) -
      dot(basis.b3.value, basis.b1.second_derivative)),
    0.5 * (
      dot(basis.b2.value, basis.b1.second_derivative) -
      dot(basis.b1.value, basis.b2.second_derivative)),
  };
}


Vector3 desired_angular_velocity_in_current_body(
  const RotationMatrix & attitude,
  const RotationMatrix & desired_attitude,
  const Vector3 & desired_angular_velocity)
{
  return rotate_inertial_to_body(
    attitude,
    rotate_body_to_inertial(
      desired_attitude,
      desired_angular_velocity));
}

}  // namespace


Controller::Controller(const Parameters & parameters)
: parameters_(parameters)
{
  require_positive_finite(
    parameters_.mass,
    "SE3 vehicle mass must be finite and positive.");

  require_positive_finite(
    parameters_.kx,
    "SE3 position gain kx must be finite and positive.");

  require_positive_finite(
    parameters_.kv,
    "SE3 velocity gain kv must be finite and positive.");

  require_nonnegative_finite(
    parameters_.attitude_gain,
    "SE3 attitude gain must be finite and non-negative.");

  require_nonnegative_finite(
    parameters_.normalized_attitude_gain,
    "SE3 normalized attitude gain must be finite and non-negative.");

  require_nonnegative_finite(
    parameters_.normalized_angular_velocity_gain,
    "SE3 normalized angular-velocity gain must be finite and non-negative.");

  require_nonnegative_finite(
    parameters_.normalized_angular_acceleration_gain,
    "SE3 normalized angular-acceleration gain must be finite and non-negative.");
}


TranslationalOutput Controller::compute_translation(
  const State & state,
  const Reference & reference) const
{
  if (
    !is_finite(state.position) ||
    !is_finite(state.velocity) ||
    !is_finite(reference.position) ||
    !is_finite(reference.velocity) ||
    !is_finite(reference.acceleration) ||
    !std::isfinite(reference.yaw))
  {
    throw std::invalid_argument(
            "SE3 state and reference values must be finite.");
  }

  const Vector3 position_error =
    state.position - reference.position;

  const Vector3 velocity_error =
    state.velocity - reference.velocity;

  // PX4 acceleration handoff expects kinematic acceleration without gravity:
  //
  //   a_cmd = x_ddot_d - (kx / m)e_x - (kv / m)e_v
  const Vector3 acceleration =
    reference.acceleration -
    (parameters_.kx / parameters_.mass) * position_error -
    (parameters_.kv / parameters_.mass) * velocity_error;

  // In NED, e3 points Down.
  const Vector3 gravity{
    0.0,
    0.0,
    kStandardGravity,
  };

  const Vector3 force_vector =
    parameters_.mass * (acceleration - gravity);

  return {
    acceleration,
    force_vector,
  };
}


Vector3 Controller::compute_force_derivative(
  const State & state,
  const Reference & reference,
  const Vector3 & force) const
{
  if (
    !is_finite(state.velocity) ||
    !is_finite(state.attitude) ||
    !is_finite(reference.velocity) ||
    !is_finite(reference.acceleration) ||
    !is_finite(reference.jerk) ||
    !is_finite(force))
  {
    throw std::invalid_argument(
            "SE3 force derivative requires finite state, reference, "
            "and force data.");
  }

  const Vector3 velocity_error =
    state.velocity - reference.velocity;

  // Lee translational control vector in NED:
  //
  //   A = -k_x e_x - k_v e_v - m g e3 + m x_ddot_d
  //
  // Rather than reading an estimated acceleration and later differentiating
  // it to obtain jerk, use the nominal quadrotor translational dynamics:
  //
  //   f       = -A . b3
  //   a_model = g e3 - (f / m) b3
  //
  // This is the model-based derivative construction used by the FDCL
  // geometric-controller implementation cited at the top of this file.
  const double collective_thrust =
    -dot(force, state.attitude.b3);

  const Vector3 gravity{
    0.0,
    0.0,
    kStandardGravity,
  };

  const Vector3 model_acceleration =
    gravity -
    (collective_thrust / parameters_.mass) *
    state.attitude.b3;

  const Vector3 acceleration_error =
    model_acceleration - reference.acceleration;

  //   A_dot = m x_d^(3) - k_x e_v - k_v e_a
  return
    parameters_.mass * reference.jerk -
    parameters_.kx * velocity_error -
    parameters_.kv * acceleration_error;
}


Vector3 Controller::compute_force_second_derivative(
  const State & state,
  const Reference & reference,
  const Vector3 & force,
  const Vector3 & force_derivative) const
{
  if (
    !is_finite(state.attitude) ||
    !is_finite(state.angular_velocity) ||
    !is_finite(reference.acceleration) ||
    !is_finite(reference.jerk) ||
    !is_finite(reference.snap) ||
    !is_finite(force) ||
    !is_finite(force_derivative))
  {
    throw std::invalid_argument(
            "SE3 force second derivative requires finite state, reference, "
            "and force data.");
  }

  const Vector3 body_z_axis =
    state.attitude.b3;

  const double collective_thrust =
    -dot(force, body_z_axis);

  const Vector3 gravity{
    0.0,
    0.0,
    kStandardGravity,
  };

  const Vector3 model_acceleration =
    gravity -
    (collective_thrust / parameters_.mass) *
    body_z_axis;

  // R_dot = R hat(Omega), so
  //
  //   b3_dot = R hat(Omega) e3.
  //
  // Omega is the measured FRD body angular velocity; b3_dot is therefore
  // expressed in NED, like A and the translational reference derivatives.
  const Vector3 body_z_axis_derivative =
    rotate_body_to_inertial(
      state.attitude,
      cross(
        state.angular_velocity,
        Vector3{0.0, 0.0, 1.0}));

  // Differentiate f = -A . b3 analytically:
  //
  //   f_dot = -A_dot . b3 - A . b3_dot.
  const double collective_thrust_derivative =
    -dot(force_derivative, body_z_axis) -
    dot(force, body_z_axis_derivative);

  // From a_model = g e3 - (f / m)b3:
  //
  //   j_model = -(f_dot / m)b3 - (f / m)b3_dot.
  //
  // This avoids the noise amplification and delay of finite-differencing an
  // estimated acceleration signal.
  const Vector3 model_jerk =
    -(collective_thrust_derivative / parameters_.mass) *
    body_z_axis -
    (collective_thrust / parameters_.mass) *
    body_z_axis_derivative;

  const Vector3 acceleration_error =
    model_acceleration - reference.acceleration;

  const Vector3 jerk_error =
    model_jerk - reference.jerk;

  //   A_ddot = m x_d^(4) - k_x e_a - k_v e_j
  return
    parameters_.mass * reference.snap -
    parameters_.kx * acceleration_error -
    parameters_.kv * jerk_error;
}


RotationMatrix Controller::compute_desired_attitude(
  const Vector3 & force,
  double yaw) const
{
  if (!is_finite(force) || !std::isfinite(yaw)) {
    throw std::invalid_argument(
            "SE3 force and yaw must be finite.");
  }

  const Vector3 b3d =
    normalized_direction(
      -force,
      "SE3 desired attitude requires a non-zero force vector.");

  const Vector3 b1c{
    std::cos(yaw),
    std::sin(yaw),
    0.0,
  };

  const Vector3 b2d =
    normalized_direction(
      cross(b3d, b1c),
      "SE3 desired attitude heading is degenerate.");

  const Vector3 b1d =
    cross(b2d, b3d);

  return {
    b1d,
    b2d,
    b3d,
  };
}


DesiredAttitudeRate Controller::compute_desired_attitude_rate(
  const Vector3 & force,
  const Vector3 & force_derivative,
  double yaw,
  double yaw_rate) const
{
  const BasisDynamics basis =
    desired_basis_dynamics(
      force,
      force_derivative,
      {},
      yaw,
      yaw_rate,
      0.0);

  return {
    rotation_from_basis(basis),
    desired_angular_velocity(basis),
  };
}


DesiredAttitudeDynamics Controller::compute_desired_attitude_dynamics(
  const Vector3 & force,
  const Vector3 & force_derivative,
  const Vector3 & force_second_derivative,
  double yaw,
  double yaw_rate,
  double yaw_acceleration) const
{
  const BasisDynamics basis =
    desired_basis_dynamics(
      force,
      force_derivative,
      force_second_derivative,
      yaw,
      yaw_rate,
      yaw_acceleration);

  return {
    rotation_from_basis(basis),
    desired_angular_velocity(basis),
    desired_angular_acceleration(basis),
  };
}


Vector3 Controller::compute_attitude_error(
  const RotationMatrix & attitude,
  const RotationMatrix & desired_attitude) const
{
  if (!is_finite(attitude) || !is_finite(desired_attitude)) {
    throw std::invalid_argument(
            "SE3 attitude error requires finite rotations.");
  }

  // Lee, Leok, McClamroch (CDC 2010), Eq. (10):
  //
  //   e_R = 1/2 (R_d^T R - R^T R_d)^vee
  //
  // R and R_d map FRD body coordinates into NED. The resulting geometric
  // attitude error is expressed in the current FRD body frame.
  return {
    0.5 * (
      dot(desired_attitude.b3, attitude.b2) -
      dot(desired_attitude.b2, attitude.b3)),
    0.5 * (
      dot(desired_attitude.b1, attitude.b3) -
      dot(desired_attitude.b3, attitude.b1)),
    0.5 * (
      dot(desired_attitude.b2, attitude.b1) -
      dot(desired_attitude.b1, attitude.b2)),
  };
}


Vector3 Controller::compute_attitude_rate_command(
  const RotationMatrix & attitude,
  const DesiredAttitudeRate & desired) const
{
  if (
    !is_finite(attitude) ||
    !is_finite(desired.attitude) ||
    !is_finite(desired.angular_velocity))
  {
    throw std::invalid_argument(
            "SE3 attitude-rate command requires finite inputs.");
  }

  const Vector3 attitude_error =
    compute_attitude_error(
      attitude,
      desired.attitude);

  const Vector3 feedforward =
    desired_angular_velocity_in_current_body(
      attitude,
      desired.attitude,
      desired.angular_velocity);

  // Toolkit cascaded geometric attitude-to-rate law:
  //
  //   Omega_sp = R^T R_d Omega_d - K_att e_R
  //
  // Omega_d is the desired-attitude kinematic reference. Omega_sp is the
  // controller-generated body-rate setpoint handed to PX4. This cascade is
  // deliberately separate from Lee's direct physical-moment controller.
  return
    feedforward -
    component_product(
      parameters_.attitude_gain,
      attitude_error);
}


GeometricNormalizedOutput Controller::compute_geometric_normalized_torque(
  const RotationMatrix & attitude,
  const Vector3 & angular_velocity,
  const DesiredAttitudeDynamics & desired) const
{
  if (
    !is_finite(attitude) ||
    !is_finite(angular_velocity) ||
    !is_finite(desired.attitude) ||
    !is_finite(desired.angular_velocity) ||
    !is_finite(desired.angular_acceleration))
  {
    throw std::invalid_argument(
            "SE3 geometric-normalized torque requires finite inputs.");
  }

  // Lee 2010, Eq. (10).
  const Vector3 attitude_error =
    compute_attitude_error(
      attitude,
      desired.attitude);

  const Vector3 desired_angular_velocity_current_body =
    desired_angular_velocity_in_current_body(
      attitude,
      desired.attitude,
      desired.angular_velocity);

  // Lee 2010, Eq. (11):
  //
  //   e_Omega = Omega - R^T R_d Omega_d
  //
  // Omega_d is the angular velocity of R_d itself, not a body-rate command.
  const Vector3 angular_velocity_error =
    angular_velocity -
    desired_angular_velocity_current_body;

  // Time derivative of R^T R_d Omega_d, expressed in the current body frame:
  //
  //   alpha_ff = -Omega x (R^T R_d Omega_d)
  //              + R^T R_d dot(Omega_d)
  const Vector3 angular_acceleration_feedforward =
    -cross(
      angular_velocity,
      desired_angular_velocity_current_body) +
    desired_angular_velocity_in_current_body(
      attitude,
      desired.attitude,
      desired.angular_acceleration);

  // Toolkit normalized geometric controller:
  //
  //   tau_n = -K_R^n e_R - K_Omega^n e_Omega
  //           + K_alpha^n alpha_ff
  //
  // This deliberately preserves Lee's SO(3) tracking errors but does not
  // implement Lee 2010 Eq. (13): there is no physical inertia J, gyroscopic
  // moment Omega x J Omega, or N*m output here. tau_n is directly in the
  // normalized coordinates expected by PX4 VehicleTorqueSetpoint.
  const Vector3 attitude_feedback =
    -component_product(
      parameters_.normalized_attitude_gain,
      attitude_error);

  const Vector3 angular_velocity_feedback =
    -component_product(
      parameters_.normalized_angular_velocity_gain,
      angular_velocity_error);

  const Vector3 acceleration_feedforward =
    component_product(
      parameters_.normalized_angular_acceleration_gain,
      angular_acceleration_feedforward);

  return {
    attitude_feedback,
    angular_velocity_feedback,
    acceleration_feedforward,
    attitude_feedback +
    angular_velocity_feedback +
    acceleration_feedforward,
  };
}


Vector3 Controller::compute_physical_moment_command(
  const RotationMatrix & attitude,
  const Vector3 & angular_velocity,
  const DesiredAttitudeDynamics & desired,
  const InertiaMatrix & inertia,
  const Vector3 & k_r,
  const Vector3 & k_omega) const
{
  if (
    !is_finite(attitude) ||
    !is_finite(angular_velocity) ||
    !is_finite(desired.attitude) ||
    !is_finite(desired.angular_velocity) ||
    !is_finite(desired.angular_acceleration) ||
    !is_finite(inertia) ||
    !is_finite(k_r) ||
    !is_finite(k_omega))
  {
    throw std::invalid_argument(
            "SE3 physical moment command requires finite inputs.");
  }

  const Vector3 attitude_error =
    compute_attitude_error(
      attitude,
      desired.attitude);

  const Vector3 angular_velocity_feedforward =
    desired_angular_velocity_in_current_body(
      attitude,
      desired.attitude,
      desired.angular_velocity);

  const Vector3 angular_velocity_error =
    angular_velocity - angular_velocity_feedforward;

  const Vector3 angular_acceleration_feedforward =
    -cross(
      angular_velocity,
      angular_velocity_feedforward) +
    desired_angular_velocity_in_current_body(
      attitude,
      desired.attitude,
      desired.angular_acceleration);

  const Vector3 angular_momentum =
    multiply(
      inertia,
      angular_velocity);

  // Lee physical moment controller:
  // M = -kR eR - kOmega eOmega + Omega x J Omega
  //     - J(hat(Omega) R^T Rd Omega_d - R^T Rd Omegadot_d).
  // The last bracket is -alpha_ff, giving the equivalent +J alpha_ff form.
  return
    -component_product(k_r, attitude_error) -
    component_product(
      k_omega,
      angular_velocity_error) +
    cross(
      angular_velocity,
      angular_momentum) +
    multiply(
      inertia,
      angular_acceleration_feedforward);
}

}  // namespace offboard_controllers::se3
