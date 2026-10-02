#pragma once

namespace offboard_controllers::se3
{

// Translational vectors use PX4 local NED coordinates unless a field
// explicitly states otherwise. Rotational body-frame vectors use FRD.
struct Vector3
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};


struct RotationMatrix
{
  // Rotation from FRD body coordinates to the NED inertial frame. The
  // columns are the current body axes expressed in NED.
  Vector3 b1{};
  Vector3 b2{};
  Vector3 b3{};
};


struct InertiaMatrix
{
  // Symmetric FRD body-frame inertia tensor [kg m^2].
  double xx{0.0};
  double xy{0.0};
  double xz{0.0};
  double yy{0.0};
  double yz{0.0};
  double zz{0.0};
};


struct State
{
  // Position [m] and velocity [m/s] in local NED.
  Vector3 position{};
  Vector3 velocity{};

  // Current FRD-to-NED attitude and measured FRD body angular velocity
  // [rad/s]. Translational acceleration and jerk are intentionally absent:
  // desired-attitude derivatives are obtained analytically from the nominal
  // quadrotor dynamics rather than by differentiating estimator outputs.
  RotationMatrix attitude{};
  Vector3 angular_velocity{};
};


struct Reference
{
  // Flat-output translation reference in NED. Jerk and snap are analytic
  // trajectory derivatives [m/s^3] and [m/s^4], not measured derivatives.
  Vector3 position{};
  Vector3 velocity{};
  Vector3 acceleration{};
  Vector3 jerk{};
  Vector3 snap{};

  // Heading reference and its first two derivatives [rad], [rad/s],
  // [rad/s^2].
  double yaw{0.0};
  double yaw_rate{0.0};
  double yaw_acceleration{0.0};
};


struct Parameters
{
  // Physical translational gains:
  //   kx [N/m]
  //   kv [N s/m]
  double mass{0.0};
  double kx{0.0};
  double kv{0.0};

  // Toolkit cascaded attitude-to-rate gain. It maps the dimensionless SO(3)
  // attitude error e_R to a corrective FRD body-rate command [rad/s].
  Vector3 attitude_gain{};

  // Toolkit geometric-normalized gains. These map the geometric tracking
  // terms directly into PX4-normalized torque coordinates; they are not
  // physical Lee moment gains.
  Vector3 normalized_attitude_gain{};
  Vector3 normalized_angular_velocity_gain{};
  Vector3 normalized_angular_acceleration_gain{};
};


struct TranslationalOutput
{
  // Kinematic acceleration command [m/s^2] in NED. Gravity is not included
  // because the PX4 acceleration handoff expects a kinematic setpoint.
  Vector3 acceleration{};

  // Lee translational control vector A [N] in NED:
  //
  //   A = -k_x e_x - k_v e_v - m g e3 + m x_ddot_d
  //
  // The desired thrust direction is b3_d = -A / ||A||. This is a physical
  // force-like control vector, not a PX4-normalized thrust command.
  Vector3 force_vector{};
};


struct DesiredAttitudeRate
{
  RotationMatrix attitude{};

  // Omega_d: angular velocity of the desired attitude trajectory, expressed
  // in the desired FRD body frame. This is a kinematic reference, not an
  // attitude-controller-generated body-rate setpoint.
  Vector3 angular_velocity{};
};


struct DesiredAttitudeDynamics
{
  RotationMatrix attitude{};

  // Omega_d and dot(Omega_d) are kinematic derivatives of R_d, expressed in
  // the desired FRD body frame. Neither quantity is an inner-loop rate command.
  Vector3 angular_velocity{};
  Vector3 angular_acceleration{};
};


struct GeometricNormalizedOutput
{
  // Controller contributions and final command in PX4-normalized FRD torque
  // coordinates. The three contributions sum to normalized_torque.
  Vector3 attitude_feedback{};
  Vector3 angular_velocity_feedback{};
  Vector3 angular_acceleration_feedforward{};
  Vector3 normalized_torque{};
};

}  // namespace offboard_controllers::se3
