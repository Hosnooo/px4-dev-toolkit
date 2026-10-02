#include <array>
#include <cmath>
#include <iostream>
#include <stdexcept>

#include <offboard_controllers/lee_px4_adapter.hpp>

namespace
{

constexpr double kTolerance = 1.0e-9;
constexpr double kSqrtHalf =
  0.70710678118654752440;


void expect_near(
  const char * name,
  double actual,
  double expected)
{
  if (std::abs(actual - expected) > kTolerance) {
    std::cerr
      << name
      << ": expected " << expected
      << ", got " << actual
      << '\n';

    throw std::runtime_error(
            "Lee/PX4 adapter test failed.");
  }
}


void expect_vector_near(
  const char * name,
  const offboard_controllers::se3::Vector3 & actual,
  const offboard_controllers::se3::Vector3 & expected)
{
  expect_near(name, actual.x, expected.x);
  expect_near(name, actual.y, expected.y);
  expect_near(name, actual.z, expected.z);
}


offboard_controllers::lee_px4_adapter::Parameters
f450_parameters()
{
  using namespace offboard_controllers;

  constexpr double arm =
    0.1626345596714;

  constexpr double allocator_arm =
    0.159;

  constexpr double physical_yaw_ratio =
    0.0137;

  constexpr double allocator_yaw_ratio =
    0.014;

  constexpr double allocator_ct =
    6.5;

  return {
    {
      lee_px4_adapter::PhysicalRotor{
        {arm, arm, -0.0115},
        physical_yaw_ratio,
      },
      lee_px4_adapter::PhysicalRotor{
        {-arm, -arm, -0.0115},
        physical_yaw_ratio,
      },
      lee_px4_adapter::PhysicalRotor{
        {arm, -arm, -0.0115},
        -physical_yaw_ratio,
      },
      lee_px4_adapter::PhysicalRotor{
        {-arm, arm, -0.0115},
        -physical_yaw_ratio,
      },
    },
    {
      lee_px4_adapter::AllocatorRotor{
        {allocator_arm, allocator_arm, 0.0},
        allocator_ct,
        allocator_yaw_ratio,
      },
      lee_px4_adapter::AllocatorRotor{
        {-allocator_arm, -allocator_arm, 0.0},
        allocator_ct,
        allocator_yaw_ratio,
      },
      lee_px4_adapter::AllocatorRotor{
        {allocator_arm, -allocator_arm, 0.0},
        allocator_ct,
        -allocator_yaw_ratio,
      },
      lee_px4_adapter::AllocatorRotor{
        {-allocator_arm, allocator_arm, 0.0},
        allocator_ct,
        -allocator_yaw_ratio,
      },
    },
    1.2e-05,
    150.0,
    1000.0,
    0.0,
  };
}


struct PhysicalWrench
{
  double collective_thrust_n{0.0};
  offboard_controllers::se3::Vector3 moment_nm{};
};


PhysicalWrench physical_wrench_from_rotor_signals(
  const std::array<double, 4> & control)
{
  using namespace offboard_controllers;

  const auto parameters =
    f450_parameters();

  PhysicalWrench wrench{};

  for (std::size_t rotor = 0; rotor < 4; ++rotor) {
    const double velocity =
      parameters.output_min_rad_s +
      (
        parameters.output_max_rad_s -
        parameters.output_min_rad_s
      ) *
      control[rotor];

    const double thrust =
      parameters.motor_constant *
      velocity *
      velocity;

    const auto & data =
      parameters.physical_rotors[rotor];

    wrench.collective_thrust_n +=
      thrust;

    wrench.moment_nm.x +=
      -data.position_frd_m.y *
      thrust;

    wrench.moment_nm.y +=
      data.position_frd_m.x *
      thrust;

    wrench.moment_nm.z +=
      data.yaw_moment_ratio *
      thrust;
  }

  return wrench;
}


void test_hover_round_trip()
{
  using namespace offboard_controllers;

  const std::array<double, 4> control{
    0.60,
    0.60,
    0.60,
    0.60,
  };

  const PhysicalWrench wrench =
    physical_wrench_from_rotor_signals(
      control);

  const auto output =
    lee_px4_adapter::adapt(
      f450_parameters(),
      wrench.collective_thrust_n,
      wrench.moment_nm);

  expect_vector_near(
    "hover normalized torque",
    output.normalized_torque,
    {});

  expect_near(
    "hover normalized thrust",
    output.normalized_thrust_z,
    -0.60);

  for (std::size_t rotor = 0; rotor < 4; ++rotor) {
    expect_near(
      "hover actuator control",
      output.actuator_control[rotor],
      control[rotor]);
  }
}


void test_roll_round_trip()
{
  using namespace offboard_controllers;

  const std::array<double, 4> control{
    0.55,
    0.65,
    0.65,
    0.55,
  };

  const PhysicalWrench wrench =
    physical_wrench_from_rotor_signals(
      control);

  const auto output =
    lee_px4_adapter::adapt(
      f450_parameters(),
      wrench.collective_thrust_n,
      wrench.moment_nm);

  expect_vector_near(
    "roll normalized torque",
    output.normalized_torque,
    {
      0.05 / kSqrtHalf,
      0.0,
      0.0,
    });

  expect_near(
    "roll normalized thrust",
    output.normalized_thrust_z,
    -0.60);

  for (std::size_t rotor = 0; rotor < 4; ++rotor) {
    expect_near(
      "roll actuator round trip",
      output.actuator_control[rotor],
      control[rotor]);
  }
}


void test_yaw_round_trip()
{
  using namespace offboard_controllers;

  const std::array<double, 4> control{
    0.62,
    0.62,
    0.58,
    0.58,
  };

  const PhysicalWrench wrench =
    physical_wrench_from_rotor_signals(
      control);

  const auto output =
    lee_px4_adapter::adapt(
      f450_parameters(),
      wrench.collective_thrust_n,
      wrench.moment_nm);

  expect_vector_near(
    "yaw normalized torque",
    output.normalized_torque,
    {
      0.0,
      0.0,
      0.02,
    });

  expect_near(
    "yaw normalized thrust",
    output.normalized_thrust_z,
    -0.60);

  for (std::size_t rotor = 0; rotor < 4; ++rotor) {
    expect_near(
      "yaw actuator round trip",
      output.actuator_control[rotor],
      control[rotor]);
  }
}


void test_nonzero_thrust_model_factor_round_trip()
{
  using namespace offboard_controllers;

  auto parameters =
    f450_parameters();

  parameters.thrust_model_factor =
    0.30;

  // These are the normalized rotor-velocity signals after PX4 has inverted
  // its thrust model, not actuator_motors values.
  const std::array<double, 4> rotor_signal{
    0.42,
    0.51,
    0.63,
    0.57,
  };

  const PhysicalWrench wrench =
    physical_wrench_from_rotor_signals(
      rotor_signal);

  const auto output =
    lee_px4_adapter::adapt(
      parameters,
      wrench.collective_thrust_n,
      wrench.moment_nm);

  for (std::size_t rotor = 0; rotor < 4; ++rotor) {
    const double signal =
      rotor_signal[rotor];

    const double expected_control =
      parameters.thrust_model_factor *
      signal *
      signal +
      (
        1.0 -
        parameters.thrust_model_factor
      ) *
      signal;

    expect_near(
      "nonzero thrust-model round trip",
      output.actuator_control[rotor],
      expected_control);
  }
}


void test_infeasible_wrench_is_rejected()
{
  using namespace offboard_controllers;

  bool threw = false;

  try {
    (void)lee_px4_adapter::adapt(
      f450_parameters(),
      1.0,
      {10.0, 0.0, 0.0});

  } catch (const std::domain_error &) {
    threw = true;
  }

  if (!threw) {
    throw std::runtime_error(
            "Lee/PX4 adapter accepted an infeasible physical wrench.");
  }
}

}  // namespace


int main()
{
  test_hover_round_trip();
  test_roll_round_trip();
  test_yaw_round_trip();
  test_nonzero_thrust_model_factor_round_trip();
  test_infeasible_wrench_is_rejected();

  std::cout
    << "Lee/PX4 physical-wrench adapter tests passed.\n";

  return 0;
}
