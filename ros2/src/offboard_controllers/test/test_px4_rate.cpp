#include <array>
#include <cmath>
#include <iostream>
#include <stdexcept>

#include <offboard_controllers/px4_rate.hpp>

namespace
{

constexpr double kTolerance = 1.0e-12;
constexpr double kPi = 3.14159265358979323846;


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
            "PX4 rate-controller test failed.");
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


offboard_controllers::px4_rate::Parameters
f450_parameters(
  double yaw_cutoff_hz = 0.0)
{
  return {
    {0.15, 0.15, 0.20},
    {0.20, 0.20, 0.10},
    {0.003, 0.003, 0.0},
    {0.0, 0.0, 0.0},
    {1.0, 1.0, 1.0},
    {0.30, 0.30, 0.30},
    yaw_cutoff_hz,
  };
}


double integration_factor(
  double error)
{
  constexpr double error_scale =
    400.0 * kPi / 180.0;

  const double ratio =
    error /
    error_scale;

  return
    std::max(
      0.0,
      1.0 - ratio * ratio);
}


void test_pid_and_integrator_update_order()
{
  using namespace offboard_controllers;

  px4_rate::Controller controller(
    f450_parameters());

  const px4_rate::Output first =
    controller.update(
      1'000'000,
      {0.1, -0.2, 0.3},
      {1.0, 0.0, -0.5},
      {2.0, -4.0, 10.0},
      false);

  expect_near(
    "first dt",
    first.dt_s,
    0.02);

  expect_vector_near(
    "rate error",
    first.rate_error,
    {0.9, 0.2, -0.8});

  expect_vector_near(
    "proportional",
    first.proportional_feedback,
    {0.135, 0.03, -0.16});

  expect_vector_near(
    "initial integral contribution",
    first.integral_feedback,
    {});

  expect_vector_near(
    "derivative",
    first.derivative_feedback,
    {-0.006, 0.012, 0.0});

  expect_vector_near(
    "unfiltered torque",
    first.unfiltered_torque,
    {0.129, 0.042, -0.16});

  expect_vector_near(
    "unfiltered yaw passthrough",
    first.normalized_torque,
    {0.129, 0.042, -0.16});

  const se3::Vector3 expected_integrator{
    integration_factor(0.9) * 0.20 * 0.9 * 0.02,
    integration_factor(0.2) * 0.20 * 0.2 * 0.02,
    integration_factor(-0.8) * 0.10 * -0.8 * 0.02,
  };

  expect_vector_near(
    "updated integrator",
    first.integrator_state,
    expected_integrator);

  const px4_rate::Output second =
    controller.update(
      1'004'000,
      {0.1, -0.2, 0.3},
      {1.0, 0.0, -0.5},
      {2.0, -4.0, 10.0},
      true);

  expect_near(
    "second dt",
    second.dt_s,
    0.004);

  expect_vector_near(
    "previous integrator contributes next cycle",
    second.integral_feedback,
    expected_integrator);

  expect_vector_near(
    "landed leaves integrator unchanged",
    second.integrator_state,
    expected_integrator);
}


void test_inactive_sample_advances_wrapper_timing()
{
  using namespace offboard_controllers;

  px4_rate::Controller controller(
    f450_parameters());

  controller.observe_timestamp_sample(
    1'000'000);

  const px4_rate::Output output =
    controller.update(
      1'004'000,
      {},
      {},
      {},
      true);

  expect_near(
    "dt after inactive gyro sample",
    output.dt_s,
    0.004);
}


void test_dt_limits_match_px4_wrapper()
{
  using namespace offboard_controllers;

  px4_rate::Controller controller(
    f450_parameters());

  expect_near(
    "initial large dt clamp",
    controller.update(
      1'000'000,
      {},
      {},
      {},
      true).dt_s,
    0.02);

  expect_near(
    "small dt clamp",
    controller.update(
      1'000'100,
      {},
      {},
      {},
      true).dt_s,
    0.000125);

  expect_near(
    "normal sample dt",
    controller.update(
      1'004'100,
      {},
      {},
      {},
      true).dt_s,
    0.004);
}


void test_allocator_saturation_blocks_matching_integration()
{
  using namespace offboard_controllers;

  px4_rate::Parameters parameters{
    {},
    {1.0, 1.0, 1.0},
    {},
    {},
    {1.0, 1.0, 1.0},
    {1.0, 1.0, 1.0},
    0.0,
  };

  px4_rate::Controller controller(
    parameters);

  controller.set_saturation_status(
    {true, false, false},
    {false, true, false});

  const px4_rate::Output output =
    controller.update(
      1'000'000,
      {},
      {1.0, -1.0, 1.0},
      {},
      false);

  expect_vector_near(
    "saturation-aware integrator",
    output.integrator_state,
    {
      0.0,
      0.0,
      integration_factor(1.0) * 0.02,
    });
}


void test_integrator_limit_and_reset()
{
  using namespace offboard_controllers;

  px4_rate::Parameters parameters{
    {},
    {100.0, 0.0, 0.0},
    {},
    {},
    {1.0, 1.0, 1.0},
    {0.30, 0.30, 0.30},
    0.0,
  };

  px4_rate::Controller controller(
    parameters);

  const px4_rate::Output output =
    controller.update(
      1'000'000,
      {},
      {1.0, 0.0, 0.0},
      {},
      false);

  expect_vector_near(
    "integrator limit",
    output.integrator_state,
    {0.30, 0.0, 0.0});

  controller.reset_integral();

  const px4_rate::Output reset =
    controller.update(
      1'004'000,
      {},
      {},
      {},
      true);

  expect_vector_near(
    "integrator reset",
    reset.integral_feedback,
    {});
}


void test_yaw_alpha_filter_matches_px4()
{
  using namespace offboard_controllers;

  px4_rate::Parameters parameters{
    {0.0, 0.0, 1.0},
    {},
    {},
    {},
    {1.0, 1.0, 1.0},
    {0.30, 0.30, 0.30},
    2.0,
  };

  px4_rate::Controller controller(
    parameters);

  const double time_constant =
    1.0 /
    (
      2.0 *
      kPi *
      2.0
    );

  const double first_dt = 0.02;
  const double first_alpha =
    first_dt /
    (
      time_constant +
      first_dt
    );

  const px4_rate::Output first =
    controller.update(
      1'000'000,
      {},
      {0.0, 0.0, 1.0},
      {},
      true);

  expect_near(
    "first raw yaw torque",
    first.unfiltered_torque.z,
    1.0);

  expect_near(
    "first filtered yaw torque",
    first.normalized_torque.z,
    first_alpha);

  const double second_dt = 0.004;
  const double second_alpha =
    second_dt /
    (
      time_constant +
      second_dt
    );

  const double expected_second =
    first_alpha +
    second_alpha *
    (
      1.0 -
      first_alpha
    );

  const px4_rate::Output second =
    controller.update(
      1'004'000,
      {},
      {0.0, 0.0, 1.0},
      {},
      true);

  expect_near(
    "second filtered yaw torque",
    second.normalized_torque.z,
    expected_second);
}

}  // namespace


int main()
{
  try {
    test_pid_and_integrator_update_order();
    test_inactive_sample_advances_wrapper_timing();
    test_dt_limits_match_px4_wrapper();
    test_allocator_saturation_blocks_matching_integration();
    test_integrator_limit_and_reset();
    test_yaw_alpha_filter_matches_px4();
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    return 1;
  }

  std::cout
    << "PX4 rate-controller tests passed.\n";

  return 0;
}
