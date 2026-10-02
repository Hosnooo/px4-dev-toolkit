#include <cmath>
#include <iostream>
#include <stdexcept>

#include <offboard_controllers/vehicle_config.hpp>

namespace
{

constexpr double kTolerance = 1.0e-12;


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
            "Vehicle configuration test failed.");
  }
}


void test_f450_lee_physical_configuration()
{
  using namespace offboard_controllers;

  const auto configuration =
    vehicle_config::load_lee_physical_configuration(
      VEHICLE_CONFIG_DIR,
      "gz_f450");

  const auto & inertia =
    configuration.inertia_frd;

  expect_near(
    "inertia xx",
    inertia.xx,
    0.021997);

  expect_near(
    "inertia xy FLU->FRD",
    inertia.xy,
    -1.0842e-19);

  expect_near(
    "inertia xz FLU->FRD",
    inertia.xz,
    -3.38813e-21);

  expect_near(
    "inertia yy",
    inertia.yy,
    0.0221599);

  expect_near(
    "inertia yz FLU->FRD",
    inertia.yz,
    3.38813e-21);

  expect_near(
    "inertia zz",
    inertia.zz,
    0.0433006);

  const auto & physical =
    configuration.adapter.physical_rotors;

  expect_near(
    "rotor 0 x",
    physical[0].position_frd_m.x,
    0.1626345596714);

  expect_near(
    "rotor 0 y FLU->FRD",
    physical[0].position_frd_m.y,
    0.1626345596714);

  expect_near(
    "rotor 0 z relative COM",
    physical[0].position_frd_m.z,
    -0.011220486);

  expect_near(
    "rotor 0 yaw ratio",
    physical[0].yaw_moment_ratio,
    0.0137);

  expect_near(
    "rotor 2 yaw ratio",
    physical[2].yaw_moment_ratio,
    -0.0137);

  const auto & allocator =
    configuration.adapter.allocator_rotors;

  expect_near(
    "allocator rotor 0 x",
    allocator[0].position_frd_m.x,
    0.159);

  expect_near(
    "allocator rotor 0 y",
    allocator[0].position_frd_m.y,
    0.159);

  expect_near(
    "allocator CT",
    allocator[0].thrust_coefficient,
    6.5);

  expect_near(
    "allocator KM",
    allocator[0].moment_ratio,
    0.014);

  expect_near(
    "motor constant",
    configuration.adapter.motor_constant,
    1.2e-05);

  expect_near(
    "ESC minimum",
    configuration.adapter.output_min_rad_s,
    150.0);

  expect_near(
    "ESC maximum",
    configuration.adapter.output_max_rad_s,
    1000.0);

  expect_near(
    "THR_MDL_FAC",
    configuration.adapter.thrust_model_factor,
    0.0);
}

}  // namespace


int main()
{
  test_f450_lee_physical_configuration();

  std::cout
    << "Vehicle configuration tests passed.\n";

  return 0;
}
