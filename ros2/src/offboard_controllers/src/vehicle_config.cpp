#include <array>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <string>

#include <yaml-cpp/yaml.h>

#include <offboard_controllers/se3/math.hpp>
#include <offboard_controllers/vehicle_config.hpp>

namespace offboard_controllers::vehicle_config
{

namespace
{

enum class BodyFrame
{
  FRD,
  FLU,
};


std::string vehicle_path(
  const std::string & config_directory,
  const std::string & vehicle)
{
  return
    config_directory +
    "/" +
    vehicle +
    ".yaml";
}


YAML::Node load_vehicle_root(
  const std::string & path)
{
  try {
    return YAML::LoadFile(path);

  } catch (const YAML::Exception & error) {
    throw std::invalid_argument(
            "Unable to load vehicle configuration '" +
            path +
            "': " +
            error.what());
  }
}


double finite_scalar(
  const YAML::Node & node,
  const std::string & name)
{
  if (!node) {
    throw std::invalid_argument(
            "Vehicle configuration does not define " +
            name +
            ".");
  }

  double value{};

  try {
    value = node.as<double>();

  } catch (const YAML::Exception & error) {
    throw std::invalid_argument(
            "Vehicle " +
            name +
            " is invalid: " +
            error.what());
  }

  if (!std::isfinite(value)) {
    throw std::invalid_argument(
            "Vehicle " +
            name +
            " must be finite.");
  }

  return value;
}


int integer_value(
  const YAML::Node & node,
  const std::string & name)
{
  if (!node) {
    throw std::invalid_argument(
            "Vehicle configuration does not define " +
            name +
            ".");
  }

  try {
    return node.as<int>();

  } catch (const YAML::Exception & error) {
    throw std::invalid_argument(
            "Vehicle " +
            name +
            " is invalid: " +
            error.what());
  }
}


std::string string_value(
  const YAML::Node & node,
  const std::string & name)
{
  if (!node) {
    throw std::invalid_argument(
            "Vehicle configuration does not define " +
            name +
            ".");
  }

  try {
    return node.as<std::string>();

  } catch (const YAML::Exception & error) {
    throw std::invalid_argument(
            "Vehicle " +
            name +
            " is invalid: " +
            error.what());
  }
}


BodyFrame parse_body_frame(
  const YAML::Node & node,
  const std::string & name)
{
  const std::string frame =
    string_value(
      node,
      name);

  if (frame == "FRD") {
    return BodyFrame::FRD;
  }

  if (frame == "FLU") {
    return BodyFrame::FLU;
  }

  throw std::invalid_argument(
          "Vehicle " +
          name +
          " must be FRD or FLU.");
}


se3::Vector3 sequence_vector3(
  const YAML::Node & node,
  const std::string & name)
{
  if (
    !node ||
    !node.IsSequence() ||
    node.size() != 3)
  {
    throw std::invalid_argument(
            "Vehicle " +
            name +
            " must contain exactly three values.");
  }

  const se3::Vector3 vector{
    finite_scalar(node[0], name + "[0]"),
    finite_scalar(node[1], name + "[1]"),
    finite_scalar(node[2], name + "[2]"),
  };

  if (!se3::is_finite(vector)) {
    throw std::invalid_argument(
            "Vehicle " +
            name +
            " must be finite.");
  }

  return vector;
}


se3::Vector3 allocator_position(
  const YAML::Node & node,
  const std::string & name)
{
  if (
    !node ||
    !node.IsSequence() ||
    (
      node.size() != 2 &&
      node.size() != 3
    ))
  {
    throw std::invalid_argument(
            "Vehicle " +
            name +
            " must contain two or three values.");
  }

  return {
    finite_scalar(node[0], name + "[0]"),
    finite_scalar(node[1], name + "[1]"),
    node.size() == 3 ?
    finite_scalar(node[2], name + "[2]") :
    0.0,
  };
}


se3::Vector3 named_vector3(
  const YAML::Node & node,
  const std::string & name)
{
  if (!node || !node.IsMap()) {
    throw std::invalid_argument(
            "Vehicle " +
            name +
            " must be an x/y/z mapping.");
  }

  return {
    finite_scalar(node["x"], name + ".x"),
    finite_scalar(node["y"], name + ".y"),
    finite_scalar(node["z"], name + ".z"),
  };
}


se3::Vector3 to_frd(
  const se3::Vector3 & vector,
  BodyFrame frame)
{
  if (frame == BodyFrame::FRD) {
    return vector;
  }

  // FLU -> FRD is a 180-degree rotation about body X.
  return {
    vector.x,
    -vector.y,
    -vector.z,
  };
}


se3::InertiaMatrix inertia_to_frd(
  const se3::InertiaMatrix & inertia,
  BodyFrame frame)
{
  if (frame == BodyFrame::FRD) {
    return inertia;
  }

  // J_FRD = C J_FLU C^T, C = diag(1, -1, -1).
  return {
    inertia.xx,
    -inertia.xy,
    -inertia.xz,
    inertia.yy,
    inertia.yz,
    inertia.zz,
  };
}


void validate_inertia(
  const se3::InertiaMatrix & inertia)
{
  if (!se3::is_finite(inertia)) {
    throw std::invalid_argument(
            "Vehicle inertia must be finite.");
  }

  const double second_minor =
    inertia.xx * inertia.yy -
    inertia.xy * inertia.xy;

  const double determinant =
    inertia.xx *
    (
      inertia.yy * inertia.zz -
      inertia.yz * inertia.yz
    ) -
    inertia.xy *
    (
      inertia.xy * inertia.zz -
      inertia.yz * inertia.xz
    ) +
    inertia.xz *
    (
      inertia.xy * inertia.yz -
      inertia.yy * inertia.xz
    );

  if (
    inertia.xx <= 0.0 ||
    second_minor <= 0.0 ||
    determinant <= 0.0)
  {
    throw std::invalid_argument(
            "Vehicle inertia must be positive definite.");
  }
}


int direction_sign(
  const YAML::Node & node,
  const std::string & name)
{
  const std::string direction =
    string_value(
      node,
      name);

  if (direction == "ccw") {
    return 1;
  }

  if (direction == "cw") {
    return -1;
  }

  throw std::invalid_argument(
          "Vehicle " +
          name +
          " must be cw or ccw.");
}

}  // namespace


LeePhysicalConfiguration load_lee_physical_configuration(
  const std::string & config_directory,
  const std::string & vehicle)
{
  const std::string path =
    vehicle_path(
      config_directory,
      vehicle);

  const YAML::Node root =
    load_vehicle_root(
      path);

  const YAML::Node plant =
    root["plant"];

  if (!plant) {
    throw std::invalid_argument(
            "Vehicle configuration '" +
            path +
            "' does not define plant.");
  }

  const BodyFrame plant_frame =
    parse_body_frame(
      plant["frame"],
      "plant.frame");

  const YAML::Node inertia_node =
    plant["inertia_kg_m2"];

  if (!inertia_node) {
    throw std::invalid_argument(
            "Vehicle configuration '" +
            path +
            "' does not define plant.inertia_kg_m2.");
  }

  const se3::InertiaMatrix inertia_native{
    finite_scalar(
      inertia_node["xx"],
      "plant.inertia_kg_m2.xx"),
    finite_scalar(
      inertia_node["xy"],
      "plant.inertia_kg_m2.xy"),
    finite_scalar(
      inertia_node["xz"],
      "plant.inertia_kg_m2.xz"),
    finite_scalar(
      inertia_node["yy"],
      "plant.inertia_kg_m2.yy"),
    finite_scalar(
      inertia_node["yz"],
      "plant.inertia_kg_m2.yz"),
    finite_scalar(
      inertia_node["zz"],
      "plant.inertia_kg_m2.zz"),
  };

  const se3::InertiaMatrix inertia_frd =
    inertia_to_frd(
      inertia_native,
      plant_frame);

  validate_inertia(
    inertia_frd);

  const se3::Vector3 center_of_mass_frd =
    to_frd(
      named_vector3(
        plant["center_of_mass_m"],
        "plant.center_of_mass_m"),
      plant_frame);

  const YAML::Node propulsion =
    root["gazebo_propulsion"];

  if (!propulsion) {
    throw std::invalid_argument(
            "Vehicle configuration '" +
            path +
            "' does not define gazebo_propulsion.");
  }

  if (
    integer_value(
      propulsion["motor_count"],
      "gazebo_propulsion.motor_count") !=
    static_cast<int>(
      lee_px4_adapter::kRotorCount))
  {
    throw std::invalid_argument(
            "Lee/PX4 adapter requires exactly four physical rotors.");
  }

  if (
    string_value(
      propulsion["motor_type"],
      "gazebo_propulsion.motor_type") !=
    "velocity")
  {
    throw std::invalid_argument(
            "Lee/PX4 adapter requires velocity-controlled Gazebo motors.");
  }

  const BodyFrame propulsion_frame =
    parse_body_frame(
      propulsion["frame"],
      "gazebo_propulsion.frame");

  const double motor_constant =
    finite_scalar(
      propulsion["motor_constant"],
      "gazebo_propulsion.motor_constant");

  const double moment_constant =
    finite_scalar(
      propulsion["moment_constant"],
      "gazebo_propulsion.moment_constant");

  const double model_max_rot_velocity =
    finite_scalar(
      propulsion["max_rot_velocity_rad_s"],
      "gazebo_propulsion.max_rot_velocity_rad_s");

  if (
    motor_constant <= 0.0 ||
    moment_constant <= 0.0 ||
    model_max_rot_velocity <= 0.0)
  {
    throw std::invalid_argument(
            "Gazebo propulsion constants must be positive.");
  }

  const YAML::Node physical_rotors =
    propulsion["rotors"];

  if (
    !physical_rotors ||
    !physical_rotors.IsSequence() ||
    physical_rotors.size() !=
    lee_px4_adapter::kRotorCount)
  {
    throw std::invalid_argument(
            "gazebo_propulsion.rotors must define exactly four rotors.");
  }

  std::array<
    lee_px4_adapter::PhysicalRotor,
    lee_px4_adapter::kRotorCount>
  physical{};

  std::array<bool, lee_px4_adapter::kRotorCount>
  physical_seen{};

  for (const YAML::Node & rotor : physical_rotors) {
    const int index =
      integer_value(
        rotor["index"],
        "gazebo_propulsion.rotors[].index");

    if (
      index < 0 ||
      index >=
      static_cast<int>(
        lee_px4_adapter::kRotorCount) ||
      physical_seen[
        static_cast<std::size_t>(index)])
    {
      throw std::invalid_argument(
              "Gazebo rotor indexes must be unique values 0..3.");
    }

    physical_seen[
      static_cast<std::size_t>(index)] = true;

    const se3::Vector3 rotor_position_frd =
      to_frd(
        sequence_vector3(
          rotor["position_m"],
          "gazebo_propulsion.rotors[].position_m"),
        propulsion_frame);

    physical[
      static_cast<std::size_t>(index)] = {
      rotor_position_frd -
      center_of_mass_frd,
      static_cast<double>(
        direction_sign(
          rotor["direction"],
          "gazebo_propulsion.rotors[].direction")) *
      moment_constant,
    };
  }

  const YAML::Node esc =
    root["px4_gz_esc"];

  if (!esc) {
    throw std::invalid_argument(
            "Vehicle configuration '" +
            path +
            "' does not define px4_gz_esc.");
  }

  const double output_min =
    finite_scalar(
      esc["min_rot_velocity_rad_s"],
      "px4_gz_esc.min_rot_velocity_rad_s");

  const double output_max =
    finite_scalar(
      esc["max_rot_velocity_rad_s"],
      "px4_gz_esc.max_rot_velocity_rad_s");

  const double thrust_model_factor =
    finite_scalar(
      esc["thrust_model_factor"],
      "px4_gz_esc.thrust_model_factor");

  if (
    output_min < 0.0 ||
    output_max <= output_min ||
    output_max > model_max_rot_velocity ||
    thrust_model_factor < 0.0 ||
    thrust_model_factor > 1.0)
  {
    throw std::invalid_argument(
            "PX4/Gazebo ESC configuration is inconsistent.");
  }

  const YAML::Node allocator =
    root["px4_allocator"];

  if (!allocator) {
    throw std::invalid_argument(
            "Vehicle configuration '" +
            path +
            "' does not define px4_allocator.");
  }

  if (
    parse_body_frame(
      allocator["frame"],
      "px4_allocator.frame") !=
    BodyFrame::FRD)
  {
    throw std::invalid_argument(
            "PX4 allocator geometry must be expressed in FRD.");
  }

  if (
    integer_value(
      allocator["airframe_type"],
      "px4_allocator.airframe_type") != 0 ||
    integer_value(
      allocator["rotor_count"],
      "px4_allocator.rotor_count") !=
    static_cast<int>(
      lee_px4_adapter::kRotorCount))
  {
    throw std::invalid_argument(
            "Lee/PX4 adapter requires the four-rotor PX4 multirotor allocator.");
  }

  const double thrust_coefficient =
    finite_scalar(
      allocator["thrust_coefficient"],
      "px4_allocator.thrust_coefficient");

  if (thrust_coefficient <= 0.0) {
    throw std::invalid_argument(
            "PX4 allocator thrust coefficient must be positive.");
  }

  const YAML::Node allocator_rotors =
    allocator["rotors"];

  if (
    !allocator_rotors ||
    !allocator_rotors.IsSequence() ||
    allocator_rotors.size() !=
    lee_px4_adapter::kRotorCount)
  {
    throw std::invalid_argument(
            "px4_allocator.rotors must define exactly four rotors.");
  }

  std::array<
    lee_px4_adapter::AllocatorRotor,
    lee_px4_adapter::kRotorCount>
  allocator_data{};

  std::array<bool, lee_px4_adapter::kRotorCount>
  allocator_seen{};

  for (const YAML::Node & rotor : allocator_rotors) {
    const int index =
      integer_value(
        rotor["index"],
        "px4_allocator.rotors[].index");

    if (
      index < 0 ||
      index >=
      static_cast<int>(
        lee_px4_adapter::kRotorCount) ||
      allocator_seen[
        static_cast<std::size_t>(index)])
    {
      throw std::invalid_argument(
              "PX4 allocator rotor indexes must be unique values 0..3.");
    }

    allocator_seen[
      static_cast<std::size_t>(index)] = true;

    const double km =
      finite_scalar(
        rotor["km"],
        "px4_allocator.rotors[].km");

    allocator_data[
      static_cast<std::size_t>(index)] = {
      allocator_position(
        rotor["position_m"],
        "px4_allocator.rotors[].position_m"),
      thrust_coefficient,
      km,
    };

    if (
      physical[
        static_cast<std::size_t>(index)].
      yaw_moment_ratio *
      km <= 0.0)
    {
      throw std::invalid_argument(
              "Gazebo and PX4 rotor yaw-moment directions disagree.");
    }
  }

  return {
    inertia_frd,
    {
      physical,
      allocator_data,
      motor_constant,
      output_min,
      output_max,
      thrust_model_factor,
    },
  };
}

}  // namespace offboard_controllers::vehicle_config
