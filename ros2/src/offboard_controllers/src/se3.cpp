/*
 * PX4 Offboard runtime for the geometric SE(3) controller.
 *
 * The controller mathematics remain independent of ROS 2 and PX4. This node
 * owns vehicle/config loading, trajectory timing, PX4 Offboard lifecycle, and
 * selection of the PX4 handoff level.
 */

#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_angular_velocity.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_attitude_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_command_ack.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_rates_setpoint.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_thrust_setpoint.hpp>
#include <px4_msgs/msg/vehicle_torque_setpoint.hpp>
#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>

#include <offboard_controllers/offboard_lifecycle.hpp>
#include <offboard_controllers/px4_wrench.hpp>
#include <offboard_controllers/runtime_timing.hpp>
#include <offboard_controllers/se3/controller.hpp>
#include <offboard_controllers/se3/math.hpp>
#include <offboard_controllers/trajectory.hpp>

namespace px4_topics
{

#define PX4_TOPIC(name, topic) constexpr char name[] = topic;
#include "px4_topics.def"
#undef PX4_TOPIC

}  // namespace px4_topics


namespace offboard_controllers
{

namespace
{

constexpr char kTrajectoryReferenceTopic[] =
  "/px4_toolkit/se3/trajectory_reference";

constexpr char kDirectRateCommandTopic[] =
  "/px4_toolkit/se3/rate_command";

constexpr char kTorqueRateFeedbackTopic[] =
  "/px4_toolkit/se3/torque_rate_feedback";

constexpr char kTorqueAngularAccelerationFeedforwardTopic[] =
  "/px4_toolkit/se3/torque_angular_acceleration_feedforward";


double load_vehicle_mass(
  const std::string & config_directory,
  const std::string & vehicle)
{
  const std::string path =
    config_directory + "/" + vehicle + ".yaml";

  YAML::Node root;

  try {
    root = YAML::LoadFile(path);
  } catch (const YAML::Exception & error) {
    throw std::invalid_argument(
            "Unable to load vehicle configuration '" +
            path +
            "': " +
            error.what());
  }

  const YAML::Node mass_node =
    root["plant"]["mass_kg"];

  if (!mass_node) {
    throw std::invalid_argument(
            "Vehicle configuration '" +
            path +
            "' does not define plant.mass_kg.");
  }

  const double mass =
    mass_node.as<double>();

  if (!std::isfinite(mass) || mass <= 0.0) {
    throw std::invalid_argument(
            "Vehicle plant.mass_kg must be finite and positive.");
  }

  return mass;
}


double load_vehicle_hover_thrust(
  const std::string & config_directory,
  const std::string & vehicle)
{
  const std::string path =
    config_directory + "/" + vehicle + ".yaml";

  YAML::Node root;

  try {
    root = YAML::LoadFile(path);
  } catch (const YAML::Exception & error) {
    throw std::invalid_argument(
            "Unable to load vehicle configuration '" +
            path +
            "': " +
            error.what());
  }

  const YAML::Node hover_thrust_node =
    root["px4_allocator"]["hover_thrust"];

  if (!hover_thrust_node) {
    throw std::invalid_argument(
            "Vehicle configuration '" +
            path +
            "' does not define px4_allocator.hover_thrust; "
            "attitude, attitude_rate, and thrust_and_torque handoffs "
            "require a validated hover-thrust value.");
  }

  const double hover_thrust =
    hover_thrust_node.as<double>();

  if (
    !std::isfinite(hover_thrust) ||
    hover_thrust <= 0.0)
  {
    throw std::invalid_argument(
            "Vehicle px4_allocator.hover_thrust must be "
            "finite and positive.");
  }

  return hover_thrust;
}


se3::Vector3 vector3_parameter(
  const std::vector<double> & values,
  const std::string & name)
{
  if (values.size() != 3) {
    throw std::invalid_argument(
            "SE3 " + name + " must contain exactly three values.");
  }

  const se3::Vector3 result{
    values[0],
    values[1],
    values[2],
  };

  if (!se3::is_finite(result)) {
    throw std::invalid_argument(
            "SE3 " + name + " must contain finite values.");
  }

  return result;
}


se3::Reference to_se3_reference(
  const trajectory::Reference & reference)
{
  return {
    {
      reference.position[0],
      reference.position[1],
      reference.position[2],
    },
    {
      reference.velocity[0],
      reference.velocity[1],
      reference.velocity[2],
    },
    {
      reference.acceleration[0],
      reference.acceleration[1],
      reference.acceleration[2],
    },
    {
      reference.jerk[0],
      reference.jerk[1],
      reference.jerk[2],
    },
    {
      reference.snap[0],
      reference.snap[1],
      reference.snap[2],
    },
    reference.yaw,
    reference.yaw_rate,
    reference.yaw_acceleration,
  };
}

}  // namespace


class Se3 final : public rclcpp::Node
{
public:
  Se3()
  : Node("se3")
  {
    vehicle_ =
      declare_parameter<std::string>(
        "vehicle",
        "");

    trajectory_name_ =
      declare_parameter<std::string>(
        "trajectory",
        "");

    handoff_ =
      declare_parameter<std::string>(
        "handoff",
        "");

    const std::string vehicle_config_dir =
      declare_parameter<std::string>(
        "vehicle_config_dir",
        "");

    const std::string trajectory_config =
      declare_parameter<std::string>(
        "trajectory_config",
        "");

    const double kx_over_mass =
      declare_parameter<double>(
        "kx_over_mass",
        std::numeric_limits<double>::quiet_NaN());

    const double kv_over_mass =
      declare_parameter<double>(
        "kv_over_mass",
        std::numeric_limits<double>::quiet_NaN());

    const se3::Vector3 attitude_gain =
      vector3_parameter(
        declare_parameter<std::vector<double>>(
          "attitude_gain",
          std::vector<double>{}),
        "attitude_gain");

    const se3::Vector3 normalized_rate_gain =
      vector3_parameter(
        declare_parameter<std::vector<double>>(
          "normalized_rate_gain",
          std::vector<double>{}),
        "normalized_rate_gain");

    const se3::Vector3 normalized_angular_acceleration_gain =
      vector3_parameter(
        declare_parameter<std::vector<double>>(
          "normalized_angular_acceleration_gain",
          std::vector<double>{}),
        "normalized_angular_acceleration_gain");

    const double control_rate_hz =
      declare_parameter<double>(
        "control_rate_hz",
        std::numeric_limits<double>::quiet_NaN());

    const double warmup_duration_s =
      declare_parameter<double>(
        "warmup_duration_s",
        std::numeric_limits<double>::quiet_NaN());

    const double offboard_retry_interval_s =
      declare_parameter<double>(
        "offboard_retry_interval_s",
        std::numeric_limits<double>::quiet_NaN());

    runtime_timing_ =
      make_runtime_timing(
        control_rate_hz,
        warmup_duration_s,
        offboard_retry_interval_s);

    if (vehicle_.empty()) {
      throw std::invalid_argument(
              "SE3 vehicle parameter must be specified.");
    }

    if (trajectory_name_.empty()) {
      throw std::invalid_argument(
              "SE3 trajectory parameter must be specified.");
    }

    if (vehicle_config_dir.empty()) {
      throw std::invalid_argument(
              "SE3 vehicle_config_dir parameter must be specified.");
    }

    if (trajectory_config.empty()) {
      throw std::invalid_argument(
              "SE3 trajectory_config parameter must be specified.");
    }

    if (
      handoff_ != "acceleration" &&
      handoff_ != "attitude" &&
      handoff_ != "attitude_rate" &&
      handoff_ != "thrust_and_torque")
    {
      throw std::invalid_argument(
              "SE3 handoff '" +
              handoff_ +
              "' is not implemented. Available handoffs: "
              "acceleration, attitude, attitude_rate, thrust_and_torque.");
    }

    if (
      !std::isfinite(kx_over_mass) ||
      kx_over_mass <= 0.0)
    {
      throw std::invalid_argument(
              "SE3 kx_over_mass must be finite and positive.");
    }

    if (
      !std::isfinite(kv_over_mass) ||
      kv_over_mass <= 0.0)
    {
      throw std::invalid_argument(
              "SE3 kv_over_mass must be finite and positive.");
    }

    const double mass =
      load_vehicle_mass(
        vehicle_config_dir,
        vehicle_);

    mass_ = mass;

    if (handoff_ != "acceleration") {
      hover_thrust_ =
        load_vehicle_hover_thrust(
          vehicle_config_dir,
          vehicle_);
    }

    configured_trajectory_ =
      std::make_unique<trajectory::ConfiguredTrajectory>(
        trajectory::load_trajectory(
          trajectory_config,
          trajectory_name_));

    normalized_rate_gain_ =
      normalized_rate_gain;

    controller_ =
      std::make_unique<se3::Controller>(
        se3::Parameters{
          mass,
          mass * kx_over_mass,
          mass * kv_over_mass,
          attitude_gain,
          normalized_rate_gain,
          normalized_angular_acceleration_gain,
        });

    offboard_control_mode_pub_ =
      create_publisher<px4_msgs::msg::OffboardControlMode>(
        px4_topics::IN_OFFBOARD_CONTROL_MODE,
        10);

    trajectory_setpoint_pub_ =
      create_publisher<px4_msgs::msg::TrajectorySetpoint>(
        px4_topics::IN_TRAJECTORY_SETPOINT,
        10);

    vehicle_attitude_setpoint_pub_ =
      create_publisher<px4_msgs::msg::VehicleAttitudeSetpoint>(
        px4_topics::IN_VEHICLE_ATTITUDE_SETPOINT,
        10);

    vehicle_rates_setpoint_pub_ =
      create_publisher<px4_msgs::msg::VehicleRatesSetpoint>(
        px4_topics::IN_VEHICLE_RATES_SETPOINT,
        10);

    vehicle_thrust_setpoint_pub_ =
      create_publisher<px4_msgs::msg::VehicleThrustSetpoint>(
        px4_topics::IN_VEHICLE_THRUST_SETPOINT,
        10);

    vehicle_torque_setpoint_pub_ =
      create_publisher<px4_msgs::msg::VehicleTorqueSetpoint>(
        px4_topics::IN_VEHICLE_TORQUE_SETPOINT,
        10);

    trajectory_reference_pub_ =
      create_publisher<px4_msgs::msg::TrajectorySetpoint>(
        kTrajectoryReferenceTopic,
        10);

    direct_rate_command_pub_ =
      create_publisher<px4_msgs::msg::VehicleRatesSetpoint>(
        kDirectRateCommandTopic,
        10);

    torque_rate_feedback_pub_ =
      create_publisher<px4_msgs::msg::VehicleTorqueSetpoint>(
        kTorqueRateFeedbackTopic,
        10);

    torque_angular_acceleration_feedforward_pub_ =
      create_publisher<px4_msgs::msg::VehicleTorqueSetpoint>(
        kTorqueAngularAccelerationFeedforwardTopic,
        10);

    vehicle_command_pub_ =
      create_publisher<px4_msgs::msg::VehicleCommand>(
        px4_topics::IN_VEHICLE_COMMAND,
        10);

    local_position_sub_ =
      create_subscription<px4_msgs::msg::VehicleLocalPosition>(
        px4_topics::OUT_VEHICLE_LOCAL_POSITION,
        rclcpp::SensorDataQoS(),
        [this](
          const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
        {
          update_state(*msg);
        });

    vehicle_attitude_sub_ =
      create_subscription<px4_msgs::msg::VehicleAttitude>(
        px4_topics::OUT_VEHICLE_ATTITUDE,
        rclcpp::SensorDataQoS(),
        [this](
          const px4_msgs::msg::VehicleAttitude::SharedPtr msg)
        {
          update_attitude(*msg);
        });

    angular_velocity_sub_ =
      create_subscription<px4_msgs::msg::VehicleAngularVelocity>(
        px4_topics::OUT_VEHICLE_ANGULAR_VELOCITY,
        rclcpp::SensorDataQoS(),
        [this](
          const px4_msgs::msg::VehicleAngularVelocity::SharedPtr msg)
        {
          update_angular_velocity(*msg);
        });

    vehicle_status_sub_ =
      create_subscription<px4_msgs::msg::VehicleStatus>(
        px4_topics::OUT_VEHICLE_STATUS_V1,
        rclcpp::SensorDataQoS(),
        [this](
          const px4_msgs::msg::VehicleStatus::SharedPtr msg)
        {
          update_vehicle_status(*msg);
        });

    command_ack_sub_ =
      create_subscription<px4_msgs::msg::VehicleCommandAck>(
        px4_topics::OUT_VEHICLE_COMMAND_ACK,
        rclcpp::SensorDataQoS(),
        [this](
          const px4_msgs::msg::VehicleCommandAck::SharedPtr msg)
        {
          handle_command_ack(*msg);
        });

    timer_ =
      create_wall_timer(
        runtime_timing_.control_period,
        [this]()
        {
          update();
        });

    RCLCPP_INFO(
      get_logger(),
      "SE3 ready: vehicle=%s, trajectory=%s, "
      "handoff=%s, mass=%.6f kg, "
      "kx/m=%.6f, kv/m=%.6f",
      vehicle_.c_str(),
      trajectory_name_.c_str(),
      handoff_.c_str(),
      mass,
      kx_over_mass,
      kv_over_mass);
  }

private:
  static uint64_t timestamp_us(
    const rclcpp::Node & node)
  {
    return static_cast<uint64_t>(
      node.get_clock()->now().nanoseconds() / 1000);
  }


  void update_state(
    const px4_msgs::msg::VehicleLocalPosition & msg)
  {
    const bool translational_state_valid =
      msg.xy_valid &&
      msg.z_valid &&
      msg.v_xy_valid &&
      msg.v_z_valid &&
      std::isfinite(msg.x) &&
      std::isfinite(msg.y) &&
      std::isfinite(msg.z) &&
      std::isfinite(msg.vx) &&
      std::isfinite(msg.vy) &&
      std::isfinite(msg.vz);

    if (!translational_state_valid) {
      state_valid_ = false;
      return;
    }

    state_.position = {
      static_cast<double>(msg.x),
      static_cast<double>(msg.y),
      static_cast<double>(msg.z),
    };

    state_.velocity = {
      static_cast<double>(msg.vx),
      static_cast<double>(msg.vy),
      static_cast<double>(msg.vz),
    };

    // Desired-attitude derivatives are model-derived in se3::Controller.
    // Do not finite-difference VehicleLocalPosition acceleration here: doing
    // so would inject differentiation noise and estimator delay into
    // Omega_d_dot.
    state_valid_ = true;
  }


  void update_attitude(
    const px4_msgs::msg::VehicleAttitude & msg)
  {
    try {
      state_.attitude =
        px4_wrench::rotation_from_quaternion(std::array<double, 4>{
          static_cast<double>(msg.q[0]),
          static_cast<double>(msg.q[1]),
          static_cast<double>(msg.q[2]),
          static_cast<double>(msg.q[3]),
        });

      attitude_valid_ = true;
    } catch (const std::invalid_argument &) {
      attitude_valid_ = false;
    }
  }


  void update_angular_velocity(
    const px4_msgs::msg::VehicleAngularVelocity & msg)
  {
    const se3::Vector3 angular_velocity{
      static_cast<double>(msg.xyz[0]),
      static_cast<double>(msg.xyz[1]),
      static_cast<double>(msg.xyz[2]),
    };

    if (!se3::is_finite(angular_velocity)) {
      angular_velocity_valid_ = false;
      return;
    }

    state_.angular_velocity = angular_velocity;
    angular_velocity_valid_ = true;
  }


  bool controller_state_ready() const
  {
    if (!state_valid_) {
      return false;
    }

    if (handoff_ == "acceleration") {
      return true;
    }

    if (!attitude_valid_) {
      return false;
    }

    // A_dot is obtained from the commanded collective thrust and current
    // attitude, so the attitude and attitude-rate handoffs do not require a
    // measured acceleration. A_ddot additionally requires current body rate.
    if (
      handoff_ == "attitude" ||
      handoff_ == "attitude_rate")
    {
      return true;
    }

    return angular_velocity_valid_;
  }


  void update_vehicle_status(
    const px4_msgs::msg::VehicleStatus & msg)
  {
    position_mode_active_ =
      msg.nav_state ==
      px4_msgs::msg::VehicleStatus::
      NAVIGATION_STATE_POSCTL;

    const bool offboard =
      msg.nav_state ==
      px4_msgs::msg::VehicleStatus::
      NAVIGATION_STATE_OFFBOARD;

    const OffboardTransition transition =
      offboard_transition(
        offboard_active_,
        offboard);

    if (transition == OffboardTransition::ENTERED) {
      if (!controller_state_ready()) {
        RCLCPP_ERROR(
          get_logger(),
          "PX4 entered Offboard before the state required by "
          "handoff '%s' was available.",
          handoff_.c_str());

        offboard_active_ = false;
        return;
      }

      offboard_active_ = true;
      return_to_position_requested_ = false;

      trajectory_origin_ =
        trajectory::stationary_reference(
          {
            state_.position.x,
            state_.position.y,
            state_.position.z,
          },
          configured_trajectory_->yaw);

      trajectory_start_time_ =
        get_clock()->now();

      RCLCPP_INFO(
        get_logger(),
        "PX4 Offboard active; starting trajectory '%s'.",
        trajectory_name_.c_str());

      return;
    }

    if (transition == OffboardTransition::LOST) {
      offboard_active_ = false;

      if (return_to_position_requested_) {
        RCLCPP_INFO(
          get_logger(),
          "PX4 left Offboard after the Position-mode return request; "
          "waiting for Position-mode confirmation.");
      } else {
        RCLCPP_WARN(
          get_logger(),
          "PX4 left Offboard before trajectory completion; "
          "the next Offboard entry will restart and re-anchor "
          "the trajectory.");
      }

      return;
    }

    offboard_active_ = offboard;
  }


  void handle_command_ack(
    const px4_msgs::msg::VehicleCommandAck & msg)
  {
    using VehicleCommand =
      px4_msgs::msg::VehicleCommand;

    using VehicleCommandAck =
      px4_msgs::msg::VehicleCommandAck;

    if (
      msg.command !=
      VehicleCommand::VEHICLE_CMD_DO_SET_MODE)
    {
      return;
    }

    if (
      msg.result ==
      VehicleCommandAck::VEHICLE_CMD_RESULT_ACCEPTED)
    {
      RCLCPP_INFO(
        get_logger(),
        "PX4 accepted mode-change command.");

      return;
    }

    RCLCPP_WARN(
      get_logger(),
      "PX4 did not accept mode-change command: result=%u",
      static_cast<unsigned>(msg.result));
  }


  void publish_offboard_control_mode()
  {
    px4_msgs::msg::OffboardControlMode msg{};

    msg.timestamp =
      timestamp_us(*this);

    msg.position = false;
    msg.velocity = false;
    msg.acceleration =
      handoff_ == "acceleration";
    msg.attitude =
      handoff_ == "attitude";
    msg.body_rate =
      handoff_ == "attitude_rate";
    msg.thrust_and_torque =
      handoff_ == "thrust_and_torque";
    msg.direct_actuator = false;

    offboard_control_mode_pub_->publish(msg);
  }


  void publish_trajectory_reference(
    const trajectory::Reference & reference)
  {
    px4_msgs::msg::TrajectorySetpoint msg{};

    msg.timestamp =
      timestamp_us(*this);

    msg.position = {
      static_cast<float>(reference.position[0]),
      static_cast<float>(reference.position[1]),
      static_cast<float>(reference.position[2]),
    };

    msg.velocity = {
      static_cast<float>(reference.velocity[0]),
      static_cast<float>(reference.velocity[1]),
      static_cast<float>(reference.velocity[2]),
    };

    msg.acceleration = {
      static_cast<float>(reference.acceleration[0]),
      static_cast<float>(reference.acceleration[1]),
      static_cast<float>(reference.acceleration[2]),
    };

    msg.jerk = {
      static_cast<float>(reference.jerk[0]),
      static_cast<float>(reference.jerk[1]),
      static_cast<float>(reference.jerk[2]),
    };

    msg.yaw =
      static_cast<float>(reference.yaw);

    msg.yawspeed =
      static_cast<float>(reference.yaw_rate);

    trajectory_reference_pub_->publish(msg);
  }


  void publish_acceleration_setpoint(
    const se3::Vector3 & acceleration,
    double yaw)
  {
    const float nan =
      std::numeric_limits<float>::quiet_NaN();

    px4_msgs::msg::TrajectorySetpoint msg{};

    msg.timestamp =
      timestamp_us(*this);

    msg.position = {
      nan,
      nan,
      nan,
    };

    msg.velocity = {
      nan,
      nan,
      nan,
    };

    msg.acceleration = {
      static_cast<float>(acceleration.x),
      static_cast<float>(acceleration.y),
      static_cast<float>(acceleration.z),
    };

    msg.jerk = {
      nan,
      nan,
      nan,
    };

    msg.yaw =
      static_cast<float>(yaw);

    msg.yawspeed = nan;

    trajectory_setpoint_pub_->publish(msg);
  }


  double projected_collective_thrust(
    const se3::Vector3 & force) const
  {
    return px4_wrench::normalized_projected_collective_thrust(
      force,
      state_.attitude,
      mass_,
      hover_thrust_);
  }


  void publish_attitude_setpoint(
    const se3::RotationMatrix & attitude,
    const se3::Vector3 & force,
    double yaw_rate)
  {
    const auto quaternion =
      px4_wrench::quaternion_from_rotation(
        attitude);

    px4_msgs::msg::VehicleAttitudeSetpoint msg{};

    msg.timestamp =
      timestamp_us(*this);

    msg.q_d = {
      static_cast<float>(quaternion[0]),
      static_cast<float>(quaternion[1]),
      static_cast<float>(quaternion[2]),
      static_cast<float>(quaternion[3]),
    };

    msg.yaw_sp_move_rate =
      static_cast<float>(yaw_rate);

    msg.thrust_body = {
      0.0F,
      0.0F,
      -static_cast<float>(projected_collective_thrust(force)),
    };

    vehicle_attitude_setpoint_pub_->publish(msg);
  }


  void publish_rate_setpoint(
    const se3::Vector3 & angular_velocity,
    const se3::Vector3 & force)
  {
    px4_msgs::msg::VehicleRatesSetpoint msg{};

    msg.timestamp =
      timestamp_us(*this);

    msg.roll =
      static_cast<float>(angular_velocity.x);
    msg.pitch =
      static_cast<float>(angular_velocity.y);
    msg.yaw =
      static_cast<float>(angular_velocity.z);

    msg.thrust_body = {
      0.0F,
      0.0F,
      -static_cast<float>(projected_collective_thrust(force)),
    };

    msg.reset_integral = false;

    vehicle_rates_setpoint_pub_->publish(msg);
  }


  void publish_thrust_and_torque_setpoint(
    const se3::Vector3 & force,
    const se3::Vector3 & normalized_torque)
  {
    const uint64_t timestamp =
      timestamp_us(*this);

    px4_msgs::msg::VehicleThrustSetpoint thrust{};
    thrust.timestamp = timestamp;
    thrust.xyz = {
      0.0F,
      0.0F,
      -static_cast<float>(projected_collective_thrust(force)),
    };

    px4_msgs::msg::VehicleTorqueSetpoint torque{};
    torque.timestamp = timestamp;
    torque.xyz = {
      static_cast<float>(normalized_torque.x),
      static_cast<float>(normalized_torque.y),
      static_cast<float>(normalized_torque.z),
    };

    vehicle_thrust_setpoint_pub_->publish(thrust);
    vehicle_torque_setpoint_pub_->publish(torque);
  }


  void publish_direct_torque_diagnostics(
    const se3::Vector3 & angular_velocity_command,
    const se3::Vector3 & rate_feedback,
    const se3::Vector3 & angular_acceleration_feedforward)
  {
    const uint64_t timestamp =
      timestamp_us(*this);

    px4_msgs::msg::VehicleRatesSetpoint rate{};
    rate.timestamp = timestamp;
    rate.roll =
      static_cast<float>(
        angular_velocity_command.x);
    rate.pitch =
      static_cast<float>(
        angular_velocity_command.y);
    rate.yaw =
      static_cast<float>(
        angular_velocity_command.z);
    rate.reset_integral = false;

    px4_msgs::msg::VehicleTorqueSetpoint feedback{};
    feedback.timestamp = timestamp;
    feedback.xyz = {
      static_cast<float>(rate_feedback.x),
      static_cast<float>(rate_feedback.y),
      static_cast<float>(rate_feedback.z),
    };

    px4_msgs::msg::VehicleTorqueSetpoint feedforward{};
    feedforward.timestamp = timestamp;
    feedforward.xyz = {
      static_cast<float>(
        angular_acceleration_feedforward.x),
      static_cast<float>(
        angular_acceleration_feedforward.y),
      static_cast<float>(
        angular_acceleration_feedforward.z),
    };

    direct_rate_command_pub_->publish(rate);
    torque_rate_feedback_pub_->publish(feedback);
    torque_angular_acceleration_feedforward_pub_->publish(
      feedforward);
  }


  void request_offboard_mode()
  {
    px4_msgs::msg::VehicleCommand msg{};

    msg.timestamp =
      timestamp_us(*this);

    // MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
    msg.param1 = 1.0F;

    // PX4 custom main mode 6 = Offboard.
    msg.param2 = 6.0F;

    msg.command =
      px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE;

    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;

    vehicle_command_pub_->publish(msg);

    RCLCPP_INFO(
      get_logger(),
      "Requesting PX4 Offboard mode.");
  }


  void request_position_mode()
  {
    px4_msgs::msg::VehicleCommand msg{};

    msg.timestamp =
      timestamp_us(*this);

    // MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
    msg.param1 = 1.0F;

    // PX4 custom main mode 3 = Position.
    msg.param2 = 3.0F;

    msg.command =
      px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE;

    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;

    vehicle_command_pub_->publish(msg);

    return_to_position_requested_ = true;
    position_return_last_request_at_ =
      get_clock()->now();

    RCLCPP_INFO(
      get_logger(),
      "Trajectory complete; requesting PX4 Position mode.");
  }


  trajectory::Reference current_trajectory_reference()
  {
    // During pre-streaming, continuously anchor t=0 to the current vehicle
    // position. The actual trajectory therefore does not advance before PX4
    // has transferred control to Offboard.
    if (!offboard_active_) {
      trajectory_origin_ =
        trajectory::stationary_reference(
          {
            state_.position.x,
            state_.position.y,
            state_.position.z,
          },
          configured_trajectory_->yaw);

      return configured_trajectory_->sequence.sample(
        0.0,
        trajectory_origin_);
    }

    const double elapsed_s =
      (get_clock()->now() - trajectory_start_time_).seconds();

    return configured_trajectory_->sequence.sample(
      elapsed_s,
      trajectory_origin_);
  }


  void publish_controller_handoff(
    const se3::Reference & reference,
    const se3::TranslationalOutput & output)
  {
    if (handoff_ == "acceleration") {
      publish_acceleration_setpoint(
        output.acceleration,
        reference.yaw);
      return;
    }

    if (handoff_ == "attitude") {
      publish_attitude_setpoint(
        controller_->compute_desired_attitude(
          output.force_vector,
          reference.yaw),
        output.force_vector,
        reference.yaw_rate);
      return;
    }

    const se3::Vector3 force_derivative =
      controller_->compute_force_derivative(
        state_,
        reference,
        output.force_vector);

    if (handoff_ == "attitude_rate") {
      const se3::DesiredAttitudeRate desired =
        controller_->compute_desired_attitude_rate(
          output.force_vector,
          force_derivative,
          reference.yaw,
          reference.yaw_rate);

      publish_rate_setpoint(
        controller_->compute_attitude_rate_command(
          state_.attitude,
          desired),
        output.force_vector);
      return;
    }

    const se3::Vector3 force_second_derivative =
      controller_->compute_force_second_derivative(
        state_,
        reference,
        output.force_vector,
        force_derivative);

    const se3::DesiredAttitudeDynamics desired =
      controller_->compute_desired_attitude_dynamics(
        output.force_vector,
        force_derivative,
        force_second_derivative,
        reference.yaw,
        reference.yaw_rate,
        reference.yaw_acceleration);

    const se3::DesiredAttitudeRate desired_rate{
      desired.attitude,
      desired.angular_velocity,
    };

    const se3::Vector3 angular_velocity_command =
      controller_->compute_attitude_rate_command(
        state_.attitude,
        desired_rate);

    const se3::Vector3 rate_error =
      angular_velocity_command -
      state_.angular_velocity;

    const se3::Vector3 rate_feedback =
      se3::component_product(
        normalized_rate_gain_,
        rate_error);

    const se3::Vector3 normalized_torque =
      controller_->compute_normalized_torque_command(
        state_.attitude,
        state_.angular_velocity,
        desired);

    const se3::Vector3
      angular_acceleration_feedforward =
      normalized_torque -
      rate_feedback;

    publish_direct_torque_diagnostics(
      angular_velocity_command,
      rate_feedback,
      angular_acceleration_feedforward);

    publish_thrust_and_torque_setpoint(
      output.force_vector,
      normalized_torque);
  }


  void update()
  {
    publish_offboard_control_mode();

    if (!controller_state_ready()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Waiting for state required by SE3 handoff '%s'.",
        handoff_.c_str());

      return;
    }

    const trajectory::Reference trajectory_reference =
      current_trajectory_reference();

    publish_trajectory_reference(
      trajectory_reference);

    if (!offboard_active_) {
      if (return_to_position_requested_) {
        if (position_mode_active_) {
          RCLCPP_INFO(
            get_logger(),
            "PX4 returned to Position mode; SE3 experiment complete.");

          rclcpp::shutdown();
        }

        return;
      }

      ++valid_setpoint_count_;

      if (
        valid_setpoint_count_ >= runtime_timing_.warmup_samples &&
        (
          valid_setpoint_count_ == runtime_timing_.warmup_samples ||
          (
            valid_setpoint_count_ -
            runtime_timing_.warmup_samples
          ) % runtime_timing_.retry_samples == 0
        ))
      {
        request_offboard_mode();
      }

      return;
    }

    const se3::Reference controller_reference =
      to_se3_reference(
        trajectory_reference);

    const se3::TranslationalOutput output =
      controller_->compute_translation(
        state_,
        controller_reference);

    publish_controller_handoff(
      controller_reference,
      output);

    const double trajectory_elapsed_s =
      (get_clock()->now() - trajectory_start_time_).seconds();

    if (
      !return_to_position_requested_ &&
      trajectory_elapsed_s >=
      configured_trajectory_->sequence.duration_s())
    {
      request_position_mode();

    } else if (
      return_to_position_requested_ &&
      (
        get_clock()->now() -
        position_return_last_request_at_
      ).seconds() >= 1.0)
    {
      request_position_mode();
    }
  }


  std::unique_ptr<se3::Controller> controller_;

  std::unique_ptr<trajectory::ConfiguredTrajectory>
    configured_trajectory_;

  se3::State state_{};

  trajectory::Reference trajectory_origin_{};

  bool state_valid_{false};
  bool attitude_valid_{false};
  bool angular_velocity_valid_{false};
  bool offboard_active_{false};
  bool position_mode_active_{false};
  bool return_to_position_requested_{false};
  rclcpp::Time position_return_last_request_at_{0, 0, RCL_ROS_TIME};

  double mass_{
    std::numeric_limits<double>::quiet_NaN()};

  double hover_thrust_{
    std::numeric_limits<double>::quiet_NaN()};

  se3::Vector3 normalized_rate_gain_{};

  RuntimeTiming runtime_timing_{};
  int valid_setpoint_count_{0};

  std::string vehicle_;
  std::string trajectory_name_;
  std::string handoff_;

  rclcpp::Time trajectory_start_time_{0, 0, RCL_ROS_TIME};

  rclcpp::Publisher<
    px4_msgs::msg::OffboardControlMode>::SharedPtr
    offboard_control_mode_pub_;

  rclcpp::Publisher<
    px4_msgs::msg::TrajectorySetpoint>::SharedPtr
    trajectory_setpoint_pub_;

  rclcpp::Publisher<
    px4_msgs::msg::TrajectorySetpoint>::SharedPtr
    trajectory_reference_pub_;

  rclcpp::Publisher<
    px4_msgs::msg::VehicleAttitudeSetpoint>::SharedPtr
    vehicle_attitude_setpoint_pub_;

  rclcpp::Publisher<
    px4_msgs::msg::VehicleRatesSetpoint>::SharedPtr
    vehicle_rates_setpoint_pub_;

  rclcpp::Publisher<
    px4_msgs::msg::VehicleThrustSetpoint>::SharedPtr
    vehicle_thrust_setpoint_pub_;

  rclcpp::Publisher<
    px4_msgs::msg::VehicleTorqueSetpoint>::SharedPtr
    vehicle_torque_setpoint_pub_;

  rclcpp::Publisher<
    px4_msgs::msg::VehicleRatesSetpoint>::SharedPtr
    direct_rate_command_pub_;

  rclcpp::Publisher<
    px4_msgs::msg::VehicleTorqueSetpoint>::SharedPtr
    torque_rate_feedback_pub_;

  rclcpp::Publisher<
    px4_msgs::msg::VehicleTorqueSetpoint>::SharedPtr
    torque_angular_acceleration_feedforward_pub_;

  rclcpp::Publisher<
    px4_msgs::msg::VehicleCommand>::SharedPtr
    vehicle_command_pub_;

  rclcpp::Subscription<
    px4_msgs::msg::VehicleLocalPosition>::SharedPtr
    local_position_sub_;

  rclcpp::Subscription<
    px4_msgs::msg::VehicleAttitude>::SharedPtr
    vehicle_attitude_sub_;

  rclcpp::Subscription<
    px4_msgs::msg::VehicleAngularVelocity>::SharedPtr
    angular_velocity_sub_;

  rclcpp::Subscription<
    px4_msgs::msg::VehicleStatus>::SharedPtr
    vehicle_status_sub_;

  rclcpp::Subscription<
    px4_msgs::msg::VehicleCommandAck>::SharedPtr
    command_ack_sub_;

  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace offboard_controllers


int main(
  int argc,
  char * argv[])
{
  rclcpp::init(
    argc,
    argv);

  rclcpp::spin(
    std::make_shared<
      offboard_controllers::Se3>());

  rclcpp::shutdown();

  return 0;
}
