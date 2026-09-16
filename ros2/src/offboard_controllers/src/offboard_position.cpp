#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>

#include <offboard_controllers/offboard_config.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_command_ack.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <rclcpp/rclcpp.hpp>


namespace px4_topics
{

#define PX4_TOPIC(name, value) inline constexpr char name[] = value;
#include "px4_topics.def"
#undef PX4_TOPIC

}  // namespace px4_topics


/*
 * Independent PX4 Offboard position controller.
 *
 * The controller depends only on PX4's fused local state. It is therefore
 * agnostic to whether EKF2 receives external vision from Gazebo, Vicon, or
 * another estimator input.
 *
 *   wait for PX4 status and a valid local position
 *       -> arm if the vehicle is not already armed
 *       -> prestream OffboardControlMode + configured TrajectorySetpoint
 *       -> request Offboard
 *       -> wait until PX4 reports Offboard mode
 *       -> keep publishing the configured position and yaw
 *
 * The initial setpoint is required from config/offboard.yaml. No fallback
 * setpoint is embedded in this source file.
 */
class OffboardPosition : public rclcpp::Node
{
public:
  OffboardPosition()
  : Node("offboard_position"),
    initial_setpoint_(
      offboard_controllers::load_initial_setpoint(*this))
  {
    const auto sensor_qos = rclcpp::SensorDataQoS();

    vehicle_status_sub_ =
      create_subscription<px4_msgs::msg::VehicleStatus>(
      px4_topics::OUT_VEHICLE_STATUS_V1,
      sensor_qos,
      [this](const px4_msgs::msg::VehicleStatus::SharedPtr msg) {
        handle_vehicle_status(*msg);
      });

    vehicle_local_position_sub_ =
      create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      px4_topics::OUT_VEHICLE_LOCAL_POSITION,
      sensor_qos,
      [this](const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
        handle_local_position(*msg);
      });

    vehicle_command_ack_sub_ =
      create_subscription<px4_msgs::msg::VehicleCommandAck>(
      px4_topics::OUT_VEHICLE_COMMAND_ACK,
      sensor_qos,
      [this](const px4_msgs::msg::VehicleCommandAck::SharedPtr msg) {
        handle_command_ack(*msg);
      });

    offboard_control_mode_pub_ =
      create_publisher<px4_msgs::msg::OffboardControlMode>(
      px4_topics::IN_OFFBOARD_CONTROL_MODE,
      10);

    trajectory_setpoint_pub_ =
      create_publisher<px4_msgs::msg::TrajectorySetpoint>(
      px4_topics::IN_TRAJECTORY_SETPOINT,
      10);

    vehicle_command_pub_ =
      create_publisher<px4_msgs::msg::VehicleCommand>(
      px4_topics::IN_VEHICLE_COMMAND,
      10);

    phase_started_at_ = SteadyClock::now();

    timer_ = create_wall_timer(
      std::chrono::milliseconds(50),
      [this]() {
        run();
      });

    RCLCPP_INFO(
      get_logger(),
      "Configured Offboard setpoint: "
      "x=%.2f y=%.2f z=%.2f yaw=%.2f",
      static_cast<double>(initial_setpoint_.x),
      static_cast<double>(initial_setpoint_.y),
      static_cast<double>(initial_setpoint_.z),
      static_cast<double>(initial_setpoint_.yaw));

    RCLCPP_INFO(
      get_logger(),
      "Waiting for PX4 vehicle status and valid local position.");
  }

  int exit_code() const
  {
    return exit_code_;
  }

private:
  using SteadyClock = std::chrono::steady_clock;

  enum class Phase
  {
    WAIT_READY,
    WAIT_PREFLIGHT,
    WAIT_ARM,
    PRESTREAM,
    WAIT_OFFBOARD,
    RUN,
    DONE,
  };

  static constexpr std::uint32_t kOffboardWarmupSamples = 30;

  static constexpr float kCustomModeEnabled = 1.0F;
  static constexpr float kOffboardMainMode = 6.0F;
  static constexpr float kOffboardSubMode = 0.0F;


  void handle_vehicle_status(
    const px4_msgs::msg::VehicleStatus & msg)
  {
    if (!status_received_ ||
      msg.arming_state != arming_state_ ||
      msg.nav_state != nav_state_)
    {
      RCLCPP_INFO(
        get_logger(),
        "PX4 status: arming_state=%u, nav_state=%u",
        static_cast<unsigned>(msg.arming_state),
        static_cast<unsigned>(msg.nav_state));
    }

    arming_state_ = msg.arming_state;
    nav_state_ = msg.nav_state;
    pre_flight_checks_pass_ = msg.pre_flight_checks_pass;
    status_received_ = true;
  }


  void handle_local_position(
    const px4_msgs::msg::VehicleLocalPosition & msg)
  {
    const bool valid =
      msg.xy_valid &&
      msg.z_valid &&
      std::isfinite(msg.x) &&
      std::isfinite(msg.y) &&
      std::isfinite(msg.z);

    if (valid && !local_position_valid_) {
      RCLCPP_INFO(
        get_logger(),
        "Local position valid: x=%.2f y=%.2f z=%.2f",
        static_cast<double>(msg.x),
        static_cast<double>(msg.y),
        static_cast<double>(msg.z));
    }

    if (!valid && local_position_valid_) {
      RCLCPP_WARN(
        get_logger(),
        "PX4 local position is no longer valid.");
    }

    local_position_valid_ = valid;
  }


  void handle_command_ack(
    const px4_msgs::msg::VehicleCommandAck & msg)
  {
    using VehicleCommand = px4_msgs::msg::VehicleCommand;
    using VehicleCommandAck = px4_msgs::msg::VehicleCommandAck;

    if (
      msg.command != VehicleCommand::VEHICLE_CMD_DO_SET_MODE &&
      msg.command != VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM)
    {
      return;
    }

    const bool accepted =
      msg.result ==
      VehicleCommandAck::VEHICLE_CMD_RESULT_ACCEPTED;

    if (accepted) {
      RCLCPP_INFO(
        get_logger(),
        "PX4 accepted command %u.",
        static_cast<unsigned>(msg.command));

    } else {
      RCLCPP_ERROR(
        get_logger(),
        "PX4 rejected command %u: result=%u",
        static_cast<unsigned>(msg.command),
        static_cast<unsigned>(msg.result));
    }
  }


  void publish_offboard_control_mode(
    std::uint64_t timestamp)
  {
    px4_msgs::msg::OffboardControlMode msg{};

    msg.timestamp = timestamp;
    msg.position = true;
    msg.velocity = false;
    msg.acceleration = false;
    msg.attitude = false;
    msg.body_rate = false;
    msg.thrust_and_torque = false;
    msg.direct_actuator = false;

    offboard_control_mode_pub_->publish(msg);
  }


  void publish_trajectory_setpoint(
    std::uint64_t timestamp)
  {
    px4_msgs::msg::TrajectorySetpoint msg{};

    msg.timestamp = timestamp;
    msg.position = {
      initial_setpoint_.x,
      initial_setpoint_.y,
      initial_setpoint_.z};

    msg.velocity = {NAN, NAN, NAN};
    msg.acceleration = {NAN, NAN, NAN};
    msg.jerk = {NAN, NAN, NAN};

    msg.yaw = initial_setpoint_.yaw;
    msg.yawspeed = NAN;

    trajectory_setpoint_pub_->publish(msg);
  }


  void publish_offboard_stream()
  {
    const std::uint64_t timestamp = now_us();

    publish_offboard_control_mode(timestamp);
    publish_trajectory_setpoint(timestamp);
  }


  void publish_arm_command()
  {
    px4_msgs::msg::VehicleCommand msg{};

    msg.param1 =
      static_cast<float>(
      px4_msgs::msg::VehicleCommand::ARMING_ACTION_ARM);

    msg.command =
      px4_msgs::msg::VehicleCommand::
      VEHICLE_CMD_COMPONENT_ARM_DISARM;

    fill_command_metadata(msg);
    vehicle_command_pub_->publish(msg);

    RCLCPP_INFO(get_logger(), "Arm command sent.");
  }


  void publish_offboard_mode_command()
  {
    px4_msgs::msg::VehicleCommand msg{};

    msg.param1 = kCustomModeEnabled;
    msg.param2 = kOffboardMainMode;
    msg.param3 = kOffboardSubMode;

    msg.command =
      px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE;

    fill_command_metadata(msg);
    vehicle_command_pub_->publish(msg);

    RCLCPP_INFO(
      get_logger(),
      "Offboard mode command sent after %u prestream samples.",
      static_cast<unsigned>(offboard_samples_published_));
  }


  void fill_command_metadata(
    px4_msgs::msg::VehicleCommand & msg)
  {
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    msg.timestamp = now_us();
  }


  void start_prestream()
  {
    offboard_samples_published_ = 0;

    RCLCPP_INFO(
      get_logger(),
      "Starting Offboard heartbeat and configured setpoint prestream.");

    set_phase(Phase::PRESTREAM);
  }


  void set_phase(Phase phase)
  {
    phase_ = phase;
    phase_started_at_ = SteadyClock::now();
  }


  double phase_elapsed_seconds() const
  {
    return std::chrono::duration<double>(
      SteadyClock::now() - phase_started_at_).count();
  }


  void fail(const char * reason)
  {
    RCLCPP_ERROR(
      get_logger(),
      "Offboard position controller failed: %s",
      reason);

    exit_code_ = 1;
    set_phase(Phase::DONE);
    timer_->cancel();
    rclcpp::shutdown();
  }


  void run()
  {
    using VehicleStatus = px4_msgs::msg::VehicleStatus;

    if (phase_ == Phase::DONE) {
      return;
    }

    if (!status_received_) {
      if (
        phase_ == Phase::WAIT_READY &&
        phase_elapsed_seconds() > 20.0)
      {
        fail("PX4 vehicle status was not available.");
      }

      return;
    }

    if (!local_position_valid_) {
      if (phase_ != Phase::WAIT_READY) {
        fail("PX4 local position became invalid.");

      } else if (phase_elapsed_seconds() > 20.0) {
        fail("A valid PX4 local position was not available.");
      }

      return;
    }

    switch (phase_) {
      case Phase::WAIT_READY:
        if (arming_state_ == VehicleStatus::ARMING_STATE_ARMED) {
          RCLCPP_INFO(
            get_logger(),
            "Vehicle already armed; proceeding to Offboard prestream.");

          start_prestream();
          return;
        }

        if (pre_flight_checks_pass_) {
          publish_arm_command();
          set_phase(Phase::WAIT_ARM);
          return;
        }

        RCLCPP_INFO(
          get_logger(),
          "Vehicle is disarmed; waiting for PX4 pre-flight checks.");

        set_phase(Phase::WAIT_PREFLIGHT);
        return;


      case Phase::WAIT_PREFLIGHT:
        if (arming_state_ == VehicleStatus::ARMING_STATE_ARMED) {
          RCLCPP_INFO(
            get_logger(),
            "Vehicle armed externally; proceeding to Offboard prestream.");

          start_prestream();
          return;
        }

        if (pre_flight_checks_pass_) {
          publish_arm_command();
          set_phase(Phase::WAIT_ARM);
          return;
        }

        if (phase_elapsed_seconds() > 10.0) {
          fail("PX4 pre-flight checks did not pass.");
        }

        return;


      case Phase::WAIT_ARM:
        if (arming_state_ == VehicleStatus::ARMING_STATE_ARMED) {
          RCLCPP_INFO(get_logger(), "Vehicle armed.");
          start_prestream();
          return;
        }

        if (phase_elapsed_seconds() > 5.0) {
          fail("PX4 did not arm.");
        }

        return;


      case Phase::PRESTREAM:
        if (arming_state_ != VehicleStatus::ARMING_STATE_ARMED) {
          fail("Vehicle disarmed before Offboard takeover.");
          return;
        }

        publish_offboard_stream();
        ++offboard_samples_published_;

        if (offboard_samples_published_ >= kOffboardWarmupSamples) {
          publish_offboard_mode_command();
          set_phase(Phase::WAIT_OFFBOARD);
        }

        return;


      case Phase::WAIT_OFFBOARD:
        if (arming_state_ != VehicleStatus::ARMING_STATE_ARMED) {
          fail("Vehicle disarmed while entering Offboard mode.");
          return;
        }

        publish_offboard_stream();

        if (nav_state_ == VehicleStatus::NAVIGATION_STATE_OFFBOARD) {
          RCLCPP_INFO(
            get_logger(),
            "PX4 Offboard mode active; tracking configured setpoint.");

          set_phase(Phase::RUN);
          return;
        }

        if (phase_elapsed_seconds() > 5.0) {
          fail("PX4 did not enter Offboard mode.");
        }

        return;


      case Phase::RUN:
        if (arming_state_ != VehicleStatus::ARMING_STATE_ARMED) {
          fail("Vehicle disarmed during Offboard control.");
          return;
        }

        if (nav_state_ != VehicleStatus::NAVIGATION_STATE_OFFBOARD) {
          fail("PX4 left Offboard mode.");
          return;
        }

        publish_offboard_stream();
        return;


      case Phase::DONE:
      default:
        return;
    }
  }


  std::uint64_t now_us() const
  {
    return static_cast<std::uint64_t>(
      get_clock()->now().nanoseconds() / 1000);
  }


  const offboard_controllers::InitialSetpoint initial_setpoint_;

  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr
    vehicle_status_sub_;

  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr
    vehicle_local_position_sub_;

  rclcpp::Subscription<px4_msgs::msg::VehicleCommandAck>::SharedPtr
    vehicle_command_ack_sub_;

  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr
    offboard_control_mode_pub_;

  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr
    trajectory_setpoint_pub_;

  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr
    vehicle_command_pub_;

  rclcpp::TimerBase::SharedPtr timer_;

  Phase phase_{Phase::WAIT_READY};
  SteadyClock::time_point phase_started_at_{};

  bool status_received_{false};
  bool local_position_valid_{false};
  bool pre_flight_checks_pass_{false};

  std::uint32_t offboard_samples_published_{0};

  std::uint8_t arming_state_{0};
  std::uint8_t nav_state_{0};

  int exit_code_{1};
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node =
    std::make_shared<OffboardPosition>();

  rclcpp::spin(node);

  const int exit_code = node->exit_code();

  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }

  return exit_code;
}
