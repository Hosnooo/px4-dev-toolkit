#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>

#include <px4_msgs/msg/manual_control_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_command_ack.hpp>
#include <px4_msgs/msg/vehicle_land_detected.hpp>
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
 * Minimal headless PX4 Position-mode flight sequence:
 *
 *   wait for PX4 readiness and a valid local position
 *       -> provide a virtual manual-control stream
 *       -> switch to Position mode
 *       -> arm
 *       -> climb approximately 2 m
 *       -> center the sticks and hover for 20 seconds
 *       -> ask PX4 to land at the current position
 *       -> wait for PX4 to land and disarm
 *
 * This node is intentionally not an Offboard controller. It supplies the
 * manual-control input normally provided by a joystick while PX4 retains
 * ownership of its position, attitude, rate, and actuator-control loops.
 */
class PositionTakeoffHover : public rclcpp::Node
{
public:
  PositionTakeoffHover()
  : Node("position_takeoff_hover")
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

    vehicle_land_detected_sub_ =
      create_subscription<px4_msgs::msg::VehicleLandDetected>(
      px4_topics::OUT_VEHICLE_LAND_DETECTED,
      sensor_qos,
      [this](const px4_msgs::msg::VehicleLandDetected::SharedPtr msg) {
        landed_ = msg->landed;
        land_detected_received_ = true;
      });

    vehicle_command_ack_sub_ =
      create_subscription<px4_msgs::msg::VehicleCommandAck>(
      px4_topics::OUT_VEHICLE_COMMAND_ACK,
      sensor_qos,
      [this](const px4_msgs::msg::VehicleCommandAck::SharedPtr msg) {
        handle_command_ack(*msg);
      });

    vehicle_command_pub_ =
      create_publisher<px4_msgs::msg::VehicleCommand>(
      px4_topics::IN_VEHICLE_COMMAND,
      10);

    manual_control_pub_ =
      create_publisher<px4_msgs::msg::ManualControlSetpoint>(
      px4_topics::IN_MANUAL_CONTROL_INPUT,
      10);

    phase_started_at_ = SteadyClock::now();

    timer_ = create_wall_timer(
      std::chrono::milliseconds(50),
      [this]() {
        run();
      });

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
    WAIT_POSITION_MODE,
    WAIT_PREFLIGHT,
    WAIT_ARM,
    CLIMB,
    HOVER,
    WAIT_LAND,
    DONE,
  };

  static constexpr float kClimbHeightM = 2.0F;
  static constexpr float kPrearmThrottle = -1.0F;
  static constexpr float kClimbThrottle = 0.60F;
  static constexpr float kHoverThrottle = 0.0F;

  static constexpr std::uint32_t kManualWarmupSamples = 20;

  /*
   * PX4's pinned custom-mode definition maps main mode 3 to POSCTL.
   * VEHICLE_CMD_DO_SET_MODE param1=1 selects the custom-mode fields.
   */
  static constexpr float kCustomModeEnabled = 1.0F;
  static constexpr float kPositionMainMode = 3.0F;
  static constexpr float kPositionSubMode = 0.0F;


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


  /*
   * Position mode depends on PX4's local estimate. VehicleLocalPosition is
   * expressed in local NED coordinates, where negative z is upward.
   */
  void handle_local_position(
    const px4_msgs::msg::VehicleLocalPosition & msg)
  {
    const bool valid =
      msg.xy_valid &&
      msg.z_valid &&
      msg.v_xy_valid &&
      msg.v_z_valid &&
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

    if (valid) {
      local_x_ = msg.x;
      local_y_ = msg.y;
      local_z_ = msg.z;
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
      msg.command != VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM &&
      msg.command != VehicleCommand::VEHICLE_CMD_NAV_LAND)
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


  float commanded_throttle() const
  {
    switch (phase_) {
      case Phase::CLIMB:
        return kClimbThrottle;

      case Phase::HOVER:
      case Phase::WAIT_LAND:
        return kHoverThrottle;

      default:
        return kPrearmThrottle;
    }
  }


  /*
   * Keep the virtual joystick alive for the complete flight. The input uses a
   * MAVLink-class source so PX4's normal joystick/manual-control selector
   * accepts it without introducing an Offboard control path.
   */
  void publish_manual_control()
  {
    px4_msgs::msg::ManualControlSetpoint msg{};

    const std::uint64_t timestamp = now_us();

    msg.timestamp = timestamp;
    msg.timestamp_sample = timestamp;
    msg.valid = true;

    msg.data_source =
      px4_msgs::msg::ManualControlSetpoint::SOURCE_MAVLINK_0;

    msg.roll = 0.0F;
    msg.pitch = 0.0F;
    msg.yaw = 0.0F;
    msg.throttle = commanded_throttle();

    msg.flaps = NAN;
    msg.aux1 = NAN;
    msg.aux2 = NAN;
    msg.aux3 = NAN;
    msg.aux4 = NAN;
    msg.aux5 = NAN;
    msg.aux6 = NAN;

    msg.sticks_moving = false;
    msg.buttons = 0;

    manual_control_pub_->publish(msg);
    ++manual_samples_published_;
  }


  void publish_position_mode_command()
  {
    px4_msgs::msg::VehicleCommand msg{};

    msg.param1 = kCustomModeEnabled;
    msg.param2 = kPositionMainMode;
    msg.param3 = kPositionSubMode;

    msg.command =
      px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE;

    fill_command_metadata(msg);
    vehicle_command_pub_->publish(msg);

    RCLCPP_INFO(
      get_logger(),
      "Position-mode command sent.");
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

    RCLCPP_INFO(
      get_logger(),
      "Arm command sent.");
  }


  void publish_land_command()
  {
    px4_msgs::msg::VehicleCommand msg{};

    /*
     * No global coordinates are supplied. PX4 handles NAV_LAND as a request
     * to land at the current position and can fall back to DESCEND if needed.
     */
    msg.param1 = NAN;
    msg.param2 = NAN;
    msg.param3 = NAN;
    msg.param4 = NAN;
    msg.param5 = NAN;
    msg.param6 = NAN;
    msg.param7 = NAN;

    msg.command =
      px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND;

    fill_command_metadata(msg);
    vehicle_command_pub_->publish(msg);

    RCLCPP_INFO(
      get_logger(),
      "Land command sent.");
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
      "Position-mode experiment failed: %s",
      reason);

    exit_code_ = 1;
    set_phase(Phase::DONE);
    timer_->cancel();
    rclcpp::shutdown();
  }


  void complete()
  {
    RCLCPP_INFO(
      get_logger(),
      "Position-mode takeoff and hover sequence complete.");

    exit_code_ = 0;
    set_phase(Phase::DONE);
    timer_->cancel();
    rclcpp::shutdown();
  }


  void run()
  {
    using VehicleStatus = px4_msgs::msg::VehicleStatus;

    publish_manual_control();

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
        if (manual_samples_published_ < kManualWarmupSamples) {
          return;
        }

        publish_position_mode_command();
        set_phase(Phase::WAIT_POSITION_MODE);
        return;


      case Phase::WAIT_POSITION_MODE:
        if (nav_state_ == VehicleStatus::NAVIGATION_STATE_POSCTL) {
          start_x_ = local_x_;
          start_y_ = local_y_;
          start_z_ = local_z_;
          target_z_ = start_z_ - kClimbHeightM;

          RCLCPP_INFO(
            get_logger(),
            "PX4 Position mode active: "
            "x=%.2f y=%.2f z=%.2f target_z=%.2f",
            static_cast<double>(start_x_),
            static_cast<double>(start_y_),
            static_cast<double>(start_z_),
            static_cast<double>(target_z_));

          set_phase(Phase::WAIT_PREFLIGHT);
          return;
        }

        if (phase_elapsed_seconds() > 5.0) {
          fail("PX4 did not enter Position mode.");
        }

        return;


      case Phase::WAIT_PREFLIGHT:
        if (pre_flight_checks_pass_) {
          publish_arm_command();
          set_phase(Phase::WAIT_ARM);
          return;
        }

        if (!waiting_for_preflight_reported_) {
          RCLCPP_INFO(
            get_logger(),
            "Waiting for PX4 pre-flight checks.");

          waiting_for_preflight_reported_ = true;
        }

        if (phase_elapsed_seconds() > 10.0) {
          fail("PX4 pre-flight checks did not pass.");
        }

        return;


      case Phase::WAIT_ARM:
        if (arming_state_ == VehicleStatus::ARMING_STATE_ARMED) {
          RCLCPP_INFO(
            get_logger(),
            "Vehicle armed; starting climb.");

          set_phase(Phase::CLIMB);
          return;
        }

        if (phase_elapsed_seconds() > 5.0) {
          fail("PX4 did not arm.");
        }

        return;


      case Phase::CLIMB:
        if (arming_state_ != VehicleStatus::ARMING_STATE_ARMED) {
          fail("Vehicle disarmed during climb.");
          return;
        }

        if (local_z_ <= target_z_) {
          RCLCPP_INFO(
            get_logger(),
            "Climb target reached: z=%.2f. "
            "Centering sticks for 20-second hover.",
            static_cast<double>(local_z_));

          set_phase(Phase::HOVER);
          return;
        }

        if (phase_elapsed_seconds() > 15.0) {
          fail("Vehicle did not reach the climb target.");
        }

        return;


      case Phase::HOVER:
        if (arming_state_ != VehicleStatus::ARMING_STATE_ARMED) {
          fail("Vehicle disarmed during hover.");
          return;
        }

        if (phase_elapsed_seconds() >= 20.0) {
          RCLCPP_INFO(
            get_logger(),
            "Hover complete; asking PX4 to land.");

          publish_land_command();
          set_phase(Phase::WAIT_LAND);
        }

        return;


      case Phase::WAIT_LAND:
        if (arming_state_ == VehicleStatus::ARMING_STATE_DISARMED) {
          RCLCPP_INFO(
            get_logger(),
            "PX4 landed and disarmed.");

          complete();
          return;
        }

        if (
          land_detected_received_ &&
          landed_ &&
          !landed_reported_)
        {
          RCLCPP_INFO(
            get_logger(),
            "PX4 reports landed; waiting for automatic disarm.");

          landed_reported_ = true;
        }

        if (phase_elapsed_seconds() > 30.0) {
          fail("PX4 did not complete landing and disarm.");
        }

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


  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr
    vehicle_status_sub_;

  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr
    vehicle_local_position_sub_;

  rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr
    vehicle_land_detected_sub_;

  rclcpp::Subscription<px4_msgs::msg::VehicleCommandAck>::SharedPtr
    vehicle_command_ack_sub_;

  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr
    vehicle_command_pub_;

  rclcpp::Publisher<px4_msgs::msg::ManualControlSetpoint>::SharedPtr
    manual_control_pub_;

  rclcpp::TimerBase::SharedPtr timer_;

  Phase phase_{Phase::WAIT_READY};
  SteadyClock::time_point phase_started_at_{};

  bool status_received_{false};
  bool local_position_valid_{false};
  bool land_detected_received_{false};
  bool landed_{true};
  bool landed_reported_{false};

  bool pre_flight_checks_pass_{false};
  bool waiting_for_preflight_reported_{false};

  std::uint32_t manual_samples_published_{0};

  float local_x_{NAN};
  float local_y_{NAN};
  float local_z_{NAN};

  float start_x_{NAN};
  float start_y_{NAN};
  float start_z_{NAN};
  float target_z_{NAN};

  std::uint8_t arming_state_{0};
  std::uint8_t nav_state_{0};

  int exit_code_{1};
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node =
    std::make_shared<PositionTakeoffHover>();

  rclcpp::spin(node);

  const int exit_code = node->exit_code();

  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }

  return exit_code;
}
