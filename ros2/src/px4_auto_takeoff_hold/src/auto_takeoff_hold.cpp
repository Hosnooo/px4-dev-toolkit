#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>

#include <px4_msgs/msg/mode_completed.hpp>
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
 * Minimal native-PX4 flight sequence:
 *
 *   wait for PX4 readiness and a valid local position
 *       -> arm
 *       -> PX4 AUTO_TAKEOFF
 *       -> wait for PX4 AUTO_LOITER
 *       -> report the hold-entry position
 *       -> exit
 *
 * This node is intentionally not an Offboard controller. It publishes only
 * discrete VehicleCommand messages for arming and takeoff; it never streams
 * external position setpoints or takes ownership of the position controller.
 *
 * After AUTO_LOITER is confirmed, the node has completed its job. PX4 keeps
 * holding the vehicle using its own internal controller after this process
 * exits.
 */
class AutoTakeoffHold : public rclcpp::Node
{
public:
  AutoTakeoffHold()
  : Node("auto_takeoff_hold")
  {
    /*
     * PX4 output topics are sensor-style streams, so use SensorDataQoS for
     * status and estimator data. VehicleCommand is an input command topic and
     * uses a normal reliable publisher queue.
     */
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

    mode_completed_sub_ =
      create_subscription<px4_msgs::msg::ModeCompleted>(
      px4_topics::OUT_MODE_COMPLETED,
      sensor_qos,
      [this](const px4_msgs::msg::ModeCompleted::SharedPtr msg) {
        handle_mode_completed(*msg);
      });

    vehicle_command_pub_ =
      create_publisher<px4_msgs::msg::VehicleCommand>(
      px4_topics::IN_VEHICLE_COMMAND,
      10);

    /*
     * The timer drives only the high-level sequence. Flight control remains
     * entirely inside PX4, so this is not a setpoint publication loop.
     */
    timer_ = create_wall_timer(
      std::chrono::milliseconds(100),
      [this]() {
        run();
      });

    RCLCPP_INFO(
      get_logger(),
      "Waiting for PX4 vehicle status and valid local position.");
  }

private:
  /*
   * Cache the PX4 state needed by the flight sequence. Status is logged only
   * when arming or navigation state changes so the console remains readable.
   */
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
   * Require a complete local position estimate before starting the flight.
   * PX4 VehicleLocalPosition is expressed in the local NED frame:
   *
   *   x = North
   *   y = East
   *   z = Down
   *
   * Keep the latest valid position so it can be reported when PX4 enters
   * AUTO_LOITER after takeoff.
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


  /*
   * Report acknowledgements only for commands this node actually sends.
   *
   * An ACK tells us whether PX4 accepted or rejected the request. It does not
   * prove the requested state has already been reached; VehicleStatus and
   * ModeCompleted provide that confirmation later in the sequence.
   */
  void handle_command_ack(
    const px4_msgs::msg::VehicleCommandAck & msg)
  {
    using VehicleCommand = px4_msgs::msg::VehicleCommand;
    using VehicleCommandAck = px4_msgs::msg::VehicleCommandAck;

    const bool accepted =
      msg.result ==
      VehicleCommandAck::VEHICLE_CMD_RESULT_ACCEPTED;

    switch (msg.command) {
      case VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM:
        if (accepted) {
          RCLCPP_INFO(
            get_logger(),
            "PX4 accepted arm command.");
        } else {
          RCLCPP_ERROR(
            get_logger(),
            "PX4 rejected arm command: %u",
            static_cast<unsigned>(msg.result));
        }
        break;

      case VehicleCommand::VEHICLE_CMD_NAV_TAKEOFF:
        if (accepted) {
          RCLCPP_INFO(
            get_logger(),
            "PX4 accepted takeoff command.");
        } else {
          RCLCPP_ERROR(
            get_logger(),
            "PX4 rejected takeoff command: %u",
            static_cast<unsigned>(msg.result));
        }
        break;

      default:
        break;
    }
  }


  /*
   * PX4 explicitly reports completion of AUTO_TAKEOFF on ModeCompleted.
   * Use that native completion event instead of inferring completion from
   * altitude, elapsed time, or velocity.
   */
  void handle_mode_completed(
    const px4_msgs::msg::ModeCompleted & msg)
  {
    using ModeCompleted = px4_msgs::msg::ModeCompleted;
    using VehicleStatus = px4_msgs::msg::VehicleStatus;

    if (msg.nav_state !=
      VehicleStatus::NAVIGATION_STATE_AUTO_TAKEOFF)
    {
      return;
    }

    if (msg.result == ModeCompleted::RESULT_SUCCESS) {
      if (!takeoff_complete_) {
        RCLCPP_INFO(
          get_logger(),
          "PX4 takeoff complete.");
      }

      takeoff_complete_ = true;
    } else {
      RCLCPP_ERROR(
        get_logger(),
        "PX4 takeoff failed: result=%u",
        static_cast<unsigned>(msg.result));
    }
  }


  void run()
  {
    using VehicleStatus = px4_msgs::msg::VehicleStatus;

    /*
     * Do not start the sequence until both PX4 status and a valid local
     * position estimate are available.
     */
    if (!status_received_ || !local_position_valid_) {
      return;
    }

    /*
     * 1. Wait for PX4 pre-flight checks, then send one arm request.
     *
     * The arm command is not repeated every timer tick. After sending it,
     * wait for VehicleStatus to confirm ARMING_STATE_ARMED.
     */
    if (arming_state_ != VehicleStatus::ARMING_STATE_ARMED) {
      if (!pre_flight_checks_pass_) {
        if (!waiting_for_preflight_reported_) {
          RCLCPP_INFO(
            get_logger(),
            "Waiting for PX4 pre-flight checks.");

          waiting_for_preflight_reported_ = true;
        }

        return;
      }

      if (!arm_command_sent_) {
        publish_arm_command();
        arm_command_sent_ = true;
      }

      return;
    }

    if (!armed_reported_) {
      RCLCPP_INFO(
        get_logger(),
        "Vehicle armed.");

      armed_reported_ = true;
    }

    /*
     * 2. Ask PX4 to execute its native AUTO_TAKEOFF sequence.
     *
     * No external position or altitude target is supplied. PX4 selects and
     * controls the takeoff behavior using its own configuration.
     */
    if (!takeoff_command_sent_) {
      publish_takeoff_command();
      takeoff_command_sent_ = true;
      return;
    }

    /*
     * 3. Wait for PX4's native AUTO_TAKEOFF completion event.
     *
     * Until ModeCompleted reports success, the node does not assume that the
     * climb has finished and does not proceed to the final state check.
     */
    if (!takeoff_complete_) {
      return;
    }

    /*
     * 4. Wait for PX4 to transition itself into AUTO_LOITER.
     *
     * No mode command is sent here. AUTO_LOITER is entered by PX4 as part of
     * its native takeoff behavior, and PX4's internal position controller owns
     * the hold from this point onward.
     */
    if (nav_state_ != VehicleStatus::NAVIGATION_STATE_AUTO_LOITER) {
      return;
    }

    /*
     * The maneuver is complete once AUTO_LOITER is confirmed.
     *
     * Record the position at which PX4 entered its native hold, then terminate
     * this ROS process. Exiting does not disturb the hover because this node
     * has never supplied continuous setpoints or owned the position controller.
     */
    RCLCPP_INFO(
      get_logger(),
      "PX4 AUTO_LOITER active.");

    RCLCPP_INFO(
      get_logger(),
      "Hold position (NED): x=%.2f y=%.2f z=%.2f",
      static_cast<double>(local_x_),
      static_cast<double>(local_y_),
      static_cast<double>(local_z_));

    RCLCPP_INFO(
      get_logger(),
      "Auto takeoff and hold sequence complete.");

    timer_->cancel();

    /*
     * Stop the ROS context so rclcpp::spin() in main() returns cleanly.
     * The process exits with status 0 after completing the native PX4 sequence.
     */
    rclcpp::shutdown();
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


  void publish_takeoff_command()
  {
    px4_msgs::msg::VehicleCommand msg{};

    /*
     * Leave every optional takeoff parameter unspecified. PX4 therefore uses
     * its configured native takeoff behavior instead of a ROS-provided target.
     */
    msg.param1 = NAN;
    msg.param2 = NAN;
    msg.param3 = NAN;
    msg.param4 = NAN;
    msg.param5 = NAN;
    msg.param6 = NAN;
    msg.param7 = NAN;

    msg.command =
      px4_msgs::msg::VehicleCommand::
      VEHICLE_CMD_NAV_TAKEOFF;

    fill_command_metadata(msg);

    vehicle_command_pub_->publish(msg);

    RCLCPP_INFO(
      get_logger(),
      "Takeoff command sent.");
  }


  /*
   * Commands enter PX4 through the uXRCE-DDS bridge as external commands.
   * Target system/component 1 addresses the primary PX4 autopilot in SITL.
   */
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


  std::uint64_t now_us() const
  {
    return static_cast<std::uint64_t>(
      get_clock()->now().nanoseconds() / 1000);
  }


  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr
    vehicle_status_sub_;

  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr
    vehicle_local_position_sub_;

  rclcpp::Subscription<px4_msgs::msg::VehicleCommandAck>::SharedPtr
    vehicle_command_ack_sub_;

  rclcpp::Subscription<px4_msgs::msg::ModeCompleted>::SharedPtr
    mode_completed_sub_;

  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr
    vehicle_command_pub_;

  rclcpp::TimerBase::SharedPtr timer_;

  bool status_received_{false};
  bool local_position_valid_{false};

  bool pre_flight_checks_pass_{false};

  bool arm_command_sent_{false};
  bool armed_reported_{false};
  bool waiting_for_preflight_reported_{false};

  bool takeoff_command_sent_{false};
  bool takeoff_complete_{false};

  float local_x_{NAN};
  float local_y_{NAN};
  float local_z_{NAN};

  std::uint8_t arming_state_{0};
  std::uint8_t nav_state_{0};
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::spin(
    std::make_shared<AutoTakeoffHold>());

  /*
   * The successful flight path shuts ROS down from the node once AUTO_LOITER
   * is confirmed. This guard also handles external termination cleanly.
   */
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }

  return 0;
}