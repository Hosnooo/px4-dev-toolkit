#pragma once

#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <stdexcept>
#include <string>

#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>


namespace px4_control_common
{

/*
 * Small client for the repository's MAVProxy virtual joystick.
 *
 * The client changes joystick state only. MAVProxy remains the single process
 * that continuously sends MAVLink MANUAL_CONTROL to PX4.
 */
class VirtualJoystickClient
{
public:
  VirtualJoystickClient()
  {
    socket_fd_ = socket(AF_UNIX, SOCK_DGRAM, 0);

    if (socket_fd_ < 0) {
      throw std::runtime_error(
              "Could not create virtual-joystick control socket: " +
              std::string(std::strerror(errno)));
    }
  }

  ~VirtualJoystickClient()
  {
    if (socket_fd_ >= 0) {
      close(socket_fd_);
    }
  }

  VirtualJoystickClient(const VirtualJoystickClient &) = delete;
  VirtualJoystickClient & operator=(const VirtualJoystickClient &) = delete;

  void set(
    double roll,
    double pitch,
    double throttle,
    double yaw) const
  {
    validate_axis("roll", roll);
    validate_axis("pitch", pitch);
    validate_axis("throttle", throttle);
    validate_axis("yaw", yaw);

    char command[160];

    const int length = std::snprintf(
      command,
      sizeof(command),
      "set %.6f %.6f %.6f %.6f",
      roll,
      pitch,
      throttle,
      yaw);

    if (length <= 0 ||
      static_cast<std::size_t>(length) >= sizeof(command))
    {
      throw std::runtime_error(
              "Could not format virtual-joystick command.");
    }

    send_command(command, static_cast<std::size_t>(length));
  }

  void center() const
  {
    set(0.0, 0.0, 0.0, 0.0);
  }

private:
  static constexpr char kControlSocketPath[] =
    "/tmp/px4_virtual_joystick.sock";


  static void validate_axis(
    const char * name,
    double value)
  {
    if (!std::isfinite(value) || value < -1.0 || value > 1.0) {
      throw std::invalid_argument(
              std::string("Virtual-joystick ") + name +
              " must be within [-1, 1].");
    }
  }


  void send_command(
    const char * command,
    std::size_t length) const
  {
    sockaddr_un address{};
    address.sun_family = AF_UNIX;

    if (std::strlen(kControlSocketPath) >= sizeof(address.sun_path)) {
      throw std::runtime_error(
              "Virtual-joystick control socket path is too long.");
    }

    std::strncpy(
      address.sun_path,
      kControlSocketPath,
      sizeof(address.sun_path) - 1);

    const ssize_t sent = sendto(
      socket_fd_,
      command,
      length,
      0,
      reinterpret_cast<const sockaddr *>(&address),
      sizeof(address));

    if (sent < 0) {
      throw std::runtime_error(
              "Could not reach MAVProxy virtual joystick at " +
              std::string(kControlSocketPath) + ": " +
              std::string(std::strerror(errno)));
    }

    if (static_cast<std::size_t>(sent) != length) {
      throw std::runtime_error(
              "Virtual-joystick control command was only partially sent.");
    }
  }


  int socket_fd_{-1};
};

}  // namespace px4_control_common
