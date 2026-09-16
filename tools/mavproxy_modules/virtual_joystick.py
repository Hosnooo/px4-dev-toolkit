"""MAVProxy virtual joystick for PX4 SITL."""

import os
import socket
import time

from MAVProxy.modules.lib import mp_module


SEND_RATE_HZ = 20.0
SEND_PERIOD_S = 1.0 / SEND_RATE_HZ

CONTROL_SOCKET_PATH = "/tmp/px4_virtual_joystick.sock"
CONTROL_MESSAGE_BYTES = 256


def _axis(value):
    value = float(value)

    if not -1.0 <= value <= 1.0:
        raise ValueError("axis value must be between -1.0 and 1.0")

    return value


def _buttons(value):
    value = int(value)

    if not 0 <= value <= 65535:
        raise ValueError("buttons must be between 0 and 65535")

    return value


def _manual_control_values(roll, pitch, throttle, yaw):
    """Convert normalized joystick axes to MAVLink MANUAL_CONTROL fields."""
    return (
        int(round(_axis(pitch) * 1000.0)),
        int(round(_axis(roll) * 1000.0)),
        int(round((_axis(throttle) + 1.0) * 500.0)),
        int(round(_axis(yaw) * 1000.0)),
    )


class VirtualJoystick(mp_module.MPModule):
    """Continuously send the current software-joystick state."""

    def __init__(self, mpstate):
        super().__init__(
            mpstate,
            "virtual_joystick",
            "PX4 virtual joystick",
            public=True,
        )

        # Safe startup state for an unarmed vehicle. Values persist like a
        # physical joystick until the console or a local control client changes
        # them.
        self.roll = 0.0
        self.pitch = 0.0
        self.throttle = -1.0
        self.yaw = 0.0
        self.buttons = 0

        self._last_send = 0.0
        self._control_socket = self._open_control_socket()

        self.add_command(
            "vjoy",
            self.cmd_vjoy,
            "PX4 virtual joystick",
            ["<status|center|set|roll|pitch|throttle|yaw|buttons>"],
        )

        print(
            "Virtual joystick active at %.0f Hz "
            "(roll=0 pitch=0 throttle=-1 yaw=0)." % SEND_RATE_HZ
        )
        print("Virtual joystick control socket: %s" % CONTROL_SOCKET_PATH)

    def _open_control_socket(self):
        try:
            os.unlink(CONTROL_SOCKET_PATH)
        except FileNotFoundError:
            pass

        control_socket = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        control_socket.bind(CONTROL_SOCKET_PATH)
        control_socket.setblocking(False)
        return control_socket

    def unload(self):
        self.remove_command("vjoy")

        try:
            self._control_socket.close()
        finally:
            try:
                os.unlink(CONTROL_SOCKET_PATH)
            except FileNotFoundError:
                pass

        super().unload()

    def idle_task(self):
        self._receive_control_commands()

        now = time.monotonic()

        if now - self._last_send < SEND_PERIOD_S:
            return

        self._last_send = now
        self._send()

    def _receive_control_commands(self):
        while True:
            try:
                data = self._control_socket.recv(CONTROL_MESSAGE_BYTES)
            except BlockingIOError:
                return
            except OSError as error:
                print("vjoy control socket: %s" % error)
                return

            try:
                command = data.decode("ascii").strip().split()
            except UnicodeDecodeError:
                print("vjoy control socket: command must be ASCII")
                continue

            if not command:
                continue

            self._apply_command(command, report=False)

    def _send(self):
        master = self.master

        if master is None:
            return

        x, y, z, r = _manual_control_values(
            self.roll,
            self.pitch,
            self.throttle,
            self.yaw,
        )

        master.mav.manual_control_send(
            self.target_system,
            x,
            y,
            z,
            r,
            self.buttons,
        )

    def _status(self):
        print(
            "vjoy: roll=%.3f pitch=%.3f throttle=%.3f yaw=%.3f "
            "buttons=%d rate=%.0fHz"
            % (
                self.roll,
                self.pitch,
                self.throttle,
                self.yaw,
                self.buttons,
                SEND_RATE_HZ,
            )
        )

    def _apply_command(self, args, report):
        usage = (
            "Usage:\n"
            "  vjoy status\n"
            "  vjoy center\n"
            "  vjoy set <roll> <pitch> <throttle> <yaw>\n"
            "  vjoy <roll|pitch|throttle|yaw> <value>\n"
            "  vjoy buttons <0..65535>\n"
            "Axis values use [-1.0, 1.0]."
        )

        command = args[0].lower()

        try:
            if command == "status" and len(args) == 1:
                if report:
                    self._status()
                return True

            if command == "center" and len(args) == 1:
                self.roll = 0.0
                self.pitch = 0.0
                self.throttle = 0.0
                self.yaw = 0.0
                self.buttons = 0

            elif command == "set" and len(args) == 5:
                self.roll = _axis(args[1])
                self.pitch = _axis(args[2])
                self.throttle = _axis(args[3])
                self.yaw = _axis(args[4])

            elif command in ("roll", "pitch", "throttle", "yaw") and len(args) == 2:
                setattr(self, command, _axis(args[1]))

            elif command == "buttons" and len(args) == 2:
                self.buttons = _buttons(args[1])

            else:
                if report:
                    print(usage)
                else:
                    print("vjoy control socket: invalid command")
                return False

        except ValueError as error:
            print("vjoy: %s" % error)
            return False

        self._send()

        if report:
            self._status()

        return True

    def cmd_vjoy(self, args):
        if not args:
            self._apply_command(["invalid"], report=True)
            return

        self._apply_command(args, report=True)


def init(mpstate):
    return VirtualJoystick(mpstate)
