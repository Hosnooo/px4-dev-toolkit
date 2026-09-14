"""Generic native-PX4 multicopter control-pipeline analysis."""

from __future__ import annotations

import math

import numpy as np
from pathlib import Path

from analysis.core import BagData
from analysis.plots import save_tracking_plot
from analysis.px4 import TOPICS, quaternion_to_euler


TRAJECTORY_TOPIC = TOPICS["OUT_TRAJECTORY_SETPOINT"]
LOCAL_POSITION_TOPIC = TOPICS["OUT_VEHICLE_LOCAL_POSITION"]
LOCAL_POSITION_SETPOINT_TOPIC = TOPICS[
    "OUT_VEHICLE_LOCAL_POSITION_SETPOINT"
]

ATTITUDE_TOPIC = TOPICS["OUT_VEHICLE_ATTITUDE"]
ATTITUDE_SETPOINT_TOPIC = TOPICS[
    "OUT_VEHICLE_ATTITUDE_SETPOINT"
]

RATES_SETPOINT_TOPIC = TOPICS["OUT_VEHICLE_RATES_SETPOINT"]
ANGULAR_VELOCITY_TOPIC = TOPICS[
    "OUT_VEHICLE_ANGULAR_VELOCITY"
]

THRUST_SETPOINT_TOPIC = TOPICS["OUT_VEHICLE_THRUST_SETPOINT"]
TORQUE_SETPOINT_TOPIC = TOPICS["OUT_VEHICLE_TORQUE_SETPOINT"]
ACTUATOR_MOTORS_TOPIC = TOPICS["OUT_ACTUATOR_MOTORS"]


CONTROL_PIPELINE_TOPICS = {
    TRAJECTORY_TOPIC,
    LOCAL_POSITION_TOPIC,
    LOCAL_POSITION_SETPOINT_TOPIC,
    ATTITUDE_TOPIC,
    ATTITUDE_SETPOINT_TOPIC,
    RATES_SETPOINT_TOPIC,
    ANGULAR_VELOCITY_TOPIC,
    THRUST_SETPOINT_TOPIC,
    TORQUE_SETPOINT_TOPIC,
    ACTUATOR_MOTORS_TOPIC,
}


def _finite3(values) -> bool:
    return (
        len(values) >= 3
        and all(
            math.isfinite(float(values[index]))
            for index in range(3)
        )
    )


def _series3(
    bag: BagData,
    topic: str,
    getter,
    *,
    valid=None,
    scale: float = 1.0,
) -> dict[str, list[float]]:
    """Extract a finite three-component time series from one PX4 topic."""
    selected = []

    for sample in bag.samples.get(topic, []):
        message = sample.message

        if valid is not None and not valid(message):
            continue

        values = getter(message)

        if not _finite3(values):
            continue

        selected.append((sample, values))

    return {
        "times_s": [
            bag.relative_seconds(sample.timestamp_ns)
            for sample, _ in selected
        ],
        "x": [
            float(values[0]) * scale
            for _, values in selected
        ],
        "y": [
            float(values[1]) * scale
            for _, values in selected
        ],
        "z": [
            float(values[2]) * scale
            for _, values in selected
        ],
    }


def _valid_position(message) -> bool:
    return (
        bool(message.xy_valid)
        and bool(message.z_valid)
        and all(
            math.isfinite(float(value))
            for value in (message.x, message.y, message.z)
        )
    )


def _valid_velocity(message) -> bool:
    return (
        bool(message.v_xy_valid)
        and bool(message.v_z_valid)
        and all(
            math.isfinite(float(value))
            for value in (message.vx, message.vy, message.vz)
        )
    )


def _attitude_series(
    bag: BagData,
    topic: str,
    field: str,
) -> dict[str, list[float]]:
    """Extract roll, pitch, yaw in degrees from a PX4 quaternion stream."""
    times_s = []
    roll = []
    pitch = []
    yaw = []

    for sample in bag.samples.get(topic, []):
        quaternion = getattr(sample.message, field)

        if len(quaternion) < 4:
            continue

        if not all(math.isfinite(float(value)) for value in quaternion):
            continue

        norm_squared = sum(float(value) ** 2 for value in quaternion)

        if norm_squared <= 1e-12:
            continue

        r, p, y = quaternion_to_euler(quaternion)

        times_s.append(
            bag.relative_seconds(sample.timestamp_ns)
        )
        roll.append(math.degrees(r))
        pitch.append(math.degrees(p))
        yaw.append(math.degrees(y))

    return {
        "times_s": times_s,
        "x": roll,
        "y": pitch,
        "z": yaw,
    }


def _motor_series(
    bag: BagData,
) -> dict[str, tuple[list[float], list[float]]]:
    """Extract finite PX4 control-allocation motor commands."""
    samples = bag.samples.get(ACTUATOR_MOTORS_TOPIC, [])

    if not samples:
        return {}

    motors = {}
    channel_count = len(samples[0].message.control)

    for index in range(channel_count):
        times_s = []
        values = []

        for sample in samples:
            value = float(sample.message.control[index])

            if not math.isfinite(value):
                continue

            times_s.append(
                bag.relative_seconds(sample.timestamp_ns)
            )
            values.append(value)

        if values:
            motors[f"motor {index + 1}"] = (
                times_s,
                values,
            )

    return motors


def analyze_control_pipeline(bag: BagData) -> dict[str, object]:
    """
    Extract the standard native-PX4 multicopter control pipeline.

    Missing topics simply produce empty series so this common analyzer can also
    be reused by experiments that intentionally bypass part of the pipeline.
    """
    actual_position = _series3(
        bag,
        LOCAL_POSITION_TOPIC,
        lambda msg: (msg.x, msg.y, msg.z),
        valid=_valid_position,
    )
    actual_velocity = _series3(
        bag,
        LOCAL_POSITION_TOPIC,
        lambda msg: (msg.vx, msg.vy, msg.vz),
        valid=_valid_velocity,
    )
    actual_acceleration = _series3(
        bag,
        LOCAL_POSITION_TOPIC,
        lambda msg: (msg.ax, msg.ay, msg.az),
    )

    trajectory_position = _series3(
        bag,
        TRAJECTORY_TOPIC,
        lambda msg: msg.position,
    )
    trajectory_velocity = _series3(
        bag,
        TRAJECTORY_TOPIC,
        lambda msg: msg.velocity,
    )
    trajectory_acceleration = _series3(
        bag,
        TRAJECTORY_TOPIC,
        lambda msg: msg.acceleration,
    )

    controller_position = _series3(
        bag,
        LOCAL_POSITION_SETPOINT_TOPIC,
        lambda msg: (msg.x, msg.y, msg.z),
    )
    controller_velocity = _series3(
        bag,
        LOCAL_POSITION_SETPOINT_TOPIC,
        lambda msg: (msg.vx, msg.vy, msg.vz),
    )
    controller_acceleration = _series3(
        bag,
        LOCAL_POSITION_SETPOINT_TOPIC,
        lambda msg: msg.acceleration,
    )

    attitude_actual = _attitude_series(
        bag,
        ATTITUDE_TOPIC,
        "q",
    )
    attitude_setpoint = _attitude_series(
        bag,
        ATTITUDE_SETPOINT_TOPIC,
        "q_d",
    )

    degrees = 180.0 / math.pi

    rate_actual = _series3(
        bag,
        ANGULAR_VELOCITY_TOPIC,
        lambda msg: msg.xyz,
        scale=degrees,
    )
    rate_setpoint = _series3(
        bag,
        RATES_SETPOINT_TOPIC,
        lambda msg: (msg.roll, msg.pitch, msg.yaw),
        scale=degrees,
    )

    thrust = _series3(
        bag,
        THRUST_SETPOINT_TOPIC,
        lambda msg: msg.xyz,
    )
    torque = _series3(
        bag,
        TORQUE_SETPOINT_TOPIC,
        lambda msg: msg.xyz,
    )

    return {
        "position": {
            "actual": actual_position,
            "trajectory": trajectory_position,
            "controller setpoint": controller_position,
        },
        "velocity": {
            "actual": actual_velocity,
            "trajectory": trajectory_velocity,
            "controller setpoint": controller_velocity,
        },
        "acceleration": {
            "actual": actual_acceleration,
            "trajectory": trajectory_acceleration,
            "controller setpoint": controller_acceleration,
        },
        "attitude": {
            "actual": attitude_actual,
            "setpoint": attitude_setpoint,
        },
        "rates": {
            "actual": rate_actual,
            "setpoint": rate_setpoint,
        },
        "thrust": {
            "setpoint": thrust,
        },
        "torque": {
            "setpoint": torque,
        },
        "motors": _motor_series(bag),
    }


def _components(
    section: dict[str, dict[str, list[float]]],
    names: list[tuple[str, str]],
):
    components = []

    for component_name, key in names:
        signals = {}

        for label, series in section.items():
            if series["times_s"]:
                signals[label] = (
                    series["times_s"],
                    series[key],
                )

        if signals:
            components.append((component_name, signals))

    return components



def _aligned_error_stats(
    actual: dict[str, list[float]],
    reference: dict[str, list[float]],
    component: str,
    *,
    wrap_degrees: bool = False,
) -> tuple[float, float] | None:
    """Return RMS and maximum absolute tracking error on common timestamps."""
    actual_times = np.asarray(actual["times_s"], dtype=float)
    reference_times = np.asarray(reference["times_s"], dtype=float)

    if actual_times.size == 0 or reference_times.size == 0:
        return None

    actual_values = np.asarray(actual[component], dtype=float)
    reference_values = np.asarray(reference[component], dtype=float)

    order = np.argsort(reference_times)
    reference_times = reference_times[order]
    reference_values = reference_values[order]

    reference_times, unique = np.unique(
        reference_times,
        return_index=True,
    )
    reference_values = reference_values[unique]

    start = max(actual_times[0], reference_times[0])
    end = min(actual_times[-1], reference_times[-1])

    mask = (
        (actual_times >= start)
        & (actual_times <= end)
    )

    if not np.any(mask):
        return None

    times = actual_times[mask]
    error = actual_values[mask] - np.interp(
        times,
        reference_times,
        reference_values,
    )

    if wrap_degrees:
        error = (error + 180.0) % 360.0 - 180.0

    return (
        float(np.sqrt(np.mean(error ** 2))),
        float(np.max(np.abs(error))),
    )


def _axis_range(
    series: dict[str, list[float]],
    component: str,
) -> tuple[float, float, float] | None:
    """Return minimum, maximum, and peak absolute value for one signal axis."""
    values = np.asarray(series[component], dtype=float)

    if values.size == 0:
        return None

    return (
        float(np.min(values)),
        float(np.max(values)),
        float(np.max(np.abs(values))),
    )


def control_pipeline_summary(
    pipeline: dict[str, object],
) -> list[str]:
    """Build a layer-by-layer summary of the native PX4 control pipeline."""

    def tracking_lines(
        actual,
        reference,
        components,
    ):
        result = [
            "                         RMS error    max |error|"
        ]

        for label, component, wrap in components:
            stats = _aligned_error_stats(
                actual,
                reference,
                component,
                wrap_degrees=wrap,
            )

            if stats is None:
                result.append(
                    f"  {label:<20} unavailable"
                )
                continue

            rms, maximum = stats
            result.append(
                f"  {label:<20} "
                f"{rms:>10.4f}    {maximum:>10.4f}"
            )

        return result

    def range_lines(series, components):
        result = [
            "                         minimum      maximum"
            "        mean    peak |value|"
        ]

        for label, component in components:
            values = np.asarray(
                series[component],
                dtype=float,
            )

            if values.size == 0:
                result.append(
                    f"  {label:<20} unavailable"
                )
                continue

            result.append(
                f"  {label:<20} "
                f"{np.min(values):>10.4f}   "
                f"{np.max(values):>10.4f}   "
                f"{np.mean(values):>10.4f}   "
                f"{np.max(np.abs(values)):>10.4f}"
            )

        return result

    lines = [
        "PX4 Control Pipeline Summary",
        "============================",
        "",
        "Layer 1 - Trajectory reference",
        "------------------------------",
        (
            "trajectory_setpoint samples: "
            f"{len(pipeline['position']['trajectory']['times_s'])}"
        ),
        "",
        "Position [m]",
        *range_lines(
            pipeline["position"]["trajectory"],
            [
                ("x / North", "x"),
                ("y / East", "y"),
                ("z / Down", "z"),
            ],
        ),
        "",
        "Velocity [m/s]",
        *range_lines(
            pipeline["velocity"]["trajectory"],
            [
                ("vx / North", "x"),
                ("vy / East", "y"),
                ("vz / Down", "z"),
            ],
        ),
        "",
        "Acceleration [m/s^2]",
        *range_lines(
            pipeline["acceleration"]["trajectory"],
            [
                ("ax / North", "x"),
                ("ay / East", "y"),
                ("az / Down", "z"),
            ],
        ),
        "",
        "Layer 2 - Position controller",
        "-----------------------------",
        (
            "vehicle_local_position_setpoint samples: "
            f"{len(pipeline['position']['controller setpoint']['times_s'])}"
        ),
        (
            "vehicle_local_position samples: "
            f"{len(pipeline['position']['actual']['times_s'])}"
        ),
        "",
        "Trajectory -> controller setpoint",
        "",
        "Position [m]",
        *tracking_lines(
            pipeline["position"]["controller setpoint"],
            pipeline["position"]["trajectory"],
            [
                ("x / North", "x", False),
                ("y / East", "y", False),
                ("z / Down", "z", False),
            ],
        ),
        "",
        "Velocity [m/s]",
        *tracking_lines(
            pipeline["velocity"]["controller setpoint"],
            pipeline["velocity"]["trajectory"],
            [
                ("vx / North", "x", False),
                ("vy / East", "y", False),
                ("vz / Down", "z", False),
            ],
        ),
        "",
        "Acceleration [m/s^2]",
        *tracking_lines(
            pipeline["acceleration"]["controller setpoint"],
            pipeline["acceleration"]["trajectory"],
            [
                ("ax / North", "x", False),
                ("ay / East", "y", False),
                ("az / Down", "z", False),
            ],
        ),
        "",
        "Controller setpoint -> actual",
        "",
        "Position [m]",
        *tracking_lines(
            pipeline["position"]["actual"],
            pipeline["position"]["controller setpoint"],
            [
                ("x / North", "x", False),
                ("y / East", "y", False),
                ("z / Down", "z", False),
            ],
        ),
        "",
        "Velocity [m/s]",
        *tracking_lines(
            pipeline["velocity"]["actual"],
            pipeline["velocity"]["controller setpoint"],
            [
                ("vx / North", "x", False),
                ("vy / East", "y", False),
                ("vz / Down", "z", False),
            ],
        ),
        "",
        "Acceleration [m/s^2]",
        *tracking_lines(
            pipeline["acceleration"]["actual"],
            pipeline["acceleration"]["controller setpoint"],
            [
                ("ax / North", "x", False),
                ("ay / East", "y", False),
                ("az / Down", "z", False),
            ],
        ),
        "",
        "Layer 3 - Attitude controller",
        "-----------------------------",
        (
            "vehicle_attitude_setpoint samples: "
            f"{len(pipeline['attitude']['setpoint']['times_s'])}"
        ),
        (
            "vehicle_attitude samples: "
            f"{len(pipeline['attitude']['actual']['times_s'])}"
        ),
        "",
        "Attitude tracking [deg]",
        *tracking_lines(
            pipeline["attitude"]["actual"],
            pipeline["attitude"]["setpoint"],
            [
                ("roll", "x", False),
                ("pitch", "y", False),
                ("yaw", "z", True),
            ],
        ),
        "",
        "Layer 4 - Rate controller",
        "-------------------------",
        (
            "vehicle_rates_setpoint samples: "
            f"{len(pipeline['rates']['setpoint']['times_s'])}"
        ),
        (
            "vehicle_angular_velocity samples: "
            f"{len(pipeline['rates']['actual']['times_s'])}"
        ),
        "",
        "Body-rate tracking [deg/s]",
        *tracking_lines(
            pipeline["rates"]["actual"],
            pipeline["rates"]["setpoint"],
            [
                ("p / roll", "x", False),
                ("q / pitch", "y", False),
                ("r / yaw", "z", False),
            ],
        ),
        "",
        "Layer 5 - Controller output",
        "---------------------------",
        (
            "vehicle_thrust_setpoint samples: "
            f"{len(pipeline['thrust']['setpoint']['times_s'])}"
        ),
        "",
        "Normalized thrust setpoint",
        *range_lines(
            pipeline["thrust"]["setpoint"],
            [
                ("x / body", "x"),
                ("y / body", "y"),
                ("z / body", "z"),
            ],
        ),
        "",
        (
            "vehicle_torque_setpoint samples: "
            f"{len(pipeline['torque']['setpoint']['times_s'])}"
        ),
        "",
        "Normalized torque setpoint",
        *range_lines(
            pipeline["torque"]["setpoint"],
            [
                ("x / roll", "x"),
                ("y / pitch", "y"),
                ("z / yaw", "z"),
            ],
        ),
        "",
        "Layer 6 - Control allocation",
        "----------------------------",
        f"Active motor channels: {len(pipeline['motors'])}",
    ]

    motor_sample_count = max(
        (
            len(times)
            for times, _ in pipeline["motors"].values()
        ),
        default=0,
    )

    lines.append(
        f"actuator_motors samples: {motor_sample_count}"
    )
    lines.append("")

    for label, (_times, values) in pipeline["motors"].items():
        array = np.asarray(values, dtype=float)

        if array.size == 0:
            continue

        lines.append(
            f"  {label:<12} "
            f"min {np.min(array):>8.4f}   "
            f"max {np.max(array):>8.4f}   "
            f"mean {np.mean(array):>8.4f}   "
            f"peak {np.max(np.abs(array)):>8.4f}"
        )

    return lines


def write_control_pipeline_plots(
    pipeline: dict[str, object],
    output_dir: Path,
    *,
    markers: list[tuple[float, str]] | None = None,
) -> list[Path]:
    """Write the standard PX4 controller and control-allocation plots."""
    output_dir.mkdir(parents=True, exist_ok=True)
    generated = []

    specifications = [
        (
            "position",
            "01_position_tracking.png",
            "PX4 Position Tracking (NED)",
            "[m]",
            [
                ("x / North", "x"),
                ("y / East", "y"),
                ("z / Down", "z"),
            ],
        ),
        (
            "velocity",
            "02_velocity_tracking.png",
            "PX4 Velocity Tracking (NED)",
            "[m/s]",
            [
                ("vx / North", "x"),
                ("vy / East", "y"),
                ("vz / Down", "z"),
            ],
        ),
        (
            "acceleration",
            "03_acceleration_tracking.png",
            "PX4 Acceleration Tracking (NED)",
            "[m/s²]",
            [
                ("ax / North", "x"),
                ("ay / East", "y"),
                ("az / Down", "z"),
            ],
        ),
        (
            "attitude",
            "04_attitude_tracking.png",
            "PX4 Attitude Tracking",
            "[deg]",
            [
                ("roll", "x"),
                ("pitch", "y"),
                ("yaw", "z"),
            ],
        ),
        (
            "rates",
            "05_rate_tracking.png",
            "PX4 Body-Rate Tracking (FRD)",
            "[deg/s]",
            [
                ("p / roll rate", "x"),
                ("q / pitch rate", "y"),
                ("r / yaw rate", "z"),
            ],
        ),
        (
            "thrust",
            "06_thrust_setpoint.png",
            "PX4 Thrust Setpoint (FRD)",
            "[normalized]",
            [
                ("x / body", "x"),
                ("y / body", "y"),
                ("z / body", "z"),
            ],
        ),
        (
            "torque",
            "07_torque_setpoint.png",
            "PX4 Torque Setpoint (FRD)",
            "[normalized]",
            [
                ("x / roll", "x"),
                ("y / pitch", "y"),
                ("z / yaw", "z"),
            ],
        ),
    ]

    for key, filename, title, unit, names in specifications:
        components = _components(pipeline[key], names)

        if not components:
            continue

        path = output_dir / filename
        save_tracking_plot(
            path,
            components=components,
            title=title,
            unit=unit,
            markers=markers,
        )
        generated.append(path)

    motors = pipeline["motors"]

    if motors:
        path = output_dir / "08_actuator_motors.png"
        save_tracking_plot(
            path,
            components=[
                ("motor command", motors),
            ],
            title="PX4 Control Allocation: Actuator Motors",
            unit="[normalized]",
            markers=markers,
        )
        generated.append(path)

    return generated
