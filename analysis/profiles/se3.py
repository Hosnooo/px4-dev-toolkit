"""Analysis profile for geometric SE3 trajectory tracking."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

from analysis.comparison import tracking_error_series
from analysis.core import (
    BagData,
    TimedSample,
    first_nav_state_time,
    mode_transitions,
    native_constant_names,
)
from analysis.pipeline import (
    CONTROL_PIPELINE_TOPICS,
    _aligned_error_stats,
    _attitude_series,
    _series3,
    analyze_control_pipeline,
    write_control_pipeline_plots,
)
from analysis.plots import (
    save_mode_timeline,
    save_tracking_plot,
)
from analysis.px4 import TOPICS


PROFILE_NAME = "se3"

STATUS_TOPIC = TOPICS["OUT_VEHICLE_STATUS_V1"]
OFFBOARD_MODE_TOPIC = TOPICS["IN_OFFBOARD_CONTROL_MODE"]

ACCELERATION_HANDOFF_TOPIC = TOPICS["IN_TRAJECTORY_SETPOINT"]
ATTITUDE_HANDOFF_TOPIC = TOPICS["IN_VEHICLE_ATTITUDE_SETPOINT"]
ATTITUDE_RATE_HANDOFF_TOPIC = TOPICS["IN_VEHICLE_RATES_SETPOINT"]
THRUST_HANDOFF_TOPIC = TOPICS["IN_VEHICLE_THRUST_SETPOINT"]
TORQUE_HANDOFF_TOPIC = TOPICS["IN_VEHICLE_TORQUE_SETPOINT"]

HANDOFF_TOPICS = (
    ACCELERATION_HANDOFF_TOPIC,
    ATTITUDE_HANDOFF_TOPIC,
    ATTITUDE_RATE_HANDOFF_TOPIC,
    THRUST_HANDOFF_TOPIC,
    TORQUE_HANDOFF_TOPIC,
)

HANDOFF_MODE_TOPICS = {
    "acceleration": (ACCELERATION_HANDOFF_TOPIC,),
    "attitude": (ATTITUDE_HANDOFF_TOPIC,),
    "attitude_rate": (ATTITUDE_RATE_HANDOFF_TOPIC,),
    "thrust_and_torque": (THRUST_HANDOFF_TOPIC, TORQUE_HANDOFF_TOPIC),
}

OFFBOARD_MODE_FIELDS = {
    "acceleration": "acceleration",
    "attitude": "attitude",
    "attitude_rate": "body_rate",
    "thrust_and_torque": "thrust_and_torque",
}

TRAJECTORY_REFERENCE_TOPIC = (
    "/px4_toolkit/se3/trajectory_reference"
)

DIRECT_RATE_COMMAND_TOPIC = (
    "/px4_toolkit/se3/rate_command"
)

TORQUE_RATE_FEEDBACK_TOPIC = (
    "/px4_toolkit/se3/torque_rate_feedback"
)

TORQUE_ANGULAR_ACCELERATION_FEEDFORWARD_TOPIC = (
    "/px4_toolkit/se3/torque_angular_acceleration_feedforward"
)

REQUIRED_TOPICS = {
    STATUS_TOPIC,
    OFFBOARD_MODE_TOPIC,
    *HANDOFF_TOPICS,
    TRAJECTORY_REFERENCE_TOPIC,
    *CONTROL_PIPELINE_TOPICS,
}

OPTIONAL_TOPICS = {
    DIRECT_RATE_COMMAND_TOPIC,
    TORQUE_RATE_FEEDBACK_TOPIC,
    TORQUE_ANGULAR_ACCELERATION_FEEDFORWARD_TOPIC,
}


@dataclass
class AnalysisResult:
    summary: str
    metrics: dict[str, float | bool | str]
    plot_data: dict[str, object]


def _required(
    bag: BagData,
    topic: str,
) -> list[TimedSample]:
    samples = bag.samples[topic]

    if not samples:
        raise RuntimeError(
            f"No samples recorded on {topic}"
        )

    return samples


def _handoff_samples(
    bag: BagData,
    offboard_mode: list[TimedSample],
) -> tuple[str, list[tuple[str, list[TimedSample]]]]:
    active_modes = set()

    for sample in offboard_mode:
        active = [
            mode
            for mode, field in OFFBOARD_MODE_FIELDS.items()
            if bool(getattr(sample.message, field, False))
        ]

        if len(active) > 1:
            raise RuntimeError(
                "SE3 OffboardControlMode enables multiple handoff levels"
            )

        if active:
            active_modes.add(active[0])

    if not active_modes:
        raise RuntimeError(
            "SE3 OffboardControlMode does not identify a handoff level"
        )

    if len(active_modes) > 1:
        raise RuntimeError(
            "SE3 bag contains multiple Offboard handoff modes"
        )

    handoff_mode = next(iter(active_modes))
    inputs = []

    for topic in HANDOFF_MODE_TOPICS[handoff_mode]:
        samples = bag.samples.get(topic, [])

        if not samples:
            raise RuntimeError(
                f"No SE3 {handoff_mode} handoff samples recorded on {topic}"
            )

        inputs.append((topic, samples))

    return handoff_mode, inputs


def _first_armed_time(
    samples: list[TimedSample],
    armed_state: int,
) -> int | None:
    for sample in samples:
        if int(sample.message.arming_state) == armed_state:
            return sample.timestamp_ns

    return None


def extract_se3_layers(
    bag: BagData,
    handoff_mode: str = "acceleration",
) -> dict[str, object]:
    """Extract toolkit-owned trajectory and selected handoff signals."""

    trajectory_samples = bag.samples[TRAJECTORY_REFERENCE_TOPIC]

    reference = {
        "position": _series3(
            bag,
            TRAJECTORY_REFERENCE_TOPIC,
            lambda msg: msg.position,
        ),
        "velocity": _series3(
            bag,
            TRAJECTORY_REFERENCE_TOPIC,
            lambda msg: msg.velocity,
        ),
        "acceleration": _series3(
            bag,
            TRAJECTORY_REFERENCE_TOPIC,
            lambda msg: msg.acceleration,
        ),
        "jerk": None,
    }

    if any(
        hasattr(sample.message, "jerk")
        for sample in trajectory_samples
    ):
        reference["jerk"] = _series3(
            bag,
            TRAJECTORY_REFERENCE_TOPIC,
            lambda msg: msg.jerk,
        )

    if handoff_mode == "acceleration":
        handoff = {
            "acceleration": _series3(
                bag,
                ACCELERATION_HANDOFF_TOPIC,
                lambda msg: msg.acceleration,
            ),
        }

    elif handoff_mode == "attitude":
        handoff = {
            "attitude": _attitude_series(
                bag,
                ATTITUDE_HANDOFF_TOPIC,
                "q_d",
            ),
        }

    elif handoff_mode == "attitude_rate":
        handoff = {
            "rates": _series3(
                bag,
                ATTITUDE_RATE_HANDOFF_TOPIC,
                lambda msg: (msg.roll, msg.pitch, msg.yaw),
                scale=180.0 / 3.14159265358979323846,
            ),
        }

    elif handoff_mode == "thrust_and_torque":
        handoff = {
            "thrust": _series3(
                bag,
                THRUST_HANDOFF_TOPIC,
                lambda msg: msg.xyz,
            ),
            "torque": _series3(
                bag,
                TORQUE_HANDOFF_TOPIC,
                lambda msg: msg.xyz,
            ),
        }

    else:
        raise RuntimeError(
            f"Unsupported SE3 handoff mode: {handoff_mode}"
        )

    return {
        "reference": reference,
        "handoff": handoff,
    }


def extract_direct_torque_diagnostics(
    bag: BagData,
) -> dict[str, object]:
    """Extract optional toolkit-owned direct-wrench diagnostics."""
    diagnostics = {}

    if bag.samples.get(DIRECT_RATE_COMMAND_TOPIC):
        diagnostics["rate_command"] = _series3(
            bag,
            DIRECT_RATE_COMMAND_TOPIC,
            lambda msg: (
                msg.roll,
                msg.pitch,
                msg.yaw,
            ),
            scale=180.0 / 3.14159265358979323846,
        )

    if bag.samples.get(TORQUE_RATE_FEEDBACK_TOPIC):
        diagnostics["rate_feedback"] = _series3(
            bag,
            TORQUE_RATE_FEEDBACK_TOPIC,
            lambda msg: msg.xyz,
        )

    if bag.samples.get(
        TORQUE_ANGULAR_ACCELERATION_FEEDFORWARD_TOPIC
    ):
        diagnostics[
            "angular_acceleration_feedforward"
        ] = _series3(
            bag,
            TORQUE_ANGULAR_ACCELERATION_FEEDFORWARD_TOPIC,
            lambda msg: msg.xyz,
        )

    return diagnostics


def _message_count(series: dict[str, object]) -> int:
    return int(
        series.get(
            "message_count",
            len(series["times_s"]),
        )
    )


def _finite_count(series: dict[str, object]) -> int:
    return int(
        series.get(
            "finite_count",
            len(series["times_s"]),
        )
    )


def _tracking_lines(
    actual,
    reference,
    components,
) -> list[str]:
    result = [
        "                         RMS value    max |value|"
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


def _se3_summary(
    layers: dict[str, object],
    pipeline: dict[str, object],
    handoff_mode: str,
) -> list[str]:
    reference_position = layers["reference"]["position"]
    reference_velocity = layers["reference"]["velocity"]
    reference_acceleration = layers["reference"]["acceleration"]

    actual_position = pipeline["position"]["actual"]
    actual_velocity = pipeline["velocity"]["actual"]
    attitude_actual = pipeline["attitude"]["actual"]
    attitude_setpoint = pipeline["attitude"]["setpoint"]
    rate_actual = pipeline["rates"]["actual"]
    rate_setpoint = pipeline["rates"]["setpoint"]
    thrust = pipeline["thrust"]["setpoint"]
    torque = pipeline["torque"]["setpoint"]

    motor_sample_count = max(
        (
            len(times)
            for times, _ in pipeline["motors"].values()
        ),
        default=0,
    )

    title = f"SE3 {handoff_mode}-Handoff Pipeline Summary"
    lines = [
        title,
        "=" * len(title),
        "",
        "Layer 1 - Generated trajectory reference",
        "----------------------------------------",
        (
            "generated reference messages: "
            f"{_message_count(reference_position)}"
        ),
        (
            "finite position references: "
            f"{_finite_count(reference_position)}"
        ),
        (
            "finite velocity references: "
            f"{_finite_count(reference_velocity)}"
        ),
        (
            "finite acceleration references: "
            f"{_finite_count(reference_acceleration)}"
        ),
        "",
        "Position tracking [m]",
        *_tracking_lines(
            actual_position,
            reference_position,
            [
                ("x / North", "x", False),
                ("y / East", "y", False),
                ("z / Down", "z", False),
            ],
        ),
        "",
        "Velocity tracking [m/s]",
        *_tracking_lines(
            actual_velocity,
            reference_velocity,
            [
                ("vx / North", "x", False),
                ("vy / East", "y", False),
                ("vz / Down", "z", False),
            ],
        ),
        "",
        "Layer 2 - Toolkit handoff",
        "-------------------------",
    ]

    handoff = layers["handoff"]

    if handoff_mode == "acceleration":
        handoff_acceleration = handoff["acceleration"]
        lines.extend(
            [
                (
                    "acceleration handoff messages: "
                    f"{_message_count(handoff_acceleration)}"
                ),
                (
                    "finite acceleration handoff samples: "
                    f"{_finite_count(handoff_acceleration)}"
                ),
                "",
                (
                    "SE3 feedback correction "
                    "(handoff - generated feed-forward) [m/s^2]"
                ),
                *_tracking_lines(
                    handoff_acceleration,
                    reference_acceleration,
                    [
                        ("ax / North", "x", False),
                        ("ay / East", "y", False),
                        ("az / Down", "z", False),
                    ],
                ),
            ]
        )
    elif handoff_mode == "attitude":
        attitude = handoff["attitude"]
        lines.extend(
            [
                f"attitude handoff samples: {len(attitude['times_s'])}",
                "",
                "Attitude handoff tracking [deg]",
                *_tracking_lines(
                    attitude_actual,
                    attitude,
                    [
                        ("roll", "x", False),
                        ("pitch", "y", False),
                        ("yaw", "z", True),
                    ],
                ),
            ]
        )
    elif handoff_mode == "attitude_rate":
        rates = handoff["rates"]
        lines.extend(
            [
                f"body-rate handoff samples: {_message_count(rates)}",
                "",
                "Body-rate handoff tracking [deg/s]",
                *_tracking_lines(
                    rate_actual,
                    rates,
                    [
                        ("p / roll", "x", False),
                        ("q / pitch", "y", False),
                        ("r / yaw", "z", False),
                    ],
                ),
            ]
        )
    else:
        handoff_thrust = handoff["thrust"]
        handoff_torque = handoff["torque"]
        lines.extend(
            [
                (
                    "normalized thrust handoff messages: "
                    f"{_message_count(handoff_thrust)}"
                ),
                (
                    "normalized torque handoff messages: "
                    f"{_message_count(handoff_torque)}"
                ),
            ]
        )

    lines.extend(
        [
            "",
            "PX4 downstream control pipeline",
            "-------------------------------",
            (
                "vehicle_attitude_setpoint samples: "
                f"{len(attitude_setpoint['times_s'])}"
            ),
            (
                "vehicle_rates_setpoint samples: "
                f"{_message_count(rate_setpoint)}"
            ),
            (
                "vehicle_angular_velocity samples: "
                f"{_message_count(rate_actual)}"
            ),
            (
                "vehicle_thrust_setpoint messages: "
                f"{_message_count(thrust)}"
            ),
            (
                "vehicle_torque_setpoint messages: "
                f"{_message_count(torque)}"
            ),
            f"Active motor channels: {len(pipeline['motors'])}",
            f"actuator_motors samples: {motor_sample_count}",
        ]
    )

    return lines


def analyze(bag: BagData) -> AnalysisResult:
    """Analyze the selected SE3 handoff through the PX4 control pipeline."""

    status = _required(bag, STATUS_TOPIC)
    offboard_mode = _required(bag, OFFBOARD_MODE_TOPIC)
    handoff_mode, handoff_inputs = _handoff_samples(
        bag,
        offboard_mode,
    )
    handoff_topics = [topic for topic, _ in handoff_inputs]
    handoff_topic = " + ".join(handoff_topics)
    reference = _required(
        bag,
        TRAJECTORY_REFERENCE_TOPIC,
    )

    status_type = type(status[0].message)

    armed_state = int(
        status_type.ARMING_STATE_ARMED
    )
    offboard_state = int(
        status_type.NAVIGATION_STATE_OFFBOARD
    )

    position_state = int(
        status_type.NAVIGATION_STATE_POSCTL
    )

    started_armed = (
        int(status[0].message.arming_state)
        == armed_state
    )

    armed_ns = _first_armed_time(
        status,
        armed_state,
    )

    if armed_ns is None:
        raise RuntimeError(
            "Armed state was not recorded"
        )

    offboard_ns = first_nav_state_time(
        status,
        offboard_state,
    )

    if offboard_ns is None:
        raise RuntimeError(
            "Offboard-mode entry was not recorded"
        )

    position_return_ns = first_nav_state_time(
        status,
        position_state,
        not_before_ns=offboard_ns,
    )

    if position_return_ns is None:
        raise RuntimeError(
            "Position-mode return after Offboard was not recorded"
        )

    offboard_mode_start_ns = offboard_mode[0].timestamp_ns
    reference_start_ns = reference[0].timestamp_ns
    handoff_start_ns = min(
        samples[0].timestamp_ns
        for _, samples in handoff_inputs
    )

    if offboard_mode_start_ns >= offboard_ns:
        raise RuntimeError(
            "OffboardControlMode prestream did not precede "
            "Offboard entry"
        )

    if reference_start_ns >= offboard_ns:
        raise RuntimeError(
            "Generated trajectory-reference prestream did not "
            "precede Offboard entry"
        )

    metrics = {
        "bag_duration_s": bag.duration_s,
        "handoff": handoff_mode,
        "handoff_topic": handoff_topic,
        "started_armed": started_armed,
        "armed_s": bag.relative_seconds(armed_ns),
        "offboard_mode_start_s": bag.relative_seconds(
            offboard_mode_start_ns
        ),
        "reference_start_s": bag.relative_seconds(
            reference_start_ns
        ),
        "offboard_entry_s": bag.relative_seconds(
            offboard_ns
        ),
        "handoff_start_s": bag.relative_seconds(
            handoff_start_ns
        ),
        "position_return_s": bag.relative_seconds(
            position_return_ns
        ),
        "offboard_mode_prestream_s": (
            offboard_ns - offboard_mode_start_ns
        ) / 1e9,
        "reference_prestream_s": (
            offboard_ns - reference_start_ns
        ) / 1e9,
        "handoff_offset_from_offboard_s": (
            handoff_start_ns - offboard_ns
        ) / 1e9,
        "offboard_duration_s": (
            position_return_ns - offboard_ns
        ) / 1e9,
    }

    state_names = native_constant_names(
        status_type,
        "NAVIGATION_STATE_",
    )

    transitions = mode_transitions(status)

    # Controller-pipeline metrics begin only after PX4 actually enters
    # Offboard. Lifecycle analysis above still uses the complete bag.
    tracking_bag = BagData(
        path=bag.path,
        start_ns=bag.start_ns,
        end_ns=bag.end_ns,
        samples={
            topic: [
                sample
                for sample in samples
                if sample.timestamp_ns >= offboard_ns
            ]
            for topic, samples in bag.samples.items()
        },
    )

    layers = extract_se3_layers(
        tracking_bag,
        handoff_mode,
    )

    # Keep the native PX4 pipeline on its native topics. In acceleration
    # handoff, OUT_TRAJECTORY_SETPOINT is not the toolkit's generated
    # geometric trajectory reference.
    pipeline = analyze_control_pipeline(
        tracking_bag
    )

    torque_diagnostics = (
        extract_direct_torque_diagnostics(
            tracking_bag
        )
    )

    lines = [
        f"Profile: {PROFILE_NAME}",
        f"Bag: {bag.path}",
        f"Bag duration: {bag.duration_s:.3f} s",
        "",
        "PX4 navigation-state transitions:",
    ]

    for timestamp_ns, state in transitions:
        lines.append(
            f"  {bag.relative_seconds(timestamp_ns):8.3f} s  "
            f"{state_names.get(state, f'STATE_{state}')} ({state})"
        )

    lines.extend(
        [
            "",
            "SE3 Offboard lifecycle:",
            f"  handoff input: {handoff_topic}",
            f"  started armed: "
            f"{'yes' if started_armed else 'no'}",
            f"  armed state first recorded: "
            f"{metrics['armed_s']:.3f} s",
            f"  OffboardControlMode start: "
            f"{metrics['offboard_mode_start_s']:.3f} s",
            f"  generated reference start: "
            f"{metrics['reference_start_s']:.3f} s",
            f"  Offboard entry: "
            f"{metrics['offboard_entry_s']:.3f} s",
            f"  OffboardControlMode before entry: "
            f"{metrics['offboard_mode_prestream_s']:.3f} s",
            f"  generated reference before entry: "
            f"{metrics['reference_prestream_s']:.3f} s",
            f"  selected handoff start: "
            f"{metrics['handoff_start_s']:.3f} s",
            f"  handoff offset from Offboard entry: "
            f"{metrics['handoff_offset_from_offboard_s']:+.3f} s",
            f"  Position return: "
            f"{metrics['position_return_s']:.3f} s",
            f"  time in Offboard: "
            f"{metrics['offboard_duration_s']:.3f} s",
            "",
            *_se3_summary(
                layers,
                pipeline,
                handoff_mode,
            ),
        ]
    )

    markers = [
        (
            metrics["reference_start_s"],
            "REFERENCE PRESTREAM",
        ),
        (
            metrics["offboard_entry_s"],
            "OFFBOARD ENTRY",
        ),
        (
            metrics["handoff_start_s"],
            "HANDOFF START",
        ),
        (
            metrics["position_return_s"],
            "POSITION RETURN",
        ),
    ]

    if not started_armed:
        markers.insert(
            0,
            (
                metrics["armed_s"],
                "ARMED",
            ),
        )

    return AnalysisResult(
        summary="\n".join(lines) + "\n",
        metrics=metrics,
        plot_data={
            "layers": layers,
            "pipeline": pipeline,
            "torque_diagnostics": torque_diagnostics,
            "handoff_mode": handoff_mode,
            "markers": markers,
            "state_names": state_names,
            "transitions": [
                (
                    bag.relative_seconds(timestamp_ns),
                    state,
                )
                for timestamp_ns, state in transitions
            ],
        },
    )


def _vector_components(
    series_by_label,
    component_names,
):
    components = []

    for component_name, key in component_names:
        signals = {}

        for label, series in series_by_label.items():
            if series["times_s"]:
                signals[label] = (
                    series["times_s"],
                    series[key],
                )

        if signals:
            components.append(
                (component_name, signals)
            )

    return components


def write_plots(
    result: AnalysisResult,
    output_dir: Path,
) -> list[Path]:
    output_dir.mkdir(
        parents=True,
        exist_ok=True,
    )

    for old_plot in output_dir.glob("*.png"):
        old_plot.unlink()

    pipeline = result.plot_data["pipeline"]
    layers = result.plot_data["layers"]
    markers = result.plot_data["markers"]

    # Reuse the existing downstream PX4 plots. The first three are replaced
    # below with SE3-specific plots whose signal ownership is explicit.
    generated = write_control_pipeline_plots(
        pipeline,
        output_dir,
        markers=markers,
    )

    save_tracking_plot(
        output_dir / "01_position_tracking.png",
        components=_vector_components(
            {
                "actual": pipeline["position"]["actual"],
                "trajectory": layers["reference"]["position"],
            },
            [
                ("x / North", "x"),
                ("y / East", "y"),
                ("z / Down", "z"),
            ],
        ),
        title="SE3 Desired Position vs Vehicle Position (NED)",
        unit="[m]",
        markers=markers,
    )

    save_tracking_plot(
        output_dir / "02_velocity_tracking.png",
        components=_vector_components(
            {
                "actual": pipeline["velocity"]["actual"],
                "trajectory": layers["reference"]["velocity"],
            },
            [
                ("vx / North", "x"),
                ("vy / East", "y"),
                ("vz / Down", "z"),
            ],
        ),
        title="SE3 Desired Velocity vs Vehicle Velocity (NED)",
        unit="[m/s]",
        markers=markers,
    )

    acceleration_signals = {
        "generated feed-forward":
            layers["reference"]["acceleration"],
        "PX4 internal":
            pipeline["acceleration"]["controller setpoint"],
        "actual":
            pipeline["acceleration"]["actual"],
    }

    if "acceleration" in layers["handoff"]:
        acceleration_signals["SE3 handoff"] = (
            layers["handoff"]["acceleration"]
        )

    save_tracking_plot(
        output_dir / "03_acceleration_tracking.png",
        components=_vector_components(
            acceleration_signals,
            [
                ("ax / North", "x"),
                ("ay / East", "y"),
                ("az / Down", "z"),
            ],
        ),
        title="SE3 / PX4 Acceleration Layers (NED)",
        unit="[m/s²]",
        markers=markers,
    )

    mode_path = (
        output_dir / "09_mode_timeline.png"
    )

    save_mode_timeline(
        mode_path,
        transitions=result.plot_data["transitions"],
        state_names=result.plot_data["state_names"],
    )

    if mode_path not in generated:
        generated.append(mode_path)

    diagnostics = result.plot_data.get(
        "torque_diagnostics",
        {},
    )

    rate_command = diagnostics.get(
        "rate_command"
    )

    if rate_command:
        rate_path = (
            output_dir
            / "10_direct_rate_tracking.png"
        )

        save_tracking_plot(
            rate_path,
            components=_vector_components(
                {
                    "command": rate_command,
                    "actual":
                        pipeline["rates"]["actual"],
                },
                [
                    ("p / roll", "x"),
                    ("q / pitch", "y"),
                    ("r / yaw", "z"),
                ],
            ),
            title=(
                "Direct Wrench Rate Command "
                "vs Measured Body Rate"
            ),
            unit="[deg/s]",
            markers=markers,
        )

        generated.append(rate_path)

        rate_error = tracking_error_series(
            pipeline["rates"]["actual"],
            rate_command,
            time_origin_s=0.0,
        )

        error_path = (
            output_dir
            / "11_direct_rate_error.png"
        )

        save_tracking_plot(
            error_path,
            components=_vector_components(
                {
                    "actual - command":
                        rate_error,
                },
                [
                    ("p / roll", "x"),
                    ("q / pitch", "y"),
                    ("r / yaw", "z"),
                ],
            ),
            title=(
                "Direct Wrench Body-Rate Error"
            ),
            unit="[deg/s]",
            markers=markers,
        )

        generated.append(error_path)

    rate_feedback = diagnostics.get(
        "rate_feedback"
    )
    angular_acceleration_feedforward = (
        diagnostics.get(
            "angular_acceleration_feedforward"
        )
    )

    if (
        rate_feedback
        and angular_acceleration_feedforward
        and "torque" in layers["handoff"]
    ):
        torque_path = (
            output_dir
            / "12_direct_torque_decomposition.png"
        )

        save_tracking_plot(
            torque_path,
            components=_vector_components(
                {
                    "rate feedback":
                        rate_feedback,
                    "angular-acceleration feed-forward":
                        angular_acceleration_feedforward,
                    "final direct command":
                        layers["handoff"]["torque"],
                },
                [
                    ("roll / x", "x"),
                    ("pitch / y", "y"),
                    ("yaw / z", "z"),
                ],
            ),
            title=(
                "Direct Wrench Normalized "
                "Torque Decomposition"
            ),
            unit="[normalized]",
            markers=markers,
        )

        generated.append(torque_path)

    return generated
