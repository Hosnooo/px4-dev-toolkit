"""Analysis profile for Position-mode takeoff and Offboard handoff."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

from analysis.core import (
    BagData,
    TimedSample,
    first_nav_state_time,
    mode_transitions,
    native_constant_names,
)
from analysis.pipeline import (
    CONTROL_PIPELINE_TOPICS,
    analyze_control_pipeline,
    control_pipeline_summary,
    write_control_pipeline_plots,
)
from analysis.plots import save_mode_timeline
from analysis.px4 import TOPICS


PROFILE_NAME = "offboard_takeoff_handoff"

STATUS_TOPIC = TOPICS["OUT_VEHICLE_STATUS_V1"]
MANUAL_CONTROL_TOPIC = TOPICS["OUT_MANUAL_CONTROL_SETPOINT"]
OFFBOARD_MODE_TOPIC = TOPICS["IN_OFFBOARD_CONTROL_MODE"]
TRAJECTORY_INPUT_TOPIC = TOPICS["IN_TRAJECTORY_SETPOINT"]

REQUIRED_TOPICS = {
    STATUS_TOPIC,
    MANUAL_CONTROL_TOPIC,
    OFFBOARD_MODE_TOPIC,
    TRAJECTORY_INPUT_TOPIC,
    *CONTROL_PIPELINE_TOPICS,
}


@dataclass
class AnalysisResult:
    summary: str
    metrics: dict[str, float]
    plot_data: dict[str, object]


def _required(bag: BagData, topic: str) -> list[TimedSample]:
    samples = bag.samples[topic]

    if not samples:
        raise RuntimeError(f"No samples recorded on {topic}")

    return samples


def _first_arming_time(
    samples: list[TimedSample],
    armed_state: int,
    *,
    not_before_ns: int,
) -> int | None:
    for sample in samples:
        if sample.timestamp_ns < not_before_ns:
            continue

        if int(sample.message.arming_state) == armed_state:
            return sample.timestamp_ns

    return None


def _first_throttle_time(
    samples: list[TimedSample],
    predicate,
    *,
    not_before_ns: int,
) -> int | None:
    for sample in samples:
        if sample.timestamp_ns < not_before_ns:
            continue

        if predicate(float(sample.message.throttle)):
            return sample.timestamp_ns

    return None


def analyze(bag: BagData) -> AnalysisResult:
    """Analyze Position staging, Offboard takeover, and the generic pipeline."""

    status = _required(bag, STATUS_TOPIC)
    manual = _required(bag, MANUAL_CONTROL_TOPIC)
    offboard_mode = _required(bag, OFFBOARD_MODE_TOPIC)
    trajectory_input = _required(bag, TRAJECTORY_INPUT_TOPIC)

    status_type = type(status[0].message)
    position_state = int(status_type.NAVIGATION_STATE_POSCTL)
    offboard_state = int(status_type.NAVIGATION_STATE_OFFBOARD)
    armed_state = int(status_type.ARMING_STATE_ARMED)

    position_ns = first_nav_state_time(status, position_state)

    if position_ns is None:
        raise RuntimeError("Position-mode entry was not recorded")

    armed_ns = _first_arming_time(
        status,
        armed_state,
        not_before_ns=position_ns,
    )

    if armed_ns is None:
        raise RuntimeError("Armed state was not recorded")

    climb_ns = _first_throttle_time(
        manual,
        lambda value: value > 0.10,
        not_before_ns=armed_ns,
    )

    if climb_ns is None:
        raise RuntimeError("Climb-stick command was not recorded")

    centered_ns = _first_throttle_time(
        manual,
        lambda value: abs(value) <= 0.05,
        not_before_ns=climb_ns,
    )

    if centered_ns is None:
        raise RuntimeError("Centered-stick staging state was not recorded")

    offboard_ns = first_nav_state_time(
        status,
        offboard_state,
        not_before_ns=centered_ns,
    )

    if offboard_ns is None:
        raise RuntimeError("Offboard takeover after staging was not recorded")

    prestream_ns = min(
        offboard_mode[0].timestamp_ns,
        trajectory_input[0].timestamp_ns,
    )

    if prestream_ns >= offboard_ns:
        raise RuntimeError("Offboard prestream did not precede Offboard entry")

    metrics = {
        "bag_duration_s": bag.duration_s,
        "position_entry_s": bag.relative_seconds(position_ns),
        "armed_s": bag.relative_seconds(armed_ns),
        "climb_start_s": bag.relative_seconds(climb_ns),
        "centered_s": bag.relative_seconds(centered_ns),
        "prestream_start_s": bag.relative_seconds(prestream_ns),
        "offboard_entry_s": bag.relative_seconds(offboard_ns),
        "centered_to_offboard_s": (offboard_ns - centered_ns) / 1e9,
        "prestream_to_offboard_s": (offboard_ns - prestream_ns) / 1e9,
    }

    state_names = native_constant_names(status_type, "NAVIGATION_STATE_")
    transitions = mode_transitions(status)
    pipeline = analyze_control_pipeline(bag)

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
            "Position-mode staging and Offboard handoff:",
            f"  Position-mode entry: {metrics['position_entry_s']:.3f} s",
            f"  armed: {metrics['armed_s']:.3f} s",
            f"  climb command: {metrics['climb_start_s']:.3f} s",
            f"  sticks centered: {metrics['centered_s']:.3f} s",
            f"  Offboard prestream start: {metrics['prestream_start_s']:.3f} s",
            f"  Offboard entry: {metrics['offboard_entry_s']:.3f} s",
            f"  centered -> Offboard: {metrics['centered_to_offboard_s']:.3f} s",
            f"  prestream -> Offboard: {metrics['prestream_to_offboard_s']:.3f} s",
            "",
            *control_pipeline_summary(pipeline),
        ]
    )

    markers = [
        (metrics["position_entry_s"], "POSCTL ENTRY"),
        (metrics["armed_s"], "ARMED"),
        (metrics["centered_s"], "STICKS CENTERED"),
        (metrics["prestream_start_s"], "OFFBOARD PRESTREAM"),
        (metrics["offboard_entry_s"], "OFFBOARD ENTRY"),
    ]

    return AnalysisResult(
        summary="\n".join(lines) + "\n",
        metrics=metrics,
        plot_data={
            "pipeline": pipeline,
            "markers": markers,
            "state_names": state_names,
            "transitions": [
                (bag.relative_seconds(timestamp_ns), state)
                for timestamp_ns, state in transitions
            ],
        },
    )


def write_plots(
    result: AnalysisResult,
    output_dir: Path,
) -> list[Path]:
    output_dir.mkdir(parents=True, exist_ok=True)

    for old_plot in output_dir.glob("*.png"):
        old_plot.unlink()

    generated = write_control_pipeline_plots(
        result.plot_data["pipeline"],
        output_dir,
        markers=result.plot_data["markers"],
    )

    mode_path = output_dir / "09_mode_timeline.png"
    save_mode_timeline(
        mode_path,
        transitions=result.plot_data["transitions"],
        state_names=result.plot_data["state_names"],
    )
    generated.append(mode_path)

    return generated
