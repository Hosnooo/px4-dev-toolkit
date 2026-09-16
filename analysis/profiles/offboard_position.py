"""Analysis profile for the PX4 Offboard position experiment."""

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


PROFILE_NAME = "offboard_position"

STATUS_TOPIC = TOPICS["OUT_VEHICLE_STATUS_V1"]
OFFBOARD_MODE_TOPIC = TOPICS["IN_OFFBOARD_CONTROL_MODE"]
TRAJECTORY_INPUT_TOPIC = TOPICS["IN_TRAJECTORY_SETPOINT"]

REQUIRED_TOPICS = {
    STATUS_TOPIC,
    OFFBOARD_MODE_TOPIC,
    TRAJECTORY_INPUT_TOPIC,
    *CONTROL_PIPELINE_TOPICS,
}


@dataclass
class AnalysisResult:
    summary: str
    metrics: dict[str, float | bool]
    plot_data: dict[str, object]


def _required(bag: BagData, topic: str) -> list[TimedSample]:
    samples = bag.samples[topic]

    if not samples:
        raise RuntimeError(f"No samples recorded on {topic}")

    return samples


def _first_arming_time(
    samples: list[TimedSample],
    armed_state: int,
) -> int | None:
    for sample in samples:
        if int(sample.message.arming_state) == armed_state:
            return sample.timestamp_ns

    return None


def analyze(bag: BagData) -> AnalysisResult:
    """Analyze Offboard entry plus the generic PX4 control pipeline."""

    status = _required(bag, STATUS_TOPIC)
    offboard_mode = _required(bag, OFFBOARD_MODE_TOPIC)
    trajectory_input = _required(bag, TRAJECTORY_INPUT_TOPIC)

    status_type = type(status[0].message)
    armed_state = int(status_type.ARMING_STATE_ARMED)
    offboard_state = int(status_type.NAVIGATION_STATE_OFFBOARD)

    started_armed = int(status[0].message.arming_state) == armed_state
    armed_ns = _first_arming_time(status, armed_state)

    if armed_ns is None:
        raise RuntimeError("Armed state was not recorded")

    offboard_ns = first_nav_state_time(status, offboard_state)

    if offboard_ns is None:
        raise RuntimeError("Offboard-mode entry was not recorded")

    prestream_ns = min(
        offboard_mode[0].timestamp_ns,
        trajectory_input[0].timestamp_ns,
    )

    if prestream_ns >= offboard_ns:
        raise RuntimeError("Offboard prestream did not precede Offboard entry")

    metrics = {
        "bag_duration_s": bag.duration_s,
        "started_armed": started_armed,
        "armed_s": bag.relative_seconds(armed_ns),
        "prestream_start_s": bag.relative_seconds(prestream_ns),
        "offboard_entry_s": bag.relative_seconds(offboard_ns),
        "prestream_duration_s": (offboard_ns - prestream_ns) / 1e9,
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
            "Offboard lifecycle:",
            f"  started armed: {'yes' if started_armed else 'no'}",
            f"  armed state first recorded: {metrics['armed_s']:.3f} s",
            f"  prestream start: {metrics['prestream_start_s']:.3f} s",
            f"  Offboard entry: {metrics['offboard_entry_s']:.3f} s",
            f"  prestream before entry: {metrics['prestream_duration_s']:.3f} s",
            "",
            *control_pipeline_summary(pipeline),
        ]
    )

    markers = [
        (metrics["prestream_start_s"], "OFFBOARD PRESTREAM"),
        (metrics["offboard_entry_s"], "OFFBOARD ENTRY"),
    ]

    if not started_armed:
        markers.insert(0, (metrics["armed_s"], "ARMED"))

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
