"""Analysis profile for the headless PX4 Position-mode takeoff/hover test."""

from __future__ import annotations

from dataclasses import dataclass
from math import hypot, isfinite
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


PROFILE_NAME = "position_takeoff_hover"

STATUS_TOPIC = TOPICS["OUT_VEHICLE_STATUS_V1"]
LOCAL_POSITION_TOPIC = TOPICS["OUT_VEHICLE_LOCAL_POSITION"]
MANUAL_CONTROL_TOPIC = TOPICS["OUT_MANUAL_CONTROL_SETPOINT"]

REQUIRED_TOPICS = {
    STATUS_TOPIC,
    LOCAL_POSITION_TOPIC,
    MANUAL_CONTROL_TOPIC,
    *CONTROL_PIPELINE_TOPICS,
}


@dataclass
class AnalysisResult:
    """Profile output consumed by the global analyzer CLI."""

    summary: str
    metrics: dict[str, float]
    plot_data: dict[str, object]


def _first_arming_state_time(
    samples: list[TimedSample],
    state: int,
    *,
    not_before_ns: int | None = None,
) -> int | None:
    """Find the first occurrence of one PX4 arming state."""

    for sample in samples:
        if (
            not_before_ns is not None
            and sample.timestamp_ns < not_before_ns
        ):
            continue

        if int(sample.message.arming_state) == state:
            return sample.timestamp_ns

    return None


def _first_throttle_time(
    samples: list[TimedSample],
    predicate,
    *,
    not_before_ns: int,
) -> int | None:
    """Find the first manual-control sample matching one flight phase."""

    for sample in samples:
        if sample.timestamp_ns < not_before_ns:
            continue

        if predicate(float(sample.message.throttle)):
            return sample.timestamp_ns

    return None


def _first_landing_state_time(
    samples: list[TimedSample],
    status_type: type,
    *,
    not_before_ns: int,
) -> tuple[int, int] | None:
    """Find PX4 Land mode or its local Descend fallback."""

    landing_states = {
        int(status_type.NAVIGATION_STATE_AUTO_LAND),
        int(status_type.NAVIGATION_STATE_DESCEND),
    }

    for sample in samples:
        if sample.timestamp_ns < not_before_ns:
            continue

        state = int(sample.message.nav_state)

        if state in landing_states:
            return sample.timestamp_ns, state

    return None


def _hover_drift(
    samples: list[TimedSample],
    start_ns: int,
    end_ns: int,
) -> tuple[float, float]:
    """Measure local-position drift while the virtual sticks are centered."""

    window = [
        sample
        for sample in samples
        if start_ns <= sample.timestamp_ns <= end_ns
        and isfinite(float(sample.message.x))
        and isfinite(float(sample.message.y))
        and isfinite(float(sample.message.z))
    ]

    if not window:
        raise RuntimeError(
            "No finite local-position samples were recorded during hover"
        )

    x0 = float(window[0].message.x)
    y0 = float(window[0].message.y)
    z0 = float(window[0].message.z)

    max_xy = max(
        hypot(
            float(sample.message.x) - x0,
            float(sample.message.y) - y0,
        )
        for sample in window
    )

    max_z = max(
        abs(float(sample.message.z) - z0)
        for sample in window
    )

    return max_xy, max_z


def analyze(bag: BagData) -> AnalysisResult:
    """Analyze Position-mode semantics plus the common PX4 control pipeline."""

    status_samples = bag.samples[STATUS_TOPIC]
    local_position_samples = bag.samples[LOCAL_POSITION_TOPIC]
    manual_samples = bag.samples[MANUAL_CONTROL_TOPIC]

    if not status_samples:
        raise RuntimeError(f"No samples recorded on {STATUS_TOPIC}")

    if not local_position_samples:
        raise RuntimeError(
            f"No samples recorded on {LOCAL_POSITION_TOPIC}"
        )

    if not manual_samples:
        raise RuntimeError(
            f"No samples recorded on {MANUAL_CONTROL_TOPIC}"
        )

    status_type = type(status_samples[0].message)

    position_state = int(
        status_type.NAVIGATION_STATE_POSCTL
    )
    armed_state = int(
        status_type.ARMING_STATE_ARMED
    )
    disarmed_state = int(
        status_type.ARMING_STATE_DISARMED
    )

    position_entry_ns = first_nav_state_time(
        status_samples,
        position_state,
    )

    if position_entry_ns is None:
        raise RuntimeError("Position-mode entry was not recorded")

    armed_ns = _first_arming_state_time(
        status_samples,
        armed_state,
        not_before_ns=position_entry_ns,
    )

    if armed_ns is None:
        raise RuntimeError("Armed state was not recorded")

    climb_start_ns = _first_throttle_time(
        manual_samples,
        lambda value: value > 0.10,
        not_before_ns=armed_ns,
    )

    if climb_start_ns is None:
        raise RuntimeError("Climb-stick command was not recorded")

    hover_start_ns = _first_throttle_time(
        manual_samples,
        lambda value: abs(value) <= 0.05,
        not_before_ns=climb_start_ns,
    )

    if hover_start_ns is None:
        raise RuntimeError("Centered-stick hover was not recorded")

    landing = _first_landing_state_time(
        status_samples,
        status_type,
        not_before_ns=hover_start_ns,
    )

    if landing is None:
        raise RuntimeError(
            "PX4 Land/Descend entry after hover was not recorded"
        )

    landing_ns, landing_state = landing

    disarmed_ns = _first_arming_state_time(
        status_samples,
        disarmed_state,
        not_before_ns=landing_ns,
    )

    if disarmed_ns is None:
        raise RuntimeError("Final disarmed state was not recorded")

    max_xy_drift_m, max_z_drift_m = _hover_drift(
        local_position_samples,
        hover_start_ns,
        landing_ns,
    )

    metrics = {
        "bag_duration_s": bag.duration_s,
        "position_entry_s": bag.relative_seconds(position_entry_ns),
        "armed_s": bag.relative_seconds(armed_ns),
        "climb_start_s": bag.relative_seconds(climb_start_ns),
        "hover_start_s": bag.relative_seconds(hover_start_ns),
        "landing_entry_s": bag.relative_seconds(landing_ns),
        "disarmed_s": bag.relative_seconds(disarmed_ns),
        "hover_duration_s": (
            landing_ns - hover_start_ns
        ) / 1_000_000_000.0,
        "max_xy_hover_drift_m": max_xy_drift_m,
        "max_z_hover_drift_m": max_z_drift_m,
    }

    state_names = native_constant_names(
        status_type,
        "NAVIGATION_STATE_",
    )
    landing_name = state_names.get(
        landing_state,
        f"STATE_{landing_state}",
    )
    transitions = mode_transitions(status_samples)
    pipeline = analyze_control_pipeline(bag)

    lines = [
        f"Profile: {PROFILE_NAME}",
        f"Bag: {bag.path}",
        f"Bag duration: {metrics['bag_duration_s']:.3f} s",
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
            "Position-mode flight:",
            f"  Position-mode entry: "
            f"{metrics['position_entry_s']:.3f} s",
            f"  armed: {metrics['armed_s']:.3f} s",
            f"  climb command: {metrics['climb_start_s']:.3f} s",
            f"  sticks centered: {metrics['hover_start_s']:.3f} s",
            f"  native landing entry: "
            f"{metrics['landing_entry_s']:.3f} s ({landing_name})",
            f"  disarmed: {metrics['disarmed_s']:.3f} s",
            f"  hover duration: {metrics['hover_duration_s']:.3f} s",
            f"  max XY hover drift: "
            f"{metrics['max_xy_hover_drift_m']:.4f} m",
            f"  max Z hover drift: "
            f"{metrics['max_z_hover_drift_m']:.4f} m",
            "",
            *control_pipeline_summary(pipeline),
        ]
    )

    markers = [
        (metrics["position_entry_s"], "POSCTL ENTRY"),
        (metrics["hover_start_s"], "STICKS CENTERED"),
        (metrics["landing_entry_s"], "LAND"),
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
    """Write common PX4 pipeline plots plus navigation-state history."""

    output_dir.mkdir(parents=True, exist_ok=True)

    # Each analyzer run owns its generated figures. Remove stale plots from
    # earlier layouts or filename conventions before writing the new report.
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
