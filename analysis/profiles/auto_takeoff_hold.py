"""Analysis profile for the native PX4 AUTO_TAKEOFF -> AUTO_LOITER test."""

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


PROFILE_NAME = "auto_takeoff_hold"

STATUS_TOPIC = TOPICS["OUT_VEHICLE_STATUS_V1"]
MODE_COMPLETED_TOPIC = TOPICS["OUT_MODE_COMPLETED"]

REQUIRED_TOPICS = {
    STATUS_TOPIC,
    MODE_COMPLETED_TOPIC,
    *CONTROL_PIPELINE_TOPICS,
}


@dataclass
class AnalysisResult:
    """Profile output consumed by the global analyzer CLI."""

    summary: str
    metrics: dict[str, float]
    plot_data: dict[str, object]


def _first_takeoff_completion(
    samples: list[TimedSample],
    *,
    takeoff_state: int,
    success_result: int,
) -> int | None:
    """Find successful completion of PX4's native AUTO_TAKEOFF mode."""
    for sample in samples:
        message = sample.message

        if (
            int(message.nav_state) == takeoff_state
            and int(message.result) == success_result
        ):
            return sample.timestamp_ns

    return None


def analyze(bag: BagData) -> AnalysisResult:
    """Analyze AUTO_TAKEOFF semantics plus the common PX4 control pipeline."""
    status_samples = bag.samples[STATUS_TOPIC]
    completed_samples = bag.samples[MODE_COMPLETED_TOPIC]

    if not status_samples:
        raise RuntimeError(f"No samples recorded on {STATUS_TOPIC}")

    if not completed_samples:
        raise RuntimeError(f"No samples recorded on {MODE_COMPLETED_TOPIC}")

    status_type = type(status_samples[0].message)
    completed_type = type(completed_samples[0].message)

    takeoff_state = int(
        status_type.NAVIGATION_STATE_AUTO_TAKEOFF
    )
    hold_state = int(
        status_type.NAVIGATION_STATE_AUTO_LOITER
    )
    success_result = int(completed_type.RESULT_SUCCESS)

    takeoff_entry_ns = first_nav_state_time(
        status_samples,
        takeoff_state,
    )

    if takeoff_entry_ns is None:
        raise RuntimeError("AUTO_TAKEOFF entry was not recorded")

    takeoff_complete_ns = _first_takeoff_completion(
        completed_samples,
        takeoff_state=takeoff_state,
        success_result=success_result,
    )

    if takeoff_complete_ns is None:
        raise RuntimeError(
            "Successful AUTO_TAKEOFF completion was not recorded"
        )

    hold_entry_ns = first_nav_state_time(
        status_samples,
        hold_state,
        not_before_ns=takeoff_entry_ns,
    )

    if hold_entry_ns is None:
        raise RuntimeError(
            "AUTO_LOITER entry after AUTO_TAKEOFF was not recorded"
        )

    metrics = {
        "bag_duration_s": bag.duration_s,
        "takeoff_entry_s": bag.relative_seconds(takeoff_entry_ns),
        "takeoff_complete_s": bag.relative_seconds(takeoff_complete_ns),
        "hold_entry_s": bag.relative_seconds(hold_entry_ns),
        "takeoff_duration_s": (
            takeoff_complete_ns - takeoff_entry_ns
        ) / 1_000_000_000.0,
        "takeoff_to_hold_s": (
            hold_entry_ns - takeoff_entry_ns
        ) / 1_000_000_000.0,
    }

    state_names = native_constant_names(
        status_type,
        "NAVIGATION_STATE_",
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
            "Takeoff:",
            f"  AUTO_TAKEOFF entry: "
            f"{metrics['takeoff_entry_s']:.3f} s",
            f"  completion event: "
            f"{metrics['takeoff_complete_s']:.3f} s",
            f"  AUTO_LOITER entry: "
            f"{metrics['hold_entry_s']:.3f} s",
            f"  Takeoff duration: "
            f"{metrics['takeoff_duration_s']:.3f} s",
            f"  Takeoff-to-hold duration: "
            f"{metrics['takeoff_to_hold_s']:.3f} s",
        ]
    )

    lines.extend(
        [
            "",
            *control_pipeline_summary(pipeline),
        ]
    )

    markers = [
        (metrics["takeoff_entry_s"], "AUTO_TAKEOFF"),
        (metrics["hold_entry_s"], "AUTO_LOITER"),
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
    """Write common PX4 pipeline plots plus takeoff navigation-state history."""

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
