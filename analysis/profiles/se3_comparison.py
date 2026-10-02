"""Multi-run comparison for SE3 trajectory-tracking experiments."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np
import yaml

from analysis.comparison import (
    activity_stats,
    effort_stats,
    interval_scalar,
    interval_series,
    motor_envelope,
    series_stats,
    shift_motor_series,
    shift_vector_series,
    tracking_error_series,
    vector_magnitude_series,
)
from analysis.plots import (
    _decorate_axis,
    _matplotlib,
)


@dataclass
class ComparisonRun:
    """Common comparison view of one analyzed SE3 run."""

    label: str
    bag_path: Path
    handoff: str
    duration_s: float
    position_error: dict[str, list[float]]
    velocity_error: dict[str, list[float]]
    thrust: dict[str, list[float]]
    torque: dict[str, list[float]]
    body_rates: dict[str, list[float]]
    motors: dict[
        str,
        tuple[list[float], list[float]],
    ]


def build_run(
    result,
    bag_path: Path,
    label: str,
) -> ComparisonRun:
    """Build one SE3 comparison run from structured analyzer output."""
    layers = result.plot_data["layers"]
    pipeline = result.plot_data["pipeline"]

    origin = float(
        result.metrics["offboard_entry_s"]
    )
    duration = float(
        result.metrics["offboard_duration_s"]
    )
    end = origin + duration

    return ComparisonRun(
        label=label,
        bag_path=bag_path,
        handoff=str(
            result.metrics["handoff"]
        ),
        duration_s=duration,
        position_error=tracking_error_series(
            pipeline["position"]["actual"],
            layers["reference"]["position"],
            time_origin_s=origin,
            end_time_s=end,
        ),
        velocity_error=tracking_error_series(
            pipeline["velocity"]["actual"],
            layers["reference"]["velocity"],
            time_origin_s=origin,
            end_time_s=end,
        ),
        thrust=shift_vector_series(
            pipeline["thrust"]["setpoint"],
            origin,
            duration,
        ),
        torque=shift_vector_series(
            pipeline["torque"]["setpoint"],
            origin,
            duration,
        ),
        body_rates=shift_vector_series(
            pipeline["rates"]["actual"],
            origin,
            duration,
        ),
        motors=shift_motor_series(
            pipeline["motors"],
            origin,
            duration,
        ),
    )


def trajectory_segments(
    config_path: Path,
    trajectory_name: str,
) -> list[dict[str, float | str]]:
    """Resolve configured segment boundaries relative to Offboard entry."""
    data = yaml.safe_load(
        config_path.read_text()
    )
    segments = data.get("segments", {})
    trajectories = data.get(
        "trajectories",
        {},
    )

    if trajectory_name not in trajectories:
        raise ValueError(
            f"Unknown trajectory: {trajectory_name}"
        )

    elapsed = 0.0
    result = []

    for index, segment_name in enumerate(
        trajectories[trajectory_name]["segments"],
        start=1,
    ):
        if segment_name not in segments:
            raise ValueError(
                f"Trajectory {trajectory_name} references "
                f"unknown segment {segment_name}"
            )

        segment = segments[segment_name]
        duration = float(
            segment["duration"]
        )

        result.append(
            {
                "index": index,
                "name": segment_name,
                "type": str(
                    segment["type"]
                ),
                "start_s": elapsed,
                "end_s": elapsed + duration,
            }
        )
        elapsed += duration

    return result


def _axis_error_stats(
    series: dict[str, list[float]],
):
    return {
        component: series_stats(
            series[component]
        )
        for component in (
            "x",
            "y",
            "z",
            "norm",
        )
    }


def summarize_run(
    run: ComparisonRun,
) -> dict[str, object]:
    """Compute whole-Offboard tracking and downstream activity metrics."""
    thrust_t, thrust_mag = (
        vector_magnitude_series(
            run.thrust
        )
    )
    motor_t, motor_mag = (
        motor_envelope(
            run.motors
        )
    )

    return {
        "position": _axis_error_stats(
            run.position_error
        ),
        "velocity": _axis_error_stats(
            run.velocity_error
        ),
        "thrust": effort_stats(
            thrust_t,
            thrust_mag,
        ),
        "torque": {
            component: effort_stats(
                run.torque["times_s"],
                run.torque[component],
            )
            for component in (
                "x",
                "y",
                "z",
            )
        },
        "body_rates": {
            component: activity_stats(
                run.body_rates["times_s"],
                run.body_rates[component],
            )
            for component in (
                "x",
                "y",
                "z",
            )
        },
        "motors": effort_stats(
            motor_t,
            motor_mag,
        ),
    }


def _format_float(
    value: float,
    width: int = 10,
) -> str:
    if np.isnan(value):
        return f"{'n/a':>{width}}"

    return f"{value:>{width}.4f}"


def _append_error_table(
    lines: list[str],
    runs: list[ComparisonRun],
    metrics: dict[str, object],
    *,
    title: str,
    section: str,
    statistic: str,
) -> None:
    lines.extend(
        [
            title,
            "-" * len(title),
            (
                "  run                         "
                "x         y         z      norm"
            ),
        ]
    )

    for run in runs:
        row = metrics[run.label][section]
        lines.append(
            f"  {run.label:<24}"
            f"{_format_float(row['x'][statistic])}"
            f"{_format_float(row['y'][statistic])}"
            f"{_format_float(row['z'][statistic])}"
            f"{_format_float(row['norm'][statistic])}"
        )

    lines.append("")


def comparison_summary(
    runs: list[ComparisonRun],
    segments: list[
        dict[str, float | str]
    ] | None = None,
) -> str:
    """Build the SE3 multi-run comparison report."""
    metrics = {
        run.label: summarize_run(run)
        for run in runs
    }

    lines = [
        "SE3 Multi-Bag Comparison",
        "========================",
        "",
        f"runs: {len(runs)}",
        "alignment: PX4 Offboard entry = 0 s",
        "statistics interval: reported PX4 Offboard interval only",
        "",
    ]

    _append_error_table(
        lines,
        runs,
        metrics,
        title="Position-error RMS [m]",
        section="position",
        statistic="rms",
    )
    _append_error_table(
        lines,
        runs,
        metrics,
        title="Position max |error| [m]",
        section="position",
        statistic="peak",
    )
    _append_error_table(
        lines,
        runs,
        metrics,
        title="Velocity-error RMS [m/s]",
        section="velocity",
        statistic="rms",
    )
    _append_error_table(
        lines,
        runs,
        metrics,
        title="Velocity max |error| [m/s]",
        section="velocity",
        statistic="peak",
    )

    lines.extend(
        [
            "Downstream normalized thrust activity",
            "-------------------------------------",
            (
                "  run                       RMS      peak"
                "   near-limit   roughness"
            ),
            "  near-limit: |u| >= 0.99",
        ]
    )

    for run in runs:
        row = metrics[
            run.label
        ]["thrust"]
        lines.append(
            f"  {run.label:<24}"
            f"{_format_float(row['rms'])}"
            f"{_format_float(row['peak'])}"
            f"{_format_float(row['near_limit_fraction'], 13)}"
            f"{_format_float(row['roughness_rms'], 12)}"
        )

    lines.extend(
        [
            "",
            "Downstream normalized torque activity",
            "-------------------------------------",
            (
                "  run / axis                 RMS      peak"
                "   near-limit   roughness"
            ),
            "  near-limit: |u| >= 0.99",
        ]
    )

    for run in runs:
        for component in (
            "x",
            "y",
            "z",
        ):
            row = metrics[
                run.label
            ]["torque"][component]
            lines.append(
                f"  {run.label + ' / ' + component:<24}"
                f"{_format_float(row['rms'])}"
                f"{_format_float(row['peak'])}"
                f"{_format_float(row['near_limit_fraction'], 13)}"
                f"{_format_float(row['roughness_rms'], 12)}"
            )

    lines.extend(
        [
            "",
            "Measured body-rate activity [deg/s]",
            "-----------------------------------",
            (
                "  run / axis                 RMS      peak"
                "   roughness [deg/s^2]"
            ),
        ]
    )

    for run in runs:
        for component in (
            "x",
            "y",
            "z",
        ):
            row = metrics[
                run.label
            ]["body_rates"][component]
            lines.append(
                f"  {run.label + ' / ' + component:<24}"
                f"{_format_float(row['rms'])}"
                f"{_format_float(row['peak'])}"
                f"{_format_float(row['roughness_rms'], 20)}"
            )

    lines.extend(
        [
            "",
            "Motor-command envelope",
            "----------------------",
            (
                "  run                       RMS      peak"
                "   near-limit   roughness"
            ),
            (
                "  envelope = max absolute motor command; "
                "near-limit: envelope >= 0.99"
            ),
        ]
    )

    for run in runs:
        row = metrics[
            run.label
        ]["motors"]
        lines.append(
            f"  {run.label:<24}"
            f"{_format_float(row['rms'])}"
            f"{_format_float(row['peak'])}"
            f"{_format_float(row['near_limit_fraction'], 13)}"
            f"{_format_float(row['roughness_rms'], 12)}"
        )

    if segments:
        lines.extend(
            [
                "",
                "Trajectory-segment error-norm RMS",
                "---------------------------------",
                (
                    "  segment                       metric  "
                    + "  ".join(
                        f"{run.label:>12}"
                        for run in runs
                    )
                ),
            ]
        )

        for segment in segments:
            position_values = []
            velocity_values = []

            for run in runs:
                position = interval_series(
                    run.position_error,
                    float(
                        segment["start_s"]
                    ),
                    float(
                        segment["end_s"]
                    ),
                )
                velocity = interval_series(
                    run.velocity_error,
                    float(
                        segment["start_s"]
                    ),
                    float(
                        segment["end_s"]
                    ),
                )

                position_values.append(
                    series_stats(
                        position["norm"]
                    )["rms"]
                )
                velocity_values.append(
                    series_stats(
                        velocity["norm"]
                    )["rms"]
                )

            segment_label = (
                f"{int(segment['index']):02d}-"
                f"{str(segment['name'])}"
            )

            lines.append(
                f"  {segment_label:<28} {'pos [m]':<8}  "
                + "  ".join(
                    _format_float(
                        value,
                        12,
                    )
                    for value in position_values
                )
            )
            lines.append(
                f"  {'':<28} {'vel [m/s]':<8}  "
                + "  ".join(
                    _format_float(
                        value,
                        12,
                    )
                    for value in velocity_values
                )
            )

        figure_eights = [
            segment
            for segment in segments
            if segment["type"]
            == "figure_eight"
        ]

        for figure_eight in figure_eights:
            start = float(
                figure_eight["start_s"]
            )
            end = float(
                figure_eight["end_s"]
            )
            name = str(
                figure_eight["name"]
            )

            lines.extend(
                [
                    "",
                    f"Figure-eight control activity: {name}",
                    "-" * (
                        len(
                            "Figure-eight control activity: "
                        )
                        + len(name)
                    ),
                    (
                        "  run / signal               RMS      peak"
                        "   near-limit   roughness"
                    ),
                ]
            )

            for run in runs:
                thrust_t, thrust_mag = (
                    vector_magnitude_series(
                        run.thrust
                    )
                )
                thrust_t, thrust_mag = (
                    interval_scalar(
                        thrust_t,
                        thrust_mag,
                        start,
                        end,
                    )
                )
                thrust = effort_stats(
                    thrust_t,
                    thrust_mag,
                )

                lines.append(
                    f"  {run.label + ' / thrust':<24}"
                    f"{_format_float(thrust['rms'])}"
                    f"{_format_float(thrust['peak'])}"
                    f"{_format_float(thrust['near_limit_fraction'], 13)}"
                    f"{_format_float(thrust['roughness_rms'], 12)}"
                )

                torque = interval_series(
                    run.torque,
                    start,
                    end,
                )

                for component in (
                    "x",
                    "y",
                    "z",
                ):
                    row = effort_stats(
                        torque["times_s"],
                        torque[component],
                    )
                    lines.append(
                        f"  {run.label + ' / torque ' + component:<24}"
                        f"{_format_float(row['rms'])}"
                        f"{_format_float(row['peak'])}"
                        f"{_format_float(row['near_limit_fraction'], 13)}"
                        f"{_format_float(row['roughness_rms'], 12)}"
                    )

                motor_t, motor_mag = (
                    motor_envelope(
                        run.motors
                    )
                )
                motor_t, motor_mag = (
                    interval_scalar(
                        motor_t,
                        motor_mag,
                        start,
                        end,
                    )
                )
                motor = effort_stats(
                    motor_t,
                    motor_mag,
                )

                lines.append(
                    f"  {run.label + ' / motors':<24}"
                    f"{_format_float(motor['rms'])}"
                    f"{_format_float(motor['peak'])}"
                    f"{_format_float(motor['near_limit_fraction'], 13)}"
                    f"{_format_float(motor['roughness_rms'], 12)}"
                )

            lines.extend(
                [
                    "",
                    (
                        "Figure-eight measured body-rate activity "
                        "[deg/s]"
                    ),
                    (
                        "-------------------------------------------"
                        "------"
                    ),
                    (
                        "  run / axis                 RMS      peak"
                        "   roughness [deg/s^2]"
                    ),
                ]
            )

            for run in runs:
                rates = interval_series(
                    run.body_rates,
                    start,
                    end,
                )

                for component in (
                    "x",
                    "y",
                    "z",
                ):
                    row = activity_stats(
                        rates["times_s"],
                        rates[component],
                    )
                    lines.append(
                        f"  {run.label + ' / ' + component:<24}"
                        f"{_format_float(row['rms'])}"
                        f"{_format_float(row['peak'])}"
                        f"{_format_float(row['roughness_rms'], 20)}"
                    )

    return "\n".join(lines) + "\n"


def _segment_markers(
    axis,
    segments,
) -> None:
    for segment in segments or []:
        axis.axvline(
            float(
                segment["start_s"]
            ),
            linestyle="--",
            linewidth=0.7,
            alpha=0.35,
        )


def _save_component_overlay(
    output_path: Path,
    runs: list[ComparisonRun],
    attribute: str,
    title: str,
    ylabel: str,
    segments,
) -> None:
    plt = _matplotlib()
    fig, axes = plt.subplots(
        3,
        1,
        sharex=True,
        figsize=(10.5, 8.0),
    )

    for axis, component in zip(
        axes,
        ("x", "y", "z"),
    ):
        for run in runs:
            series = getattr(
                run,
                attribute,
            )
            axis.plot(
                series["times_s"],
                series[component],
                label=run.label,
            )

        axis.axhline(
            0.0,
            linewidth=0.8,
            alpha=0.5,
        )
        _segment_markers(
            axis,
            segments,
        )
        _decorate_axis(axis)
        axis.set_ylabel(
            f"{component}\n{ylabel}"
        )

    handles, labels = (
        axes[0].get_legend_handles_labels()
    )

    fig.suptitle(
        title,
        y=0.985,
    )

    if handles:
        fig.legend(
            handles,
            labels,
            frameon=False,
            loc="upper center",
            bbox_to_anchor=(0.5, 0.945),
            borderaxespad=0.0,
            ncol=min(
                4,
                len(labels),
            ),
        )

    axes[-1].set_xlabel(
        "Time from Offboard entry [s]"
    )
    fig.tight_layout(
        rect=(0.02, 0.02, 0.99, 0.88)
    )
    fig.savefig(
        output_path,
        bbox_inches="tight",
    )
    plt.close(fig)


def _save_scalar_overlay(
    output_path: Path,
    runs: list[ComparisonRun],
    getter,
    title: str,
    ylabel: str,
    segments,
) -> None:
    plt = _matplotlib()
    fig, axis = plt.subplots(
        figsize=(10.5, 4.8)
    )

    for run in runs:
        times, values = getter(run)
        axis.plot(
            times,
            values,
            label=run.label,
        )

    _segment_markers(
        axis,
        segments,
    )
    _decorate_axis(axis)
    axis.set_xlabel(
        "Time from Offboard entry [s]"
    )
    axis.set_ylabel(ylabel)
    axis.set_title(
        title,
        pad=12,
    )
    axis.legend(
        frameon=False,
        loc="best",
    )
    fig.tight_layout()
    fig.savefig(
        output_path,
        bbox_inches="tight",
    )
    plt.close(fig)


def write_comparison_plots(
    runs: list[ComparisonRun],
    output_dir: Path,
    segments: list[
        dict[str, float | str]
    ] | None = None,
) -> list[Path]:
    """Write SE3 tracking and control-activity comparison plots."""
    output_dir.mkdir(
        parents=True,
        exist_ok=True,
    )

    paths = [
        output_dir / "01_position_error.png",
        output_dir / "02_velocity_error.png",
        output_dir / "03_position_error_norm.png",
        output_dir / "04_velocity_error_norm.png",
        output_dir / "05_thrust_activity.png",
        output_dir / "06_torque_activity.png",
        output_dir / "07_motor_activity.png",
        output_dir / "08_body_rate_activity.png",
    ]

    _save_component_overlay(
        paths[0],
        runs,
        "position_error",
        "SE3 Position Tracking Error",
        "error [m]",
        segments,
    )
    _save_component_overlay(
        paths[1],
        runs,
        "velocity_error",
        "SE3 Velocity Tracking Error",
        "error [m/s]",
        segments,
    )
    _save_scalar_overlay(
        paths[2],
        runs,
        lambda run: (
            run.position_error["times_s"],
            run.position_error["norm"],
        ),
        "SE3 Position-Error Norm",
        "error norm [m]",
        segments,
    )
    _save_scalar_overlay(
        paths[3],
        runs,
        lambda run: (
            run.velocity_error["times_s"],
            run.velocity_error["norm"],
        ),
        "SE3 Velocity-Error Norm",
        "error norm [m/s]",
        segments,
    )
    _save_scalar_overlay(
        paths[4],
        runs,
        lambda run: vector_magnitude_series(
            run.thrust
        ),
        "PX4 Normalized Thrust Magnitude",
        "normalized thrust",
        segments,
    )
    _save_component_overlay(
        paths[5],
        runs,
        "torque",
        "PX4 Normalized Torque Setpoint",
        "normalized torque",
        segments,
    )
    _save_scalar_overlay(
        paths[6],
        runs,
        lambda run: motor_envelope(
            run.motors
        ),
        "PX4 Motor-Command Envelope",
        "max |motor command|",
        segments,
    )
    _save_component_overlay(
        paths[7],
        runs,
        "body_rates",
        "Measured Body-Rate Activity",
        "deg/s",
        segments,
    )

    return paths
