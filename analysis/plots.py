"""Publication-style plotting helpers for PX4 flight-data analysis."""

from __future__ import annotations

from pathlib import Path

import numpy as np


def _matplotlib():
    """Configure a consistent publication-style Matplotlib backend."""
    try:
        import matplotlib

        matplotlib.use("Agg")
        matplotlib.rcParams.update(
            {
                "font.family": "STIXGeneral",
                "mathtext.fontset": "stix",
                "font.size": 10,
                "axes.titlesize": 11,
                "axes.labelsize": 10,
                "legend.fontsize": 9,
                "xtick.labelsize": 9,
                "ytick.labelsize": 9,
                "axes.spines.top": False,
                "axes.spines.right": False,
                "lines.linewidth": 1.35,
                "savefig.dpi": 200,
            }
        )

        import matplotlib.pyplot as plt

    except ImportError as exc:
        raise RuntimeError(
            "matplotlib is not installed; summary.txt was still generated."
        ) from exc

    return plt


def _decorate_axis(axis) -> None:
    """Apply the common time-history axis appearance."""
    axis.grid(
        True,
        which="major",
        linewidth=0.5,
        alpha=0.30,
    )
    axis.margins(x=0.01)


def _add_markers(
    axis,
    markers: list[tuple[float, str]] | None,
) -> None:
    """Draw experiment-event boundaries without adding legend entries."""
    for marker_time_s, _ in markers or []:
        axis.axvline(
            marker_time_s,
            linestyle="--",
            linewidth=0.9,
            alpha=0.65,
        )


def _add_marker_labels(
    axis,
    markers: list[tuple[float, str]] | None,
) -> None:
    """Label event boundaries once on the first subplot."""
    for marker_time_s, label in markers or []:
        axis.annotate(
            label,
            xy=(marker_time_s, 1.0),
            xycoords=("data", "axes fraction"),
            xytext=(-4, -5),
            textcoords="offset points",
            rotation=90,
            ha="right",
            va="top",
            fontsize=8,
        )


def _tracking_error(
    actual_times: list[float],
    actual_values: list[float],
    reference_times: list[float],
    reference_values: list[float],
    *,
    wrap_degrees: bool = False,
) -> tuple[list[float], list[float]]:
    """
    Compute actual - reference on actual-signal timestamps.

    The reference is interpolated only over the common recorded interval.
    """
    if not actual_times or not reference_times:
        return [], []

    actual_t = np.asarray(actual_times, dtype=float)
    actual_y = np.asarray(actual_values, dtype=float)
    reference_t = np.asarray(reference_times, dtype=float)
    reference_y = np.asarray(reference_values, dtype=float)

    order = np.argsort(reference_t)
    reference_t = reference_t[order]
    reference_y = reference_y[order]

    start = max(actual_t[0], reference_t[0])
    end = min(actual_t[-1], reference_t[-1])

    mask = (actual_t >= start) & (actual_t <= end)

    if not np.any(mask):
        return [], []

    times = actual_t[mask]
    error = actual_y[mask] - np.interp(
        times,
        reference_t,
        reference_y,
    )

    if wrap_degrees:
        error = (error + 180.0) % 360.0 - 180.0

    return times.tolist(), error.tolist()


def save_series_plot(
    output_path: Path,
    *,
    times_s: list[float],
    series: dict[str, list[float]],
    title: str,
    ylabel: str,
    markers: list[tuple[float, str]] | None = None,
) -> None:
    """Save a clean single-axis time-history figure."""
    plt = _matplotlib()

    fig, axis = plt.subplots(figsize=(10, 4.6))

    for label, values in series.items():
        axis.plot(times_s, values, label=label)

    _add_markers(axis, markers)
    _add_marker_labels(axis, markers)
    _decorate_axis(axis)

    axis.set_xlabel(r"Time from bag start, $t$ [s]")
    axis.set_ylabel(ylabel)
    axis.set_title(title, pad=12)

    axis.legend(
        frameon=False,
        loc="best",
    )

    fig.tight_layout()
    fig.savefig(output_path, bbox_inches="tight")
    plt.close(fig)


def save_tracking_plot(
    output_path: Path,
    *,
    components: list[
        tuple[
            str,
            dict[str, tuple[list[float], list[float]]],
        ]
    ],
    title: str,
    unit: str,
    markers: list[tuple[float, str]] | None = None,
) -> None:
    """
    Save a controller-tracking figure.

    Three physical components are shown individually. When an actual/reference
    pair exists, a final subplot contains all component tracking errors.
    """
    if not components:
        return

    plt = _matplotlib()

    line_styles = {
        "actual": "-",
        "trajectory": "--",
        "controller setpoint": "-.",
        "setpoint": "--",
    }

    errors = []

    for component_name, signals in components:
        actual = signals.get("actual")

        if actual is None:
            continue

        if "controller setpoint" in signals:
            reference_label = "controller setpoint"
        elif "setpoint" in signals:
            reference_label = "setpoint"
        elif "trajectory" in signals:
            reference_label = "trajectory"
        else:
            continue

        reference = signals[reference_label]

        wrap_degrees = (
            unit == "[deg]"
            and "yaw" in component_name.lower()
        )

        error_times, error_values = _tracking_error(
            actual[0],
            actual[1],
            reference[0],
            reference[1],
            wrap_degrees=wrap_degrees,
        )

        if error_times:
            errors.append(
                (
                    component_name,
                    error_times,
                    error_values,
                )
            )

    axis_count = len(components) + (1 if errors else 0)

    fig, axes = plt.subplots(
        axis_count,
        1,
        sharex=True,
        figsize=(10.5, 2.25 * axis_count + 1.2),
    )

    if axis_count == 1:
        axes = [axes]

    # Main tracking panels.
    for axis, (component_name, signals) in zip(
        axes,
        components,
    ):
        for label, (times_s, values) in signals.items():
            axis.plot(
                times_s,
                values,
                label=label,
                linestyle=line_styles.get(label, "-"),
            )

        _add_markers(axis, markers)
        _decorate_axis(axis)

        axis.set_ylabel(
            f"{component_name}\n{unit}"
        )

    _add_marker_labels(axes[0], markers)

    # One common legend for the entire figure.
    handles, labels = axes[0].get_legend_handles_labels()

    if handles:
        fig.legend(
            handles,
            labels,
            frameon=False,
            loc="upper center",
            ncol=len(labels),
            bbox_to_anchor=(0.5, 0.935),
        )

    # Final tracking-error panel.
    if errors:
        error_axis = axes[-1]

        for component_name, times_s, values in errors:
            error_axis.plot(
                times_s,
                values,
                label=component_name,
            )

        error_axis.axhline(
            0.0,
            linewidth=0.8,
            alpha=0.6,
        )

        _add_markers(error_axis, markers)
        _decorate_axis(error_axis)

        error_axis.set_ylabel(
            f"error\n{unit}"
        )

        error_axis.legend(
            frameon=False,
            loc="best",
            ncol=min(3, len(errors)),
        )

    axes[-1].set_xlabel(
        r"Time from bag start, $t$ [s]"
    )

    # Separate vertical space for title and global legend.
    fig.suptitle(
        title,
        fontsize=14,
        y=0.985,
    )

    fig.tight_layout(
        rect=(0.02, 0.02, 0.99, 0.89)
    )

    fig.savefig(
        output_path,
        bbox_inches="tight",
    )
    plt.close(fig)


def save_mode_timeline(
    output_path: Path,
    *,
    transitions: list[tuple[float, int]],
    state_names: dict[int, str],
) -> None:
    """Save the PX4 navigation-state history."""
    if not transitions:
        return

    plt = _matplotlib()

    times_s = [time for time, _ in transitions]
    states = [state for _, state in transitions]
    unique_states = sorted(set(states))

    fig, axis = plt.subplots(figsize=(10, 4.5))

    axis.step(
        times_s,
        states,
        where="post",
        linewidth=1.5,
    )

    axis.set_yticks(
        unique_states,
        [
            f"{state_names.get(state, 'STATE')} ({state})"
            for state in unique_states
        ],
    )

    axis.set_xlabel(r"Time from bag start, $t$ [s]")
    axis.set_ylabel("PX4 navigation state")
    axis.set_title(
        "PX4 Navigation-State Timeline",
        pad=12,
    )

    _decorate_axis(axis)

    fig.tight_layout()
    fig.savefig(output_path, bbox_inches="tight")
    plt.close(fig)
