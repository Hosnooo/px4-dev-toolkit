"""Reusable math for comparing multiple experiment runs."""

from __future__ import annotations

from pathlib import Path

import numpy as np


def validate_profile_names(names: list[str]) -> str:
    """Require at least two runs from one analysis profile."""
    if len(names) < 2:
        raise ValueError("Comparison requires at least two bags.")

    unique = sorted(set(names))

    if len(unique) != 1:
        raise ValueError(
            "Comparison bags use different profiles: " + ", ".join(unique)
        )

    return unique[0]


def unique_labels(
    labels: list[str],
    bag_paths: list[Path],
) -> list[str]:
    """Keep natural labels while disambiguating duplicate run names."""
    counts = {
        label: labels.count(label)
        for label in set(labels)
    }
    used: dict[str, int] = {}
    result = []

    for label, path in zip(labels, bag_paths):
        candidate = label

        if counts[label] > 1:
            candidate = f"{label}:{path.name}"

        number = used.get(candidate, 0) + 1
        used[candidate] = number

        if number > 1:
            candidate = f"{candidate}#{number}"

        result.append(candidate)

    return result


def _ordered_unique_series(
    series: dict[str, list[float]],
    components: tuple[str, ...],
):
    times = np.asarray(series["times_s"], dtype=float)
    order = np.argsort(times)
    times = times[order]
    times, unique = np.unique(times, return_index=True)

    values = {
        component: np.asarray(
            series[component],
            dtype=float,
        )[order][unique]
        for component in components
    }

    return times, values


def tracking_error_series(
    actual: dict[str, list[float]],
    reference: dict[str, list[float]],
    *,
    time_origin_s: float,
    end_time_s: float | None = None,
    components: tuple[str, ...] = ("x", "y", "z"),
) -> dict[str, list[float]]:
    """Return actual-reference error over the selected experiment interval."""
    actual_times, actual_values = _ordered_unique_series(
        actual,
        components,
    )
    reference_times, reference_values = _ordered_unique_series(
        reference,
        components,
    )

    empty = {
        "times_s": [],
        **{component: [] for component in components},
        "norm": [],
    }

    if actual_times.size == 0 or reference_times.size == 0:
        return empty

    start = max(
        actual_times[0],
        reference_times[0],
        time_origin_s,
    )
    end = min(
        actual_times[-1],
        reference_times[-1],
    )

    if end_time_s is not None:
        end = min(end, end_time_s)

    mask = (
        (actual_times >= start)
        & (actual_times <= end)
    )

    if not np.any(mask):
        return empty

    selected_times = actual_times[mask]
    errors = {}

    for component in components:
        errors[component] = (
            actual_values[component][mask]
            - np.interp(
                selected_times,
                reference_times,
                reference_values[component],
            )
        )

    norm = np.sqrt(
        sum(
            errors[component] ** 2
            for component in components
        )
    )

    return {
        "times_s": (
            selected_times - time_origin_s
        ).tolist(),
        **{
            component: errors[component].tolist()
            for component in components
        },
        "norm": norm.tolist(),
    }


def shift_vector_series(
    series: dict[str, list[float]],
    time_origin_s: float,
    duration_s: float,
) -> dict[str, list[float]]:
    """Shift and clip a three-axis series to the experiment interval."""
    times = np.asarray(series["times_s"], dtype=float)
    start = time_origin_s
    end = time_origin_s + duration_s
    mask = (
        (times >= start)
        & (times <= end)
    )

    return {
        "times_s": (
            times[mask] - time_origin_s
        ).tolist(),
        **{
            component: np.asarray(
                series[component],
                dtype=float,
            )[mask].tolist()
            for component in ("x", "y", "z")
        },
    }


def shift_motor_series(
    motors: dict[str, tuple[list[float], list[float]]],
    time_origin_s: float,
    duration_s: float,
) -> dict[str, tuple[list[float], list[float]]]:
    """Shift and clip motor histories to the experiment interval."""
    start = time_origin_s
    end = time_origin_s + duration_s
    result = {}

    for name, (raw_times, raw_values) in motors.items():
        times = np.asarray(raw_times, dtype=float)
        values = np.asarray(raw_values, dtype=float)
        mask = (
            (times >= start)
            & (times <= end)
        )

        result[name] = (
            (times[mask] - time_origin_s).tolist(),
            values[mask].tolist(),
        )

    return result


def series_stats(values: list[float]) -> dict[str, float]:
    """Return RMS and peak absolute value for a scalar series."""
    data = np.asarray(values, dtype=float)

    if data.size == 0:
        return {
            "rms": float("nan"),
            "peak": float("nan"),
        }

    return {
        "rms": float(
            np.sqrt(np.mean(data ** 2))
        ),
        "peak": float(
            np.max(np.abs(data))
        ),
    }


def activity_stats(
    times_s: list[float],
    values: list[float],
) -> dict[str, float]:
    """Measure scalar magnitude and time-normalized roughness."""
    times = np.asarray(times_s, dtype=float)
    data = np.asarray(values, dtype=float)

    base = series_stats(values)

    if data.size < 2:
        return {
            **base,
            "roughness_rms": float("nan"),
        }

    dt = np.diff(times)
    du = np.diff(data)
    valid = dt > 0.0

    if not np.any(valid):
        roughness = float("nan")
    else:
        derivative = du[valid] / dt[valid]
        roughness = float(
            np.sqrt(np.mean(derivative ** 2))
        )

    return {
        **base,
        "roughness_rms": roughness,
    }


def effort_stats(
    times_s: list[float],
    values: list[float],
    *,
    limit: float = 1.0,
    near_limit_ratio: float = 0.99,
) -> dict[str, float]:
    """Measure command activity and fraction near its nominal unit limit."""
    data = np.asarray(values, dtype=float)
    result = activity_stats(
        times_s,
        values,
    )

    if data.size == 0:
        near_limit_fraction = float("nan")
    else:
        near_limit_fraction = float(
            np.mean(
                np.abs(data)
                >= near_limit_ratio * limit
            )
        )

    return {
        **result,
        "near_limit_fraction": near_limit_fraction,
    }


def vector_magnitude_series(
    series: dict[str, list[float]],
) -> tuple[list[float], list[float]]:
    """Return Euclidean magnitude for one three-axis vector series."""
    if not series["times_s"]:
        return [], []

    values = np.column_stack(
        [
            np.asarray(
                series[component],
                dtype=float,
            )
            for component in ("x", "y", "z")
        ]
    )

    magnitude = np.linalg.norm(
        values,
        axis=1,
    )

    return (
        list(series["times_s"]),
        magnitude.tolist(),
    )


def motor_envelope(
    motors: dict[str, tuple[list[float], list[float]]],
) -> tuple[list[float], list[float]]:
    """Return the maximum absolute motor command on a common time grid."""
    if not motors:
        return [], []

    first_times, _ = next(
        iter(motors.values())
    )
    grid = np.asarray(
        first_times,
        dtype=float,
    )

    if grid.size == 0:
        return [], []

    channels = []

    for raw_times, raw_values in motors.values():
        times = np.asarray(
            raw_times,
            dtype=float,
        )
        values = np.asarray(
            raw_values,
            dtype=float,
        )

        if times.size == 0:
            continue

        order = np.argsort(times)
        times = times[order]
        values = values[order]
        times, unique = np.unique(
            times,
            return_index=True,
        )
        values = values[unique]

        channels.append(
            np.interp(
                grid,
                times,
                values,
            )
        )

    if not channels:
        return [], []

    envelope = np.max(
        np.abs(np.vstack(channels)),
        axis=0,
    )

    return (
        grid.tolist(),
        envelope.tolist(),
    )


def interval_series(
    series: dict[str, list[float]],
    start_s: float,
    end_s: float,
) -> dict[str, list[float]]:
    """Slice one aligned series by comparison-relative time."""
    times = np.asarray(
        series["times_s"],
        dtype=float,
    )
    mask = (
        (times >= start_s)
        & (times < end_s)
    )

    return {
        key: (
            times[mask].tolist()
            if key == "times_s"
            else np.asarray(
                values,
                dtype=float,
            )[mask].tolist()
        )
        for key, values in series.items()
    }


def interval_scalar(
    times_s: list[float],
    values: list[float],
    start_s: float,
    end_s: float,
) -> tuple[list[float], list[float]]:
    """Slice one scalar time series by comparison-relative time."""
    times = np.asarray(
        times_s,
        dtype=float,
    )
    data = np.asarray(
        values,
        dtype=float,
    )
    mask = (
        (times >= start_s)
        & (times < end_s)
    )

    return (
        times[mask].tolist(),
        data[mask].tolist(),
    )
