"""Experiment-specific analysis profiles and profile discovery."""

from __future__ import annotations

import json
from pathlib import Path

from . import (
    offboard_position,
    offboard_takeoff_handoff,
    position_takeoff_hover,
    se3,
)


PROFILES = {
    position_takeoff_hover.PROFILE_NAME: position_takeoff_hover,
    offboard_position.PROFILE_NAME: offboard_position,
    offboard_takeoff_handoff.PROFILE_NAME: offboard_takeoff_handoff,
    se3.PROFILE_NAME: se3,
}


def get_profile(name: str):
    """Return one registered experiment profile by its stable name."""
    try:
        return PROFILES[name]
    except KeyError as exc:
        available = ", ".join(
            sorted(PROFILES)
        )
        raise ValueError(
            f"unknown analysis profile {name!r}; "
            f"available: {available}"
        ) from exc


def detect_profile_name(
    bag_path: Path,
) -> str:
    """Return explicit bag profile, falling back to its parent directory."""
    metadata_path = (
        bag_path / "experiment.json"
    )

    if metadata_path.is_file():
        try:
            data = json.loads(
                metadata_path.read_text()
            )
        except json.JSONDecodeError as exc:
            raise RuntimeError(
                "Invalid experiment metadata JSON: "
                f"{metadata_path}"
            ) from exc

        profile = data.get("profile")

        if (
            isinstance(profile, str)
            and profile
        ):
            return profile

        raise RuntimeError(
            "Invalid analysis profile metadata: "
            f"{metadata_path}"
        )

    return bag_path.parent.name
