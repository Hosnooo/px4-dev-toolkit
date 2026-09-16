
"""Experiment-specific analysis profiles."""

from . import (
    offboard_position,
    offboard_takeoff_handoff,
    position_takeoff_hover,
)


PROFILES = {
    position_takeoff_hover.PROFILE_NAME: position_takeoff_hover,
    offboard_position.PROFILE_NAME: offboard_position,
    offboard_takeoff_handoff.PROFILE_NAME: offboard_takeoff_handoff,
}


def get_profile(name: str):
    """Return one registered experiment profile by its stable name."""

    try:
        return PROFILES[name]
    except KeyError as exc:
        available = ", ".join(sorted(PROFILES))
        raise ValueError(
            f"unknown analysis profile {name!r}; available: {available}"
        ) from exc
