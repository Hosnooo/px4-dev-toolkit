
"""Experiment-specific analysis profiles."""

from . import auto_takeoff_hold


PROFILES = {
    auto_takeoff_hold.PROFILE_NAME: auto_takeoff_hold,
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
