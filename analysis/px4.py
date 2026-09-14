"""Common PX4 conventions used by flight-data analysis."""

from __future__ import annotations

from collections.abc import Sequence
from pathlib import Path
import re

from scipy.spatial.transform import Rotation


_TOPIC_FILE = (
    Path(__file__).resolve().parents[1]
    / "config"
    / "px4_topics.def"
)

_TOPIC_PATTERN = re.compile(
    r'^PX4_TOPIC\(([A-Z0-9_]+),\s*"([^"]+)"\)$'
)


def _load_topics() -> dict[str, str]:
    """Load the project-wide PX4 ROS topic catalog."""
    topics: dict[str, str] = {}

    for raw_line in _TOPIC_FILE.read_text().splitlines():
        line = raw_line.strip()

        if not line or line.startswith("//"):
            continue

        match = _TOPIC_PATTERN.fullmatch(line)

        if match is None:
            raise RuntimeError(
                f"Invalid PX4 topic definition: {raw_line}"
            )

        name, topic = match.groups()
        topics[name] = topic

    return topics


TOPICS = _load_topics()


def quaternion_to_euler(
    quaternion: Sequence[float],
) -> tuple[float, float, float]:
    """Convert PX4 [w, x, y, z] quaternion to roll, pitch, yaw in radians."""
    w, x, y, z = (float(value) for value in quaternion)

    roll, pitch, yaw = Rotation.from_quat(
        [x, y, z, w]
    ).as_euler("xyz")

    return float(roll), float(pitch), float(yaw)
