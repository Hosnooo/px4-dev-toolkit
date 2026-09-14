
"""Common ROS bag loading and PX4 state-analysis utilities."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Iterable


@dataclass(frozen=True)
class TimedSample:
    """One deserialized ROS message with its rosbag receive timestamp."""

    timestamp_ns: int
    message: object


@dataclass
class BagData:
    """
    The subset of one bag requested by an experiment profile.

    Rosbag timestamps are the common analysis clock. They avoid mixing PX4
    boot-relative message timestamps with host wall-clock timestamps.
    """

    path: Path
    start_ns: int
    end_ns: int
    samples: dict[str, list[TimedSample]]

    def relative_seconds(self, timestamp_ns: int) -> float:
        return (timestamp_ns - self.start_ns) / 1_000_000_000.0

    @property
    def duration_s(self) -> float:
        return self.relative_seconds(self.end_ns)


def storage_identifier(metadata_path: Path) -> str:
    """
    Read the rosbag storage backend from metadata.yaml.

    The analyzer stays storage-agnostic: MCAP and SQLite bags use the same
    analysis path as long as the corresponding rosbag2 storage plugin exists.
    """

    for raw_line in metadata_path.read_text().splitlines():
        line = raw_line.strip()

        if line.startswith("storage_identifier:"):
            value = line.split(":", 1)[1].strip()

            if value:
                return value

    raise RuntimeError(
        f"Could not find storage_identifier in {metadata_path}"
    )


def read_bag(
    bag_path: Path,
    requested_topics: Iterable[str],
) -> BagData:
    """
    Deserialize only the topics needed by the selected experiment profile.

    ROS imports are intentionally local to this function. Pure analysis logic
    remains testable without a running ROS installation, while real bag reads
    use the ROS 2 environment sourced by the user.
    """

    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except ImportError as exc:
        raise RuntimeError(
            "ROS 2 Python bag libraries are unavailable. "
            "Run `source ros2/runtime_env.sh` before px4_analyze."
        ) from exc

    bag_path = bag_path.expanduser().resolve()
    metadata_path = bag_path / "metadata.yaml"

    if not metadata_path.is_file():
        raise RuntimeError(
            f"Not a ROS 2 bag directory (metadata.yaml missing): {bag_path}"
        )

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(
            uri=str(bag_path),
            storage_id=storage_identifier(metadata_path),
        ),
        rosbag2_py.ConverterOptions("", ""),
    )

    topic_types = {
        entry.name: entry.type
        for entry in reader.get_all_topics_and_types()
    }

    requested = set(requested_topics)
    missing = sorted(requested - topic_types.keys())

    if missing:
        raise RuntimeError(
            "Bag is missing required topic(s): " + ", ".join(missing)
        )

    message_types = {
        topic: get_message(topic_types[topic])
        for topic in requested
    }
    samples = {topic: [] for topic in requested}

    bag_start_ns: int | None = None
    bag_end_ns: int | None = None

    while reader.has_next():
        topic, serialized, timestamp_ns = reader.read_next()

        if bag_start_ns is None:
            bag_start_ns = timestamp_ns

        bag_end_ns = timestamp_ns

        if topic not in requested:
            continue

        samples[topic].append(
            TimedSample(
                timestamp_ns=timestamp_ns,
                message=deserialize_message(
                    serialized,
                    message_types[topic],
                ),
            )
        )

    if bag_start_ns is None or bag_end_ns is None:
        raise RuntimeError(f"Bag contains no messages: {bag_path}")

    return BagData(
        path=bag_path,
        start_ns=bag_start_ns,
        end_ns=bag_end_ns,
        samples=samples,
    )


def native_constant_names(
    message_type: type,
    prefix: str,
) -> dict[int, str]:
    """
    Build display names from generated PX4 message constants.

    Analysis follows the installed px4_msgs definitions instead of maintaining
    a duplicate hand-written navigation-state mapping.
    """

    names: dict[int, str] = {}

    for name in dir(message_type):
        if not name.startswith(prefix):
            continue

        value = getattr(message_type, name)

        if isinstance(value, int):
            names.setdefault(value, name.removeprefix(prefix))

    return names


def mode_transitions(
    samples: list[TimedSample],
) -> list[tuple[int, int]]:
    """Return navigation-state changes without repeated identical samples."""

    transitions: list[tuple[int, int]] = []
    previous_state: int | None = None

    for sample in samples:
        state = int(sample.message.nav_state)

        if state != previous_state:
            transitions.append((sample.timestamp_ns, state))
            previous_state = state

    return transitions


def first_nav_state_time(
    samples: list[TimedSample],
    state: int,
    *,
    not_before_ns: int | None = None,
) -> int | None:
    """Find the first occurrence of one native PX4 navigation state."""

    for sample in samples:
        if (
            not_before_ns is not None
            and sample.timestamp_ns < not_before_ns
        ):
            continue

        if int(sample.message.nav_state) == state:
            return sample.timestamp_ns

    return None
