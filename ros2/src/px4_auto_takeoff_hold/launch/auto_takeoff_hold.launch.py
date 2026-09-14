from datetime import datetime
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.events import Shutdown, matches_action
from launch.events.process import ShutdownProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# The flight node exits immediately after PX4 confirms AUTO_LOITER.
# Keep recording for a fixed window afterward so hold drift and velocity can
# be measured without keeping the C++ flight node alive.
HOLD_RECORD_SECONDS = 20.0

# ros2 bag is started before the maneuver. Give DDS discovery a short window
# before the flight node starts so early arm/takeoff traffic is not missed.
RECORDER_STARTUP_SECONDS = 1.0


def find_repo_root() -> Path:
    """
    Locate px4_env from either the source or installed launch-file path.

    colcon installs this launch file below ros2/install/, which is still inside
    the repository. Repository markers keep bag output anchored to px4_env/bags
    instead of depending on the shell's current working directory.
    """
    start = Path(__file__).resolve()

    for path in (start.parent, *start.parents):
        if (
            (path / "px4_env.repos").is_file()
            and (path / "config").is_dir()
            and (path / "ros2").is_dir()
        ):
            return path

    raise RuntimeError(
        f"Could not locate px4_env repository root from {start}"
    )


def read_topic_catalog(path: Path) -> dict[str, str]:
    """Load the project-wide PX4 topic-name catalog."""
    topics = {}

    for raw_line in path.read_text().splitlines():
        line = raw_line.strip()

        if not line or line.startswith("//"):
            continue

        prefix = "PX4_TOPIC("
        if not line.startswith(prefix) or not line.endswith(")"):
            raise RuntimeError(
                f"Invalid PX4 topic definition: {raw_line}"
            )

        name, value = line[len(prefix):-1].split(",", 1)
        topics[name.strip()] = value.strip().strip('"')

    return topics


def read_recording_topics(
    path: Path,
    topic_catalog: dict[str, str],
) -> list[str]:
    """Resolve this experiment's recording selection through the topic catalog."""
    topics = []

    for raw_line in path.read_text().splitlines():
        name = raw_line.strip()

        if not name or name.startswith("#"):
            continue

        if name not in topic_catalog:
            raise RuntimeError(
                f"Unknown PX4 topic name in {path}: {name}"
            )

        topics.append(topic_catalog[name])

    if not topics:
        raise RuntimeError(f"Recording topic list is empty: {path}")

    return topics


def launch_argument_is_true(context, name: str) -> bool:
    """Interpret a ROS launch boolean argument using the usual truthy forms."""
    value = LaunchConfiguration(name).perform(context).strip().lower()
    return value in {"1", "true", "yes", "on"}


def make_flight_node() -> Node:
    """Create the finite native-PX4 takeoff/hold node used by both launch modes."""
    return Node(
        package="px4_auto_takeoff_hold",
        executable="auto_takeoff_hold",
        name="auto_takeoff_hold",
        output="screen",
        emulate_tty=True,
    )


def launch_setup(context):
    """
    Build either the simple flight launch or the recording orchestration.

    Without recording, the launch contains only the finite flight node.
    With recording, rosbag starts first, then the flight runs, then rosbag
    remains alive for HOLD_RECORD_SECONDS after successful AUTO_LOITER entry.
    """
    record = launch_argument_is_true(context, "record")
    flight_node = make_flight_node()

    if not record:
        def on_flight_exit(event, _context):
            reason = (
                "auto_takeoff_hold completed."
                if event.returncode == 0
                else
                f"auto_takeoff_hold exited with status {event.returncode}."
            )

            return [
                EmitEvent(
                    event=Shutdown(reason=reason)
                )
            ]

        return [
            flight_node,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=flight_node,
                    on_exit=on_flight_exit,
                )
            ),
        ]

    repo_root = find_repo_root()
    package_share = Path(
        get_package_share_directory("px4_auto_takeoff_hold")
    )
    topic_file = package_share / "config" / "recording_topics.txt"
    topic_catalog_file = repo_root / "config" / "px4_topics.def"

    topic_catalog = read_topic_catalog(topic_catalog_file)
    topics = read_recording_topics(topic_file, topic_catalog)

    timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    bag_root = repo_root / "bags" / "auto_takeoff_hold"
    bag_path = bag_root / timestamp

    # rosbag creates the timestamp directory itself, but its parent must exist.
    bag_root.mkdir(parents=True, exist_ok=True)

    bag_process = ExecuteProcess(
        cmd=[
            "ros2",
            "bag",
            "record",
            "--output",
            str(bag_path),
            *topics,
        ],
        name="auto_takeoff_hold_recorder",
        output="screen",
        emulate_tty=True,
    )

    def on_flight_exit(event, _context):
        if event.returncode != 0:
            return [
                LogInfo(
                    msg=(
                        "auto_takeoff_hold failed; stopping recording "
                        "without the post-hold window."
                    )
                ),
                EmitEvent(
                    event=ShutdownProcess(
                        process_matcher=matches_action(bag_process)
                    )
                ),
            ]

        return [
            LogInfo(
                msg=(
                    "AUTO_LOITER confirmed. Continuing recording for "
                    f"{HOLD_RECORD_SECONDS:.0f} seconds."
                )
            ),
            TimerAction(
                period=HOLD_RECORD_SECONDS,
                actions=[
                    LogInfo(msg="Post-hold recording complete."),
                    EmitEvent(
                        event=ShutdownProcess(
                            process_matcher=matches_action(bag_process)
                        )
                    ),
                ],
            ),
        ]

    def on_bag_exit(_event, _context):
        return [
            LogInfo(msg=["Bag saved: ", str(bag_path)]),
            EmitEvent(
                event=Shutdown(reason="ROS bag recorder stopped.")
            ),
        ]

    return [
        LogInfo(msg=["Recording bag: ", str(bag_path)]),

        # Start rosbag first. The short timer begins only after the recorder
        # process has actually started, keeping early flight traffic in the bag.
        bag_process,
        RegisterEventHandler(
            OnProcessStart(
                target_action=bag_process,
                on_start=[
                    TimerAction(
                        period=RECORDER_STARTUP_SECONDS,
                        actions=[flight_node],
                    )
                ],
            )
        ),

        RegisterEventHandler(
            OnProcessExit(
                target_action=flight_node,
                on_exit=on_flight_exit,
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=bag_process,
                on_exit=on_bag_exit,
            )
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "record",
                default_value="false",
                description=(
                    "Record the core PX4 experiment topics and keep recording "
                    "for 20 seconds after AUTO_LOITER is confirmed."
                ),
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
