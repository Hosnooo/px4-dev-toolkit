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


# ros2 bag is started before the node. Give DDS discovery a short window before
# control starts so the beginning of the handoff is not missed.
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
    """Resolve this package's recording selection through the topic catalog."""
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


def make_control_node() -> Node:
    """Create the Offboard position controller node used by both launch modes."""
    package_share = Path(
        get_package_share_directory("offboard_controllers")
    )
    config_file = package_share / "config" / "offboard.yaml"

    if not config_file.is_file():
        raise RuntimeError(
            f"Missing Offboard configuration: {config_file}"
        )

    return Node(
        package="offboard_controllers",
        executable="offboard_position",
        name="offboard_position",
        output="screen",
        emulate_tty=True,
        parameters=[str(config_file)],
    )


def launch_setup(context):
    """
    Build either the simple control launch or the recording orchestration.

    Without recording, the launch contains only the control node. With
    recording, rosbag starts first and stops when the control node exits.
    """
    record = launch_argument_is_true(context, "record")
    control_node = make_control_node()

    if not record:
        def on_control_exit(event, _context):
            reason = (
                "offboard_position completed."
                if event.returncode == 0
                else
                f"offboard_position exited with status {event.returncode}."
            )

            return [
                EmitEvent(
                    event=Shutdown(reason=reason)
                )
            ]

        return [
            control_node,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=control_node,
                    on_exit=on_control_exit,
                )
            ),
        ]

    repo_root = find_repo_root()
    package_share = Path(
        get_package_share_directory("offboard_controllers")
    )
    topic_file = package_share / "config" / "recording_topics_position.txt"
    topic_catalog_file = repo_root / "config" / "px4_topics.def"

    topic_catalog = read_topic_catalog(topic_catalog_file)
    topics = read_recording_topics(topic_file, topic_catalog)

    timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    bag_root = repo_root / "bags" / "offboard_position"
    bag_path = bag_root / timestamp

    bag_root.mkdir(parents=True, exist_ok=True)

    bag_process = ExecuteProcess(
        cmd=[
            "ros2",
            "bag",
            "record",
            "--output",
            str(bag_path),
            "--topics",
            *topics,
        ],
        name="offboard_position_recorder",
        output="screen",
        emulate_tty=True,
    )

    def on_control_exit(event, _context):
        if event.returncode == 0:
            message = (
                "Offboard position controller complete; stopping recording."
            )

        else:
            message = (
                "Offboard position controller failed; stopping recording."
            )

        return [
            LogInfo(msg=message),
            EmitEvent(
                event=ShutdownProcess(
                    process_matcher=matches_action(bag_process)
                )
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
        # process has actually started, keeping early control traffic in the bag.
        bag_process,
        RegisterEventHandler(
            OnProcessStart(
                target_action=bag_process,
                on_start=[
                    TimerAction(
                        period=RECORDER_STARTUP_SECONDS,
                        actions=[control_node],
                    )
                ],
            )
        ),

        RegisterEventHandler(
            OnProcessExit(
                target_action=control_node,
                on_exit=on_control_exit,
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
                    "Record the Offboard position controller run."
                ),
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
