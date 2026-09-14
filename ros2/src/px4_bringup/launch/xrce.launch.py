"""Launch the Micro XRCE-DDS Agent used by PX4 SITL and ROS 2."""

from pathlib import Path
import shutil

from launch import LaunchDescription
from launch.actions import ExecuteProcess, LogInfo


def find_repo_root() -> Path:
    """Locate px4_env from the installed or source launch-file location."""
    start = Path(__file__).resolve()

    for path in (start.parent, *start.parents):
        if (
            (path / "px4_env.repos").is_file()
            and (path / "config").is_dir()
            and (path / "ros2").is_dir()
        ):
            return path

    raise RuntimeError(f"Could not locate px4_env repository root from {start}")


def read_env_file(path: Path) -> dict[str, str]:
    """Read the repository's simple KEY=VALUE configuration files."""
    values = {}

    for raw_line in path.read_text().splitlines():
        line = raw_line.strip()

        if not line or line.startswith("#"):
            continue

        if "=" not in line:
            raise RuntimeError(f"Invalid config line in {path}: {raw_line}")

        key, value = line.split("=", 1)
        values[key.strip()] = value.strip()

    return values


def generate_launch_description() -> LaunchDescription:
    root = find_repo_root()
    runtime = read_env_file(root / "config" / "env.env")

    transport = runtime["XRCE_AGENT_TRANSPORT"]
    port = runtime["XRCE_AGENT_PORT"]

    # The agent executable comes from the built ROS workspace. Requiring it on
    # PATH here catches an unsourced/unbuilt workspace before starting PX4 I/O.
    agent = shutil.which("MicroXRCEAgent")

    if agent is None:
        raise RuntimeError(
            "MicroXRCEAgent was not found. "
            "Source the ROS workspace before launching."
        )

    return LaunchDescription(
        [
            LogInfo(
                msg=[
                    "XRCE Agent: ",
                    transport,
                    " port ",
                    port,
                ]
            ),
            ExecuteProcess(
                cmd=[
                    agent,
                    transport,
                    "-p",
                    port,
                ],
                output="screen",
            ),
        ]
    )
