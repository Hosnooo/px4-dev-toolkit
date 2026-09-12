from pathlib import Path
import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess, LogInfo


def find_repo_root() -> Path:
    start = Path(__file__).resolve()

    for path in (start.parent, *start.parents):
        if (
            (path / "px4_env.repos").is_file()
            and (path / "config").is_dir()
            and (path / "px4").is_dir()
        ):
            return path

    raise RuntimeError(f"Could not locate px4_env repository root from {start}")


def read_env_file(path: Path) -> dict[str, str]:
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
    sitl = read_env_file(root / "config" / "sitl.env")

    build_target = sitl["PX4_BUILD_TARGET"]
    sim_model = sitl["PX4_SIM_MODEL"]
    world = sitl["PX4_GZ_WORLD"]
    gazebo_gui = sitl["GAZEBO_GUI"]

    px4_dir = root / "px4" / "PX4-Autopilot"

    if not px4_dir.is_dir():
        raise RuntimeError(f"PX4 source directory does not exist: {px4_dir}")

    environment = os.environ.copy()
    environment["PX4_GZ_WORLD"] = world

    if gazebo_gui == "true":
        environment.pop("HEADLESS", None)
    elif gazebo_gui == "false":
        environment["HEADLESS"] = "1"
    else:
        raise RuntimeError("GAZEBO_GUI must be either 'true' or 'false'.")

    return LaunchDescription(
        [
            LogInfo(
                msg=[
                    "PX4 SITL: ",
                    sim_model,
                    " world ",
                    world,
                    " Gazebo GUI=",
                    gazebo_gui,
                ]
            ),
            ExecuteProcess(
                cmd=[
                    "make",
                    build_target,
                    sim_model,
                ],
                cwd=str(px4_dir),
                env=environment,
                output="screen",
                emulate_tty=True,
            ),
        ]
    )
