from pathlib import Path

from launch import LaunchDescription
from launch.actions import ExecuteProcess, LogInfo


def find_repo_root() -> Path:
    start = Path(__file__).resolve()

    for path in (start.parent, *start.parents):
        if (
            (path / "px4_env.repos").is_file()
            and (path / "config").is_dir()
            and (path / "tools").is_dir()
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
    versions = read_env_file(root / "config" / "versions.env")

    qgc_path = (
        root
        / "tools"
        / "qgroundcontrol"
        / versions["QGC_FILENAME"]
    )

    if not qgc_path.is_file():
        raise RuntimeError(f"QGroundControl does not exist: {qgc_path}")

    if not qgc_path.stat().st_mode & 0o111:
        raise RuntimeError(f"QGroundControl is not executable: {qgc_path}")

    return LaunchDescription(
        [
            LogInfo(msg=["QGroundControl: ", str(qgc_path)]),
            ExecuteProcess(
                cmd=[str(qgc_path)],
                output="screen",
            ),
        ]
    )
