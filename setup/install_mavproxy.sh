#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

MAVPROXY_SOURCE="${ROOT}/tools/MAVProxy"
MAVPROXY_VENV="${ROOT}/.venv/mavproxy"
MAVPROXY_REQUIREMENTS="${ROOT}/config/mavproxy-requirements.txt"
MAVPROXY_PYTHON="${MAVPROXY_VENV}/bin/python"
MAVPROXY_BIN="${MAVPROXY_VENV}/bin/mavproxy.py"

if [[ ! -d "${MAVPROXY_SOURCE}/.git" ]]; then
    echo "ERROR: MAVProxy source repository does not exist:"
    echo "  ${MAVPROXY_SOURCE}"
    exit 1
fi

if [[ ! -f "${MAVPROXY_REQUIREMENTS}" ]]; then
    echo "ERROR: MAVProxy requirements file does not exist:"
    echo "  ${MAVPROXY_REQUIREMENTS}"
    exit 1
fi

# Keep MAVProxy isolated from the user's Python environment. This avoids ROS,
# user-site, or shell PYTHONPATH settings changing the pinned dependency set.
mkdir -p "${ROOT}/.venv"

if [[ ! -x "${MAVPROXY_PYTHON}" ]]; then
    echo "Creating MAVProxy Python environment..."
    python3 -m venv "${MAVPROXY_VENV}"
fi

run_python() {
    env \
        -u PYTHONPATH \
        -u PYTHONHOME \
        PYTHONNOUSERSITE=1 \
        "$@"
}

# Dependencies and MAVProxy itself are installed without dependency resolution.
# The exact versions are owned by this repo, not by whatever pip selects today.
echo "Installing pinned MAVProxy Python dependencies..."

run_python "${MAVPROXY_PYTHON}" -m pip install \
    --disable-pip-version-check \
    --no-deps \
    -r "${MAVPROXY_REQUIREMENTS}"

echo "Installing MAVProxy from pinned source checkout..."

run_python "${MAVPROXY_PYTHON}" -m pip install \
    --disable-pip-version-check \
    --no-deps \
    --no-build-isolation \
    "${MAVPROXY_SOURCE}"

if [[ ! -x "${MAVPROXY_BIN}" ]]; then
    echo "ERROR: MAVProxy executable was not installed:"
    echo "  ${MAVPROXY_BIN}"
    exit 1
fi

# Verify both dependency consistency and the exact environment versions that
# were validated for this project. This catches accidental drift in the venv.
run_python "${MAVPROXY_PYTHON}" -m pip check

run_python "${MAVPROXY_PYTHON}" - <<'PY'
from importlib.metadata import version

expected = {
    "MAVProxy": "1.8.74",
    "setuptools": "68.1.2",
    "wheel": "0.42.0",
    "future": "1.0.0",
    "pymavlink": "2.4.49",
    "pyserial": "3.5",
    "numpy": "2.5.3",
    "pynmeagps": "1.1.7",
    "lxml": "6.1.3",
    "fastcrc": "0.3.6",
}

for package, required in expected.items():
    installed = version(package)
    if installed != required:
        raise SystemExit(
            f"ERROR: {package} {installed} installed; expected {required}"
        )

print("MAVProxy 1.8.74 environment verified.")
PY

"${MAVPROXY_BIN}" --help >/dev/null

echo "MAVProxy ready."
