#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MANIFEST="${ROOT}/px4_env.repos"

REPOSITORIES=(
    "${ROOT}/px4/PX4-Autopilot"
    "${ROOT}/ros2/src/px4_msgs"
    "${ROOT}/ros2/src/Micro-XRCE-DDS-Agent"
    "${ROOT}/ros2/src/px4_ros_com"
)

if ! command -v vcs >/dev/null 2>&1; then
    echo "ERROR: vcs is not installed."
    exit 1
fi

vcs validate < "${MANIFEST}"

existing=0

for repo in "${REPOSITORIES[@]}"; do
    if [[ -d "${repo}" ]]; then
        ((existing += 1))
    fi
done

if [[ "${existing}" -eq 0 ]]; then
    echo "Fetching pinned source repositories..."
    vcs import --input "${MANIFEST}" "${ROOT}"
elif [[ "${existing}" -eq "${#REPOSITORIES[@]}" ]]; then
    echo "Source repositories already exist; keeping current checkouts."
else
    echo "ERROR: Source tree is partially populated."
    echo "Refusing to modify it automatically."
    exit 1
fi

echo "Initializing PX4 nested submodules..."

git -C "${ROOT}/px4/PX4-Autopilot" \
    submodule update --init --recursive

echo "Source repositories ready."
