#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# ROS-generated setup scripts are not safe under Bash nounset.
set +u
source /opt/ros/jazzy/setup.bash
set -u

cd "${ROOT}/ros2"

colcon build

echo
echo "ROS 2 workspace built successfully."
