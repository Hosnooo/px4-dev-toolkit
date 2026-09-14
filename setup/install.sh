#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

echo "========================================"
echo " Installing PX4 environment"
echo "========================================"

# Keep installation stages explicit and independently runnable. The order is
# intentional: host tooling -> pinned sources -> source-specific dependencies
# -> optional ground-control application -> ROS workspace build.
"${ROOT}/setup/install_ros2_jazzy.sh"
"${ROOT}/setup/install_common_tools.sh"
"${ROOT}/setup/fetch_sources.sh"
"${ROOT}/setup/install_mavproxy.sh"
"${ROOT}/setup/install_px4_dependencies.sh"
"${ROOT}/setup/install_qgroundcontrol.sh"
"${ROOT}/ros2/build.sh"

echo
echo "========================================"
echo " PX4 environment installation complete"
echo "========================================"
