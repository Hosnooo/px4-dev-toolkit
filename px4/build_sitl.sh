#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# Reuse the same PX4 target selected for runtime SITL. This script only
# prebuilds PX4; tools/sitl remains responsible for launching the simulator.
source "${ROOT}/config/env.env"

cd "${ROOT}/px4/PX4-Autopilot"
make "${PX4_BUILD_TARGET}_default"

echo "PX4 SITL build successful."
