#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

source "${ROOT}/config/sitl.env"

cd "${ROOT}/px4/PX4-Autopilot"

make "${PX4_BUILD_TARGET}_default"

echo "PX4 SITL build successful."
