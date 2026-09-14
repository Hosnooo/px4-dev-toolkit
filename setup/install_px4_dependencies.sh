#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PX4="${ROOT}/px4/PX4-Autopilot"

if [[ ! -f "${PX4}/Tools/setup/ubuntu.sh" ]]; then
    echo "ERROR: PX4 source is missing."
    echo "Run fetch_sources.sh first."
    exit 1
fi

# Use the dependency installer shipped by the pinned PX4 checkout instead of
# duplicating PX4's Ubuntu package list in this repository.
echo "Installing PX4 development dependencies..."

bash "${PX4}/Tools/setup/ubuntu.sh"

echo "PX4 dependencies ready."
