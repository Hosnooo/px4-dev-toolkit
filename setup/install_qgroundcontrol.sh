#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

source "${ROOT}/config/versions.env"

QGC_DIR="${ROOT}/tools/qgroundcontrol"
QGC_PATH="${QGC_DIR}/${QGC_FILENAME}"
TMP_PATH="${QGC_PATH}.download"

# QGroundControl is kept as a repository-managed AppImage rather than a system
# package. Install only the host libraries required to run that AppImage.
sudo apt update
sudo apt install -y \
    libfuse2 \
    libxcb-xinerama0 \
    libxkbcommon-x11-0 \
    libxcb-cursor0

mkdir -p "${QGC_DIR}"

# Reuse an existing download only when it matches the pinned checksum.
if [[ -f "${QGC_PATH}" ]]; then
    if echo "${QGC_SHA256}  ${QGC_PATH}" | sha256sum --check --status; then
        echo "QGroundControl ${QGC_VERSION} already installed and verified."
        exit 0
    fi

    echo "Existing QGroundControl checksum does not match."
    rm -f "${QGC_PATH}"
fi

echo "Downloading QGroundControl ${QGC_VERSION}..."

curl -L "${QGC_URL}" -o "${TMP_PATH}"

# Verify before moving the temporary download into the runnable location.
echo "${QGC_SHA256}  ${TMP_PATH}" | sha256sum --check

mv "${TMP_PATH}" "${QGC_PATH}"
chmod +x "${QGC_PATH}"

echo "QGroundControl ${QGC_VERSION} installed."
