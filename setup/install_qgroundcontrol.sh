#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

source "${ROOT}/config/versions.env"

QGC_DIR="${ROOT}/tools/qgroundcontrol"
QGC_PATH="${QGC_DIR}/${QGC_FILENAME}"
TMP_PATH="${QGC_PATH}.download"

sudo apt update
sudo apt install -y \
    libfuse2 \
    libxcb-xinerama0 \
    libxkbcommon-x11-0 \
    libxcb-cursor0

mkdir -p "${QGC_DIR}"

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

echo "${QGC_SHA256}  ${TMP_PATH}" | sha256sum --check

mv "${TMP_PATH}" "${QGC_PATH}"
chmod +x "${QGC_PATH}"

echo "QGroundControl ${QGC_VERSION} installed."
