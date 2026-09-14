#!/usr/bin/env bash
set -euo pipefail

source /etc/os-release

# The repository has only been validated on Ubuntu 24.04. Refuse a different
# host release instead of attempting a best-effort ROS installation.
if [[ "${VERSION_ID}" != "24.04" ]]; then
    echo "ERROR: This environment targets Ubuntu 24.04."
    echo "Detected: ${PRETTY_NAME}"
    exit 1
fi

if [[ -f /opt/ros/jazzy/setup.bash ]]; then
    echo "ROS 2 Jazzy is already installed."
else
    # Configure the official ROS 2 apt source, then install the Jazzy desktop
    # environment plus the standard ROS development tools used by this repo.
    sudo apt update
    sudo apt install -y locales software-properties-common curl

    sudo locale-gen en_US en_US.UTF-8
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

    sudo add-apt-repository -y universe

    ROS_APT_SOURCE_VERSION="$(
        curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
        | grep -F '"tag_name"' \
        | awk -F'"' '{print $4}'
    )"

    curl -L \
        -o /tmp/ros2-apt-source.deb \
        "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.${VERSION_CODENAME}_all.deb"

    sudo dpkg -i /tmp/ros2-apt-source.deb
    sudo apt update

    sudo apt install -y \
        ros-jazzy-desktop \
        ros-dev-tools
fi

echo "ROS 2 Jazzy and repository tooling ready."
