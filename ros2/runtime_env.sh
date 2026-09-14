#!/usr/bin/env bash

# Shared interactive runtime environment for ROS 2 + PX4/Gazebo.
# Source this file before running ROS 2 commands from a normal terminal.

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
    echo "ERROR: source ros2/runtime_env.sh instead of executing it."
    exit 1
fi

_PX4_ENV_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

source "${_PX4_ENV_ROOT}/config/env.env"

_PX4_ENV_ROS_SETUP="/opt/ros/${ROS_DISTRO}/setup.bash"
_PX4_ENV_WS_SETUP="${_PX4_ENV_ROOT}/ros2/install/setup.bash"
_PX4_ENV_SYSTEM_GZ_CONFIG="/usr/share/gz"

# Fail early with a useful message instead of letting a later ros2/gz command
# fail because the base ROS install or this workspace has not been prepared.
if [[ ! -f "${_PX4_ENV_ROS_SETUP}" ]]; then
    echo "ERROR: ROS 2 ${ROS_DISTRO} is not installed." >&2
    return 1
fi

if [[ ! -f "${_PX4_ENV_WS_SETUP}" ]]; then
    echo "ERROR: ROS 2 workspace has not been built." >&2
    return 1
fi

source "${_PX4_ENV_ROS_SETUP}"
source "${_PX4_ENV_WS_SETUP}"

if [[ ! -d "${_PX4_ENV_SYSTEM_GZ_CONFIG}" ]]; then
    echo "ERROR: system Gazebo configuration was not found:" >&2
    echo "  ${_PX4_ENV_SYSTEM_GZ_CONFIG}" >&2
    return 1
fi

# ROS 2 Gazebo vendor packages can add their own command descriptors. Keep
# those paths while also exposing the system Gazebo installation installed by
# PX4's Ubuntu setup; this makes the `gz` command work in the sourced shell.
case ":${GZ_CONFIG_PATH:-}:" in
    *":${_PX4_ENV_SYSTEM_GZ_CONFIG}:"*)
        ;;
    *)
        export GZ_CONFIG_PATH="${_PX4_ENV_SYSTEM_GZ_CONFIG}${GZ_CONFIG_PATH:+:${GZ_CONFIG_PATH}}"
        ;;
esac

# Do not leave helper variables in the caller's interactive shell.
unset _PX4_ENV_ROOT
unset _PX4_ENV_ROS_SETUP
unset _PX4_ENV_WS_SETUP
unset _PX4_ENV_SYSTEM_GZ_CONFIG
