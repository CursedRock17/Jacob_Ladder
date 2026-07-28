#!/bin/bash
# Shared path resolution for every script in this repo.
#
# Nothing in this repo may hardcode a home directory: the workspace root is
# derived from THIS file's own location, so a clone works under any username,
# at any path, on the host or inside the container.
#
# Source it from any script in a subdirectory:
#     source "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/../jl_env.sh"
#
# Everything is overridable from the environment:
#   JL_WS_ROOT     workspace root            (default: dir holding this file)
#   JL_PX4_DIR     PX4-Autopilot checkout    (default: first sibling/home match)
#   JL_ROS_DISTRO  ROS 2 distro to source    (default: $ROS_DISTRO, else humble)
#   JL_VENV        Python venv to activate   (default: $JL_WS_ROOT/.venv)

_jl_env_file="${BASH_SOURCE[0]:-$0}"
: "${JL_WS_ROOT:=$(cd "$(dirname "$(readlink -f "$_jl_env_file")")" && pwd)}"
unset _jl_env_file
export JL_WS_ROOT

: "${JL_ROS_DISTRO:=${ROS_DISTRO:-humble}}"
export JL_ROS_DISTRO

# PX4-Autopilot is a separate checkout; look in the usual spots relative to the
# workspace before falling back to a sibling directory.
if [ -z "${JL_PX4_DIR:-}" ]; then
    for _jl_candidate in \
        "$JL_WS_ROOT/PX4-Autopilot" \
        "$(dirname "$JL_WS_ROOT")/PX4-Autopilot" \
        "$HOME/PX4-Autopilot"; do
        if [ -d "$_jl_candidate" ]; then
            JL_PX4_DIR="$_jl_candidate"
            break
        fi
    done
    unset _jl_candidate
    : "${JL_PX4_DIR:=$(dirname "$JL_WS_ROOT")/PX4-Autopilot}"
fi
export JL_PX4_DIR

: "${JL_VENV:=$JL_WS_ROOT/.venv}"
export JL_VENV

# Source /opt/ros/<distro> and the workspace overlay, if they exist.
# Safe under `set -e`: a missing file is skipped, not an error.
jl_source_ros() {
    if [ -f "/opt/ros/$JL_ROS_DISTRO/setup.bash" ]; then
        # shellcheck disable=SC1090
        source "/opt/ros/$JL_ROS_DISTRO/setup.bash"
    fi
    if [ -f "$JL_WS_ROOT/install/setup.bash" ]; then
        # shellcheck disable=SC1091
        source "$JL_WS_ROOT/install/setup.bash"
    else
        echo "jl_env: $JL_WS_ROOT/install/setup.bash missing — build the workspace first" >&2
    fi
    return 0
}

# Activate a Python venv if one is present (services need the ultralytics /
# depthai wheels that live there).
jl_source_venv() {
    local venv="${1:-$JL_VENV}"
    if [ -f "$venv/bin/activate" ]; then
        # shellcheck disable=SC1091
        source "$venv/bin/activate"
    fi
    return 0
}
