#!/bin/bash
# Source our Environment (paths resolved from this script's location — see jl_env.sh)
source "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/../jl_env.sh"

# aruco_tracker lives in the separate tracktor-beam workspace, not this one.
# Point JL_TRACKTOR_BEAM_DIR at your checkout; the fallbacks cover the usual spots.
if [ -z "${JL_TRACKTOR_BEAM_DIR:-}" ]; then
    for candidate in \
        "$(dirname "$JL_WS_ROOT")/tracktor-beam" \
        "$HOME/tracktor-beam" \
        "$HOME/ARK/tracktor-beam"; do
        if [ -d "$candidate" ]; then
            JL_TRACKTOR_BEAM_DIR="$candidate"
            break
        fi
    done
fi

if [ ! -d "${JL_TRACKTOR_BEAM_DIR:-}" ]; then
    echo "start_aruco_tracker: tracktor-beam workspace not found." >&2
    echo "Set JL_TRACKTOR_BEAM_DIR=/path/to/tracktor-beam and retry." >&2
    exit 1
fi

cd "$JL_TRACKTOR_BEAM_DIR" || exit 1
if [ -f "/opt/ros/$JL_ROS_DISTRO/setup.bash" ]; then
    source "/opt/ros/$JL_ROS_DISTRO/setup.bash"
fi
source install/setup.bash

# Run the tracker
ros2 run aruco_tracker aruco_tracker
