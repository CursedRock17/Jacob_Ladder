#!/bin/bash
# Source our Environment (paths resolved from this script's location — see jl_env.sh)
source "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/../jl_env.sh"
cd "$JL_WS_ROOT" || exit 1
jl_source_ros
# The odometry node needs the depthai wheels: prefer the package-local venv,
# and fall back to the workspace venv ($JL_VENV) when it isn't there.
jl_source_venv "$JL_WS_ROOT/src/oak_d_visual_odometry/venv"

# Run the ROS2 usb_cam node with specified parameters
ros2 run oak_d_visual_odometry vo_publisher_px4_node
