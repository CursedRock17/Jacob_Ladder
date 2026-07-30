#!/bin/bash
# Source our Environment (paths resolved from this script's location — see jl_env.sh)
source "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/../jl_env.sh"
cd "$JL_WS_ROOT" || exit 1
jl_source_ros
# The odometry node needs the depthai wheels: prefer the package-local venv,
# and fall back to the workspace venv ($JL_VENV) when it isn't there.
jl_source_venv "$JL_WS_ROOT/src/oak_d_visual_odometry/venv"

# Run the OAK-D visual-odometry publisher with PX4 VehicleOdometry output.
# (Node name per `ros2 pkg executables oak_d_visual_odometry`; this file used to
# name vo_publisher_px4_node, which has never existed and made the unit
# restart-loop.)
ros2 run oak_d_visual_odometry cuvslam_publisher_px4_node
