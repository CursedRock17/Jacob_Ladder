#!/bin/bash
# Source our Environment (paths resolved from this script's location — see jl_env.sh)
source "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/../jl_env.sh"
cd "$JL_WS_ROOT" || exit 1
jl_source_ros

# The odometry node needs wheels that only exist in a venv (pyrealsense2,
# depthai, cuvslam, numpy<2): prefer a package-local venv, and fall back to the
# workspace venv ($JL_VENV) when it isn't there.
#
# The fallback has to be spelled out here. jl_source_venv() uses $JL_VENV only
# when called with NO argument -- given an explicit path that does not exist it
# silently sources nothing, which is what used to happen: this script passed the
# package-local path unconditionally, no venv was ever activated under systemd
# (which does not read the shell profile that puts .venv on PATH), and the node
# ran under /usr/bin/python3 and died on `numpy.core.multiarray failed to
# import` -- NumPy 2.x against modules built for 1.x.
JL_PKG_VENV="$JL_WS_ROOT/src/oak_d_visual_odometry/venv"
if [ -f "$JL_PKG_VENV/bin/activate" ]; then
    jl_source_venv "$JL_PKG_VENV"
else
    jl_source_venv
fi

# Run the visual-odometry publisher with PX4 VehicleOdometry output, through the
# same launch file the tmux launcher (launch_scripts/super_real.sh) uses, so the
# service and an interactive session load identical parameters.
#
# camera:= picks the backend: "realsense" (Intel D435i, the launch default) or
# "oak" (OAK-D S2). Passed explicitly so this unit does not silently change
# camera if that default moves. One at a time -- each backend claims its device
# exclusively, so this service and super_real.sh's cuVSLAM window must not run
# together.
#
# exec so `ros2 launch` is the unit's MAINPID: systemd then delivers SIGTERM
# straight to it on stop and it can close the camera and tear the pipeline down,
# instead of SIGTERM landing on this shell and the launch being swept up by the
# cgroup kill afterwards.
exec ros2 launch oak_d_visual_odometry cuvslam_px4.launch.py camera:=realsense
