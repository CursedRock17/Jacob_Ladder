#!/bin/bash
# Check the simulated RGB-D camera headless (spec section 8, L3): start the
# rgbd_demo world with the x500_rgbd drone, bridge the camera into ROS and
# check what arrives (test/check_rgbd.py). The drone does not fly.
# Run inside the sim container after colcon build. Exit 0 = every check passed.
HERE="$(dirname "$(readlink -f "$0")")"
source "$HERE/sitl_common.sh"
export GZ_PARTITION=sitl_rgbd
trap kill_flight EXIT
echo "== flight: rgbd camera =="
logs=$(mktemp -d)
start_px4_rgbd "$logs" || { echo "logs: $logs"; exit 1; }
python3 "$HERE/check_rgbd.py"; rc=$?
[ $rc -ne 0 ] && echo "logs: $logs"
exit $rc
