#!/bin/bash
# Every L3 flight (spec section 8), each from a fresh PX4 start. Stops at the
# first script that fails. Run inside the sim container after colcon build.
HERE="$(dirname "$(readlink -f "$0")")"
ROOT="$HERE/.."
set -e
"$ROOT/src/jl_mission/test/sitl_jl_mission.sh"
"$HERE/sitl_mission.sh" "$ROOT/missions/takeoff_hold_land.yaml" \
  --expect "takeoff hold land" --finished
"$HERE/sitl_runner_smoke.sh"
"$HERE/sitl_mission.sh" "$ROOT/missions/aruco_standoff.yaml" --world aruco \
  --expect "takeoff search track" --camera-standoff "0 0 3"
echo "== all L3 flights passed =="
