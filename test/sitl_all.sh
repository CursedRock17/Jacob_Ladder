#!/bin/bash
# Every L3 flight, each from a fresh PX4 start. Stops at the first script that
# fails. Run inside the sim container after colcon build; worlds with cameras
# need rendering (GZ_RENDER=x11 where headless EGL fails).
HERE="$(dirname "$(readlink -f "$0")")"
ROOT="$HERE/.."
set -e
"$ROOT/src/jl_mission/test/sitl_jl_mission.sh"
"$HERE/sitl_mission_dies.sh"
"$HERE/sitl_flight.sh" "$ROOT/missions/takeoff_hold_land.py" --landed-at "0 0" --tolerance 0.3
"$HERE/sitl_flight.sh" "$HERE/missions/fly_smoke.py" --landed-at "0 0" --ring 1.4 --tolerance 0.4
"$HERE/sitl_rgbd.sh"
"$HERE/sitl_flight.sh" "$HERE/missions/fly_smoke.py" --world rgbd --landed-at "0 0" --ring 1.4 --tolerance 0.4
"$HERE/sitl_flight.sh" "$ROOT/missions/land_before_apriltag.py" --world apriltag \
  --landed-at "0 5.898" --ring 1.5 --tolerance 0.3
"$HERE/sitl_flight.sh" "$ROOT/missions/apriltag_approach.py" --world apriltag \
  --holding-at "0 5.898" --ring 3.0 --after 60 --tolerance 0.25
"$HERE/sitl_flight.sh" "$ROOT/missions/aruco_standoff.py" --world aruco \
  --holding-at "0 1.0" --ring 3.0 --after 60 --tolerance 0.3
echo "== all L3 flights passed =="
