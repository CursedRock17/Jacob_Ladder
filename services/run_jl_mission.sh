#!/bin/bash
# One mission from config/flight.yaml, as a boot service: jl_mission (the mode
# shown in QGC), the perception helpers the mission declares, and the mission
# itself, via jl_flight's mission.launch.py.
#   $1: the mission file's stem, e.g. takeoff_hold_land for missions/takeoff_hold_land.py
# The mission restarts after it ends (or dies) and waits for its mode to be
# selected again: it never resumes a flight that was already active
# (jl_mission holds, then lands, meanwhile).
source "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/../jl_env.sh"
cd "$JL_WS_ROOT" || exit 1
jl_source_ros

mission="$JL_WS_ROOT/missions/$1.py"
if [ ! -f "$mission" ]; then
    echo "run_jl_mission: $mission not found (check config/flight.yaml, then ./deploy.sh)" >&2
    exit 1
fi
airframe="$(python3 -m jl_flight flight airframe "$JL_WS_ROOT/config/flight.yaml" --root "$JL_WS_ROOT")" \
    || { echo "run_jl_mission: config/flight.yaml does not check out (run ./deploy.sh)" >&2; exit 1; }
exec ros2 launch jl_flight mission.launch.py mission:="$mission" airframe:="$airframe" respawn:=true
