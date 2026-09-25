#!/bin/bash
# One mission from config/flight.yaml, as a boot service: jl_mission (the mode
# shown in QGC) plus the mission_runner, via mission.launch.py.
#   $1: the mission file's stem, e.g. takeoff_hold_land for missions/takeoff_hold_land.yaml
# The runner is respawned if it dies; it will not resume a flight that was
# already active (jl_mission holds, then lands, meanwhile).
source "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/../jl_env.sh"
cd "$JL_WS_ROOT" || exit 1
jl_source_ros

mission="$JL_WS_ROOT/missions/$1.yaml"
if [ ! -f "$mission" ]; then
    echo "run_jl_mission: $mission not found (check config/flight.yaml, then ./deploy.sh)" >&2
    exit 1
fi
# jl_blocks itself isn't on PATH (setup.cfg installs its console scripts to
# lib/jl_blocks); the sourced workspace does put it on PYTHONPATH, so invoke
# it as a module instead, same as deploy.sh.
blocks="$(python3 -m jl_blocks.cli flight blocks "$JL_WS_ROOT/config/flight.yaml" --root "$JL_WS_ROOT")"

args=(mission_file:="$mission" respawn_runner:=true)
[ -n "$blocks" ] && args+=(blocks:="$blocks")
exec ros2 launch jl_blocks mission.launch.py "${args[@]}"
