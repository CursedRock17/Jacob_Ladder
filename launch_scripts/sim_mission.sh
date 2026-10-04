#!/bin/bash
# Fly one mission file in Gazebo SITL, with the GUI, from a single terminal.
#
#   ./launch_scripts/sim_mission.sh [MISSION.yaml] [--expect "step names"] [--no-start]
#
#   MISSION.yaml   default: missions/takeoff_hold_land.yaml
#   --expect       steps that must start, in order (default: "takeoff hold land")
#   --no-start     do not select the mission for you; pick it in QGC instead
#
# Run it on the host: it re-runs itself inside the jacob_ladder_sim container
# (./docker/run_sim_container.sh), which needs PX4 and the workspace built.
# PX4, the DDS agent, the translation node and the mission runner run as
# background jobs of this one shell; their logs are in the directory it prints.
# This is test/sitl_mission.sh with the Gazebo window turned on.
source "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/../jl_env.sh"

if [ ! -f /.dockerenv ]; then
  exec docker exec -it --user user -w "$JL_WS_ROOT" "${JL_CONTAINER:-jacob_ladder_sim}" \
    bash -c "$(printf '%q ' "$JL_WS_ROOT/launch_scripts/sim_mission.sh" "$@")"
fi

source "$JL_WS_ROOT/test/sitl_common.sh"

MISSION="$JL_WS_ROOT/missions/takeoff_hold_land.yaml"; EXPECT="takeoff hold land"; START=true
while [ $# -gt 0 ]; do
  case "$1" in
    --expect) EXPECT="$2"; shift 2 ;;
    --no-start) START=false; shift ;;
    -*) echo "unknown option: $1"; exit 2 ;;
    *) MISSION="$(readlink -f "$1")"; shift ;;
  esac
done

ros2 run jl_blocks jl_blocks check "$MISSION" || { echo "FAIL mission file $MISSION"; exit 1; }
NAME=$(python3 -c "import sys, yaml; print(yaml.safe_load(open(sys.argv[1]))['name'])" "$MISSION")
trap kill_flight EXIT
kill_flight   # nothing from an earlier run may be left over

logs=$(mktemp -d)
echo "== $NAME: starting Gazebo + PX4 (logs: $logs) =="
mkdir -p "$logs/rootfs" && cp "$PX4_BUILD/rootfs/gz_env.sh" "$logs/rootfs/"
# No HEADLESS here: that is what opens the Gazebo window.
PX4_SIM_MODEL=gz_x500 GZ_IP=127.0.0.1 \
  "$PX4_BUILD/bin/px4" -d -w "$logs/rootfs" "$PX4_BUILD/etc" > "$logs/px4.log" 2>&1 &
MicroXRCEAgent udp4 -p 8888 > "$logs/agent.log" 2>&1 &
ros2 run translation_node translation_node_bin > "$logs/translation.log" 2>&1 &
for _ in $(seq 120); do grep -q "synchronized with time offset" "$logs/px4.log" && break; sleep 1; done
grep -q "synchronized with time offset" "$logs/px4.log" || { echo "FAIL PX4 did not start (see $logs/px4.log)"; exit 1; }
(cd "$logs/rootfs" && "$PX4_BUILD/bin/px4-param" set NAV_DLL_ACT 0) >> "$logs/commander.log" 2>&1  # sim: no data-link failsafe

echo "== starting the mission runner =="
ros2 launch jl_blocks mission.launch.py mission_file:="$MISSION" > "$logs/launch.log" 2>&1 &
for _ in $(seq 60); do grep -q "Registered '$NAME'" "$logs/launch.log" && break; sleep 1; done
grep -q "Registered '$NAME'" "$logs/launch.log" || { echo "FAIL $NAME never registered with PX4 (see $logs/launch.log)"; exit 1; }
echo "== $NAME is registered with PX4 =="

ros2 run jl_blocks mission_watch --ros-args -p mission_name:="$NAME" -p expect:="$EXPECT" \
  -p finished:=true -p timeout_s:=300 > "$logs/watch.log" 2>&1 &
watch=$!
sleep 3
if $START; then
  echo "== selecting $NAME (watch the Gazebo window) =="
  (cd "$logs/rootfs" && "$PX4_BUILD/bin/px4-commander" mode ext1) >> "$logs/commander.log" 2>&1
else
  echo "== select $NAME in QGC (or: px4-commander mode ext1) =="
fi

wait $watch; rc=$?
cat "$logs/watch.log"
echo "logs: $logs"
read -r -p "Press Enter to close the simulation... "
exit $rc
