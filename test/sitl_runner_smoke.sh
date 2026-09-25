#!/bin/bash
# SITL smoke flight for the Phase 3 mission_runner, headless: check that a
# dying runner leaves jl_mission to hold, then land, with no resume and no
# crash. The takeoff->hold->land flight now runs through test/sitl_mission.sh.
# Run inside the sim container after colcon build.
HERE="$(dirname "$(readlink -f "$0")")"
source "$HERE/sitl_common.sh"
export GZ_PARTITION=sitl_runner_smoke
trap kill_flight EXIT

rc=0

# Flight 2: a dying runner must leave jl_mission running (F1): it holds on
# silence, then lands, with no resume and no crash.
echo "== flight: runner dies =="
logs2=$(mktemp -d)
start_px4 "$logs2"
NAME2=RunnerDies
MISSION2="$logs2/runner_dies.yaml"
cat > "$MISSION2" <<'EOF'
name: RunnerDies
steps:
  - takeoff: {height: 1.5}
  - hold: {duration: 60}
EOF
ros2 launch jl_blocks mission.launch.py mission_file:="$MISSION2" > "$logs2/launch.log" 2>&1 &
for _ in $(seq 60); do grep -q "Registered '$NAME2'" "$logs2/launch.log" && break; sleep 1; done
sleep 2
(cd "$logs2/rootfs" && "$PX4_BUILD/bin/px4-commander" mode ext1) >> "$logs2/commander.log" 2>&1

for _ in $(seq 60); do grep -q "event: hold: started" "$logs2/launch.log" && break; sleep 1; done
sleep 3
pkill -9 -f lib/jl_blocks/mission_runner

# jl_mission's watchdog: ~0.5 s to hold, ~5 s more to hand over to landing,
# then a real landing (~10 s): give it up to 40 s total.
for _ in $(seq 40); do grep -q "handing over to land" "$logs2/launch.log" && break; sleep 1; done
cat "$logs2/launch.log"

rc2=0
if grep -q "mission_runner exited" "$logs2/launch.log"; then
  echo "PASS runner exit was logged, launch kept running"
else
  echo "FAIL runner exit was logged, launch kept running"; rc2=1
fi
if grep -q "No valid setpoint" "$logs2/launch.log" && grep -q "holding" "$logs2/launch.log"; then
  echo "PASS jl_mission held on silence"
else
  echo "FAIL jl_mission held on silence"; rc2=1
fi
if grep -q "handing over to land" "$logs2/launch.log"; then
  echo "PASS jl_mission handed over to land on silence"
else
  echo "FAIL jl_mission handed over to land on silence"; rc2=1
fi

state2=""
for _ in $(seq 40); do
  state2=$(timeout 3 ros2 topic echo --once \
    "/jl/$NAME2/mode_state" std_msgs/msg/String 2>/dev/null | grep -o "data: .*")
  [ "$state2" = "data: landed" ] && break
  sleep 1
done
if [ "$state2" = "data: landed" ]; then
  echo "PASS jl_mission reached landed"
else
  echo "FAIL jl_mission reached landed (last state '${state2:-none}')"; rc2=1
fi

if grep -qE "Traceback|Assertion|terminate called" "$logs2/launch.log"; then
  echo "FAIL no crash after the runner died"; rc2=1
else
  echo "PASS no crash after the runner died"
fi
[ $rc2 -ne 0 ] && echo "logs: $logs2"
[ $rc2 -ne 0 ] && rc=1

exit $rc
