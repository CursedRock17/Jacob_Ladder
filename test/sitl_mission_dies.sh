#!/bin/bash
# SITL: a mission process that dies mid-flight must leave jl_mission to hold on
# silence, then land, with no resume and no crash. Run inside the sim container
# after colcon build.
HERE="$(dirname "$(readlink -f "$0")")"
source "$HERE/sitl_common.sh"
export GZ_PARTITION=sitl_mission_dies
trap kill_flight EXIT

MISSION="$HERE/missions/hover_long.py"
NAME=MissionDies
echo "== flight: the mission dies =="
logs=$(mktemp -d)
start_px4 "$logs" || { echo "logs: $logs"; exit 1; }
ros2 launch jl_flight mission.launch.py mission:="$MISSION" > "$logs/launch.log" 2>&1 &
wait_mode_ready "$logs/launch.log" "$NAME" || { echo "logs: $logs"; exit 1; }
for _ in $(seq 30); do grep -q "Waiting for '$NAME'" "$logs/launch.log" && break; sleep 1; done
select_mission "$logs" "$NAME"

for _ in $(seq 60); do grep -q "event: Hold: started" "$logs/launch.log" && break; sleep 1; done
sleep 3
pkill -9 -f "python3 -m jl_flight run $MISSION"

# jl_mission's watchdog: ~0.5 s to hold, ~5 s more to hand over to landing,
# then a real landing (~10 s): give it up to 40 s total.
for _ in $(seq 40); do grep -q "handing over to land" "$logs/launch.log" && break; sleep 1; done

rc=0
check() { if eval "$2"; then echo "PASS $1"; else echo "FAIL $1"; rc=1; fi; }
check "the mission's death was noticed, and the launch kept running" \
  'grep -q "mission_missiondies-[0-9]*\]: process has died" "$logs/launch.log"'
check "jl_mission held on silence" \
  'grep -q "No valid setpoint" "$logs/launch.log" && grep -q "holding" "$logs/launch.log"'
check "jl_mission handed over to land on silence" 'grep -q "handing over to land" "$logs/launch.log"'

state=""
for _ in $(seq 40); do
  state=$(timeout 3 ros2 topic echo --once "/jl/$NAME/mode_state" std_msgs/msg/String 2>/dev/null | grep -o "data: .*")
  [ "$state" = "data: landed" ] && break
  sleep 1
done
check "jl_mission reached landed (last state '${state:-none}')" '[ "$state" = "data: landed" ]'
check "no crash after the mission died" '! grep -qE "Traceback|Assertion|terminate called" "$logs/launch.log"'
[ $rc -ne 0 ] && echo "logs: $logs"
exit $rc
