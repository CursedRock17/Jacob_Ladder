#!/bin/bash
# Fly one mission file headless in SITL and check what it did (spec section 8, L3).
#
#   test/sitl_mission.sh MISSION.yaml --expect "takeoff hold land" [--finished]
#       [--world default|aruco] [--camera-standoff "X Y Z"] [--tolerance M]
#       [--hold S] [--timeout S]
#
#   --expect           step names that must start, in this order (required)
#   --finished         the mission must also finish ("mission finished")
#   --world            default: PX4's gz_x500 in an empty world.
#                      aruco: gazebo/worlds/aruco_dual_ids.sdf, x500_dual_cam,
#                      and the front aruco_tracker (tag id 0)
#   --camera-standoff  the front camera must see the tag at X Y Z metres
#                      (optical frame: +x right, +y down, +z forward), within
#                      --tolerance (0.25) per axis, for --hold (5) seconds
#   --timeout          seconds from selecting the mission to giving up (120)
#
# Run inside the sim container after colcon build. Exit 0 = every check passed.
HERE="$(dirname "$(readlink -f "$0")")"
source "$HERE/sitl_common.sh"

usage() { sed -n '2,18p' "$0" | sed 's/^# \{0,1\}//'; exit 2; }
[ $# -ge 1 ] || usage
MISSION="$(readlink -f "$1")"; shift
EXPECT=""; FINISHED=false; WORLD=default; CAMERA=""; TOL=0.25; HOLD=5; TIMEOUT=120
needs_value() { echo "$1 needs a value"; usage; }
while [ $# -gt 0 ]; do
  case "$1" in
    --expect) [ $# -ge 2 ] || needs_value "$1"; EXPECT="$2"; shift 2 ;;
    --finished) FINISHED=true; shift ;;
    --world) [ $# -ge 2 ] || needs_value "$1"; WORLD="$2"; shift 2 ;;
    --camera-standoff) [ $# -ge 2 ] || needs_value "$1"; CAMERA="$2"; shift 2 ;;
    --tolerance) [ $# -ge 2 ] || needs_value "$1"; TOL="$2"; shift 2 ;;
    --hold) [ $# -ge 2 ] || needs_value "$1"; HOLD="$2"; shift 2 ;;
    --timeout) [ $# -ge 2 ] || needs_value "$1"; TIMEOUT="$2"; shift 2 ;;
    *) echo "unknown option: $1"; usage ;;
  esac
done
[ -n "$EXPECT" ] || { echo "--expect is required"; usage; }
case "$WORLD" in default|aruco) ;; *) echo "unknown world: $WORLD"; usage ;; esac

# A bad mission file fails here, in seconds, before anything starts.
ros2 run jl_blocks jl_blocks check "$MISSION" || {
  echo "FAIL mission file $MISSION (jl_blocks check)"; exit 1
}
NAME=$(python3 -c "import sys, yaml; print(yaml.safe_load(open(sys.argv[1]))['name'])" "$MISSION")
export GZ_PARTITION="sitl_mission_$NAME"
trap kill_flight EXIT

echo "== flight: $NAME ($WORLD world) =="
logs=$(mktemp -d)
if [ "$WORLD" = aruco ]; then
  start_px4_aruco "$logs" || { echo "logs: $logs"; exit 1; }
else
  start_px4 "$logs"
fi

ros2 launch jl_blocks mission.launch.py mission_file:="$MISSION" > "$logs/launch.log" 2>&1 &
for _ in $(seq 60); do grep -q "Registered '$NAME'" "$logs/launch.log" && break; sleep 1; done
if ! grep -q "Registered '$NAME'" "$logs/launch.log"; then
  echo "FAIL $NAME registered with PX4 (see $logs/launch.log)"; echo "logs: $logs"; exit 1
fi

# The watcher must be listening before the mission starts, because events are
# not latched. Its timeout counts from here.
watch_args=(-p mission_name:="$NAME" -p expect:="$EXPECT" -p finished:=$FINISHED
            -p tolerance:=$TOL -p hold_s:=$HOLD -p timeout_s:=$TIMEOUT)
[ -n "$CAMERA" ] && watch_args+=(-p camera_target:="$CAMERA")
ros2 run jl_blocks mission_watch --ros-args "${watch_args[@]}" > "$logs/watch.log" 2>&1 &
watch=$!
sleep 3
# Select the mission; retry slowly (spec section 8: >= 10 s apart, since
# re-selecting mid-takeoff restarts the executor) only if it never activated.
for attempt in 1 2 3; do
  (cd "$logs/rootfs" && "$PX4_BUILD/bin/px4-commander" mode ext1) >> "$logs/commander.log" 2>&1
  for _ in $(seq 12); do grep -q "event: activated" "$logs/watch.log" && break 2; sleep 1; done
  echo "mission not active after selection attempt $attempt; selecting again" >> "$logs/commander.log"
done

wait $watch; rc=$?
if [ $rc -ne 0 ] && ! grep -q FAIL "$logs/watch.log"; then
  echo "FAIL mission_watch exited $rc (see $logs/watch.log)"
fi
cat "$logs/watch.log"
if grep -qE "Traceback|Assertion|terminate called" "$logs/launch.log"; then
  echo "FAIL no crash in the runner or the mode"; rc=1
else
  echo "PASS no crash in the runner or the mode"
fi
[ $rc -ne 0 ] && echo "logs: $logs"
exit $rc
