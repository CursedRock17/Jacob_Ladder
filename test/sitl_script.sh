#!/bin/bash
# Fly a jl_blocks.fly script headless in SITL (see flights/ for examples).
#
#   test/sitl_script.sh SCRIPT.py NAME [--timeout S] [--rgbd]
#
#   NAME      the name the script gives its Drone (the mode name in QGC)
#   --timeout seconds the script may take once the mode is selected (default 120)
#   --rgbd    fly the x500_rgbd drone in the rgbd_demo world (needs rendering: on a
#             machine where EGL cannot open the GPU, set GZ_RENDER=x11)
#
# Exit 0 means the script ran to the end (so it landed) with no assertion failing.
# Run inside the sim container after colcon build.
HERE="$(dirname "$(readlink -f "$0")")"
source "$HERE/sitl_common.sh"

[ $# -ge 2 ] || { sed -n '2,12p' "$0" | sed 's/^# \{0,1\}//'; exit 2; }
SCRIPT="$(readlink -f "$1")"; NAME="$2"; shift 2
TIMEOUT=120; RGBD=false
while [ $# -gt 0 ]; do
  case "$1" in
    --timeout) TIMEOUT="$2"; shift 2 ;;
    --rgbd) RGBD=true; shift ;;
    *) echo "unknown option: $1"; exit 2 ;;
  esac
done

export GZ_PARTITION="sitl_script_$NAME"
trap kill_flight EXIT
echo "== flight: $SCRIPT as $NAME =="
logs=$(mktemp -d)
if $RGBD; then
  start_px4_rgbd "$logs" || { echo "logs: $logs"; exit 1; }
else
  start_px4 "$logs"
fi

ros2 launch jl_mission jl_mission.launch.py mission_name:="$NAME" > "$logs/launch.log" 2>&1 &
for _ in $(seq 60); do grep -q "Registered '$NAME'" "$logs/launch.log" && break; sleep 1; done
if ! grep -q "Registered '$NAME'" "$logs/launch.log"; then
  echo "FAIL $NAME registered with PX4 (see $logs/launch.log)"; echo "logs: $logs"; exit 1
fi

# The script waits for the mode to be selected, so start it first.
timeout "$TIMEOUT" python3 "$SCRIPT" > "$logs/script.log" 2>&1 &
script=$!
sleep 3
# Retry slowly (>= 10 s apart) only if the script never saw the mode activate.
for attempt in 1 2 3; do
  (cd "$logs/rootfs" && "$PX4_BUILD/bin/px4-commander" mode ext1) >> "$logs/commander.log" 2>&1
  for _ in $(seq 12); do grep -q "executor — active" "$logs/launch.log" && break 2; sleep 1; done
  echo "mode not active after selection attempt $attempt; selecting again" >> "$logs/commander.log"
done

wait $script; rc=$?
cat "$logs/script.log"
if [ $rc -eq 0 ]; then
  echo "PASS $NAME script finished"
else
  echo "FAIL $NAME script exited $rc (124 = timed out)"
  echo "--- jl_mission log ---"; tail -30 "$logs/launch.log"; echo "logs: $logs"
fi
exit $rc
