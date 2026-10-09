#!/bin/bash
# Fly a Python mission (missions/*.py) headless in SITL and check where it ends up.
#
#   test/sitl_flight.sh MISSION.py [--world default|apriltag]
#       [--landed-at "N E"] [--holding-at "N E" --after S] [--ring R]
#       [--tolerance M] [--timeout S]
#
#   --world       default: PX4's gz_x500 in an empty world (airframe generic).
#                 apriltag: apriltag_wall, a tag36h11 id 0 on a wall (face at east 5.898,
#                 centre 1.2 m up), with the x500_rgbd drone. rgbd: rgbd_demo, obstacles
#                 ahead, x500_rgbd. aruco: aruco_dual_ids, ArUco 4x4_250 id 0 vertical
#                 at (east 1, up 1), with the x500_dual_cam drone. Worlds with cameras
#                 need rendering; GZ_RENDER=x11 if EGL cannot open the GPU.
#   --landed-at   the mission must finish, and the drone be on the ground within
#                 --tolerance of NORTH EAST (metres from where it spawned)
#   --holding-at  for missions that never finish (they hold station): --after S
#                 seconds from selecting it, the drone must be in the air there
#   --ring        with either check: N E is the target, and the drone must be R metres
#                 from it in any direction (standoffs are measured along the drone's line)
#   --timeout     seconds from selecting the mission to giving up (default 120)
#
# The launch is jl_flight's mission.launch.py, as on the drone: jl_mission, the
# helpers the mission declares, and the mission. Run inside the sim container
# after colcon build. Exit 0 = every check passed.
HERE="$(dirname "$(readlink -f "$0")")"
source "$HERE/sitl_common.sh"

usage() { sed -n '2,24p' "$0" | sed 's/^# \{0,1\}//'; exit 2; }
[ $# -ge 1 ] || usage
MISSION="$(readlink -f "$1")"; shift
WORLD=default; LANDED=""; HOLDING=""; AFTER=""; RING=(); TOL=0.35; TIMEOUT=120
needs_value() { echo "$1 needs a value"; usage; }
while [ $# -gt 0 ]; do
  case "$1" in
    --world) [ $# -ge 2 ] || needs_value "$1"; WORLD="$2"; shift 2 ;;
    --landed-at) [ $# -ge 2 ] || needs_value "$1"; LANDED="$2"; shift 2 ;;
    --holding-at) [ $# -ge 2 ] || needs_value "$1"; HOLDING="$2"; shift 2 ;;
    --after) [ $# -ge 2 ] || needs_value "$1"; AFTER="$2"; shift 2 ;;
    --ring) [ $# -ge 2 ] || needs_value "$1"; RING=(--ring "$2"); shift 2 ;;
    --tolerance) [ $# -ge 2 ] || needs_value "$1"; TOL="$2"; shift 2 ;;
    --timeout) [ $# -ge 2 ] || needs_value "$1"; TIMEOUT="$2"; shift 2 ;;
    *) echo "unknown option: $1"; usage ;;
  esac
done
read -r GZ_WORLD AIRFRAME < <(world_setup "$WORLD") || { echo "unknown world: $WORLD"; usage; }
[ -n "$LANDED$HOLDING" ] || { echo "say what to check: --landed-at or --holding-at"; usage; }
[ -z "$HOLDING" ] || [ -n "$AFTER" ] || { echo "--holding-at needs --after"; usage; }

# A bad mission fails here, in seconds, before anything starts.
python3 -m jl_flight check "$MISSION" --airframe "$AIRFRAME" || exit 1
NAME=$(python3 -m jl_flight name "$MISSION")
export GZ_PARTITION="sitl_flight_$NAME"
trap kill_flight EXIT

echo "== flight: $NAME ($WORLD world, airframe $AIRFRAME) =="
logs=$(mktemp -d)
if [ "$WORLD" = default ]; then
  start_px4 "$logs" || { echo "logs: $logs"; exit 1; }
else
  start_px4_world "$logs" "$GZ_WORLD" "$AIRFRAME" || { echo "logs: $logs"; exit 1; }
fi
TAG_TOPIC=$(python3 -c "import sys; from jl_flight.missions import load; n = load(sys.argv[1]).needs; print(n[0].topic if n else '')" "$MISSION")
python3 "$HERE/record_pose.py" "$logs/pose.csv" $TAG_TOPIC > /dev/null 2>&1 &   # where the drone really went

ros2 launch jl_flight mission.launch.py mission:="$MISSION" airframe:="$AIRFRAME" > "$logs/launch.log" 2>&1 &
wait_mode_ready "$logs/launch.log" "$NAME" || { echo "logs: $logs"; exit 1; }
for _ in $(seq 30); do grep -q "Waiting for '$NAME'" "$logs/launch.log" && break; sleep 1; done

# Select the mission; retry slowly (>= 10 s apart) only if it never activated.
for attempt in 1 2 3; do
  select_mission "$logs" "$NAME"
  for _ in $(seq 12); do grep -q "executor — active" "$logs/launch.log" && break 2; sleep 1; done
  echo "mission not active after selection attempt $attempt; selecting again" >> "$logs/commander.log"
done
selected=$(date +%s)

rc=0
if [ -n "$LANDED" ]; then
  finished=""
  while [ $(( $(date +%s) - selected )) -lt "$TIMEOUT" ]; do
    finished=$(grep -oE "mission_[a-z0-9_]+-[0-9]+\]: process has finished cleanly|mission_[a-z0-9_]+-[0-9]+\]: process has died.*" "$logs/launch.log" | head -1)
    [ -n "$finished" ] && break
    sleep 1
  done
  if [[ "$finished" == *"finished cleanly"* ]]; then
    echo "PASS $NAME finished"
  else
    echo "FAIL $NAME finished within ${TIMEOUT} s (${finished:-still running})"; rc=1
  fi
  python3 "$HERE/check_position.py" $LANDED "$TOL" --landed "${RING[@]}" || rc=1
else
  sleep $(( AFTER - ($(date +%s) - selected) > 0 ? AFTER - ($(date +%s) - selected) : 0 ))
  python3 "$HERE/check_position.py" $HOLDING "$TOL" --airborne "${RING[@]}" || rc=1
fi
if [ "$WORLD" != default ]; then
  read -r gx gy gz_ < <(gz_truth "$GZ_WORLD" "${AIRFRAME}_0")
  echo "gazebo says the drone is at east $gx, north $gy, up $gz_"
fi
echo "--- what the mission said"; grep -E "event: |jl_flight: |Traceback|Error" "$logs/launch.log" | sed -E 's/.*\] (event: )?//' | head -30
[ $rc -ne 0 ] && echo "logs: $logs"
exit $rc
