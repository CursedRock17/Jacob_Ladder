#!/bin/bash
# Fly one mission (missions/*.py) in Gazebo SITL, with the GUI, from a single terminal.
#
#   ./launch_scripts/sim_mission.sh MISSION.py [--world default|apriltag] [--no-start]
#
#   --world      default: PX4's gz_x500 in an empty world (airframe generic).
#                apriltag: a wall 6 m ahead with a tag36h11 id 0 tag on it, x500_rgbd drone.
#                rgbd: obstacles ahead for the RGB-D camera, x500_rgbd drone.
#                aruco: ArUco tags on the ground and on a post, x500_dual_cam drone.
#   --no-start   do not select the mission for you; pick it in QGC instead
#
# Run it on the host: it re-runs itself inside the jacob_ladder_sim container
# (./docker/run_sim_container.sh), which needs PX4 and the workspace built.
# Gazebo, PX4, the DDS agent and the translation node run as background jobs of
# this shell, and so does jl_flight's mission.launch.py: jl_mission, the helpers
# the mission declares, and the mission. The mission's own output (what it is
# doing, and any error) is shown here; everything else goes to the logs folder it
# prints. Ctrl+C or Enter stops it all.
source "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/../jl_env.sh"

if [ ! -f /.dockerenv ]; then
  exec docker exec -it --user user -w "$JL_WS_ROOT" "${JL_CONTAINER:-jacob_ladder_sim}" \
    bash -c "$(printf '%q ' "$JL_WS_ROOT/launch_scripts/sim_mission.sh" "$@")"
fi

source "$JL_WS_ROOT/test/sitl_common.sh"

usage() { sed -n '2,18p' "$0" | sed 's/^# \{0,1\}//'; exit 2; }
[ $# -ge 1 ] || usage
MISSION="$(readlink -f "$1")"; shift
WORLD=default; START=true
while [ $# -gt 0 ]; do
  case "$1" in
    --world) [ $# -ge 2 ] || usage; WORLD="$2"; shift 2 ;;
    --no-start) START=false; shift ;;
    *) echo "unknown option: $1"; usage ;;
  esac
done
read -r GZ_WORLD AIRFRAME < <(world_setup "$WORLD") || { echo "unknown world: $WORLD"; usage; }

python3 -m jl_flight check "$MISSION" --airframe "$AIRFRAME" || exit 1
NAME=$(python3 -m jl_flight name "$MISSION")
trap kill_flight EXIT
kill_flight   # nothing from an earlier run may be left over

logs=$(mktemp -d)
echo "== $NAME: starting Gazebo ($WORLD) + PX4 (logs: $logs) =="
if [ "$WORLD" = default ]; then
  GZ_GUI=1 start_px4 "$logs" || { echo "logs: $logs"; exit 1; }
else
  GZ_GUI=1 start_px4_world "$logs" "$GZ_WORLD" "$AIRFRAME" || { echo "logs: $logs"; exit 1; }
fi

echo "== starting $NAME =="
ros2 launch jl_flight mission.launch.py mission:="$MISSION" airframe:="$AIRFRAME" > "$logs/launch.log" 2>&1 &
wait_mode_ready "$logs/launch.log" "$NAME" || exit 1
# Show what the mission says as it flies.
tail -n +1 -f "$logs/launch.log" | grep --line-buffered -E "event: |Waiting for|jl_flight: |Traceback|Error" \
  | sed -u -E 's/.*\]: (event: )?/  /' &

if $START; then
  for _ in $(seq 30); do grep -q "Waiting for '$NAME'" "$logs/launch.log" && break; sleep 1; done
  echo "== selecting $NAME (watch the Gazebo window) =="
  select_mission "$logs" "$NAME"
else
  echo "== select $NAME in QGC =="
fi
read -r -p "Press Enter to stop the simulation... "
