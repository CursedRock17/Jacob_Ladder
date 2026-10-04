#!/bin/bash
# Fly a jl_blocks.fly script in Gazebo SITL, with the GUI, from a single terminal.
#
#   ./launch_scripts/sim_script.sh flights/square.py SquareFlight [--no-start] [--rgbd]
#
#   SCRIPT.py    your flight (see flights/)
#   NAME         the name the script gives its Drone: the mode name in QGC
#   --no-start   do not select the mode for you; pick it in QGC instead
#   --rgbd       fly the x500_rgbd drone (RealSense-style RGB-D camera) in the
#                rgbd_demo world, with the camera on /camera/... in ROS
#
# Run it on the host: it re-runs itself inside the jacob_ladder_sim container
# (./docker/run_sim_container.sh), which needs PX4 and the workspace built.
# PX4, the DDS agent, the translation node and jl_mission run as background jobs
# of this shell, with logs in the directory it prints; your script runs in the
# foreground, so its prints and errors show up here.
source "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/../jl_env.sh"

if [ ! -f /.dockerenv ]; then
  exec docker exec -it --user user -w "$JL_WS_ROOT" "${JL_CONTAINER:-jacob_ladder_sim}" \
    bash -c "$(printf '%q ' "$JL_WS_ROOT/launch_scripts/sim_script.sh" "$@")"
fi

source "$JL_WS_ROOT/test/sitl_common.sh"

[ $# -ge 2 ] || { sed -n '2,15p' "$0" | sed 's/^# \{0,1\}//'; exit 2; }
SCRIPT="$(readlink -f "$1")"; NAME="$2"; shift 2
START=true; RGBD=false
while [ $# -gt 0 ]; do
  case "$1" in
    --no-start) START=false; shift ;;
    --rgbd) RGBD=true; shift ;;
    *) echo "unknown option: $1"; exit 2 ;;
  esac
done
[ -f "$SCRIPT" ] || { echo "no such script: $SCRIPT"; exit 2; }

trap kill_flight EXIT
kill_flight   # nothing from an earlier run may be left over

logs=$(mktemp -d)
if $RGBD; then
  echo "== starting Gazebo (rgbd_demo) + PX4 (logs: $logs) =="
  GZ_GUI=1 start_px4_rgbd "$logs" || { echo "logs: $logs"; exit 1; }
else
  echo "== starting Gazebo + PX4 (logs: $logs) =="
  mkdir -p "$logs/rootfs" && cp "$PX4_BUILD/rootfs/gz_env.sh" "$logs/rootfs/"
  # No HEADLESS here: that is what opens the Gazebo window.
  PX4_SIM_MODEL=gz_x500 GZ_IP=127.0.0.1 \
    "$PX4_BUILD/bin/px4" -d -w "$logs/rootfs" "$PX4_BUILD/etc" > "$logs/px4.log" 2>&1 &
  MicroXRCEAgent udp4 -p 8888 > "$logs/agent.log" 2>&1 &
  ros2 run translation_node translation_node_bin > "$logs/translation.log" 2>&1 &
  for _ in $(seq 120); do grep -q "synchronized with time offset" "$logs/px4.log" && break; sleep 1; done
  grep -q "synchronized with time offset" "$logs/px4.log" || { echo "FAIL PX4 did not start (see $logs/px4.log)"; exit 1; }
  (cd "$logs/rootfs" && "$PX4_BUILD/bin/px4-param" set NAV_DLL_ACT 0) >> "$logs/commander.log" 2>&1  # sim: no data-link failsafe
fi

echo "== starting jl_mission as '$NAME' =="
ros2 launch jl_mission jl_mission.launch.py mission_name:="$NAME" > "$logs/launch.log" 2>&1 &
for _ in $(seq 60); do grep -q "Registered '$NAME'" "$logs/launch.log" && break; sleep 1; done
grep -q "Registered '$NAME'" "$logs/launch.log" || { echo "FAIL $NAME never registered with PX4 (see $logs/launch.log)"; exit 1; }

if $START; then
  # The script is waiting for the mode: select it a few seconds after it starts.
  ( sleep 4; echo "== selecting $NAME =="; cd "$logs/rootfs" && "$PX4_BUILD/bin/px4-commander" mode ext1 >> "$logs/commander.log" 2>&1 ) &
fi

echo "== running $SCRIPT =="
python3 "$SCRIPT"; rc=$?
echo "== script exited with $rc; logs: $logs =="
read -r -p "Press Enter to close the simulation... "
exit $rc
