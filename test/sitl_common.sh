# Shared by the headless SITL checks: start PX4 + agent + translation node in
# a fresh rootfs, and kill everything between flights. Source it, don't run it.
# Each script sets its own GZ_PARTITION before calling start_px4.
source "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/../jl_env.sh"
source /opt/ros/humble/setup.bash
source "$JL_WS_ROOT/install/setup.bash"
PX4_BUILD="$JL_PX4_DIR/build/px4_sitl_default"

wait_for_no_px4() {
  for _ in $(seq 30); do
    pgrep -f "$PX4_BUILD/bin/px4" >/dev/null || return 0
    sleep 1
  done
  echo "warning: a bin/px4 process is still running after 30s" >&2
}

# Kill everything from the previous flight before starting the next one.
kill_flight() {
  kill $(jobs -p) 2>/dev/null
  pkill -f "gz sim" 2>/dev/null
  pkill -f "$PX4_BUILD/bin/px4" 2>/dev/null
  pkill -f "MicroXRCEAgent" 2>/dev/null
  pkill -f "translation_node_bin" 2>/dev/null
  pkill -f "ros2 run jl_mission jl_mission" 2>/dev/null
  pkill -f "lib/jl_mission/jl_mission" 2>/dev/null
  pkill -f "ros2 launch jl_blocks" 2>/dev/null
  pkill -f "lib/jl_blocks/mission_runner" 2>/dev/null
  pkill -f "lib/jl_blocks/mission_watch" 2>/dev/null
  pkill -f "aruco_tracker" 2>/dev/null
  pkill -f "parameter_bridge" 2>/dev/null
  pkill -f "fake_runner.py" 2>/dev/null
  pkill -f "fake_pilot.py" 2>/dev/null
  wait_for_no_px4
}

# Start PX4 + agent + translation node in a fresh rootfs, and wait until PX4
# is up. $1: log dir.
start_px4() {
  local logs="$1"
  mkdir -p "$logs/rootfs" && cp "$PX4_BUILD/rootfs/gz_env.sh" "$logs/rootfs/"
  HEADLESS=1 PX4_SIM_MODEL=gz_x500 GZ_IP=127.0.0.1 \
    "$PX4_BUILD/bin/px4" -d -w "$logs/rootfs" "$PX4_BUILD/etc" > "$logs/px4.log" 2>&1 &
  MicroXRCEAgent udp4 -p 8888 > "$logs/agent.log" 2>&1 &
  ros2 run translation_node translation_node_bin > "$logs/translation.log" 2>&1 &
  for _ in $(seq 120); do grep -q "synchronized with time offset" "$logs/px4.log" && break; sleep 1; done
  (cd "$logs/rootfs" && "$PX4_BUILD/bin/px4-param" set NAV_DLL_ACT 0) >> "$logs/commander.log" 2>&1  # no GCS in this test
}

# Start the ArUco world headless: Gazebo with rendering (the cameras need it)
# loading gazebo/worlds/aruco_dual_ids.sdf, the x500_dual_cam vehicle, PX4
# attached to it in standalone mode, the agent, the translation node, and the
# front aruco_tracker. The same setup as launch_scripts/aruco_smooth_planner.sh,
# without the GUI. $1: log dir. Returns 1 if Gazebo or PX4 never comes up.
# Callers must `trap kill_flight EXIT` before calling it: it leaves processes
# running on its failure paths.
start_px4_aruco() {
  local logs="$1" world=aruco_dual_ids
  mkdir -p "$logs/rootfs" && cp "$PX4_BUILD/rootfs/gz_env.sh" "$logs/rootfs/"
  source "$PX4_BUILD/rootfs/gz_env.sh"
  export GZ_SIM_RESOURCE_PATH="$GZ_SIM_RESOURCE_PATH:$JL_WS_ROOT/gazebo/models:$JL_WS_ROOT/gazebo/worlds"
  export GZ_IP=127.0.0.1
  gz sim -s -r --headless-rendering "$JL_WS_ROOT/gazebo/worlds/$world.sdf" > "$logs/gz.log" 2>&1 &
  for _ in $(seq 60); do
    gz service -l 2>/dev/null | grep -q "/world/$world/create" && break; sleep 1
  done
  if ! gz service -l 2>/dev/null | grep -q "/world/$world/create"; then
    echo "FAIL gazebo world $world did not start (see $logs/gz.log)"; return 1
  fi
  gz service -s "/world/$world/create" --reqtype gz.msgs.EntityFactory \
    --reptype gz.msgs.Boolean --timeout 5000 \
    --req "sdf_filename: \"$JL_WS_ROOT/gazebo/models/x500_dual_cam/model.sdf\", name: \"x500_dual_cam_0\"" \
    >> "$logs/gz.log" 2>&1
  PX4_GZ_STANDALONE=1 PX4_GZ_WORLD=$world PX4_GZ_MODEL_NAME=x500_dual_cam_0 \
    PX4_SIM_MODEL=gz_x500_dual_cam \
    "$PX4_BUILD/bin/px4" -d -w "$logs/rootfs" "$PX4_BUILD/etc" > "$logs/px4.log" 2>&1 &
  MicroXRCEAgent udp4 -p 8888 > "$logs/agent.log" 2>&1 &
  ros2 run translation_node translation_node_bin > "$logs/translation.log" 2>&1 &
  for _ in $(seq 120); do grep -q "synchronized with time offset" "$logs/px4.log" && break; sleep 1; done
  if ! grep -q "synchronized with time offset" "$logs/px4.log"; then
    echo "FAIL PX4 did not start (see $logs/px4.log)"; return 1
  fi
  (cd "$logs/rootfs" && "$PX4_BUILD/bin/px4-param" set NAV_DLL_ACT 0) >> "$logs/commander.log" 2>&1  # no GCS in this test
  ros2 launch aruco_tracker front_camera_aruco.launch.py > "$logs/tracker.log" 2>&1 &
}
