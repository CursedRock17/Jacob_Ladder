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
  pkill -f "ros2 launch jl_flight" 2>/dev/null
  pkill -f "python3 -m jl_flight run" 2>/dev/null
  pkill -f "aruco_tracker" 2>/dev/null
  pkill -f "ros2_yolo_image_processing" 2>/dev/null
  pkill -f "parameter_bridge" 2>/dev/null
  pkill -f "fake_runner.py" 2>/dev/null
  pkill -f "fake_pilot.py" 2>/dev/null
  pkill -f "record_pose.py" 2>/dev/null
  wait_for_no_px4
}

# Wait until PX4's preflight checks pass ("Ready for takeoff!"). PX4 reports its
# clock synchronised seconds before it will arm, and a mission selected in that
# gap has its arm rejected. $1: log dir. Returns 1 if it never gets there.
wait_ready_to_arm() {
  for _ in $(seq 90); do grep -q "Ready for takeoff" "$1/px4.log" && return 0; sleep 1; done
  echo "FAIL PX4 never became ready to arm (see $1/px4.log)"; return 1
}

# Wait until a freshly started mission can be selected and armed: jl_mission has
# registered NAME with PX4, PX4 has sent the new mode its first arming-check
# request, and the reply has had time to land. An arm requested before that is
# refused ("Resolve system health failures first"). On the drone the pilot selects
# long after boot, so this only matters to scripts that select straight away.
# $1: the launch log, $2: the mission name. Returns 1 if it never registers.
wait_mode_ready() {
  for _ in $(seq 60); do
    grep -q "Registered '$2'" "$1" && grep -q "Arming check request" "$1" && { sleep 2; return 0; }
    sleep 1
  done
  echo "FAIL $2 never registered with PX4 (see $1)"; return 1
}

# The external-mode slot jl_mission registered NAME in, for `commander mode extN`
# (usually ext1; see jl_mission's main.cpp for when it is not). $1: the launch
# log, $2: the mission name.
mode_slot() {
  grep -o "Registered '$2' with PX4 (mode ext[0-9]*)" "$1" | tail -1 | grep -o "ext[0-9]*" || echo ext1
}

# Select NAME's mode, in whichever slot it registered. $1: log dir (with
# launch.log), $2: the mission name.
select_mission() {
  python3 "$JL_WS_ROOT/test/select_mode.py" "$(mode_slot "$1/launch.log" "$2")" >> "$1/commander.log" 2>&1
}

# Start PX4 + agent + translation node in a fresh rootfs, and wait until PX4
# is ready to arm. $1: log dir. GZ_GUI=1 opens the Gazebo window. Returns 1 if
# PX4 never becomes ready.
start_px4() {
  local logs="$1"
  mkdir -p "$logs/rootfs" && cp "$PX4_BUILD/rootfs/gz_env.sh" "$logs/rootfs/"
  # PX4 starts Gazebo itself; HEADLESS keeps its window closed unless GZ_GUI is set.
  local headless=1; [ -n "${GZ_GUI:-}" ] && headless=
  HEADLESS=$headless PX4_SIM_MODEL=gz_x500 GZ_IP=127.0.0.1 \
    "$PX4_BUILD/bin/px4" -d -w "$logs/rootfs" "$PX4_BUILD/etc" > "$logs/px4.log" 2>&1 &
  MicroXRCEAgent udp4 -p 8888 > "$logs/agent.log" 2>&1 &
  ros2 run translation_node translation_node_bin > "$logs/translation.log" 2>&1 &
  for _ in $(seq 120); do grep -q "synchronized with time offset" "$logs/px4.log" && break; sleep 1; done
  (cd "$logs/rootfs" && "$PX4_BUILD/bin/px4-param" set NAV_DLL_ACT 0) >> "$logs/commander.log" 2>&1  # no GCS in this test
  wait_ready_to_arm "$logs"
}

# Start a world from gazebo/worlds with one of our drones in it, for missions
# that see: Gazebo with rendering (cameras need it) loading the world, the
# airframe's model spawned at the origin facing +X, PX4 attached to it in
# standalone mode with the stock gz_x500 airframe (so nothing has to be added to
# PX4-Autopilot), the agent, the translation node, and the bridge that puts the
# airframe's cameras on ROS. Trackers are not started here: the mission's
# declared perception starts its own (jl_flight's mission.launch.py).
#   $1 log dir, $2 world (rgbd_demo, apriltag_wall, aruco_dual_ids),
#   $3 airframe (x500_rgbd or x500_dual_cam).
# GZ_GUI=1 opens the Gazebo window; GZ_RENDER=x11 renders headless through
# $DISPLAY instead of EGL (needed when EGL cannot open the GPU, which gives
# "eglInitialize failed"); RGBD_POINTS=true bridges x500_rgbd's heavy point cloud.
# Returns 1 if Gazebo or PX4 never comes up. Callers must `trap kill_flight EXIT`
# first: it leaves processes running on its failure paths.
start_px4_world() {
  local logs="$1" world="$2" airframe="$3" bridge
  case "$airframe" in
    x500_rgbd)
      bridge=(ros2 launch "$JL_WS_ROOT/gazebo/launch/rgbd_bridge.launch.py" points:="${RGBD_POINTS:-false}") ;;
    x500_dual_cam)
      bridge=(ros2 launch "$JL_WS_ROOT/gazebo/launch/dualcam_bridge.launch.py" world:="$world" model:="${airframe}_0") ;;
    *) echo "FAIL no Gazebo model for airframe $airframe"; return 1 ;;
  esac
  mkdir -p "$logs/rootfs" && cp "$PX4_BUILD/rootfs/gz_env.sh" "$logs/rootfs/"
  source "$PX4_BUILD/rootfs/gz_env.sh"
  export GZ_SIM_RESOURCE_PATH="$GZ_SIM_RESOURCE_PATH:$JL_WS_ROOT/gazebo/models:$JL_WS_ROOT/gazebo/worlds"
  export GZ_IP=127.0.0.1
  if [ -n "${GZ_GUI:-}" ]; then
    gz sim -r "$JL_WS_ROOT/gazebo/worlds/$world.sdf" > "$logs/gz.log" 2>&1 &
  elif [ "${GZ_RENDER:-egl}" = x11 ]; then
    gz sim -s -r "$JL_WS_ROOT/gazebo/worlds/$world.sdf" > "$logs/gz.log" 2>&1 &
  else
    gz sim -s -r --headless-rendering "$JL_WS_ROOT/gazebo/worlds/$world.sdf" > "$logs/gz.log" 2>&1 &
  fi
  for _ in $(seq 60); do
    gz service -l 2>/dev/null | grep -q "/world/$world/create" && break; sleep 1
  done
  if ! gz service -l 2>/dev/null | grep -q "/world/$world/create"; then
    echo "FAIL gazebo world $world did not start (see $logs/gz.log)"
    grep -q "eglInitialize failed" "$logs/gz.log" &&
      echo "hint: headless EGL rendering does not work on this GPU setup; rerun with GZ_RENDER=x11 to render through the X display instead"
    return 1
  fi
  gz service -s "/world/$world/create" --reqtype gz.msgs.EntityFactory \
    --reptype gz.msgs.Boolean --timeout 5000 \
    --req "sdf_filename: \"$JL_WS_ROOT/gazebo/models/$airframe/model.sdf\", name: \"${airframe}_0\"" \
    >> "$logs/gz.log" 2>&1
  PX4_GZ_STANDALONE=1 PX4_GZ_WORLD=$world PX4_GZ_MODEL_NAME="${airframe}_0" \
    PX4_SIM_MODEL=gz_x500 \
    "$PX4_BUILD/bin/px4" -d -w "$logs/rootfs" "$PX4_BUILD/etc" > "$logs/px4.log" 2>&1 &
  MicroXRCEAgent udp4 -p 8888 > "$logs/agent.log" 2>&1 &
  ros2 run translation_node translation_node_bin > "$logs/translation.log" 2>&1 &
  for _ in $(seq 120); do grep -q "synchronized with time offset" "$logs/px4.log" && break; sleep 1; done
  if ! grep -q "synchronized with time offset" "$logs/px4.log"; then
    echo "FAIL PX4 did not start (see $logs/px4.log)"; return 1
  fi
  (cd "$logs/rootfs" && "$PX4_BUILD/bin/px4-param" set NAV_DLL_ACT 0) >> "$logs/commander.log" 2>&1  # no GCS in this test
  "${bridge[@]}" > "$logs/bridge.log" 2>&1 &
  wait_ready_to_arm "$logs"
}

# The SITL worlds by short name: "WORLD AIRFRAME" for start_px4_world, or
# "default generic" for PX4's own empty world (start_px4).
world_setup() {
  case "$1" in
    default) echo "default generic" ;;
    apriltag) echo "apriltag_wall x500_rgbd" ;;
    rgbd) echo "rgbd_demo x500_rgbd" ;;
    aruco) echo "aruco_dual_ids x500_dual_cam" ;;
    *) return 1 ;;
  esac
}

# Print where Gazebo really has a model: "x y z" in Gazebo's world frame (ENU, so x is
# east and y is north), to compare with what PX4 believes. $1: world, $2: model name.
gz_truth() {
  gz topic -e -t "/world/$1/pose/info" -n 1 2>/dev/null | python3 -c '
import re, sys
text = sys.stdin.read()
for block in re.split(r"\npose \{", text):
    if re.search(r"name: \"" + sys.argv[1] + "\"", block):
        xyz = [re.search(axis + r": (-?[0-9.e+-]+)", block.split("position {")[1]).group(1) for axis in "xyz"]
        print(" ".join(xyz)); break
' "$2"
}
