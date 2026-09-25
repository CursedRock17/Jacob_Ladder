#!/bin/bash
# SITL smoke test: fly one mode headless and check it publishes its debug topics.
#
#   ./sitl_debug_topics.sh <executable> <params yaml in cfg/>
#   ./sitl_debug_topics.sh takeoff_land takeoff_land_params.yaml
#
# Passes when the mode publishes /tracking_error and <node>/state after being
# selected. Run from the workspace root, after colcon build, inside the container.

EXE=$1
PARAMS=$2
source "$(dirname "$(readlink -f "$0")")/../../../jl_env.sh"
source /opt/ros/humble/setup.bash
source "$JL_WS_ROOT/install/setup.bash"
LOGS=$(mktemp -d)
# Explicit partition: gz derives the default from the username, which fails for
# a container uid with no passwd entry, and it keeps this sim isolated anyway
export GZ_PARTITION=sitl_debug_topics
PX4_BUILD="$JL_PX4_DIR/build/px4_sitl_default"
trap 'kill $(jobs -p) 2>/dev/null; pkill -f "gz sim"; pkill -f "$PX4_BUILD/bin/px4"' EXIT

# PX4 + Gazebo (no GUI), the DDS agent, the translation node, then the mode itself
# -d: no interactive shell (it would spin on the closed stdin)
# -w: fresh working dir, so default params and your saved SITL params stay untouched
mkdir -p "$LOGS/rootfs" && cp "$PX4_BUILD/rootfs/gz_env.sh" "$LOGS/rootfs/"  # gz model/world paths
HEADLESS=1 PX4_SIM_MODEL=gz_x500 GZ_IP=127.0.0.1 \
  "$PX4_BUILD/bin/px4" -d -w "$LOGS/rootfs" "$PX4_BUILD/etc" > "$LOGS/px4.log" 2>&1 &
MicroXRCEAgent udp4 -p 8888 > "$LOGS/agent.log" 2>&1 &
ros2 run translation_node translation_node_bin > "$LOGS/translation.log" 2>&1 &
px4cmd() { (cd "$LOGS/rootfs" && "$PX4_BUILD/bin/px4-$1" "${@:2}") >> "$LOGS/commander.log" 2>&1; }
# Start the mode only once PX4's DDS link is up: registration gives up after ~20 s
for _ in $(seq 120); do grep -q "synchronized with time offset" "$LOGS/px4.log" && break; sleep 1; done
px4cmd param set NAV_DLL_ACT 0  # no GCS in this test, so don't make arming wait for one
ros2 run precision_land "$EXE" --ros-args \
  --params-file "$(ros2 pkg prefix precision_land)/share/precision_land/cfg/$PARAMS" \
  -r __node:="$EXE" > "$LOGS/mode.log" 2>&1 &

# Wait for PX4 to list our external mode, then select it (the executor arms and takes off)
# Retry slowly: re-selecting the mode mid-takeoff would restart the executor
for _ in $(seq 12); do
  px4cmd commander mode ext1
  sleep 10
  grep -q "executor —" "$LOGS/mode.log" && break  # every executor logs this on activation
done

check() {  # check <topic>: pass if one message arrives within 90 s
  if timeout 90 ros2 topic echo --once "$1" > /dev/null 2>&1; then echo "PASS $1"; else echo "FAIL $1"; return 1; fi
}
check /tracking_error; rc=$?
check "/$EXE/state" || rc=1
[ $rc -ne 0 ] && echo "logs: $LOGS" && tail -20 "$LOGS/mode.log"
exit $rc
