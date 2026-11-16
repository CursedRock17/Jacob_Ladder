#!/bin/bash
# Fly a jl_flight mission while the Arducam records 64x64 frames for flow_cnn.
#
#   ./launch_scripts/flow_capture.sh                        # the MISSION below
#   ./launch_scripts/flow_capture.sh missions/square.py     # any other mission
#
# Every run gets one name, flow_<date>_<time>_<MissionName>, used for all of
# its outputs so they can be matched up later:
#
#   $FLOW_CNN/captures/<run>/     frames + frames.csv (host receive time, unix ns)
#   flight_logs/<run>/mission.txt the mission's log (setpoints with report:=)
#   flight_logs/<run>/bag/        mission events + PX4 position and attitude
#   flight_logs/<run>/run.txt     what was run, and when
#
# The bag and frames.csv are stamped on the same Jetson clock, so a frame can
# be matched to the drone's state, and to the mission step it was taken in.
# The camera starts straight away: select the mission in QGC once it is up.
# Ctrl+C in the camera window stops it cleanly; `tmux kill-session -t flow`
# stops everything.

# ── edit these ──
MISSION="${1:-missions/flow_data.py}"
AIRFRAME="realsense_drone"
REPORT="0.5"                  # print setpoints every this many s ("" for none)
MODE_SLOT=7                   # register as ext7 every time (0: whatever PX4 gives)
CAMERA_DEVICE=0               # /dev/video0: the Arducam OV9281
CAPTURE_SECONDS=300           # how long the camera records
CAMERA_FORMAT="MJPG"          # YUYV hangs while the RealSense shares the USB 2.0 hub
FLOW_CNN="$HOME/Documents/flow_cnn"

SESSION="flow"

source "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/../jl_env.sh"
cd "$JL_WS_ROOT" || exit 1
jl_source_ros
jl_source_venv

if [ ! -f "$MISSION" ]; then
    echo "No such mission: $MISSION"
    exit 1
fi
# Validate before anything starts, and get the name QGC shows.
NAME=$(PYTHONPATH="$JL_WS_ROOT/src/jl_flight" python3 -m jl_flight name "$MISSION") || exit 1
PYTHONPATH="$JL_WS_ROOT/src/jl_flight" python3 -m jl_flight check "$MISSION" --airframe "$AIRFRAME" || exit 1

# With TakeoffHold registered too, PX4 stops sending arming check requests and
# both external modes time out ("no request received from FMU") ~10 s after the
# second registers. Run one mode at a time: `sudo systemctl start
# takeoff_hold.service` to bring it back (it also comes back on reboot).
if systemctl is-active --quiet takeoff_hold.service; then
    echo "Stopping takeoff_hold.service (it knocks $NAME's mode out of PX4)..."
    sudo systemctl stop takeoff_hold.service || exit 1
fi

RUN="flow_$(date +%Y%m%d_%H%M%S)_${NAME}"
LOGS="$JL_WS_ROOT/flight_logs/$RUN"
CAPTURE="$FLOW_CNN/captures/$RUN"
mkdir -p "$LOGS"
cat > "$LOGS/run.txt" <<EOF
run:      $RUN
started:  $(date --iso-8601=seconds)
mission:  $MISSION ($NAME)
airframe: $AIRFRAME
camera:   /dev/video$CAMERA_DEVICE, ${CAPTURE_SECONDS} s, $CAMERA_FORMAT, 64x64 grayscale
frames:   $CAPTURE
git:      $(git rev-parse --short HEAD)$(git diff --quiet || echo " (uncommitted changes)")
EOF

# Each tmux window is a fresh bash that does not source ROS: do it explicitly.
JL_SETUP="source $JL_WS_ROOT/jl_env.sh && jl_source_ros && jl_source_venv && cd $JL_WS_ROOT"
tmux_run() {
    tmux send-keys -t "$SESSION:$1" "$JL_SETUP && $2" Enter
}

tmux kill-session -t "$SESSION" 2>/dev/null

echo "=== Flow capture: $RUN ==="

tmux new-session -d -s "$SESSION" -n "Mission"
REPORT_ARG=${REPORT:+report:=$REPORT}
tmux_run "Mission" "ros2 launch jl_flight mission.launch.py mission:=$MISSION airframe:=$AIRFRAME mode_slot:=$MODE_SLOT $REPORT_ARG 2>&1 | tee $LOGS/mission.txt"

tmux new-window -t "$SESSION" -n "Bag"
tmux_run "Bag" "ros2 bag record -o $LOGS/bag /jl/$NAME/events /fmu/out/vehicle_local_position /fmu/out/vehicle_attitude /fmu/out/vehicle_status /fmu/out/vehicle_land_detected"

# 64x64 grayscale is arducam_capture.py's default (no --raw).
tmux new-window -t "$SESSION" -n "Camera"
tmux send-keys -t "$SESSION:Camera" "cd $FLOW_CNN && source .venv/bin/activate && python3 arducam_capture.py --device $CAMERA_DEVICE --seconds $CAPTURE_SECONDS --fourcc $CAMERA_FORMAT --output $CAPTURE" Enter

tmux new-window -t "$SESSION" -n "VIO"
tmux send-keys -t "$SESSION:VIO" "journalctl -u vio.service -f" Enter

echo "  mission: $NAME  (select it in QGC)"
echo "  frames:  $CAPTURE"
echo "  logs:    $LOGS"
echo "  Ctrl+B then W: window picker   Ctrl+B then D: detach"
tmux select-window -t "$SESSION:Mission"
tmux attach-session -t "$SESSION"
