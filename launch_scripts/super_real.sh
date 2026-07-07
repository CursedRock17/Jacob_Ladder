#!/bin/bash

SESSION="drone"

# Kill existing session if it exists
tmux kill-session -t $SESSION 2>/dev/null

echo "=== Drone Launch Script ==="
echo ""
echo "NOTE: uXRCE Agent and Translation Node now run as systemd services"
echo "(dds_agent.service, translation_node.service) — started at boot."
echo "Their tmux windows just tail the service logs."
echo ""

# Window 1: uXRCE Agent (systemd service — log view only)
echo "[1/5] uXRCE Agent (service log)..."
tmux new-session -d -s $SESSION -n "uXRCE Agent"
tmux send-keys -t $SESSION:"uXRCE Agent" "systemctl status dds_agent.service --no-pager; journalctl -u dds_agent.service -f" Enter

# Window 2: Translation Node (systemd service — log view only)
echo "[2/5] Translation Node (service log)..."
tmux new-window -t $SESSION -n "Translation Node"
tmux send-keys -t $SESSION:"Translation Node" "systemctl status translation_node.service --no-pager; journalctl -u translation_node.service -f" Enter

# Window 3: Camera Node
echo "[3/5] Starting Camera Node..."
tmux new-window -t $SESSION -n "Camera Node"
tmux send-keys -t $SESSION:"Camera Node" "ros2 run depthai_ros_driver_v3 driver_node --ros-args -r /driver/rgb/image_raw:=/image_raw -r /driver/rgb/camera_info:=/camera_info" Enter
# tmux send-keys -t $SESSION:"Camera Node" "ros2 run depthai_ros_driver camera_node --ros-args -r /oak/rgb/image_raw:=/front/camera/image_raw -r /oak/rgb/camera_info:=/front/camera/camera_info" Enter

# Window 4: Aruco Tracker
echo "[4/5] Starting Aruco Tracker..."
# tmux new-window -t $SESSION -n "Aruco Tracker"
# tmux send-keys -t $SESSION:"Aruco Tracker" "ros2 launch aruco_tracker front_camera.launch.py" Enter

# Window 5: Precision Land
echo "[5/5] Starting Precision Land..."
# tmux new-window -t $SESSION -n "Precision Land"
# tmux send-keys -t $SESSION:"Precision Land" "ros2 launch precision_land front_approach.launch.py" Enter

# Windoow 6: VIO
echo "[6/6] Starting VIO via cuVSLAM..."
tmux new-window -t $SESSION -n "cuVSLAM"
tmux send-keys -t $SESSION:"cuVSLAM" "ros2 launch oak_d_visual_odometry cuvslam_px4.launch.py" Enter

echo ""
echo "=== All nodes launched ==="
echo ""
echo "Attaching to tmux session..."
echo ""
echo "--- Cheatsheet ---"
echo "  Ctrl+B then W       : visual window picker"
echo "  Ctrl+B then 0-4     : jump to window by number"
echo "  Ctrl+B then D       : detach (everything keeps running)"
echo "  tmux attach -t drone: reattach after disconnect"
echo "  Restart a service   : sudo systemctl restart dds_agent translation_node"
echo ""

# Attach on the first window
tmux select-window -t $SESSION:"uXRCE Agent"
tmux attach-session -t $SESSION
