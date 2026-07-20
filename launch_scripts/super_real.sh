#!/bin/bash

SESSION="drone"
FLIGHT_NUMBER="twelve"

# Create a logging directory

mkdir -p flight_logs/${FLIGHT_NUMBER}_flight

# Kill existing session if it exists
tmux kill-session -t $SESSION 2>/dev/null

echo "=== Drone Launch Script ==="
echo ""
echo "NOTE: uXRCE Agent and Translation Node now run as systemd services"
echo "(dds_agent.service, translation_node.service) — started at boot."
echo "Their tmux windows just tail the service logs."
echo ""

# Window 1: uXRCE Agent (systemd service — log view only)
echo "[1/9] uXRCE Agent Status (service log)..."
tmux new-session -d -s $SESSION -n "uXRCE Agent"
tmux send-keys -t $SESSION:"uXRCE Agent" "systemctl status dds_agent.service --no-pager; journalctl -u dds_agent.service -f" Enter

# Window 2: Translation Node (systemd service — log view only)
echo "[2/9] Translation Node Status (service log)..."
tmux new-window -t $SESSION -n "Translation Node"
tmux send-keys -t $SESSION:"Translation Node" "systemctl status translation_node.service --no-pager; journalctl -u translation_node.service -f" Enter

# tmux send-keys -t $SESSION:"Camera 
# Window 3: VIO
echo "[3/9] Starting VIO via cuVSLAM..."
tmux new-window -t $SESSION -n "cuVSLAM"
tmux send-keys -t $SESSION:"cuVSLAM" "ros2 launch oak_d_visual_odometry cuvslam_px4.launch.py" Enter

# Window 4: OAK-D Lite Camera Node — RETIRED 2026-07-16. The Lite was removed
# from the airframe; the cuVSLAM node (window 3) now publishes the OAK-D S2's
# CAM_A stream on /front/camera/image_raw + /front/camera/camera_info
# (see rgb_topic in cuvslam_params.yaml). If the Lite returns, re-enable this
# AND revert rgb_topic/rgb_camera_info_topic in cuvslam_params.yaml to
# /rgb/image + /rgb/camera_info — otherwise both cameras publish on
# /front/camera/image_raw at once.
echo "[4/9] Camera Node retired (front camera now served by cuVSLAM window)..."
tmux new-window -t $SESSION -n "Camera Node"
#tmux send-keys -t $SESSION:"Camera Node" "ros2 run depthai_ros_driver_v3 driver_node --ros-args -p driver.i_device_id:=18443010B17F0C1300 -r /driver/rgb/image_raw:=/front/camera/image_raw -r /driver/rgb/camera_info:=/front/camera/camera_info" Enter

# Window 5: Aruco Tracker (Option 1)
echo "[5/9] Starting Aruco Tracker..."
tmux new-window -t $SESSION -n "Aruco Tracker"
#tmux send-keyu -t $SESSION:"Aruco Tracker" "python3 ~/Downloads/tegrastats_viewer.py --log ~/logs/fifth_flight.csv" Enter

# Window 5: Drogue Detection (Option 2)
echo "[6/9] Starting Drouge Ranging..."
tmux new-window -t $SESSION -n "Drogue Detection"
tmux send-keys -t $SESSION:"Drogue Detection" "ros2 run ros2_yolo_image_processing drogue_detection_node --ros-args -r image_raw:=/front/camera/image_raw" Enter

# Window 9 (Additional): Drogue Pose (Option 2)
echo "[9/9] Starting Drouge Pose..."
tmux new-window -t $SESSION -n "Drogue Pose"
tmux send-keys -t $SESSION:"Drogue Pose" "ros2 run ros2_yolo_image_processing pose_estimation_node --ros-args -r camera_info:=/front/camera/camera_info" Enter

# Window 6: Autonomous ROS Code
echo "[6/9] Starting Autonomous ROS Code..."
tmux new-window -t $SESSION -n "Autonomous ROS Code"
#tmux send-keys -t $SESSION:"Autonomous ROS Code" "ros2 launch precision_land takeoff_land.launch.py | tee flight_logs/${FLIGHT_NUMBER}_flight/precision_land_takeoff_land.txt" Enter

# Window 7 ROS Bag
echo "[7/9] Starting ROS Bag..."
tmux new-window -t $SESSION -n "ROS Bag"
tmux send-keys -t $SESSION:"ROS Bag" "./launch_scripts/foxglove_wifi.sh" Enter
#tmux send-keys -t $SESSION:"ROS Bag" "ros2 bag record --all -s mcap -o flight_logs/${FLIGHT_NUMBER}_flight/bag" Enter

# Window 8: Tegrastats Logger
echo "[8/9] Starting Tegrastats Logger..."
tmux new-window -t $SESSION -n "Tegrastats Logger"
#tmux send-keys -t $SESSION:"Tegrastats Logger" "python3 ~/Downloads/tegrastats_viewer.py --log /home/usmsm/Jacob_Ladder/flight_logs/${FLIGHT_NUMBER}_flight/tegrastats_${FLIGHT_NUMBER}.csv" Enter

echo ""
echo "=== All nodes launched ==="
echo ""
echo "Attaching to tmux session..."
echo ""
echo "--- Cheatsheet ---"
echo "  Ctrl+B then W       : visual window picker"
echo "  Ctrl+B then 0-9     : jump to window by number"
echo "  Ctrl+B then D       : detach (everything keeps running)"
echo "  tmux attach -t drone: reattach after disconnect"
echo "  Restart a service   : sudo systemctl restart dds_agent translation_node"
echo ""

# Attach on the first window
tmux select-window -t $SESSION:"uXRCE Agent"
tmux attach-session -t $SESSION
