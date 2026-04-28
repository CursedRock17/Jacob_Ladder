#!/bin/bash

SESSION="drone"

# Kill existing session if it exists
tmux kill-session -t $SESSION 2>/dev/null

echo "=== Drone Launch Script ==="
echo ""

# Window 1: uXRCE Agent
echo "[1/5] Starting uXRCE Agent..."
tmux new-session -d -s $SESSION -n "uXRCE Agent"
tmux send-keys -t $SESSION:"uXRCE Agent" "docker exec -it --user user jacob_drone bash -c 'cd /home/cursedrock17/Documents/Electrical/Matrix_Lab/jacob_drone_ws/src/PX4-Autopilot && make px4_sitl gz_x500_dual_cam_aruco_dual_ids'" Enter

# Window 2: Translation Node
echo "[2/5] Starting Translation Node..."
tmux new-window -t $SESSION -n "Translation Node"
tmux send-keys -t $SESSION:"Translation Node" "docker exec -it --user user jacob_drone bash -c 'cd /home/cursedrock17/Documents/Electrical/Matrix_Lab/jacob_drone_ws/src/PX4-Autopilot && make px4_sitl gz_x500_dual_cam_aruco_dual_ids'" Enter

# Window 3: Camera Node
echo "[3/5] Starting Camera Node..."
tmux new-window -t $SESSION -n "Camera Node"
tmux send-keys -t $SESSION:"Camera Node" "docker exec -it --user user jacob_drone bash -c 'cd /home/cursedrock17/Documents/Electrical/Matrix_Lab/jacob_drone_ws/src/Jacob_Ladder && source install/setup.bash && ros2 run translation_node translation_node_bin'" Enter

# Window 4: Aruco Tracker
echo "[4/5] Starting Aruco Tracker..."
tmux new-window -t $SESSION -n "Aruco Tracker"
tmux send-keys -t $SESSION:"Aruco Tracker" "docker exec -it --user user jacob_drone bash -c 'cd /home/cursedrock17/Documents/Electrical/Matrix_Lab/jacob_drone_ws/src/Jacob_Ladder && source install/setup.bash && ros2 launch aruco_tracker front_camera_aruco.launch.py'" Enter

# Window 5: Precision Land
echo "[5/5] Starting Precision Land..."
tmux new-window -t $SESSION -n "Precision Land"
tmux send-keys -t $SESSION:"Precision Land" "docker exec -it --user user jacob_drone bash -c 'sleep 15; cd /home/cursedrock17/Documents/Electrical/Matrix_Lab/jacob_drone_ws/src/Jacob_Ladder && source install/setup.bash && ros2 launch jacob_manual front_approach.launch.py'" Enter

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
echo ""

# Attach on the first window
tmux select-window -t $SESSION:"uXRCE Agent"
tmux attach-session -t $SESSION
