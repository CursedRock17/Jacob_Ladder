#!/bin/bash

container_name="jacob_drone"
user="user"

# Tab names
tab_names=("PX4-SITL" "DDS-Agent" "Translation-Node" "Takeoff & Hover & Land")

# Commands to run in each tab
commands=(
    "cd /home/cursedrock17/Documents/Electrical/Matrix_Lab/jacob_drone_ws/src/PX4-Autopilot && make px4_sitl gz_x500_dual_cam"
    "cd /home/cursedrock17/Documents/Electrical/Matrix_Lab/jacob_drone_ws/src/Jacob_Ladder && source install/setup.bash && MicroXRCEAgent udp4 -p 8888"
    "cd /home/cursedrock17/Documents/Electrical/Matrix_Lab/jacob_drone_ws/src/Jacob_Ladder && source install/setup.bash && ros2 run translation_node translation_node_bin"
    "cd /home/cursedrock17/Documents/Electrical/Matrix_Lab/jacob_drone_ws/src/Jacob_Ladder && source install/setup.bash && ros2 launch precision_land takeoff_land.launch.py"
)

# Start gnome-terminal with the first tab
docker_cmd="docker exec -it --user ${user} ${container_name} bash -c \"${commands[0]}\""
gnome-terminal --tab --title="${tab_names[0]}" -- bash -c "${docker_cmd}; exec bash"
echo "Docker: ${docker_cmd}"

# Open the rest of the tabs
for i in "${!commands[@]}"; do
    echo "Current Command: ${i}"
    if [ $i -eq 0 ]; then
        continue
    fi

    # Add 20 sec delay only for Precision-Land
    if [ $i -eq 4 ]; then
        docker_cmd="docker exec -it --user ${user} ${container_name} bash -c \"sleep 5; ${commands[$i]}\""
    else
        docker_cmd="docker exec -it --user ${user} ${container_name} bash -c \"${commands[$i]}\""
    fi

    echo "Docker: ${docker_cmd}"
    gnome-terminal --tab --title="${tab_names[$i]}" -- bash -c "${docker_cmd}; exec bash"
    sleep 1
done
