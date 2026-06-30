#!/bin/bash

container_name="px4-latest"
user="user"

# Tab names
tab_names=("DDS-Agent" "RQT-Image" "Translation-Node" "Aruco-Tracker" "Precision-Land")

# Commands to run in each tab
commands=(
    # "cd /src/PX4-Autopilot && make px4_sitl gz_x500_mono_cam_down_aruco; exec bash"
    "cd /aruco_land/Jacob_Ladder && source install/setup.bash && MicroXRCEAgent udp4 -p 8888; exec bash"
    "cd /aruco_land/Jacob_Ladder && source install/setup.bash && ros2 run rqt_image_view rqt_image_view; exec bash"
    "cd /aruco_land/Jacob_Ladder && source install/setup.bash && ros2 run translation_node translation_node_bin; exec bash"
    "cd /aruco_land/Jacob_Ladder && source install/setup.bash && ros2 launch aruco_tracker v1_16_tracker.launch.py; exec bash"
    "cd /aruco_land/Jacob_Ladder && source install/setup.bash && ros2 launch precision_land track_follow.launch.py; exec bash"
)

# Start gnome-terminal with the first tab
docker_cmd="docker exec -it --user ${user} ${container_name} bash -c \"${commands[0]}\""
gnome-terminal --tab --title="${tab_names[0]}" -- bash -c "${docker_cmd}"

# Open the rest of the tabs
for i in "${!commands[@]}"; do
    if [ $i -eq 0 ]; then
        continue
    fi

    # Add delay only for Precision-Land
    if [ $i -eq 4 ]; then
        docker_cmd="docker exec -it --user ${user} ${container_name} bash -c \"sleep 5; ${commands[$i]}\""
    else
        docker_cmd="docker exec -it --user ${user} ${container_name} bash -c \"${commands[$i]}\""
    fi

    gnome-terminal --tab --title="${tab_names[$i]}" -- bash -c "${docker_cmd}"
    sleep 1
done

