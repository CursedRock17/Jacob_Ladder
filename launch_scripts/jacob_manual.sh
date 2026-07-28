#!/bin/bash

container_name="capstone_drone"
user="user"
# Paths used inside the container. The `docker run` in the README mounts the
# workspace at the same path it has on the host, so the host-resolved values are
# the defaults; set JL_DOCKER_WS_DIR / JL_DOCKER_PX4_DIR if you mount elsewhere.
source "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/../jl_env.sh"
WS_DIR="${JL_DOCKER_WS_DIR:-$JL_WS_ROOT}"
PX4_DIR="${JL_DOCKER_PX4_DIR:-$JL_PX4_DIR}"

# Tab names
tab_names=("PX4-SITL" "DDS-Agent" "Translation-Node" "Front-Tracker" "Front-Approach")

# Commands to run in each tab
commands=(
    "cd ${PX4_DIR} && make px4_sitl gz_x500_dual_cam_aruco_dual_ids"
    "cd ${WS_DIR} && source install/setup.bash && MicroXRCEAgent udp4 -p 8888"
    "cd ${WS_DIR} && source install/setup.bash && ros2 run translation_node translation_node_bin"
    "cd ${WS_DIR} && source install/setup.bash && ros2 launch aruco_tracker front_camera_aruco.launch.py"
    "cd ${WS_DIR} && source install/setup.bash && ros2 launch jacob_manual front_approach.launch.py"
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
        docker_cmd="docker exec -it --user ${user} ${container_name} bash -c \"sleep 15; ${commands[$i]}\""
    else
        docker_cmd="docker exec -it --user ${user} ${container_name} bash -c \"${commands[$i]}\""
    fi

    echo "Docker: ${docker_cmd}"
    gnome-terminal --tab --title="${tab_names[$i]}" -- bash -c "${docker_cmd}; exec bash"
    sleep 1
done
