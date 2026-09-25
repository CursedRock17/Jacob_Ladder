#!/bin/bash
# DroneSmoothPlanner (drogue_flight) in SITL, chasing an ArUco tag instead of the YOLO drogue.
# The planner runs unmodified; aruco_to_ranging stands in for the YOLO pose node.
# Create the container first with ./docker/run_sim_container.sh, then select
# "DroneSmoothPlanner" in QGC (or `commander mode ext1` in the PX4 tab).

container_name="${JL_CONTAINER:-jacob_ladder_sim}"
user="user"
source "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/../jl_env.sh"
WS_DIR="${JL_DOCKER_WS_DIR:-$JL_WS_ROOT}"
PX4_DIR="${JL_DOCKER_PX4_DIR:-$JL_PX4_DIR}"

# Gazebo loads the world and vehicle straight from this repo's gazebo/ folder, so
# nothing has to be copied into PX4-Autopilot. PX4 then attaches to that vehicle.
GZ_ENV="source ${PX4_DIR}/build/px4_sitl_default/rootfs/gz_env.sh && export GZ_SIM_RESOURCE_PATH=\\\$GZ_SIM_RESOURCE_PATH:${WS_DIR}/gazebo/models:${WS_DIR}/gazebo/worlds"
SPAWN="gz service -s /world/aruco_dual_ids/create --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 5000 --req 'sdf_filename: \\\"${WS_DIR}/gazebo/models/x500_dual_cam/model.sdf\\\", name: \\\"x500_dual_cam_0\\\"'"
PX4_RUN="cd ${PX4_DIR}/build/px4_sitl_default/rootfs && PX4_GZ_STANDALONE=1 PX4_GZ_MODEL_NAME=x500_dual_cam_0 PX4_SIM_MODEL=gz_x500_dual_cam ../bin/px4"

# Tab names
tab_names=("Gazebo" "PX4-SITL" "DDS-Agent" "Translation-Node" "Front-Tracker" "ArUco-to-Ranging" "Smooth-Planner")

# Commands to run in each tab
commands=(
    "${GZ_ENV} && gz sim -r ${WS_DIR}/gazebo/worlds/aruco_dual_ids.sdf"
    "sleep 15; ${GZ_ENV} && ${SPAWN} && ${PX4_RUN}"
    "cd ${WS_DIR} && source install/setup.bash && MicroXRCEAgent udp4 -p 8888"
    "cd ${WS_DIR} && source install/setup.bash && ros2 run translation_node translation_node_bin"
    "sleep 20; cd ${WS_DIR} && source install/setup.bash && ros2 launch aruco_tracker front_camera_aruco.launch.py"
    "cd ${WS_DIR} && source install/setup.bash && ros2 run drogue_flight aruco_to_ranging.py"
    "cd ${WS_DIR} && source install/setup.bash && ros2 launch drogue_flight autonomous_smooth_flight.launch.py"
)

# Start gnome-terminal with the first tab
docker_cmd="docker exec -it --user ${user} ${container_name} bash -c \"${commands[0]}\""
gnome-terminal --tab --title="${tab_names[0]}" -- bash -c "${docker_cmd}; exec bash"

# Open the rest of the tabs
for i in "${!commands[@]}"; do
    if [ $i -eq 0 ]; then
        continue
    fi
    docker_cmd="docker exec -it --user ${user} ${container_name} bash -c \"${commands[$i]}\""
    gnome-terminal --tab --title="${tab_names[$i]}" -- bash -c "${docker_cmd}; exec bash"
    sleep 1
done
