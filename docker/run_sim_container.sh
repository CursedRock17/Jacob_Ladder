#!/bin/bash
# Create the GUI simulation container for this checkout.
#
#   ./docker/run_sim_container.sh            # creates + starts "jacob_ladder_sim"
#   JL_CONTAINER=my_name ./docker/run_sim_container.sh
#
# The workspace and PX4 are mounted at the SAME paths they have on the host, so
# launch scripts can `docker exec ... cd <host path>` and colcon/PX4 build dirs
# stay valid on both sides. LOCAL_USER_ID makes the image's entrypoint remap its
# `user` to your uid, so files built in the container stay yours on the host.
set -e
source "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/../jl_env.sh"
CONTAINER="${JL_CONTAINER:-jacob_ladder_sim}"
IMAGE="${JL_IMAGE:-lucaswendland/jacob_ladder:latest}"

docker run -d --name "$CONTAINER" \
  --network host \
  --device /dev/dri \
  --group-add "$(stat -c %g /dev/dri/card*)" \
  --group-add "$(stat -c %g /dev/dri/renderD128)" \
  -e DISPLAY="${DISPLAY:-:0}" \
  -e LOCAL_USER_ID="$(id -u)" \
  -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
  -v "$JL_WS_ROOT:$JL_WS_ROOT:rw" \
  -v "$JL_PX4_DIR:$JL_PX4_DIR:rw" \
  "$IMAGE" sleep infinity

echo "Started $CONTAINER. Build inside it once:"
echo "  docker exec -it --user user $CONTAINER bash -c 'cd $JL_PX4_DIR && make px4_sitl'"
echo "  docker exec -it --user user $CONTAINER bash -c 'cd $JL_WS_ROOT && source /opt/ros/humble/setup.bash && colcon build'"
