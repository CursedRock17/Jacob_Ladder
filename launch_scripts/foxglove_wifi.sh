#!/bin/bash
# Low-bandwidth Foxglove viewing stack for hotspot/WiFi debugging.
#
# Replaces the bare `ros2 launch foxglove_bridge foxglove_bridge_launch.xml`:
#   1. JPEG-compresses the two image topics on the Jetson (image_transport
#      republish) so the websocket carries ~0.5 Mbps per stream instead of
#      ~120+ Mbps of raw BGR.
#   2. Whitelists the bridge to the topics we actually debug — by default the
#      bridge serializes EVERY topic a panel touches, raw images included.
#   3. Caps the per-client send buffer at 1 MB so a slow radio drops stale
#      frames instead of queueing seconds of latency (default is 10 MB).
#
# In Foxglove, point image panels at the /compressed topics.
# Details: general_docs/reviewing_flight_data.md ("Watching live flight data")

set -e
source "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/../jl_env.sh"
cd "$JL_WS_ROOT"
jl_source_ros

export FASTRTPS_DEFAULT_PROFILES_FILE="$JL_WS_ROOT/config/fastdds_wifi_profile.xml"

JPEG_QUALITY="${JPEG_QUALITY:-60}"

cleanup() { kill 0 2>/dev/null; }
trap cleanup EXIT

# YOLO annotated output (the 15-20 Hz drogue view)
ros2 run image_transport republish raw compressed --ros-args \
    -r __node:=republish_detections_image \
    -r in:=/detections_image \
    -r out/compressed:=/detections_image/compressed \
    -p out.jpeg_quality:="${JPEG_QUALITY}" &

# Raw front camera (useful when the detector is down)
ros2 run image_transport republish raw compressed --ros-args \
    -r __node:=republish_front_camera \
    -r in:=/front/camera/image_raw \
    -r out/compressed:=/front/camera/image_raw/compressed \
    -p out.jpeg_quality:="${JPEG_QUALITY}" &

# Features Image for cuVSLAM (useful when the detector is down)
ros2 run image_transport republish raw compressed --ros-args \
    -r __node:=republish_features_image \
    -r in:=/features/image \
    -r out/compressed:=/features/image/compressed \
    -p out.jpeg_quality:="${JPEG_QUALITY}" &

# Bridge: compressed images + small telemetry topics only.
# Edit topic_whitelist below when a new topic needs remote viewing —
# just never let a bare Image topic through.
ros2 launch foxglove_bridge foxglove_bridge_launch.xml \
    send_buffer_limit:=1000000 \
    num_threads:=4 \
    topic_whitelist:="['.*/compressed$', \
'/detections', '/vision_info', '/tag_detections', \
'/drogue.*', '/slam/.*', '.*/state$', '.*/diagnostics$', '/tf', '/tf_static', \
'/fmu/out/.*', '/imu/data', \
'/front/camera/camera_info', \
'/rosout', '/foxglove_bridge/sysinfo']"
