## Recording & Replaying Data

Every launch file in this project automatically records a [rosbag](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Recording-And-Playing-Back-Data/Recording-And-Playing-Back-Data.html) — a timestamped log of every ROS 2 topic during the flight. Bags are saved to the `flight_rosbags/` directory with a timestamp in the filename (e.g. `flight_bag_2026-04-10_14-30-00`).

This is useful for:

- **Debugging** — Replay a flight and watch what happened in RViz without needing the drone
- **Testing perception** — Run the ArUco tracker or YOLO detector against recorded camera images
- **Tuning** — Examine the exact sensor data and setpoints from a flight to adjust parameters

### What gets recorded

By default, the launch files run `ros2 bag record -a`, which captures **all topics** — PX4 telemetry, camera images, target poses, setpoints, everything. The bags use the [MCAP](https://mcap.dev/) storage format.

If you running `ros2 bag record` on the drone, you may have to move files over ssh, which can take a minute especially since the bags will often be 5+ GB in size (so don't let them fill up on-drone storage). Using `rsync` will process them faster:
```bash
rsync -avz --progress jacob@10.42.0.1:~/Jacob_Ladder/flight_rosbags/ ~/new_location/some_flights/
```

### Inspecting a bag (Foxglove) - Preferred
First you will have to install foxglove (TODO: Add foxglove to lucas' docker)
- Install Foxglove support (to docker)
 - Foxglove (for visualization) : https://docs.foxglove.dev/docs/data/primary-sites/installation
 - Foxglove-ros-bridge: https://docs.foxglove.dev/docs/visualization/ros-foxglove-bridge

[Foxglove](https://docs.foxglove.dev/docs/getting-started/frameworks/px4) has the ability to playback both [PX4 uLogs](https://docs.px4.io/main/en/dev_log/ulog_file_format) and [ROSBAGs](https://docs.foxglove.dev/docs/getting-started/frameworks/ros2?modality=recorded) providing extreme flexibility when reviewing data.
They also provide the ability to monitor all of our data during flight in an easy manner. I leave an example work flow in the [config section](../config/chimera_drone_foxglove.json) section of the project

### Watching live flight data over WiFi (Foxglove)

For live monitoring, run `launch_scripts/foxglove_wifi.sh` on the drone instead
of a bare `foxglove_bridge` launch (super_real.sh window 7 already does), then
connect Foxglove Studio on the laptop to `ws://<jetson-wifi-ip>:8765`.

The script exists because raw images don't fit the radio: one 640×400 BGR8
frame is 768 KB, so a 20 Hz stream needs ~123 Mbps against the ~15–30 Mbps a
hotspot really delivers — that mismatch is what turned 20 Hz tracking into a
3 Hz slideshow. The script fixes it by:

- JPEG-compressing the image topics on the Jetson via `image_transport
  republish` (~25–40 KB/frame → ~5 Mbps at 20 Hz). Quality defaults to 60:
  `JPEG_QUALITY=40 ./launch_scripts/foxglove_wifi.sh` if the link is bad.
- Whitelisting the bridge so raw `Image` topics never reach the websocket —
  **point image panels at the `/compressed` topics**. Edit the whitelist in
  the script when a new topic needs remote viewing.
- Capping the bridge send buffer at 1 MB so a congested link drops stale
  frames instead of queueing seconds of latency.

If the compressed stream still stutters, lower `JPEG_QUALITY` first —
bandwidth is always suspect #1 on this link (`ros2 topic bw` to confirm).

### Inspecting a bag (ROS 2 CLI)

We can use the ROS 2 CLI to also inspect our bags:
```bash
# See what topics were recorded and how many messages each has
ros2 bag info flight_rosbags/flight_bag_2026-04-10_14-30-00

# Echo a specific topic from the bag
ros2 bag play flight_rosbags/flight_bag_2026-04-10_14-30-00 --topics /target_pose
```

### Inspecting ULog Files (PX4)
The native form of debugging that PX4 provides is in the form of a [uLog File](https://docs.px4.io/main/en/dev_log/ulog_file_format) which contains thousands of pieces of information. Very valuable information, but lots of it.
To handle this, I enjoy using [Plotjuggler](https://github.com/PlotJuggler/PlotJuggler) to debug this data, but PX4 also provides a Web App.

There are many ways to visualize the data, but a sample with one of our various configs:

```bash
plotjuggler --datafile some_path/to/our_log.ulog --layout config/plotjuggler_baro_pressure.xml
```

### QoS overrides

PX4 topics come through uXRCE-DDS with non-standard QoS settings that can cause errors in the recorder. The file `config/rosbag_qos_override.yaml` provides overrides that fix this — some launch files apply it automatically.
