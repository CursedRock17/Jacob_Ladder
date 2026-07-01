# Jacob's Ladder
Welcome, it's recommended that you start **HERE**, as this main README will allow you to navigate this large and complex project.
Each of the subsections have their own respective READMEs and docs for full explanations.

Jacob's Ladder is a modular, system-agnostic UAV command-and-control (C2) framework built on **ROS 2 Humble** and **PX4**. It lets you write autonomous drone missions entirely in ROS 2 — the same code runs in Gazebo simulation on your laptop and on real hardware in the field, with no rewrites needed.

The framework uses PX4 **external modes** instead of the traditional offboard API. External modes register directly with PX4 through the companion computer, appearing as selectable flight modes in QGroundControl alongside the built-in ones (Stabilized, Position, Mission, etc.). If the companion computer ever stops communicating, PX4 automatically failsafes — so the system is safe by design.

## Jacob's Ladder Structure
Within the project you'll find several pacakges, this README serves as a higher level overview of all the components involved in the project.
For a deeper view, there are embedded markdown files which break down specific rationale, processes, and objectives.

START HERE:

| Package | Objective |
|---|---|
| [Micro-XRCE-DDS-Agent](src/Micro-XRCE-DDS-Agent) | Should NOT be Altered : Use given branch -> Communication Agent from Drone to Host Computer |
| [aruco_tracker](src/aruco_tracker) | ROS 2 Wrapper for OpenCV detection of an ArUco Marker |
| [drogue_flight](src/drogue_flight) | TODO: In-Progress Porting for flying to a detected KC-130 drogue |
| [jacob_manual](src/jacob_manual)  | ROS 2 External Modes that Require Manual Control to get in the air, but fly autonomous missions after |
| [precision_land](src/precision_land)  | ROS 2 External Modes that fly autonomous missions for object detection, trajectory planning, and landing
| [oak_d_visual_odometry](src/oak_d_visual_odometry)  | ROS 2 nodes for OAK-D visual odometry using NVIDIA cuVSLAM, with optional PX4 `VehicleOdometry` output for flight. |
| [px4-ros2-interface-lib](src/px4-ros2-interface-lib) | Should NOT be Altered : Use Given Branch -> ROS 2 <-> PX4 Bridge, allows us to create External Modes |
| [px4_msgs](src/px4_msgs) | Should NOT be Altered : Use Given Branch -> PX4 Messages for ROS 2 Communication |
| [px4_msgs_old](src/px4_msgs_old) | Should NOT be Altered : Use Given Branch -> More PX4 Messages for ROS 2 Communication |
| [translation_node](src/translation_node) | Should NOT be Altered : Use Given Branch -> Introduced in PX4 v1.15.0, translates old px4 messages necessary for past versions to new px4 messages |
| [vision_opencv](src/vision_opencv) | Should NOT be Altered : Use Given Branch -> OpenCV STD library |

## What Can It Do?

| Capability | Description |
|---|---|
| **Precision Landing** | Detect an ArUco marker with a downward camera, approach it, and autonomously land on it — even on a moving platform |
| **Front-Camera Approach** | Fly toward a target detected by a forward-facing camera using PID control |
| **Combined Approach + Land** | Use a front camera to approach, then seamlessly hand off to a downward camera for precision landing |
| **Drogue Collection** | Detect a drogue target via YOLO, plan a smooth S-curve trajectory, and fly to it |
| **Takeoff & Hold** | Simple building-block modes for taking off, holding altitude, and landing |

## If You're New to ROS 2

A few concepts that will help you navigate this project:

- **Node** — A single process that does one job (e.g. track ArUco markers, send flight commands). Nodes communicate by publishing and subscribing to **topics**.
- **Topic** — A named data channel. One node publishes messages to a topic; other nodes subscribe to receive them. For example, the ArUco tracker publishes marker positions to a topic that the precision landing mode subscribes to.
- **Package** — A folder containing related code, a `CMakeLists.txt` (or `setup.py` for Python), and a `package.xml` manifest. Each folder under `src/` is a package.
- **Launch file** — A Python script that starts multiple nodes at once with the right parameters. You'll find these in each package's `launch/` folder.
- **Workspace** — The overall project directory. You build everything with `colcon build` and source `install/setup.bash` to make your nodes available.

### How the Pieces Fit Together

```
┌───────────────────────────────────────────────────────────┐
│                     QGroundControl                        │
│              (select External Mode here)                  │
└───────────────────┬───────────────────────────────────────┘
                    │ MAVLink
┌───────────────────▼───────────────────────────────────────┐
│                   PX4 Autopilot                           │
│            (flight controller firmware)                   │
└───────────────────┬───────────────────────────────────────┘
                    │ uXRCE-DDS (serial or UDP)
┌───────────────────▼───────────────────────────────────────┐
│              Micro XRCE-DDS Agent                         │
│         (bridges PX4 topics ↔ ROS 2 topics)               │
└───────────────────┬───────────────────────────────────────┘
                    │ ROS 2 topics
       ┌────────────┼────────────────────┐
       │            │                    │
       ▼            ▼                    ▼
┌─────────────┐ ┌──────────┐ ┌────────────────────┐
│ Translation │ │  ArUco   │ │   External Mode    │
│    Node     │ │ Tracker  │ │ (e.g. PrecisionLand│
│             │ │          │ │  TakeoffHold, etc.) │
│ Converts    │ │ Detects  │ │                    │
│ PX4 msg     │ │ markers  │ │ Reads sensor data, │
│ versions    │ │ from     │ │ runs state machine,│
│             │ │ camera   │ │ sends setpoints    │
└─────────────┘ └──────────┘ └────────────────────┘
```

## Prerequisites

- Ubuntu Linux (22.04 recommended)
- Docker Engine
- PX4 Autopilot (v1.16.0 recommended)
- QGroundControl Daily Build

## Quick Start

### 1. Clone PX4

Create your workspace on your system and add in PX4-Autopilot
```bash
mkdir -p ~/jacob_ladder_ws/src && cd ~/jacob_ladder_ws/src
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
```

The v1.16.0 tag is **highly** recommended. The included translation node provides compatibility with most PX4 versions from 1.16 onward, but some versions may lack the Gazebo worlds and models needed for simulation — refer to PX4's official documentation if that's the case.
```bash
cd PX4-Autopilot 
git status
git checkout v1.16.0
```

### 2. Install QGroundControl

Download the daily build: https://docs.qgroundcontrol.com/master/en/qgc-user-guide/releases/daily_builds.html

### 3. Clone Jacob's Ladder

```bash
git clone https://github.com/CursedRock17/Jacob_Ladder.git
cd Jacob_Ladder
git checkout drogue_collector
git submodule update --init --recursive
```

### 4. Set Up Docker

The `docker/` directory contains a layered Dockerfile stack built on `px4io/px4-dev-base-jammy`. 
Pre-built images are available on [Docker Hub](https://hub.docker.com/repositories/jacobsafeer), so you can skip manual dependency installation entirely.

Install Docker Engine: https://docs.docker.com/engine/install/ubuntu/

Start the container:

*If on Linux:*
First make sure your user has docker permissions, a quick check:
```bash
sudo groupadd docker
sudo usermod -aG docker $USER
newgrp docker
# Make sure to refresh computer for groups to kick into effect
docker run hello-world
```

Then open up the container
```bash
xhost +
export CONTAINER_NAME=your_container_name

docker run -it --privileged \
  -u $(id -u):$(id -g) \ 
  -v ~/src/PX4-Autopilot:/src/PX4-Autopilot/:rw \
  -v ~/path/to/Jacob_Ladder:/path/to/Jacob_Ladder/:rw \
  -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
  -e DISPLAY=:0 \
  --network host \
  --name=$CONTAINER_NAME \
  jacobsafeer/px4-dev-harmonic-jammy-humble-opencv-rqt:latest bash
```

*If on Windows*:
```bash
export CONTAINER_NAME=your_container_name
docker run -it --privileged \
    --user root \
    -v "C:\path\PX4":/src/PX4/:rw \
    -v "C:\path\Jacob_Ladder":/src/Jacob_Ladder/:rw \
    -e DISPLAY=:0 \
    --network host \
    --name $CONTAINER_NAME jacobsafeer/px4-dev-harmonic-jammy-humble-opencv-rqt:latest bash
```

**Not** tested on MacOS
**Maybe** If you running a windowing system other than X11 or notably running simulation with a GPU
you may have to make `-e` and `-v` alterations to the the windowing and display sections to allow it run.

Replace `/path/to` with your actual path to Jacob_Ladder, and `$CONTAINER_NAME` with whatever you'd like.

If you're not already in the container, start and enter
```bash
docker start $CONTAINER_NAME
docker exec -it $CONTAINER_NAME bash
```
When inside the container, build the workspace:

```bash
cd /path/to/Jacob_Ladder
source /opt/ros/humble/setup.bash
colcon build
```

### 5. Run the Example Simulation

Customize a launch script using the template in the existing `launch_scripts/` directory. Each script opens multiple terminal tabs — one for PX4 SITL, one for the DDS agent, one for each ROS node — so everything starts together.

```bash
# On the host machine
./QGroundControl.AppImage           # Start QGC
docker start $CONTAINER_NAME        # Start the container
./launch_scripts/precision_land.sh  # Run the launch script
```

Select the precision landing mode from the QGroundControl dropdown:

![](Precision.png)

### Other Flight Mode Scripts

`precision_land.sh` is just one of several ready-to-run scripts in `launch_scripts/`. Each one starts PX4 SITL in a specific Gazebo world and launches the ROS nodes needed for that flight mode. Pick whichever matches what you want to test.

| Script | Flight Mode | Gazebo World | Purpose |
|---|---|---|---|
| `precision_land.sh` | `precision_land/front_to_precision_land.launch.py` | `gz_x500_dual_cam_aruco_dual_ids` | Front-camera approach, then downward precision landing on an ArUco tag |
| `approach_aruco.sh` | `precision_land/front_approach.launch.py` | `gz_x500_dual_cam_aruco_dual_ids` | Front-camera ArUco tag approach only (no landing) |
| `jacob_manual.sh` | `jacob_manual/front_approach.launch.py` | `gz_x500_dual_cam_aruco_dual_ids` | Same as `approach_aruco.sh`, but runs the newer `jacob_manual` package version |
| `takeoff_hover.sh` | `precision_land/takeoff_hold.launch.py` | `gz_x500_dual_cam` | Takeoff and hold indefinitely — good for hover tuning |
| `takeoff_hover_land.sh` | `precision_land/takeoff_land.launch.py` | `gz_x500_dual_cam` | Takeoff, hold briefly, then land — simplest end-to-end mode |
| `offboard_blank.sh` | `precision_land/blank_mode.launch.py` | `gz_x500_dual_cam` | Empty external-mode template — use as a starting point when writing a new mode |
| `moving_launch.sh` | `precision_land/track_follow.launch.py` | Custom moving platform world | Track and land on a moving ArUco platform (uses the `v1_16_tracker` aruco launch) |
| `real_launch.sh` | *(edit before use)* | Real hardware | DDS agent + RViz shell for Jetson/Pixhawk flight testing — all mode commands are commented out so you can uncomment the one you want |
| `fake_drogue.sh` | *(no flight mode)* | Real hardware | Drogue camera feed sanity check — runs the DDS agent and `ros2 topic hz` on the camera topic |

Every script follows the same shape: the first tab is PX4 SITL (or omitted for real hardware), then the DDS agent, the translation node, any trackers, and the mode itself. If a script doesn't match your container name, workspace path, or PX4 version, open it and edit the `container_name` and path variables at the top — they're the same in every script.

> **Note:** Most scripts use the absolute path `/home/user/jacob_ladder_ws/...` hardcoded in them. Update these paths to match your own machine before running.

### Launch Script Options

| Tab | Options |
|---|---|
| **precision_land** | `precision_land.launch.py` (default ArUco world) or `track_follow.launch.py` (moving platform world) |
| **aruco_tracker** | `aruco_tracker.launch.py` (PX4 1.15.x), `v1_16_tracker.launch.py` (PX4 1.16.x default), or `moving_aruco.launch.py` (PX4 1.16.x moving platform) |


## Features in the repository
For more detailed tutorials checkout the [`general_docs`](general_docs) section of the project
which contains information on how to:
  - [Playback Data](general_docs/reviewing_flight_data.md) after the flights
  - [Running on](general_docs/real_hardware.md) real hardware
  - **[`general_docs/external_modes.md`](general_docs/external_modes.md)** — Building your own modes
  - More to Come
 
 That way you have access to more in-depth tutorials.

Also, all of the code in the repository can be totalled expanded.

### Flight Parameters
Different configurations of physical drones may require different PX4 Parameter configurations
any and all tested forms of parameters are labelled in the [`config/params`](config/params) directory.

### Before You Fly

1. **Manual flight first** — Get the drone flying reliably in Stabilized and Position modes before running any autonomy
2. **Networked sim test** — Run PX4 SITL on your PC and the ROS nodes on the Jetson over the same network to verify communication
3. **Props-off test** — Run your flight mode with propellers removed and verify PX4 receives setpoints via QGroundControl
4. **Verify topics** — Confirm PX4 topics are visible: `ros2 topic list` should show `/fmu/out/vehicle_odometry`, `/fmu/out/vehicle_status`, etc.

## Additional Resources

- [Linux Cheat Sheet](https://www.geeksforgeeks.org/linux-commands-cheat-sheet/)
- [ROS 2 Cheat Sheet](https://www.theconstruct.ai/wp-content/uploads/2021/10/ROS2-Command-Cheat-Sheets-updated.pdf)
- [CMake Basics](https://nu-msr.github.io/navigation_site/lectures/cmake_basics.html)

## Questions

Email: lwendlan@umd.edu

## License

See [LICENSE](LICENSE) for details.
