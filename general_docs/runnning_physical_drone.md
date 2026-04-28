## Running on Physical Drone

Real flights use an **Nvidia Jetson** as the companion computer, connected to a **Pixhawk Cube Orange+** flight controller via USB-to-TTL serial. You SSH into the Jetson from your laptop and run everything in a few terminal sessions.

### Hardware Requirements

- Nvidia Jetson Orin Nano Super Developer Kit
- Pixhawk Orange Cube Plus
- USB-to-TTL converter
- Optical flow sensor or other pose estimation source
- Camera (e.g. OAK-D USB)

### Generating the Firmware
TODO: Make nicer statement:
Essentially, we need to build our own firmware on a desired version (probably v1.16.0) and flash it to the flight controller in order to get desired physical setup. There are some saved versions in the [config](../config/firmware) firmware folder, but you may have to adjust.
[PX4 docs](https://docs.px4.io/main/en/dev_setup/dev_env_linux_ubuntu) provide an official way for setting up firmware
You may also have to clean out the old firmware as well, [through GIT](https://docs.px4.io/main/en/contribute/git_examples)

1) Exit the docker container and find your desired PX4 variant. As of right now I don't know of a way to build in the docker container, for the real world. You may want a brand new variant of PX4 as to not muddle the two. Then navigate to your desired branch with changes you want:
    ```bash
    git clone https://github.com/PX4/PX4-Autopilot.git --recursive
    cd PX4-Autopilot
    git branch && git checkout v1.16.0
    ```

2) Setup the scripting environment on your system. For Ubuntu:
    ```bash
    bash ./Tools/setup/ubuntu.sh
    ```

3) Restart your computer to make sure it loads
4) When you get back, navigate back to the PX4-Autopilot repository and build for you desired firmware ([OrangeCube+ on PX4](https://docs.px4.io/main/en/flight_controller/cubepilot_cube_orangeplus#building-firmware))
    ```bash
    make cubepilot_cubeorangeplus
    ```

    4a) If you get submodule complaints from the submodule repositories:
    ```bash
    git submodule sync recursive
    git submodule update --init --recursive
    ```

### Wiring

Connect the USB-to-TTL converter between the Jetson USB port and the Cube TELEM2 port:

| Jetson | Cube |
|---|---|
| TX | RX |
| RX | TX |
| GND | GND |
| **Do NOT connect 5V** | |

### PX4 Parameters

Set these in QGroundControl, then reboot the flight controller:

```
MAV_1_CONFIG   = TELEM1
MAV_1_RATE     = 57600
UXRCE_DDS_CFG  = TELEM2
UXRCE_DDS_BAUD = 921600
```

**Ensure**: no MAVLink instances are assigned to TELEM2.

There are complete PX4 Parameter files that you can load into QGC by going to:
Menu > Configure > Parameters > Load Params from File
There are examples in the [params folder](../config/params). Like v1.16.0_arkflow.params.

### Flying:
Start with all of the devices turned off, except your Laptop.

#### RC Connection
As reviewed when [setting up](setting_up_real_hardware.md) your drone, we use a RC Handset to communicate with the drone for back up pilot reasons.

QGC is finicky when it comes to connected, so you must do it in the following setps:
1) Open QGroundControl (QGC) from your laptop
2) Turn on RC Handset (Ensure no warning on screen)
3) Connect mRo Radio to Ground Station (your laptop)
4) Provide power to the Flight Controller

When it works, you should get get a message on QGC Screen:
"MAVLink v1 traffic detected on link 'SiK Radio on ttyUSB0 (AutoConnect)'. QGroundControl Daily only supports MAVLink v2. Please ensure your vehicle is configured to use MAVLink 2."

You can double check by cycling through your modes to make sure they show up in QGC

#### Connecting to the Drone for ROS 2
Now that our standard RC is all set up and you've verified the settings, we can connect to the Jetson over the WiFi hotpsot that it sets up.

1) Power on the drone
2) Check for the hotspot to come up (`nmcli device wifi list`)
3) Get access to the password for the hotspot (`export PASS=some_password`)
4) Once the drone hotspot is up connect to it (`nmcli device wifi connect jacob-jetson password $PASS`)
5) Once your on the network, shell into the drone (`ssh jacob@10.42.0.1`)
6) Enter the `Jacob_Ladder` workspace, source the environment (`cd ~/Jacob_Ladder && source install/setup.bash`)

We can now move onto the steps of onboard nodes.

#### 5 SSH Shells:
There are various commands that we need to run in parallel when running our physical drone, that are almost identical to running in simulation. The reference for this code lives in the [launch_scripts section](../launch_scripts/super_real.sh)
That command utilizes a [Terminal Multiplexor (tmux)](https://github.com/tmux/tmux/wiki) session that makes opening the various windows 10x easier, otherwise you'd have to open a new terminal, shell into it and give it new permissions. TMUX also allows you to resume sessions. 
If for some reason you choose not to use tmux, you must remember to enter Jacob_Ladder and source the environment for each and every terminal:

```bash
# Only if running separately: 
cd ~/path/to/Jacob_Ladder && source install/setup.bash
```

**Shell 1 — DDS Agent** (bridges PX4 ↔ ROS 2)
As mentioned, the [uXRCE-DDS](https://docs.px4.io/main/en/middleware/uxrce_dds#uxrce-dds-px4-ros-2-dds-bridge)  serves as middleware, "to allow uORB messages to be published and subscriped on a companion computer (the jetson) as though they were ROS 2 topics"

There are a few requirements to running the Agent:
- The USB port you utilize must be the same as the TTL->Serial Converter onboard the Jetson
- The USB port must have valid port permissions on Linux
- The serial baud rate must match the rate you provide it in the PX4 Parameters.

```bash
MicroXRCEAgent serial --dev /dev/ttyUSB0 -b 921600
```

**Shell 2 — Translation Node** (PX4 message version compatibility)
This translation node allows us to use ROS 2 applications compiled against different versions of the PX4 msgs.

```bash
ros2 run translation_node translation_node_bin
```

**Shell 3 — Camera Driver** (example: OAK-D)
This node is different than the simulation as Gazebo provides Camera sensors underneath the hood, so in the real world we need to run our own camera node. Most larger companies have ROS 2 wrappers for their camera SDK
Luxonis provides [open source documentation](https://docs.luxonis.com/software-v3/depthai/ros/driver/) for their ROS 2 camera drivers that'll work with something like the OAK-D.
We have to do a remap such that the camera provides the correct topics for the `aruco_tracker` remaps. It doesn't necessarily have to be done here, as long as both sides map, but this is useful if you're using more than one camera. You'd have to then remap for each of the cameras so that they each have different [namespaces](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Using-ROS2-Launch-For-Large-Projects.html#namespaces)

```bash
export OG_NAMESPACE=/oak/rgb/
export NEW_NAMESPACE=/front/camera/

ros2 run depthai_ros_driver camera_node
  --ros-args \
  -r ${OG_NAMESPACE}/image_raw:=${NEW_NAMESPACE}/image_raw \
  -r ${OG_NAMESPACE}/camera_info:=${NEW_NAMESPACE}/camera_info
```

**Shell 4 — ArUco Tracker**
ROS 2 wrapper that interacts with OpenCV to provide ArUco tag detection and pose estimation to the tag. The [aruco_tracker repository introductory](../../src/aruco_tracker/README.md) will provide information in much greater detail about naming conventions, parameters, and units of measurement.

```bash
ros2 run aruco_tracker aruco_tracker
```

When running your nodes for the Aruco tracker, make sure you have the correct remappings. The remap
for the physical camera node must be correct to match these remaps 

**Shell 5 — Flight Mode** (pick whichever mode you're flying)
```bash
ros2 launch jacob_manual precision_land.launch.py
```

Make sure this mode comes up in QGC. You can see which External Mode it took over by going to the Mavlink console and typing: `commander status`

The `launch_scripts/` directory has bash scripts (like `real_launch.sh`) that automate opening these shells

#### Using the MAVLink Console
The MAVLink console serves as a mini terminal which exposes various MAVLink messages and features that are easy to utilize

##### Ensuring External Mode Register
In the MAVLink Console run:
```bash
commander status
```
The result of the of External Mode channel should show up along with it's given channel.

##### Reseting Vehicle Position
Much of the onboard code relies on accurate vehicle position which can eventually become inaccurate due to things like [odometry](https://robotics.growbotics.ai/glossary/terms/odometry) drift. 
The vehicle position is initialized when the drone is powered on, [according to PX4](https://docs.px4.io/main/en/msg_docs/VehicleLocalPosition) "The coordinate system origin is the vehicle position at the time when the EKF2-module was started."

To check the current state of the vehicle, you can see ROS 2 side or the PX4 (uORB) side with either:
- In a sourced terminal : `ros2 topic echo /fmu/out/vehicle_local_position`
- In a MAVLink Console  : `listener vehicle_local_position`

To restart this position (for example, you picked up the drone to put it in a better spot), you must simply restart the EKF2-module, which doesn't require a full reboot.
In a MAVLink Console:
```bash
ekf2 stop
ekf2 start
```
