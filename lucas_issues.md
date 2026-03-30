# Lucas Issues
Any issues that Lucas encountered while installing.

1) xhost + doesn't seem to fix the permissions issue, I add to create a new root user group on
my Linux system, add the docker permissions there and join the group:
```shell
sudo groupadd docker
sudo usermod -aG docker $USER
newgrp docker
docker run hello-world

```
3) Make sure to refresh computer for groups to kick into effect
2) Prerequites in 2 different sections with different ROS 2 and Ubuntu Reqs. Fine during install
3) Had to locally build px4_sitl when importing custom worlds. Otherwise you just copy the world over
4) Add `--recursive` to end of PX4 clone to load gz assets

My Command:
```shell
docker run -it --privileged --env=LOCAL_USER_ID="$(id -u)" \
-v /home/cursedrock17/Documents/Electrical/Matrix_Lab/jacob_drone_ws/src/PX4-Autopilot:/home/cursedrock17/Documents/Electrical/Matrix_Lab/jacob_drone_ws/src/PX4-Autopilot/:rw \
-v /home/cursedrock17/Documents/Electrical/Matrix_Lab/jacob_drone_ws/src/Jacob_Ladder:/home/cursedrock17/Documents/Electrical/Matrix_Lab/jacob_drone_ws/src/Jacob_Ladder/:rw \
-v /tmp/.X11-unix:/tmp/.X11-unix:ro -e DISPLAY=:0 --network host --name=jacob_drone \
jacobsafeer/px4-dev-harmonic-jammy-humble-opencv-rqt:latest bash
```

```shell
docker run -it --privileged --env=LOCAL_USER_ID="$(id -u)" \
-v /home/cursedrock17/Documents/Electrical/Matrix_Lab/jacob_drone_ws/src/PX4-Autopilot:/home/cursedrock17/Documents/Electrical/Matrix_Lab/jacob_drone_ws/src/PX4-Autopilot/:rw \
-v /home/cursedrock17/Documents/Electrical/Matrix_Lab/jacob_drone_ws/src/Jacob_Ladder:/home/cursedrock17/Documents/Electrical/Matrix_Lab/jacob_drone_ws/src/Jacob_Ladder/:rw \
-v /tmp/.X11-unix:/tmp/.X11-unix:ro -e DISPLAY=:0 --network host --name=jacob_drone \
lucaswendland/jacob_ladder:latest bash
```

## Adding Custom Models/Worlds
Before you start, set a shell environment variable that points to the ABSOLUTE 
location of your PX4 installation:
```shell
export PX4_DIR=~/ws/src/PX4-Autopilot
```
### Worlds
1. Copy the world files from wherever you want (i.e `~/ws/src/Jacob_Ladder/gazebo/worlds`) 
into the simulation directory in PX4:
```shell
cp -r ~/ws/src/Jacob_Ladder/gazebo/worlds/ "$PX4_DIR/Tools/simulation/gz/"
```
2. Then navigate to the `gz_bridge` directory to add any worlds you created, to the 
gz_worlds directory in the `CMakeLists.txt`:
```shell
cd "$PX4_DIR/src/modules/simulation/gz_bridge/CMakeLists.txt"
```
You're looking for "set(gz worlds)", when you find it, you can simply add any world names
to the list which is separated by spaces.

### Models
1. Adding a model must be added to the gz/simulation/models directory and you must create a frame
2. Copy the model files from wherever you want (i.e `~/ws/src/Jacob_Ladder/gazebo/models`) 
into the simulation directory in PX4:
```shell
cp -r ~/ws/src/Jacob_Ladder/gazebo/models/ "$PX4_DIR/Tools/simulation/gz/"
```
3. Now, we must any imported models that are vehicles to the `airframes` list.
4. Enter the `airframes` directory from within the `ROMFS`:
```shell
cd "$PX4_DIR/ROMFS/px4fmu_common/init.d-posix/airframes"
```
5. You must then create a new vehicle configuration file, it has the naming convention of
an unused 5 digit number (between 22,000 and 22999) along with the vehicle name. Inside the file,
you must add the PX4 `.sdf` model of the vehicle along with any parameters PX4 may need ahead of time.
For an example we're going to go with the provided dual cam variant of the x500 drone:
```shell
touch "$PX4_DIR/ROMFS/px4fmu_common/init.d-posix/airframes/22001_gz_x500_dual_cam"
# In your favorite code editor:
code "$PX4_DIR/ROMFS/px4fmu_common/init.d-posix/airframes/22001_gz_x500_dual_cam"
```
The file will be created as such:
```CMakeLists
#!/bin/sh
#
# @name Gazebo x500 dual cam cam
#
# @type Quadrotor
#

PX4_SIM_MODEL=${PX4_SIM_MODEL:=x500_dual_cam}

. ${R}etc/init.d-posix/airframes/4001_gz_x500
```
6. We must then add this model to the corresponding `CMakeLists.txt` in the `airframes`
directory. It's simply adding the name of the vehicle file in the space separated
`px4_add_romfs_files` command. Of course, add all models in numerical ordering.

### Building
With any custom worlds and models you should now be able to build px4. This can be done from the
docker image with the format to load any custom vehicle and world together as 
```shell
cd "$PX4_DIR"
make px4_sitl ${vehicle_name}_${world_name}
# For example:
# make px4_sitl gz_x500_aruco
```
You must rerun the copy steps to insert the custom models into the docker anytime the
`.sdf` files change.

You may run into errors with the building. As always check spelling and make sure your 
directory and CMakeLists files are correct. You may also have to remove the `PX4-Autopilot/build`
directory or the `/tmp/px4-sock*` directory within the docker image.

### Setting up PX4
4) Make sure you v1.16.0 Release Tag for PX4-Autopilot
5) Make sure to enable joystick in general settings, fly view, virtual joystick > enabled
6) Make sure PrecisinoLandingCustom is available

Ask about wiring config
Ask about radio config

Windows Run:

docker run -it --privileged --user root -v "C:\path\PX4":/src/PX4/:rw -v "C:\path\Jacob_Ladder":/src/Jacob_Ladder/:rw -e DISPLAY=:0 --network host --name 4some_name jacobsafeer/px4-some-path
## Changes to Code:
- Created a dualcam launch file which boxes things up a bit nicer
- Install Foxglove support (to docker)
 - Foxglove (for visualization) : https://docs.foxglove.dev/docs/data/primary-sites/installation
 - Foxglove-ros-bridge: https://docs.foxglove.dev/docs/visualization/ros-foxglove-bridge


## Changes to PX4

### Adding the MTF-02P Op Flow
Old:
  UAVCAN_ENABLE: Disabled
  UAVCAN_SUB_FLOW: Disabled
  UAVCAN_SUB_RNG: Disabled
  COM_FLTMODE1: Stabilized
  COM_FLTMODE4: Position
  COM_FLTMODE6: Offboard

New:
    MAV_1_CONFIG: TELEM 2
    --reboot--
    MAV_1_MODE: Normal
    SER_TEL2_BAUD 115200 8N1
    EKF2_OF_CTRL Enabled
    EKF2_RNG_CTRL Enabled
    EKF2_RNG_A_HMAX 6.0m
    EKF2_MIN_RNG 0.08m
    EKF2_HGT_REF Range sensor
    EKF2_OF_POS_X 0.0m
    EKF2_OF_POS_Y 0.0m
    EKF2_OF_POS_Z 0.070m
    --reboot--
    SENS_FLOW_ROT No rotation
    COM_FLTMODE1: Stabilized
    COM_FLTMODE4: Altitude
    COM_FLTMODE6: Position
    SDLOG_PROFILE: 19->3
    MAV_1_FORWARD: 1
    UAVCAN_ENABLE: Disabled
    UAVCAN_SUB_GPS: Disabled
    UAVCAN_SUB_GPS_R: Disabled
    UAVCAN_SUB_MAG: Disabled

### Adding the ARKFlow
New:
    EKF2_OF_POS_X +0.025m
    EKF2_OF_POS_Y -0.010m
    EKF2_OF_POS_Z +0.070m
    UAVCAN_ENABLE: Sensors Automatic Config
    --reboot-
    EKF2_OF_CTRL: Enabled (1)
    EKF2_GPS_CTRL: Disabled (-)
    UAVCAN_SUB_FLOW: Enabled (1)
    UAVCAN_SUB_RNG: Enabled (1)
    UAVCAN_SUB_GPS: Disabled(0)
    UAVCAN_SUB_GPS_R: Disabled(0)
    EKF2_RNG_CTRL: Enabled (1)
    EKF2_RNG_A_HMAX: 10.
    EKF2_RNG_QLTY_T: 0.2.
    UAVCAN_RNG_MIN: 0.08.
    UAVCAN_RNG_MAX: 30.
    SENS_FLOW_MINHGT: 0.08.
    SENS_FLOW_MAXHGT: 25.
    SENS_FLOW_MAXR: 7.4
    EKF2_OF_POS_X, EKF2_OF_POS_Y and EKF2_OF_POS_Z can be set to account for the offset of the Ark Flow from the vehicle centre of gravity.
    MPC_XY_P: 0.5


### Scripts to Run
Blank Offboard Mode
Take off & Hover
Take Off, Hover Land
Take Off (to known Height), Locate Tag, Go to Tag Hover in Front of Tag
Take Off (to known Height), Locate Tag, Go to Tag Hover in Front of Tag, Land
