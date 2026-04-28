# Lucas Issues
Any issues that Lucas encountered while installing.

3) Had to locally build px4_sitl when importing custom worlds. Otherwise you just copy the world over
Drift on Takeoff, but holds position. Do we need to 0 position of FLow sensors

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
-v /tmp/.X11-unix:/tmp/.X11-unix:ro -e DISPLAY=:0 --network host --name=capstone_drone \
lucaswendland/jacob_ladder:latest bash
```

## Changes to PX4

### More issues:
- TODO: Don't run most of the code on the docker when running the physical drone. Run as many containers
possible in the ssh'd terminals.

### Adding the ARKFlow
New:
    "Need to adjust the position of the Optical Flow Sensor Relative to the CG"
    EKF2_OF_POS_X +0.025m
    EKF2_OF_POS_Y -0.010m
    EKF2_OF_POS_Z +0.070m
    "All CAN Protocol to grab sensors on CAN ports connected to cube and fuse in EKF2"
    "Where EKF2 is the Extended Kalman Filter and allows us to stabilize flight"
    UAVCAN_ENABLE: Sensors Automatic Config
    --reboot-
    "Enable Optical Flow Input to EKF2, we DONT want to use GPS"
    EKF2_OF_CTRL: Enabled (1)
    EKF2_GPS_CTRL: Disabled (-)
    UAVCAN_SUB_FLOW: Enabled (1)
    UAVCAN_SUB_RNG: Enabled (1)
    UAVCAN_SUB_GPS: Disabled(0)
    UAVCAN_SUB_GPS_R: Disabled(0)
    EKF2_RNG_CTRL: Enabled (1)
    "Check Quality and max/min heights for the ArkFlow Sensors"
    EKF2_RNG_A_HMAX: 10.
    EKF2_RNG_QLTY_T: 0.2.
    UAVCAN_RNG_MIN: 0.08.
    UAVCAN_RNG_MAX: 30.
    SENS_FLOW_MINHGT: 0.08.
    SENS_FLOW_MAXHGT: 25.
    SENS_FLOW_MAXR: 7.4
    "MPC: Position Control, so MPC_XY_P is the gain of XY axis controller for position hold"
    MPC_XY_P: 0.5
    "We want to get up to speed faster into the air on Takeoff"
    MPC_ACC_UP_MAX: 5.0
    MPC_TKO_SPEED: 2.0
    "If not specified, takeoff to half a meter instead of 2.5 meters"
    MIS_TAKEOFF_ALT: 0.5m


### Issues + Solutions Seen
- Git + Compiler Permissions issues in PX4 -> Make sure you both provide permissions to the docker container
and remove any possible parasitic build traces :
```bash
# Inside Jacob_Ladder
sudo rm -rf build log install
# Inside PX4
sudo rm -rf build
sudo rm -rf /tmp/px4*
```
- Losing Access to Position Hold due to lost Optical Flow in Flight -> Apply PR Fix [#26960](https://github.com/PX4/PX4-Autopilot/pull/26960)
