## Setting Up Real Hardware

Real flights use an **Nvidia Jetson** as the companion computer, connected to a **Pixhawk Cube Orange+** flight controller via USB-to-TTL serial.

### Hardware Requirements

- Nvidia Jetson Orin Nano Super Developer Kit
- Pixhawk Orange Cube Plus
- USB-to-TTL converter
- Optical flow sensor or other pose estimation source : ARKFlow
- Camera : OAK-D Pro

### Setting Up The Companion Computer

Gather the following supplies:
- Jetson Orin Nano
- USB Mouse 
- USB Keyboard
- Monitor
- HDMI<-->HDMI Cable
- Access to Inertnet (Ethernet Cable Preferred)
- Power for the Jetson : Either Battery/Power Supply through the ESC, or a Separate Power Supply for Jetson


#### Adding the repository

TODO: When you set up the new computer, add in all this information
There will be some slight difference between the docker container you have on your laptop and the repository running on the Jetson, but we want them to be as similar as possible
1) Login into the Jetson with the Keyboard/Mouse/Monitor combo
2) Check the top right corner of the Ubuntu 22.04 Menu (TODO: Maybe need a guide to set that up), ensure you have access to the internet
3) Open up a Terminal Window and a Chromium Window side by side. Navigate to the [Jacob_Ladder Repository](https://github.com/CursedRock17/Jacob_Ladder)


#### Additional Packages:
We'll want some additional packages to make our life a bit easier:

```bash
sudo apt update && sudo apt install \
tmux ros-humble-rosbag2-storage-mcap ros-humble-depthai-ros-v3
```

### Generating the Flight Controller Firmware
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



### Communication Flow

#### RC Handset
In order to have a back up pilot, we utilize QGroundControl (QGC) along with a RC Handset and Radio, QGC uses a certain [communication](https://docs.qgroundcontrol.com/master/en/qgc-dev-guide/communication_flow.html) flow, but essentially needs the radio to function.
It communicates via MAVLink messages to the drone, the LinkManager is a UDP port that detects a known SiK Radio


### Serial Port Setup
PX4 has a [great guide](https://docs.px4.io/main/en/companion_computer/pixhawk_companion#serial-port-setup) for Serial Port Setup with the Pixhawk, essentially we're expected to communicate with the Pixhawk controller on port `TELEM2`, which by default uses MAVLink, so we'll disable and use the XRCE DDS Client
