## Setting Up Real Hardware

Real flights use an **Nvidia Jetson** as the companion computer, connected to a **Pixhawk Cube Orange+** flight controller via USB-to-TTL serial.

### Hardware Requirements

- Nvidia Jetson Orin Nano Super Developer Kit
- Pixhawk Orange Cube Plus
- USB-to-TTL converter
- Optical flow sensor or other pose estimation source
- Camera (e.g. OAK-D USB)

### Setting Up

Gather the following supplies:
- Jetson Orin Nano
- USB Mouse 
- USB Keyboard
- Monitor
- HDMI<-->HDMI Cable
- Access to Inertnet (Ethernet Cable Preferred)
- Power for the Jetson : Either Battery/Power Supply through the ESC, or a Separate Power Supply for Jetson

#### Adding the repository

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
