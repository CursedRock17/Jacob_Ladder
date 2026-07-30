## Setting Up Real Hardware

Real flights use an **Nvidia Jetson** as the companion computer, connected to a **Pixhawk Cube Orange+** flight controller via USB-to-TTL serial.

### Hardware Requirements

- Nvidia Jetson Orin Nano Super Developer Kit
- Pixhawk Orange Cube Plus
- USB-to-TTL converter
- Optical flow sensor or other pose estimation source : ARKFlow
- Camera : OAK-D Pro

### Setting Up The Companion Computer
**Estimated Setup Time**: 5 minutes

Gather the following supplies:
- Jetson Orin Nano
- Micro SD Card (Not Included w/Jetson) with at least 64 GB of Space
- Access to a secondary Laptop (for flashing)
- USB Mouse 
- USB Keyboard
- Monitor
- DP<-->HDMI/DP Cable
- Access to Inertnet (Ethernet Cable Preferred)
- Power for the Jetson : Either Battery/Power Supply through the ESC, or a Separate Power Supply for Jetson


#### Flashing Jetpack
**Estimated Setup Time**: 1 Hour (*Active*: 15 minutes)

The core operating system of the Jetson Orin Nano is Jetpack which is an offshoot of Ubuntu, this will be based on the [Getting Started](https://developer.nvidia.com/embedded/learn/get-started-jetson-orin-nano-devkit#intro) guide from NVIDIA

To start, follow the [inital setup](https://www.jetson-ai-lab.com/tutorials/initial-setup-jetson-orin-nano/) guide from NVIDIA, you first need to check if the UEFI Firmware is > 36.0.
1) Attach the monitor, keyboard, and mouse to the developer kit
2) Plug in the Jetson to power, spam the ESC key to enter the UEFI menu
3) IN the top left, you should see a version number hopefully > 36.0
4) Download the 10GB Zip file [found on official page](https://developer.nvidia.com/downloads/embedded/L4T/r36_Release_v4.4/jp62-r1-orin-nano-sd-card-image.zip), then use something like [BalenaEtcher](https://etcher.balena.io/) to flash the raw image file to the MicroSD Card
5) Insert the MicroSD card in the slot on the jetson found opposite of the USB ports (it's hard to get out, you may want tweezers).
6) Begin the basic Ubuntu setup with a user/password, etc. You'll want to install Chromium and won't have to worry about connecting to the internet if you have an ethernet cord

#### Adding the repository
**Estimated Setup Time**: 5 minutes

There will be some slight difference between the docker container you have on your laptop and the repository running on the Jetson, but we want them to be as similar as possible
1) Login into the Jetson with the Keyboard/Mouse/Monitor combo
2) Check the top right corner of the Ubuntu 22.04 Menu, ensure you have access to the internet
3) Open up a Terminal Window and a Chromium Window side by side. Navigate to the [Jacob_Ladder Repository](https://github.com/CursedRock17/Jacob_Ladder)
    3a) If you cannot open a Chromium Window, install firefox through the terminal (`sudo apt update && sudo apt upgrade && sudo apt install firefox`)
4) Clone the repository with the desired branch to the root directory:
```bash
cd ~ && git clone https://github.com/CursedRock17/Jacob_Ladder.git -b drogue_collector --recursive
```

#### Adding our packages
**Estimated Setup Time**: 2 Hours (*Active*: 20 Minutes)

We'll need to add in our desired packages to get this drone off the ground and into the air. The simulation portion of Jacob's Ladder is all dockerized, maybe that will need to be the case for drones, depending on what the project needs (TODO: Maybe dockerize setup), until then we've provided some shell scripts that will recreate the docker installation.
1) Enter the Jacob_Ladder repository on the Jetson
1a) Apply the vendored-library patch — `git apply patch.diff` from the repo root.
    It adds `setSkipMessageCompatibilityCheck()` to `ModeExecutorBase` in the
    `px4-ros2-interface-lib` submodule; without it `precision_land`,
    `drogue_flight` and `jacob_manual` will not compile. Any
    `git submodule update` reverts it, so re-apply after one. See the
    [README](../README.md#3-clone-jacobs-ladder) for the check-if-applied one-liner.
2) Navigate to the [Installation Scripts Directory](../installation_scripts) and make sure all of the shell scripts have permissions `chmod +x *.sh`
3) Assert that you have the correct permissions on the Jetson, otherwise you have to run all commands with `sudo`:
    ```bash
    groups
    export USER=some_user
    sudo usermod -aG sudo $USER
    ```
3) Ensure your packages are up to date: ```sudo apt update && sudo apt upgrade```
4) Ensure the packages all have permissions: `sudo chmod +x *.sh` other wise run each command with `sudo`.
4) Install the base tools first: `./base_tools.sh`
5) Install the ROS 2 Humble Ecosystem: `./ros2_humble.sh`
6) Install the OpenCV Packages: `REMOVE_DEFAULT_OPENCV=no ./install_opencv.sh`

    This builds a CUDA-enabled OpenCV from source and is the single longest step
    (60–120 minutes on an Orin NX, ~10 GB of disk). **Do not skip it and do not
    try to use the OpenCV that ships with the ARK Electronics / JetPack image** —
    that one has no CUDA support *and* no contrib modules, so `aruco_tracker`
    will not even compile against it. Full explanation, environment variables,
    and the required `colcon` flags are in the
    [OpenCV on the Jetson](../README.md#opencv-on-the-jetson) section of the
    top-level README.

    Answer `no` if it asks whether to remove the default OpenCV (which is what
    `REMOVE_DEFAULT_OPENCV=no` above does for you) — purging it takes
    `ros-humble-cv-bridge` and the rest of `image_pipeline` with it. If cmake
    fails to detect cuDNN, re-run with `OPENCV_WITH_CUDNN=OFF`; nothing in this
    repo needs it.
7) Install the Drone packages to work with ros2: `./drone_packages.sh`
8) Install the Python packages for our Python environment: `./python_packages.sh`

   Note that `opencv-python` is intentionally **not** installed — the CPU-only
   pip wheel would shadow the CUDA build from step 6. See the README section
   linked above.


#### Additional Packages:
**Estimated Setup Time**: 5 minutes
We'll want some additional packages to make our life a bit easier:

```bash
sudo apt update && sudo apt install \
tmux ros-humble-rosbag2-storage-mcap ros-humble-depthai-ros-v3
```

#### Environment Setup
**Estimated Setup Time**: 5 minutes
It's important to have a smooth development experience on the drone, you should add some lines to your `.bashrc`
```bash
vi ~/.bashrc
```

At the bottom:
```bash
# Source ROS Ecosystem for each terminal
source /opt/ros/humble/setup.bash
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

### Virtual Environment
TODO: If we need a virtual environment, create a high level one in the repo

### Enabling CUDA
If Jetpack doesn't already install it for you, you need to make CUDA is enabled to get the best use out of the onboard GPU, particularly in a PyTorch environment.
Access a python venv in `Jacob_Ladder`, such as within `src/ros2_yolo_image_processing`, source the venv, run python and call torch:

```bash
cd ~/Jacob_Ladder && source install/setup.bash
cd src/ros2_yolo_image_processing
source venv/bin/activate
python3
```

Within the python3 environment:
```python
import torch
torch.cuda.is_available()
```

If it's false, we need to assert that CUDA works, we should be on arm64 (aarch64) within Ubuntu 22.04, we can check with a few important Linux commands:
```bash
uname -m
lspci | grep -i nvidia
hostnamectl
gcc --version
```

We can grab the correct version of CUDA which at the time of writing is 13.2 for arm64, [Native Ubuntu 22.04](https://developer.nvidia.com/cuda-downloads?target_os=Linux&target_arch=arm64-sbsa&Compilation=Native&Distribution=Ubuntu&target_version=22.04&target_type=deb_local)



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
