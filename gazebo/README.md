# Gazebo Asset Installation for PX4

This directory holds custom Gazebo models (`./models`) and a world (`./worlds`)
that extend the default PX4 SITL environment. Follow the steps below to copy
these assets into your local `PX4-Autopilot` checkout so they are available to
the simulator.

## Prerequisites

- A local clone of `PX4-Autopilot`
- Write access to `PX4-Autopilot/Tools/simulation/gz`

## Adding Custom Models/Worlds
Before you start, set a shell environment variable that points to the ABSOLUTE 
location of your PX4 installation:

```bash
export PX4_DIR=~/jacob_ladder_ws/src/PX4-Autopilot
```

### Worlds
1. Copy the world files from wherever you want (i.e `~/jacob_ladder_ws/src/Jacob_Ladder/gazebo/worlds`) 
into the simulation directory in PX4:
```shell
cp -r ~/jacob_ladder_ws/src/Jacob_Ladder/gazebo/worlds/ "$PX4_DIR/Tools/simulation/gz/"
```

*Note:*Depending on the version you may have to navigate to the `gz_bridge` directory to add any worlds you created, to the 
gz_worlds directory in the `CMakeLists.txt`:
```shell
cd "$PX4_DIR/src/modules/simulation/gz_bridge/CMakeLists.txt"
```
You're looking for "set(gz worlds)", when you find it, you can simply add any world names
to the list which is separated by spaces.

### Models
1. Adding a model must be added to the gz/simulation/models directory and you must create a frame
2. Copy the model files from wherever you want (i.e `~/jacob_ladder_ws/src/Jacob_Ladder/gazebo/models`) 
into the simulation directory in PX4:
```shell
cp -r ~/jacob_ladder_ws/src/Jacob_Ladder/gazebo/models/ "$PX4_DIR/Tools/simulation/gz/"
```
3. Now, we must any imported models that are vehicles to the `airframes` list.
4. Enter the `airframes` directory from within the `ROMFS`:
K```shell
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

## Using the assets

With any custom worlds and models you should now be able to build px4. This can be done from the
docker image with the format to load any custom vehicle and world together as 
```bash
cd "$PX4_DIR"
make px4_sitl ${vehicle_name}_${world_name}
# For example:
make px4_sitl gz_x500_aruco
```
You must rerun the copy steps to insert the custom models into the docker anytime the
`.sdf` files change.

You may run into errors with the building. As always check spelling and make sure your 
directory and CMakeLists files are correct. You may also have to remove the `PX4-Autopilot/build`
directory or the `/tmp/px4-sock*` directory within the docker image.

Re-run the copy steps whenever the assets in this repository change.

### Setting up PX4 and QGC
1) Make sure to enable joystick in general settings, fly view, virtual joystick > enabled
