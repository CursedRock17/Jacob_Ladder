# Flights: fly a drone from a plain Python script

A flight is one short script, written like a Crazyflie `MotionCommander` program.
It runs on the companion computer, talks to PX4 through the same `jl_mission`
mode as a mission file, and so keeps its safety limits: no setpoint for 0.5 s
holds, 5 s lands, and more than 1 m/s is clamped. Switching modes in QGC stops
the script, and the pilot has control.

```python
from jl_blocks.fly import Drone

with Drone("SquareFlight") as drone:  # waits for you to select the mode, takes off
    for _ in range(4):
        drone.forward(1.5)
        drone.turn_left(90)
# lands here, even if the script crashed or you pressed Ctrl+C
```

## Try it in the simulator

```bash
./docker/run_sim_container.sh         # once, then build PX4 and the workspace (see the README)
./launch_scripts/sim_script.sh flights/square.py SquareFlight
```

The second argument is the name the script gives its `Drone`: it is the mode
name in QGC. The launcher starts Gazebo, PX4, the DDS agent and `jl_mission`,
selects the mode, runs your script in that terminal, and cleans up after.
`--no-start` leaves selecting the mode to you in QGC.

To run a script yourself (anywhere PX4, the DDS agent, `translation_node` and
`jl_mission` are already running with `mission_name:=NAME`):

```bash
source install/setup.bash
python3 flights/square.py
```

Do not run a `mission_runner` for the same name at the same time.

## The commands

Distances are metres, angles are degrees, "up" is positive.

| | |
|---|---|
| `Drone(name, default_height=1.5, velocity=0.5, take_off_on_enter=True)` | `with` it: waits until the mode is selected, takes off, and lands on exit |
| `take_off(height)` / `land()` | PX4's own takeoff and land, blocking until done |
| `arm()` | arm and stay on the ground (PX4 will not lift off from position setpoints in an external mode: use `take_off()` next) |
| `forward / back / left / right / up / down (distance, velocity=None)` | straight-line move relative to where the drone faces; waits until it arrives |
| `move_distance(forward, left, up, velocity=None)` | the same, in all three axes at once |
| `goto(north, east, up)` | straight line to a spot measured from where PX4 started |
| `turn_left(deg)` / `turn_right(deg)` | turn in place |
| `hover(seconds)` | stay put |
| `start_forward(v)` ... `start_down(v)`, then `stop()` | keep moving without waiting |
| `position` -> `Position(north, east, up)`, `heading` | where the drone is now |

### Your own loop

```python
with Drone("HoverTest", take_off_on_enter=False) as drone:
    drone.arm()
    drone.take_off(1.5)
    hover = drone.position
    for t in drone.every(5):          # the body runs 50 times a second for 5 s
        drone.send_position(hover)
    drone.land()
```

`send_position(position, heading=None)` sets the setpoint and `tick()` sends it
and waits for the next 50 Hz step; `every(seconds)` does the `tick()` for you.
`drone.now()` is a clock if you prefer `while drone.now() - start < 5:`. Between
calls the last setpoint keeps being sent, so a `time.sleep()` will not trip the
silence watchdog.

## RGB-D camera (RealSense-style)

`x500_rgbd` is an x500 with a forward-facing RGB-D camera modelled on an Intel
RealSense D435, flown in a small world (`rgbd_demo`) with a box, a crate, a pillar
and a wall ahead of the spawn point. Add `--rgbd` to any launcher:

```bash
./launch_scripts/sim_script.sh flights/square.py SquareFlight --rgbd
```

The camera appears in ROS 2 under the names the real `realsense2_camera` driver
uses, so code written against the simulator ports to the drone's camera:

| Topic | Type | |
|---|---|---|
| `/camera/color/image_raw` | `sensor_msgs/Image` | `rgb8`, 640x480, 30 Hz |
| `/camera/color/camera_info` | `sensor_msgs/CameraInfo` | |
| `/camera/aligned_depth_to_color/image_raw` | `sensor_msgs/Image` | `32FC1`, metres |
| `/camera/depth/color/points` | `sensor_msgs/PointCloud2` | frame `camera_link` (x forward) |

Look at it from a second terminal (the container shares your display):

```bash
docker exec -it --user user jacob_ladder_sim bash -c \
  'source /opt/ros/humble/setup.bash && ros2 run rqt_image_view rqt_image_view /camera/color/image_raw'
```

Pick `/camera/aligned_depth_to_color/image_raw` in the dropdown for depth, or run
`rviz2` and add a PointCloud2 display with the Fixed Frame set to `camera_link`.

Differences from the real camera: simulated depth is 32-bit float metres where
the driver publishes 16-bit millimetres (`16UC1`); the depth uses the colour
camera's intrinsics (one rendered viewpoint); there is no stereo noise or
dropout, and the range is 0.3 to 10 m. The mount (0.12 m ahead of and 0.05 m
above `base_link`) and the optics are in `gazebo/models/x500_rgbd/model.sdf`.

Headless rendering uses EGL, which fails on some GPU setups with
`eglInitialize failed`. Run the headless tests with `GZ_RENDER=x11` there, to
render through your display instead:
`docker exec -e GZ_RENDER=x11 ... test/sitl_rgbd.sh`.

## Good to know

- `forward`, `left` and `turn_*` follow the drone's **heading**. The simulated
  x500 spawns facing east (about 96 degrees), not north.
- Moves are straight lines at `velocity` (default 0.5 m/s, at most 1 m/s). A move
  that does not arrive within its expected time plus 10 s raises `FlightError`,
  and leaving the `with` block lands.
- Positions are measured from where PX4's position estimate started, which is
  where the drone was switched on.
- The flow is tested in plain Python (`make check`) and flown in SITL by
  `test/sitl_script.sh test/fly_smoke.py FlySmoke` (part of `make sitl-test`).
