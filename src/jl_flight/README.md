# jl_flight: missions as plain Python

A mission is a Python function that flies a `Drone`:

```python
from jl_flight import AprilTag, mission

TAG = AprilTag(id=0, size=0.30)

@mission("LandBeforeAprilTag", needs=[TAG])
def land_before_apriltag(drone):
    drone.take_off(1.5)
    drone.search(TAG, timeout=30)
    drone.approach(TAG, standoff=1.5, timeout=60)
    drone.hover(2)
    drone.land()
```

Missions live in [`missions/`](../../missions). Leaving the function, normally or by an
exception, lands the drone. Switching modes in QGC stops the mission and the pilot has
control. Everything flies through `jl_mission`, so its safety limits always apply.

## The layers

| Layer | What it is | Where |
|---|---|---|
| **Mission** | `@mission(name, needs=[...])` on a function. The name is what QGC shows; `needs` lists the perception it uses | `missions/*.py` |
| **Drone** | Blocking commands: `take_off`, `forward`, `turn_left`, `search`, `approach`, `follow`, `land`, and `run(skill)` for anything else | `jl_flight/drone.py` |
| **Skill** | One thing done over time, ticked at 50 Hz: `GoTo`, `Turn`, `Hold`, `Search`, `Track`, `PrecisionDescend` | `jl_flight/skills/` |
| **Perception** | What can be seen: `AprilTag`, `ArucoTag`, `YoloDrogue`. Each knows the processes that produce its detections, its topic, and its frame | `jl_flight/perception/` |
| **Airframe** | What is on the aircraft: each camera's topics and mount (offset, facing, pitch). `generic`, and the simulator's `x500_rgbd` and `x500_dual_cam` | `jl_flight/airframes.py`, picked in `config/flight.yaml` or with `JL_AIRFRAME` |

Skills, perception and airframes are pure Python and tested without ROS. The only ROS code is
`jl_flight/ros/vehicle_link.py`.

## The Drone's commands

Distances are metres, angles degrees, "up" positive. Each blocks until it is done.

| | |
|---|---|
| `take_off(height)` / `land()` | PX4's own takeoff and land |
| `arm()` | arm and stay on the ground (PX4 will not lift off from position setpoints in an external mode: `take_off()` next) |
| `forward / back / left / right / up / down (distance, velocity=None)` | straight-line move relative to where the drone faces |
| `move_distance(forward, left, up)`, `goto(north, east, up)` | the same in three axes; a spot measured from where PX4 started |
| `turn_left(deg)` / `turn_right(deg)` / `hover(seconds)` | turn in place / stay put |
| `search(target, pattern="hold"\|"spiral", timeout=)` | wait (or fly a spiral) until the target is seen |
| `approach(target, standoff=, timeout=)` | fly to `standoff` metres short of it, facing it, and stop once settled |
| `follow(target, standoff=, duration=)` | hold station there, re-tracking it as it moves |
| `seen(target)`, `position`, `heading` | what the drone sees and where it is |
| `run(skill, target=, controller=, timeout=, duration=, until=)` | run any skill; the commands above are wrappers around this |
| `start_forward(v)` ... `start_down(v)`, then `stop()` | keep moving without waiting |
| `say(text)` | report progress (logged, and published on `/jl/NAME/events`) |

`Timeout` and `TargetLost` (both `FlightError`s) can be caught to handle a failure, e.g.
`except TargetLost: drone.land()`.

Your own loop: `send_position(position, heading=None)` sets the setpoint and `tick()` sends it and
waits for the next 50 Hz step; `for t in drone.every(5):` does the `tick()` for you. Between calls
the last setpoint keeps being sent, so a `time.sleep()` does not trip jl_mission's silence
watchdog. See `missions/hover_5s.py`.

## Swapping what the drone looks for

Every Drone command that takes a target takes any declaration, so a mission swaps what it
looks for by changing one line. These two missions fly the same steps:

```python
DROGUE = YoloDrogue()                    # YOLO + ranging, on the front camera

@mission("DrogueApproach", needs=[DROGUE])
def drogue_approach(drone):
    drone.take_off(1.75)
    try:
        drone.follow(DROGUE, standoff=3.0)   # hold 3 m behind it until the pilot takes over
    except TargetLost:
        drone.say("lost the drogue: landing")
        drone.land()
```

`missions/apriltag_approach.py` is the same with `TAG = AprilTag(id=0, size=0.30)`, the
simulator's stand-in for the drogue. Mission logic lives in the mission file, so each one can be
tuned on its own.

A new kind of target is a new `Perception` subclass. It says which processes to start, which
topic their `PoseStamped` detections arrive on, and which frame those are in (`"optical"`
like `aruco_tracker`, or `"ranging"` like the YOLO pose node). Nothing else changes.

## Running

```bash
python3 -m jl_flight check missions/*.py --airframe x500_rgbd   # load and validate
python3 -m jl_flight helpers missions/drogue_approach.py         # what it would start
ros2 launch jl_flight mission.launch.py mission:=missions/apriltag_approach.py airframe:=x500_rgbd
```

The launch file starts `jl_mission` (the QGC mode), each helper the mission declares, and the
mission, which waits to be selected.

In the simulator, with the Gazebo window (from the host):

```bash
./launch_scripts/sim_mission.sh missions/apriltag_approach.py --world apriltag
```

Headless, with a check on where the drone ends up (inside the sim container):

```bash
test/sitl_flight.sh missions/land_before_apriltag.py --world apriltag \
  --landed-at "0 5.898" --ring 1.5 --tolerance 0.3
test/sitl_flight.sh missions/apriltag_approach.py --world apriltag \
  --holding-at "0 5.898" --ring 3.0 --after 60 --tolerance 0.25
```

On the drone, `config/flight.yaml` names the airframe and the missions to register at boot;
`./deploy.sh` does the rest (see the top-level README).

Set `GZ_RENDER=x11` where headless EGL rendering fails (`eglInitialize failed`). `--ring R`
checks the drone is R metres from the target in any direction, which is how standoffs are
defined: along the drone's own line to the target.

`make check` lints, type-checks and tests this package and checks every mission.

## On the real drone

The same mission file flies in the simulator and on the aircraft. What differs is the
airframe (its cameras), which `config/flight.yaml` names: `realsense_drone`, the aircraft
with a forward RealSense D435i whose colour stream the VIO service publishes.

The camera's mount is the lever arm VIO already uses (12.5 cm forward of the flight
controller, 2.5 cm above it, from `oak_d_visual_odometry/config/cuvslam_params.yaml`). Print
the tag the mission declares: `apriltag_approach.py` wants tag36h11 id 0 with a 0.30 m black
square (change its `TAG` line for a different tag).

**Props off.** Run the mission exactly as it would fly, with its setpoints printed. With the
drone powered and the flight-controller link, VIO and `translation_node` running (the boot
services do this), on the Jetson:

```bash
ros2 launch jl_flight mission.launch.py mission:=missions/apriltag_approach.py \
  airframe:=realsense_drone report:=0.5
```

Then select `AprilTagApproach` in QGC. It arms and spins the motors exactly as in flight, and
twice a second prints what it is commanding, where the drone is, and where it sees the tag:

```text
  26.6 s Track    setpoint N  -0.35 E  +3.01 U  1.01 hdg   96
                 drone    N  -0.03 E  -0.00 U  0.01 hdg   96
                 apriltag_36h11_0  N  -0.67 E  +5.99 U  1.01, 6.1 m away
```

North, east and up are metres from where PX4's position estimate started. Without propellers
the drone cannot climb, so a props-off run ends at the takeoff: PX4's own takeoff never
reaches the height, PX4 disarms an aircraft that has not left the ground within
`COM_DISARM_PRFLT` seconds, and `jl_mission` gives up on the takeoff after 30 s and lands. During
it the report shows `setpoint none (PX4 is flying its own takeoff)` (PX4, not the mission, is
in control), the drone's position, and the tag estimate, so you can check the motors, the
arming, and that the tag is seen where it really is. Stop the `jl_mission@apriltag_approach`
service first if `deploy.sh` already installed it (`sudo systemctl stop jl_mission@apriltag_approach`),
so only one copy runs.

**Flying it.** Either let `./deploy.sh` register it at boot and select it in QGC, or run it by
hand as above, with the props on, to see its setpoints as it flies.
