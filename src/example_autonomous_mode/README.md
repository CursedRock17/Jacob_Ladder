# example_autonomous_mode

A complete, working PX4 **external flight mode** in one file pair, meant to be
read start to finish and then copied. It takes off to a low hover, holds, and
lands — nothing else. Every other mode in this workspace is this shape with more
states in the middle.

If you want an *empty* skeleton rather than a working mode, see
`precision_land/BlankMode.cpp` (launched by `launch_scripts/offboard_blank.sh`).
That one registers with PX4 and does nothing, which is useful when you already
know the framework. Start here instead if you want to see a mode that actually
flies, with parameters, a state machine, and debug topics wired up.

## What is an External Mode?

PX4 supports custom modes that run on the companion computer instead of the
flight controller. The node registers a mode with PX4 through `px4_ros2`, and it
then appears in QGroundControl next to Stabilized, Position, and Mission. While
the mode is active it must keep sending setpoints; if the companion computer
stops talking, PX4 failsafes on its own.

## State Machine

```
InitialTakeoffAltitude --> Holding --> Descending --> Finished
```

| State | What it does |
|-------|-------------|
| **Idle** | Not active. Entered on `onDeactivate()`. |
| **InitialTakeoffAltitude** | Rise to `optical_flow_height` and hover there for `optical_flow_hold_time` seconds, giving the optical flow sensor a close, textured surface to lock onto before climbing away. |
| **Holding** | Hover at the current position for `hold_duration` seconds. **This is where your own states go.** |
| **Descending** | Command a constant downward velocity (`descent_vel`) until PX4's landing detector fires, or until the drone is within `landing_height` of where it took off. |
| **Finished** | Hold position and report success to PX4 (once, on entry). |

## Build and Run

```bash
# From the workspace root, with PX4 SITL and the DDS agent already running
source /opt/ros/humble/setup.bash
colcon build --packages-select example_autonomous_mode
source install/setup.bash

ros2 launch example_autonomous_mode example_autonomous_mode.launch.py
```

Then select **ExampleAutonomousMode** in QGroundControl. The mode arms nothing
on its own — take off manually or switch into it from a hover.

## Parameters

Configured in `cfg/example_autonomous_mode_params.yaml`:

| Parameter | Default | Unit | Description |
|-----------|---------|------|-------------|
| `optical_flow_height` | 0.25 | m | Height of the initial low hover |
| `optical_flow_hold_time` | 3.0 | s | How long to hold at that height |
| `delta_position` | 0.05 | m | "Close enough" tolerance for reaching a target |
| `hold_duration` | 7.5 | s | How long to hover before descending |
| `descent_vel` | 0.5 | m/s | Downward speed during landing |
| `landing_height` | 0.10 | m | Height above takeoff counted as landed, as a backstop if the landing detector never fires |

Override one at launch:

```bash
ros2 launch example_autonomous_mode example_autonomous_mode.launch.py
# or, to change a value permanently, edit cfg/example_autonomous_mode_params.yaml
```

## Debug Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/drone_state` | `std_msgs/String` | The state name, published on every transition |
| `/tracking_error` | `geometry_msgs/Vector3Stamped` | Commanded minus actual position, in NED. If this grows instead of shrinking, PX4 is not following your setpoints. |

```bash
ros2 topic echo /drone_state
```

## Coordinate Frames

PX4 uses **NED** (North-East-Down), so **up is negative z**. That is why the
takeoff target is computed by *subtracting*:

```cpp
_hold_position.z() = _base_position.z() - _optical_flow_height;
```

and why "how much altitude have we gained" is `base - current`. Getting this
backwards is the single most common mistake when writing a new mode.

All heights here are relative to `_base_position`, the position recorded in
`onActivate()` — not to the local origin. The two are only the same when the
mode happens to be activated at the origin's altitude.

## Writing Your Own Mode

1. Copy this package to `src/your_mode`, then rename the package in
   `package.xml` and `CMakeLists.txt` (`project()`, the `add_executable` target,
   and the `install(TARGETS ...)` entry). ROS 2 package names must be lowercase
   with underscores — no hyphens.
2. Change `kExampleAutonomousModeName` in the header. **This string must be
   unique across every mode registered with the same autopilot**, and PX4 caps
   it at 24 characters. Two nodes claiming one name will collide.
3. Add states to the `enum class State`, a `case` in `updateSetpoint()`, and a
   name in `stateName()`. The compiler will tell you if you miss the last one —
   the switch has no `default`, so `-Werror` catches unhandled values.
4. Add parameters in `loadParameters()` and to the YAML. Declare the compiled
   default as the fallback so the two cannot drift apart.
5. Point your states at real work between `InitialTakeoffAltitude` and
   `Descending`. Leave the takeoff and landing states alone until you have a
   reason not to.

`updateSetpoint()` runs at roughly 50 Hz and is passed `dt_s`, the time since
the last call. Use it to integrate — `_hold_position.z() -= climb_rate * dt_s`
is a rate-limited climb — rather than assuming a fixed loop period.

## Formatting

This package uses `clang-format` (see `.clang-format`), not the astyle
configuration the older packages use:

```bash
clang-format -i src/example_autonomous_mode/*.cpp src/example_autonomous_mode/*.hpp
```
