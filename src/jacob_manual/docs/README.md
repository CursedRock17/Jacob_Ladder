# Jacob Manual — External Flight Modes

This package provides custom PX4 **external flight modes** for the Jacob Ladder drone platform. Each mode registers itself with PX4 through the `px4_ros2` library, meaning it shows up as a selectable flight mode just like the built-in ones (Stabilized, Position, Mission, etc.) — but the logic runs on the companion computer instead of the flight controller.

## What is an External Mode?

PX4 supports **external modes** via ROS 2. Instead of running control logic on the autopilot, the companion computer:

1. Registers a custom mode with PX4
2. Sends trajectory setpoints (position or velocity commands) at a steady rate
3. Reads sensor data (position, attitude, landing detection) from PX4 topics

If the companion computer stops sending setpoints, PX4 will automatically failsafe — so these modes are only active when the ROS node is alive and healthy.

## Available Modes

| Mode | Launch Command | Camera | Description |
|------|---------------|--------|-------------|
| [TakeoffLand](./TakeoffLand.md) | `ros2 launch jacob_manual takeoff_land.launch.py` | None | Take off, hold, land |
| [TakeoffHold](./TakeoffHold.md) | `ros2 launch jacob_manual takeoff_hold.launch.py` | None | Take off, hold indefinitely |
| [FrontApproach](./FrontApproach.md) | `ros2 launch jacob_manual front_approach.launch.py` | Front | Fly toward a front-camera ArUco tag |
| [PrecisionLand](./PrecisionLand.md) | `ros2 launch jacob_manual precision_land.launch.py` | Down | Search, approach, and land on a downward-camera tag |
| [FrontToPrecisionLand](./FrontToPrecisionLand.md) | `ros2 launch jacob_manual front_to_precision_land.launch.py` | Front + Down | Approach via front camera, then land via downward camera |

## Quick Start

Each launch file starts three things automatically:

1. **The flight mode node** — registers with PX4 and runs the state machine
2. **The Visualizer node** — publishes RViz-compatible poses, paths, and TF frames
3. **A rosbag recorder** — saves all topics to `rosbags/` for post-flight analysis

```bash
# 1. Make sure PX4 SITL and the DDS agent are running first

# 2. Source the workspace
cd ~/Documents/Electrical/Matrix_Lab/jacob_drone_ws/src/Jacob_Ladder
source install/setup.bash

# 3. Launch a mode (example: TakeoffLand)
ros2 launch jacob_manual takeoff_land.launch.py
```

## How the Modes are Built

All modes follow the same pattern:

- **Header (`.hpp`)** — Defines a class that inherits from `px4_ros2::ModeBase` with a state machine `enum class State`
- **Implementation (`.cpp`)** — Implements three key methods:
  - `onActivate()` — Called when PX4 switches to this mode. Resets state and records the starting position
  - `onDeactivate()` — Called when PX4 leaves this mode. Cleans up
  - `updateSetpoint(float dt_s)` — Called every control loop (~50Hz). Contains a `switch` statement over the state machine that sends position/velocity commands to PX4
- **Config (`.yaml`)** — Tunable parameters loaded at startup via `declare_parameter` / `get_parameter`
- **Launch (`.launch.py`)** — Starts the mode node, Visualizer, RViz, and rosbag

## Coordinate Frames

PX4 uses the **NED** (North-East-Down) coordinate frame:
- **X** = North
- **Y** = East  
- **Z** = Down (negative values = higher altitude)

This means going *up* is a *negative* z value. You'll see this throughout the code — for example, `_hold_position.z() = _base_position.z() - _target_height` subtracts height because "up" is negative.

RViz uses **ENU** (East-North-Up), so the Visualizer node converts between the two.

## Tuning Parameters

Each mode loads parameters from a YAML file in `cfg/`. You can also override them at launch:

```bash
ros2 launch jacob_manual front_approach.launch.py front_pid_kp:=1.0 front_hold_distance:=2.0
```

See each mode's documentation page for its full parameter list.

## File Overview

```
jacob_manual/
  docs/                  # Documentation (you are here)
  cfg/                   # Parameter YAML files
  launch/                # ROS 2 launch files
  TakeoffLand.hpp/cpp    # Takeoff, hold, land
  TakeoffHold.hpp/cpp    # Takeoff, hold forever
  FrontApproach.hpp/cpp  # Front camera approach
  PrecisionLand.hpp/cpp  # Downward camera precision land
  FrontApproachPrecisionLandCombined.hpp/cpp  # Both cameras combined
  Visualizer.hpp/cpp     # RViz bridge (NED -> ENU)
  CMakeLists.txt         # Build configuration
  package.xml            # ROS 2 package manifest
```
