# FrontApproach Mode

Uses a **forward-facing camera** to detect an ArUco tag and fly toward it.
The drone searches for the tag, then uses a PID controller to smoothly
approach and stop at a configurable hold distance.

## State Machine

```
Search --> Approach --> Finished
  ^          |
  +----------+  (if target lost)
```

| State | What it does |
|-------|-------------|
| **Search** | Hover in place, waiting for the front camera to detect a tag. |
| **Approach** | PID-controlled flight toward the tag. Sends velocity commands in XY (horizontal) and Z (vertical). Points the drone's nose at the tag. Stops at `front_hold_distance` from the tag. |
| **Finished** | Hold position and report success to PX4. |

If the target is lost during Approach, the mode falls back to Search.

## How the PID Controller Works

The Approach state uses a standard PID controller for horizontal (XY) velocity:

```
velocity = Kp * error + Ki * integral(error) + Kd * derivative(error)
```

- **Error** = distance from the drone to the target position (tag minus hold distance)
- **Integral** = accumulated error over time (clamped to prevent windup)
- **Derivative** = rate of change of error (smooths out oscillation)

The vertical (Z) axis uses simpler proportional-only control.

The resulting velocity is clamped to `front_pid_max_velocity` to prevent
the drone from flying too fast.

## Camera Frame Transform

The ArUco detector reports the tag's position in the **camera's optical frame**
(Z = forward through the lens). This needs to be converted to the **world frame**
(NED) so the drone can navigate. The transform chain is:

```
world <-- drone position/attitude <-- camera rotation <-- tag position
```

The camera rotation matrix is defined in the constructor and accounts for
how the camera is mounted on the drone.

## Launch

```bash
ros2 launch jacob_manual front_approach.launch.py
```

## Parameters

Configured in `cfg/front_approach_params.yaml`:

| Parameter | Default | Unit | Description |
|-----------|---------|------|-------------|
| `front_hold_distance` | 1.25 | m | Stop this far from the tag |
| `front_delta_position` | 0.25 | m | Position tolerance for "arrived" |
| `front_delta_velocity` | 0.25 | m/s | Speed tolerance for "arrived" |
| `front_target_timeout` | 15.0 | s | Seconds before a tag is considered lost |
| `front_pid_kp` | 0.8 | - | Proportional gain (XY) |
| `front_pid_ki` | 0.02 | - | Integral gain (XY) |
| `front_pid_kd` | 0.3 | - | Derivative gain (XY) |
| `front_pid_max_velocity` | 1.0 | m/s | Max horizontal speed |
| `front_pid_integral_limit` | 0.5 | - | Anti-windup clamp |
| `front_pid_kp_z` | 0.6 | - | Proportional gain (Z) |
| `front_pid_max_velocity_z` | 0.6 | m/s | Max vertical speed |

## Topics

| Topic | Direction | Type | Purpose |
|-------|-----------|------|---------|
| `/target_pose` | Subscribe | `PoseStamped` | ArUco tag detection from front camera |
| `/fmu/in/trajectory_setpoint` | Publish | `TrajectorySetpoint` | Velocity commands |
| `/fmu/out/vehicle_local_position` | Subscribe | `VehicleLocalPosition` | Current drone position |
| `/fmu/out/vehicle_attitude` | Subscribe | `VehicleAttitude` | Current drone orientation |

## Source Files

- `FrontApproach.hpp` — Class definition, ArucoTag struct, PID state
- `FrontApproach.cpp` — PID controller, coordinate transform, state machine
