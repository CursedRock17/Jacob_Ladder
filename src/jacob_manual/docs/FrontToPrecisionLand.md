# FrontToPrecisionLand Mode

The most advanced mode — combines a **front-camera approach** with a
**downward-camera precision landing** in a single flight. Uses two separate
ArUco tags detected by two different cameras.

This is the full pipeline for scenarios where the drone must first fly
toward a target it can see ahead of it, then switch to the downward camera
to land precisely on top of it.

## State Machine

```
FrontSearch --> FrontApproach --> PrecisionDescend --> Finished
    ^               |                   |
    +---------------+                   |
     (front target lost)          (land detected)
```

An optional `PrecisionApproach` state exists for centering above the
downward tag before descending, but the current flow skips it — going
directly from FrontApproach to PrecisionDescend.

| State | Camera | What it does |
|-------|--------|-------------|
| **FrontSearch** | Front | Hover in place, wait for the front camera to detect a tag. |
| **FrontApproach** | Front | PID-controlled flight toward the front tag. Stops at `front_hold_distance`. Points the nose at the tag. |
| **PrecisionApproach** | Down | *(Optional)* Center above the downward tag at the current altitude before descending. |
| **PrecisionDescend** | Down | Descend at constant velocity until PX4 detects landing. |
| **Finished** | - | Hold position, report success. |

## Two Cameras, Two Tags

This mode subscribes to **two separate tag detection topics**:

| Camera | Topic | Rotation Matrix | Purpose |
|--------|-------|----------------|---------|
| Front | `/front/target_pose` | `[0,0,1; 1,0,0; 0,1,0]` | Find and approach the target from a distance |
| Down | `/target_pose` | `[0,-1,0; 1,0,0; 0,0,1]` | Precision tracking during descent |

Each camera has its own optical-to-body rotation matrix because they point
in different directions. The transform chain (camera -> body -> world) ensures
both tags end up in the same NED world frame.

## PID Controller (Front Approach)

The FrontApproach phase uses the same PID controller as the standalone
FrontApproach mode:

```
velocity = Kp * error + Ki * integral + Kd * derivative
```

Applied independently to X and Y axes, with anti-windup clamping on the
integral term. The Z axis uses proportional-only control.

When the drone reaches `front_hold_distance` from the front tag, it:
1. Resets the PID controller
2. Clears stale downward tag detections
3. Switches to PrecisionDescend

## Launch

```bash
ros2 launch jacob_manual front_to_precision_land.launch.py
```

## Parameters

Configured in `cfg/front_to_precision_params.yaml`:

### Front Approach Parameters

| Parameter | Default | Unit | Description |
|-----------|---------|------|-------------|
| `front_hold_distance` | 1.0 | m | Stop distance from the front tag |
| `front_target_timeout` | 3.0 | s | Front tag lost timeout |
| `front_pid_kp` | 0.8 | - | Proportional gain (XY) |
| `front_pid_ki` | 0.02 | - | Integral gain (XY) |
| `front_pid_kd` | 0.3 | - | Derivative gain (XY) |
| `front_pid_max_velocity` | 1.0 | m/s | Max horizontal speed |
| `front_pid_integral_limit` | 0.5 | - | Anti-windup clamp |
| `front_pid_kp_z` | 0.6 | - | Proportional gain (Z) |
| `front_pid_max_velocity_z` | 0.6 | m/s | Max vertical speed |

### Precision Landing Parameters

| Parameter | Default | Unit | Description |
|-----------|---------|------|-------------|
| `precision_target_timeout` | 3.0 | s | Downward tag lost timeout |
| `precision_descent_velocity` | 0.5 | m/s | Descent speed |
| `precision_vel_p` | 1.5 | - | Descent tracking P gain |
| `precision_vel_i` | 0.0 | - | Descent tracking I gain |
| `precision_max_velocity` | 1.0 | m/s | Max descent tracking speed |
| `precision_delta_position` | 0.25 | m | Position tolerance |
| `precision_delta_velocity` | 0.25 | m/s | Speed tolerance |

## Topics

| Topic | Direction | Type | Purpose |
|-------|-----------|------|---------|
| `/front/target_pose` | Subscribe | `PoseStamped` | Front camera ArUco tag |
| `/target_pose` | Subscribe | `PoseStamped` | Downward camera ArUco tag |
| `/fmu/in/trajectory_setpoint` | Publish | `TrajectorySetpoint` | Velocity commands |
| `/fmu/out/vehicle_local_position` | Subscribe | `VehicleLocalPosition` | Drone position |
| `/fmu/out/vehicle_attitude` | Subscribe | `VehicleAttitude` | Drone orientation |
| `/fmu/out/vehicle_land_detected` | Subscribe | `VehicleLandDetected` | Landing detection |

## Source Files

- `FrontApproachPrecisionLandCombined.hpp` — Class definition, dual-tag state
- `FrontApproachPrecisionLandCombined.cpp` — Two PID controllers, two transforms, state machine
