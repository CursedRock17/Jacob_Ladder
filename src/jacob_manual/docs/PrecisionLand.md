# PrecisionLand Mode

Full autonomous precision landing sequence using a **downward-facing camera**.
The drone takes off, climbs to search altitude, flies a spiral pattern to find
an ArUco tag, positions itself above the tag, then descends while tracking it
with a PI controller until touchdown.

## State Machine

```
OpticalFlowInit --> Climbing --> Search --> Approach --> Descend --> Finished
                                  ^                       |
                                  |    (if target lost)   |
                                  +----- FAIL <-----------+
```

| State | What it does |
|-------|-------------|
| **OpticalFlowInit** | Rise to `optical_flow_height` and hold for `optical_flow_hold_time` seconds. |
| **Climbing** | Ascend to `target_height` at `climb_rate` m/s. Generates search waypoints when altitude is reached. |
| **Search** | Fly an expanding spiral pattern. Each waypoint is visited in order, looping back to the start if needed. Switches to Approach when the downward camera detects a tag. |
| **Approach** | Fly horizontally to position directly above the tag at the current altitude. |
| **Descend** | Descend at `descent_vel` m/s while a PI controller keeps the drone centered over the tag. The yaw tracks the tag's orientation. |
| **Finished** | PX4 reports landing detected. Hold position and report success. |

If the target is lost during Approach or Descend, the mode reports failure.

## Search Pattern

The search generates an **expanding spiral** pattern:

1. Start at the center (0, 0) at the current altitude
2. Spiral outward to `max_radius` (2m) over 16 points
3. Drop one layer (`layer_spacing` = 0.5m)
4. Spiral back inward at the new altitude
5. Drop again and repeat

This covers a wide area while gradually descending, giving the downward
camera the best chance of spotting the tag.

## PI Controller (Descent Tracking)

During descent, a PI controller generates horizontal velocity commands
to stay centered over the tag:

```
velocity_x = -(error_x * Kp + integral_x * Ki)
velocity_y = -(error_y * Kp + integral_y * Ki)
```

The negative sign ensures the drone moves *toward* the tag (reducing error).
The integral is clamped to `max_velocity` to prevent windup.

## Launch

```bash
ros2 launch jacob_manual precision_land.launch.py
```

## Parameters

Configured in `cfg/params.yaml`:

| Parameter | Default | Unit | Description |
|-----------|---------|------|-------------|
| `optical_flow_height` | 0.5 | m | Height for optical flow init |
| `optical_flow_hold_time` | 3.0 | s | Hold time at flow height |
| `target_height` | 1.0 | m | Search altitude |
| `climb_rate` | 0.3 | m/s | Climb speed |
| `descent_vel` | 0.6 | m/s | Descent speed |
| `vel_p_gain` | 1.7 | - | Proportional gain for descent XY tracking |
| `vel_i_gain` | 0.0 | - | Integral gain for descent XY tracking |
| `max_velocity` | 3.0 | m/s | Max horizontal speed |
| `target_timeout` | 5.0 | s | Tag lost timeout |
| `delta_position` | 0.2 | m | Position tolerance |
| `delta_velocity` | 0.2 | m/s | Speed tolerance |

## Topics

| Topic | Direction | Type | Purpose |
|-------|-----------|------|---------|
| `/target_pose` | Subscribe | `PoseStamped` | ArUco tag from downward camera |
| `/fmu/in/trajectory_setpoint` | Publish | `TrajectorySetpoint` | Position/velocity commands |
| `/fmu/out/vehicle_local_position` | Subscribe | `VehicleLocalPosition` | Drone position |
| `/fmu/out/vehicle_attitude` | Subscribe | `VehicleAttitude` | Drone orientation |
| `/fmu/out/vehicle_land_detected` | Subscribe | `VehicleLandDetected` | Landing detection |

## Source Files

- `PrecisionLand.hpp` — Class definition, search/descend state
- `PrecisionLand.cpp` — Spiral generation, PI controller, state machine
