# TakeoffLand Mode

The simplest flight mode. Takes off, holds altitude for a set duration, then lands.
No cameras or target tracking required — useful for testing basic flight and tuning
altitude parameters.

## State Machine

```
OpticalFlowInit --> Climbing --> Holding --> Descending --> Finished
```

| State | What it does |
|-------|-------------|
| **OpticalFlowInit** | Rise to a low height (`optical_flow_height`) and hover for `optical_flow_hold_time` seconds. This gives the optical flow sensor time to lock on and provide stable velocity estimates. |
| **Climbing** | Gradually ascend to `target_height` at `climb_rate` m/s. *Currently skipped — jumps straight to Holding at the flow height.* |
| **Holding** | Hover at the current position for `hold_duration` seconds. |
| **Descending** | Command a constant downward velocity (`descent_vel`) until PX4 reports that it has landed. |
| **Finished** | Hold the current position and tell PX4 the mode completed successfully. |

## Launch

```bash
ros2 launch jacob_manual takeoff_land.launch.py
```

## Parameters

Configured in `cfg/takeoff_land_params.yaml`:

| Parameter | Default | Unit | Description |
|-----------|---------|------|-------------|
| `optical_flow_height` | 0.25 | m | Height to hover at for optical flow initialization |
| `optical_flow_hold_time` | 3.0 | s | How long to hold at optical flow height |
| `target_height` | 1.0 | m | Target cruise altitude (used if Climbing state is enabled) |
| `climb_rate` | 0.3 | m/s | Vertical speed during climb |
| `delta_position` | 0.05 | m | "Close enough" tolerance for reaching a target |
| `hold_duration` | 7.5 | s | How long to hover before descending |
| `descent_vel` | 0.5 | m/s | Downward speed during landing |

## Topics

| Topic | Direction | Type | Purpose |
|-------|-----------|------|---------|
| `/fmu/in/offboard_control_mode` | Publish | `OffboardControlMode` | Heartbeat (handled by px4_ros2) |
| `/fmu/in/trajectory_setpoint` | Publish | `TrajectorySetpoint` | Position/velocity commands |
| `/fmu/out/vehicle_local_position` | Subscribe | `VehicleLocalPosition` | Current drone position |
| `/fmu/out/vehicle_land_detected` | Subscribe | `VehicleLandDetected` | Landing detection |

## Source Files

- `TakeoffLand.hpp` — Class definition and state machine enum
- `TakeoffLand.cpp` — State machine implementation
