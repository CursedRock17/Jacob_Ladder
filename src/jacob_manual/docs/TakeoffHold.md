# TakeoffHold Mode

Takes off and holds position indefinitely. Like TakeoffLand but without
the descent — the drone stays in the air until the mode is manually deactivated.

Useful for testing stable hover, verifying optical flow, or as a starting
point before switching to a different mode.

## State Machine

```
OpticalFlowInit --> Climbing --> Holding (indefinitely)
```

| State | What it does |
|-------|-------------|
| **OpticalFlowInit** | Rise to `optical_flow_height` and hover for `optical_flow_hold_time` seconds so the optical flow sensor initializes. |
| **Climbing** | Gradually ascend to `target_height`. *Currently skipped — jumps straight to Holding.* |
| **Holding** | Hover at the current position forever. The mode never completes on its own. |

## Launch

```bash
ros2 launch jacob_manual takeoff_hold.launch.py
```

## Parameters

Configured in `cfg/takeoff_hold_params.yaml`:

| Parameter | Default | Unit | Description |
|-----------|---------|------|-------------|
| `optical_flow_height` | 0.25 | m | Height for optical flow initialization |
| `optical_flow_hold_time` | 3.0 | s | How long to hover at flow height |
| `target_height` | 1.0 | m | Target cruise altitude (if Climbing is enabled) |
| `climb_rate` | 0.3 | m/s | Vertical speed during climb |
| `delta_position` | 0.05 | m | "Close enough" tolerance |

## Topics

| Topic | Direction | Type | Purpose |
|-------|-----------|------|---------|
| `/fmu/in/trajectory_setpoint` | Publish | `TrajectorySetpoint` | Position commands |
| `/fmu/out/vehicle_local_position` | Subscribe | `VehicleLocalPosition` | Current drone position |

## Source Files

- `TakeoffHold.hpp` — Class definition and state machine enum
- `TakeoffHold.cpp` — State machine implementation
