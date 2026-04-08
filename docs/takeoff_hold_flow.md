# TakeoffHold State Machine

## Overview

TakeoffHold is a two-layer state machine that arms the drone, performs a low takeoff
for optical-flow sensor initialization, then smoothly climbs to the target altitude
and holds position.

**Parameters** (set via ROS 2 launch/yaml):

| Parameter | Default | Description |
|---|---|---|
| `optical_flow_height` | 0.5 m | Low hover altitude for optical flow lock |
| `optical_flow_hold_time` | 3.0 s | Time to hold at optical flow height |
| `target_height` | 1.25 m | Final hold altitude |
| `climb_rate` | 0.3 m/s | Vertical climb speed |
| `delta_position` | 0.25 m | Position tolerance for "reached" checks |

---

## Executor Flow (TakeoffHoldExecutor)

The executor handles arming and the initial PX4 takeoff command, then hands control
to the mode's internal state machine.

```mermaid
flowchart TD
    Start([Executor Activated]) --> Arming

    Arming["**Arming**\nSend arm command"]
    Arming -- "Success" --> TakingOff
    Arming -- "Failure" --> Error([Error — log & stop])

    TakingOff["**TakingOff**\nPX4 takeoff to optical_flow_height AMSL"]
    TakingOff -- "Success" --> Hold
    TakingOff -- "Failure" --> Error

    Hold["**Hold**\nSchedule TakeoffHoldMode\n(internal state machine takes over)"]
    Hold --> ModeEntry([Mode State Machine])
```

---

## Mode Flow (TakeoffHoldMode)

Once the executor schedules the mode, the internal state machine manages the climb
profile and final hold.

```mermaid
flowchart TD
    Activate([onActivate]) --> Init

    Init["Record base_position\nSet hold_position.z = base - optical_flow_height\nstate = OpticalFlowInit"]
    Init --> OFI

    OFI["**OpticalFlowInit**\nHold at optical_flow_height\nWait for sensor stabilization"]
    OFI -- "altitude_gained >= optical_flow_height - delta_position" --> ReachedFlag
    ReachedFlag["Set reached_flow_height = true\nReset hold timer"]
    ReachedFlag -- "state_elapsed >= optical_flow_hold_time" --> Climbing

    OFI -- "not yet at height" --> OFI

    Climbing["**Climbing**\nRamp hold_position.z downward at climb_rate\n(position + velocity feedforward)"]
    Climbing -- "altitude_gained >= target_height - delta_position" --> Holding
    Climbing -- "still climbing" --> Climbing

    Holding["**Holding**\nMaintain position at target_height\n(indefinite hold)"]
    Holding --> Holding

    style OFI fill:#2d6a4f,stroke:#1b4332,color:#fff
    style Climbing fill:#e76f51,stroke:#b5451b,color:#fff
    style Holding fill:#264653,stroke:#1d3640,color:#fff
```

---

## Combined End-to-End View

```mermaid
flowchart LR
    subgraph Executor
        A([Start]) --> B[Arming]
        B --> C[TakingOff]
        C --> D[Schedule Mode]
    end

    subgraph Mode
        D --> E[OpticalFlowInit]
        E -->|height reached + hold time elapsed| F[Climbing]
        F -->|target height reached| G[Holding]
    end
```
