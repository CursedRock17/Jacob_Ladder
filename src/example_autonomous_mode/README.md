# example_autonomous_mode

A complete PX4 **external mode + executor workflow** meant to be read from
start to finish and then copied. It arms, takes off to a low hover, waits for
optical flow or VIO to settle, holds, performs a controlled descent, hands
touchdown back to PX4, and waits for disarm.

Use this package when you want to learn the full autonomous workflow. For a
smaller mode without an executor, see
[`precision_land/BlankMode.cpp`](../precision_land/BlankMode.cpp). `BlankMode`
captures the activation position and keeps sending that position as its
setpoint; the pilot must arm and select it separately.

> **Safety:** Selecting `ExampleAutonomousMode` starts an automatic arm,
> takeoff, descent, and landing sequence. Use SITL first. On hardware, remove
> propellers during initial integration and keep a pilot ready to take over.

## Files in This Example

| File | Purpose |
| --- | --- |
| [`ExampleAutonomousMode.hpp`](ExampleAutonomousMode.hpp) | Declares both state machines, parameters, ROS interfaces, and flight-mode name. |
| [`ExampleAutonomousMode.cpp`](ExampleAutonomousMode.cpp) | Implements registration, state transitions, setpoints, executor operations, and the ROS node. |
| [`cfg/example_autonomous_mode_params.yaml`](cfg/example_autonomous_mode_params.yaml) | Supplies the runtime parameter values. |
| [`launch/example_autonomous_mode.launch.py`](launch/example_autonomous_mode.launch.py) | Starts the combined executor/mode node and loads the YAML file. |
| [`CMakeLists.txt`](CMakeLists.txt) and [`package.xml`](package.xml) | Declare how ROS 2 builds, installs, and discovers the package. |

## External Mode and Executor

PX4 external modes run on the companion computer instead of inside the flight
controller. This node registers through `px4_ros2`; after registration,
`ExampleAutonomousMode` can be selected in QGroundControl like a built-in
flight mode. While the mode is active, the companion computer must continue
sending setpoints.

The two classes have different responsibilities:

- `ExampleAutonomousModeExecutor` owns the operations around the custom mode:
  arm, PX4-native takeoff, schedule the mode, PX4-native landing, and wait for
  disarm.
- `ExampleAutonomousMode` owns continuous flight behavior while scheduled:
  settle, hold, descend, and continuously send position or velocity setpoints.

`Settings::Activation::ActivateAlways` lets the executor become active while
the vehicle is disarmed so it can issue the arm command. It does **not** select
the mode immediately after registration. The workflow begins when the operator
selects `ExampleAutonomousMode` in QGroundControl or commands the equivalent
mode change through another PX4 interface.

## State Machines

```text
Executor: Arming -> TakingOff -> RunningMode -> Landing -> WaitingForDisarm
                                      |
Mode:                  OpticalFlowSettling -> Holding -> Descending -> Finished
```

`Idle` is the mode's inactive state before and after the scheduled portion.

### Executor states

| State | Entry and completion condition |
| --- | --- |
| **Arming** | Calls `arm()`. A successful callback advances to `TakingOff`. |
| **TakingOff** | Calls PX4's native `takeoff()` for `optical_flow_height`. It advances when either the takeoff callback succeeds or local NED z crosses the relative altitude threshold. The two paths share a guard so the transition happens once. |
| **RunningMode** | Calls `scheduleMode()` for the owned `ExampleAutonomousMode`. A successful mode result advances to `Landing`. |
| **Landing** | Calls PX4's native `land()` after the mode hands off near the ground. |
| **WaitingForDisarm** | Calls `waitUntilDisarmed()` and logs completion when PX4 reports the vehicle disarmed. |

If an executor operation fails, `runState()` logs the result and does not
advance. If the scheduled mode ends with a non-success result—for example,
because of pilot takeover or a failsafe—the executor deliberately does **not**
issue an automatic landing command.

### Mode states

| State | Setpoint and transition |
| --- | --- |
| **Idle** | Sends no setpoint. This is the initial state and the state entered by `onDeactivate()`. |
| **OpticalFlowSettling** | Holds the position reached by native takeoff. After `optical_flow_hold_time`, advances to `Holding`. |
| **Holding** | Holds the same position for `hold_duration`. **Insert mission-specific states after this settling step and before descent.** |
| **Descending** | Sends `(0, 0, descent_vel)` as an NED velocity setpoint. Advances when PX4 reports landed or the vehicle reaches `landing_height` above the recorded/inferred ground plane. |
| **Finished** | Reports `Result::Success` once, on entry, so the executor can start native landing. |

`onActivate()` records the current local position as the hold position and
enters `OpticalFlowSettling`. `switchToState()` resets `_state_elapsed` for each
new state and publishes the new mode-state name on `/drone_state`.

## Build and Run

PX4 (or PX4 SITL) and the Micro XRCE-DDS agent must already be running and
connected. The workspace must also contain or provide `px4_ros2_cpp` and the
matching `px4_msgs` used by this repository.

```bash
# From the workspace root
source /opt/ros/humble/setup.bash
colcon build --packages-up-to example_autonomous_mode
source install/setup.bash

ros2 launch example_autonomous_mode example_autonomous_mode.launch.py
```

After the node reports successful registration, select
**ExampleAutonomousMode** in QGroundControl. The executor will then arm and run
the entire workflow; do not take off manually first.

The package uses this repository's required
`setSkipMessageCompatibilityCheck()` calls in both the mode and executor
because its PX4 message versions intentionally differ from the upstream
compatibility check. Do not remove those calls when copying this example unless
the repository's PX4/`px4_msgs` integration has been updated and verified.

## Parameters

The launch file loads
[`cfg/example_autonomous_mode_params.yaml`](cfg/example_autonomous_mode_params.yaml):

| Parameter | Default | Unit | Description |
| --- | ---: | --- | --- |
| `optical_flow_height` | 0.25 | m | Height requested from PX4's native takeoff operation. |
| `optical_flow_hold_time` | 3.0 | s | Time spent holding after takeoff so optical flow or VIO can settle. |
| `delta_position` | 0.05 | m | Altitude tolerance used by the executor's local-position takeoff watcher. Keep it smaller than `optical_flow_height`. |
| `hold_duration` | 7.5 | s | Time spent in the example `Holding` state. |
| `descent_vel` | 0.5 | m/s | Positive-down NED speed during the controlled-descent portion. |
| `landing_height` | 0.10 | m | Height above the ground plane where the mode hands final touchdown to PX4-native landing. |

The current launch file declares no launch arguments. To change a value, edit
the YAML file and rebuild the package before launching:

```bash
colcon build --packages-select example_autonomous_mode
source install/setup.bash
ros2 launch example_autonomous_mode example_autonomous_mode.launch.py
```

The YAML's top-level `example_autonomous_mode` key must match the node name in
the launch file. Otherwise ROS 2 ignores those entries and the compiled
fallbacks in `ExampleAutonomousMode.hpp` are used.

## Inputs and Debug Topics

| Topic | Direction | Type | Used for |
| --- | --- | --- | --- |
| `/fmu/out/vehicle_local_position` | Subscribe | `px4_msgs/msg/VehicleLocalPosition` | Executor's explicit takeoff-completion watcher. |
| `/fmu/out/vehicle_local_position_v1` | Subscribe | `px4_msgs/msg/VehicleLocalPosition` | Mode position input created by `OdometryLocalPosition`; `_v1` comes from this checkout's message version. |
| `/fmu/out/vehicle_land_detected` | Subscribe | `px4_msgs/msg/VehicleLandDetected` | Records the ground plane while landed and provides a descent stop condition. |
| `/drone_state` | Publish | `std_msgs/msg/String` | Mode-state transitions only; executor states are currently log messages, not a topic. |
| `/tracking_error` | Publish | `geometry_msgs/msg/Vector3Stamped` | Commanded minus actual NED position while `commandPosition()` is used. It is not published during velocity-controlled `Descending`. |

`/tracking_error` uses `frame_id = "odom"`, but its vector components follow
the PX4 local NED axes described below.

```bash
ros2 topic echo /drone_state
ros2 topic echo /tracking_error
```

The `px4_ros2` interface also creates the versioned PX4 registration, command,
control-mode, and trajectory-setpoint topics used internally by the mode and
executor.

## Coordinate Frames and Ground Reference

PX4 local position uses **NED** (North-East-Down): x is north, y is east, and z
increases downward. Climbing therefore makes z more negative, while a positive
z velocity commands descent.

![NED coordinate-frame model](assets/NED.excalidraw.svg)

The takeoff watcher computes its threshold relative to the latest local z
rather than assuming ground is always `z = 0`:

```cpp
const float takeoff_start_z = _have_local_position ? _latest_local_z : 0.0f;
_takeoff_target_z =
    takeoff_start_z - (_optical_flow_height - _delta_position);
```

While PX4 reports landed, the mode records the current z as `_ground_z`. If no
landed sample was received before activation, it infers ground from the reached
takeoff position plus `optical_flow_height`. During descent, the handoff plane
is `_ground_z - _landing_height`; PX4-native landing owns the remaining descent,
touchdown detection, and disarm.

## Writing Your Own Mode

1. Copy this package to `src/your_mode`. Rename the package in `package.xml`
   and `CMakeLists.txt`, including `project()`, the executable target, and the
   `install(TARGETS ...)` entry. ROS 2 package names use lowercase letters,
   numbers, and underscores.
2. Rename the C++ classes, namespace, node/executable, YAML top-level key, and
   launch-file package references together. A mismatch can build successfully
   while silently preventing parameters from loading.
3. Change `kExampleAutonomousModeName`. It must be unique among modes registered
   with the same autopilot and shorter than 25 characters (24 characters
   maximum).
4. Add each mission state to `ExampleAutonomousMode::State`, handle it in
   `updateSetpoint()`, and add its string in `stateName()`. Keep the switch
   exhaustive; this package's `-Werror` build helps catch omitted cases.
5. Use `switchToState()` for transitions so the elapsed-state timer and
   `/drone_state` stay correct. Call `commandPosition()` for position-controlled
   states so `/tracking_error` remains available.
6. Add each parameter in the header fallback, `loadParameters()`, and the YAML.
   If the node or package is renamed, update the launch file and YAML key too.
7. Put continuous mission behavior between `OpticalFlowSettling` and
   `Descending`. Extend the executor only when the mission needs another PX4
   operation around the custom mode.
8. Preserve the non-success behavior intentionally: a pilot takeover or
   failsafe should not be turned into an unconditional autonomous landing.

`TrajectorySetpointType` requests a 50 Hz update rate through this repository's
`px4_ros2` interface. `updateSetpoint(float dt_s)` receives measured elapsed
time, which can vary around 0.02 seconds. Use `dt_s` when integrating motion—for
example, `position.z() -= climb_rate * dt_s`—instead of assuming a fixed loop
period.
