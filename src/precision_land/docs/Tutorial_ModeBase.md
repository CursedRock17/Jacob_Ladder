# Tutorial: Writing a Custom Flight Mode with `ModeBase`

This tutorial walks through exactly what happens when you create a custom PX4 flight
mode using the `px4_ros2` library. By the end you should be able to write your own
mode from scratch by filling in a template with your own logic.

All examples come directly from this repository — so you can open the referenced
files alongside this doc.

---

## Background: What is an External Flight Mode?

When you fly a drone, PX4 runs in a **flight mode** — Position, Stabilized, Loiter,
Mission, etc. These are all built into the flight controller firmware.

The `px4_ros2` library lets you add your own mode that runs on the **companion
computer** (Raspberry Pi, Jetson, etc.) instead of inside PX4. PX4 still controls
the motors and handles low-level stabilization — but your ROS 2 node decides *where*
the drone goes.

Your mode looks just like a built-in mode to PX4. The pilot can select it from a GCS
or RC switch. If your ROS 2 node crashes or stops publishing, PX4 automatically
failsafes.

---

## The `ModeBase` Class

Every custom mode is a C++ class that **inherits from `px4_ros2::ModeBase`**:

```cpp
#include <px4_ros2/components/mode.hpp>

class MyMode : public px4_ros2::ModeBase
{
public:
    explicit MyMode(rclcpp::Node& node);

    void onActivate()  override;   // PX4 just selected your mode
    void onDeactivate() override;  // PX4 left your mode
    void updateSetpoint(float dt_s) override;  // ~50 Hz control loop
};
```

You must implement these three methods. Everything else is optional.

---

## Step 1: Constructor — Register the Mode and Set Up Interfaces

The constructor does three things:
1. Tells PX4 what to call this mode (its name)
2. Creates the **setpoint publisher** — how you move the drone
3. Creates **odometry subscribers** — how you read position/attitude

```cpp
MyMode::MyMode(rclcpp::Node& node)
    : ModeBase(node, ModeBase::Settings{"MyModeName"})  // ← mode name shown in GCS
    , _node(node)
{
    setSkipMessageCompatibilityCheck();  // skip version check during development

    // Setpoint publisher — sends trajectory commands to PX4
    _trajectory_setpoint = std::make_shared<px4_ros2::TrajectorySetpointType>(*this);

    // Odometry — read the drone's current position (NED frame)
    _vehicle_local_position = std::make_shared<px4_ros2::OdometryLocalPosition>(*this);

    // Odometry — read the drone's current orientation (quaternion)
    _vehicle_attitude = std::make_shared<px4_ros2::OdometryAttitude>(*this);
}
```

**Real example:** `PrecisionLand.cpp` lines 21–64  
The mode name `"PrecisionLandCustom"` is defined as a constant in the header:
```cpp
inline constexpr char kPrecisionLandModeName[] = "PrecisionLandCustom";
```
This is the string that appears in QGroundControl's flight mode dropdown.

### What is `TrajectorySetpointType`?

This is your "steering wheel." Calling its `update()` method sends position or
velocity commands to PX4 at the current tick. If you stop calling it, PX4 detects
the silence and failsafes.

The `px4_ros2` library automatically publishes the required `OffboardControlMode`
heartbeat for you — you don't need to manage that manually.

---

## Step 2: `onActivate()` — Reset State When Mode is Selected

PX4 calls `onActivate()` once when the pilot (or an executor) selects your mode.
Use it to snapshot the starting position and reset all state:

```cpp
void MyMode::onActivate()
{
    // Record where the drone is right now
    _start_position = _vehicle_local_position->positionNed();

    // Reset your state machine to the beginning
    _state = State::MyFirstState;

    // Clear any accumulated integrators, timers, etc.
    _elapsed = 0.0f;
}
```

**Real example:** `PrecisionLand.cpp` lines 142–159  
PrecisionLand snapshots `_base_position`, clears the PI integrators, and resets
`_search_started` so it ignores tag callbacks during takeoff.

> **Why snapshot position here and not in the constructor?**  
> The constructor runs when the node starts — the drone might not be flying yet.
> `onActivate()` runs the moment control is handed to your mode, when the drone is
> in the actual position you want as your reference.

---

## Step 3: `onDeactivate()` — Clean Up

Called when PX4 leaves your mode (pilot switches away, mode completes, failsafe, etc.).
Reset any running state:

```cpp
void MyMode::onDeactivate()
{
    _vel_x_integral = 0.f;  // clear PI integrators
    _vel_y_integral = 0.f;
}
```

Avoid heavy work here. Just zero out state that shouldn't carry over to the next
activation.

---

## Step 4: `updateSetpoint(float dt_s)` — Your Control Loop

This is called by PX4 at roughly 50 Hz while your mode is active. `dt_s` is the
time since the last call in seconds (typically ~0.02s). This is where your state
machine lives.

```cpp
void MyMode::updateSetpoint(float dt_s)
{
    _elapsed += dt_s;

    switch (_state) {

    case State::HoverInPlace: {
        // Command the drone to stay at the starting position
        _trajectory_setpoint->updatePosition(_start_position);

        // After 5 seconds, move to the next state
        if (_elapsed >= 5.0f) {
            _elapsed = 0.0f;
            _state = State::MoveNorth;
        }
        break;
    }

    case State::MoveNorth: {
        // Move 2m north of start
        Eigen::Vector3f target = _start_position;
        target.x() += 2.0f;
        _trajectory_setpoint->updatePosition(target);
        break;
    }

    case State::Finished: {
        // Signal PX4 that the mission is done — it will switch to Hold
        ModeBase::completed(px4_ros2::Result::Success);
        break;
    }

    } // end switch
}
```

**Real example:** `PrecisionLand.cpp` lines 168–311  
PrecisionLand's `updateSetpoint` runs a 6-state machine: init → climb → search →
approach → descend → finished. Each case uses `_trajectory_setpoint` to send either
a position command (early states) or a velocity command (descent).

### Sending Position vs. Velocity Commands

```cpp
// Position setpoint — "go to this NED coordinate"
_trajectory_setpoint->updatePosition(Eigen::Vector3f(x, y, z));

// Velocity setpoint — "move at this velocity"
_trajectory_setpoint->update(
    Eigen::Vector3f(vx, vy, vz),  // velocity in m/s (NED)
    std::nullopt,                  // acceleration (omit for velocity-only)
    yaw_rad                        // desired heading
);
```

> **NED reminder:** Z is positive downward. So `vz = +0.5f` means descend at 0.5 m/s,
> and `z = -2.0f` means 2 meters above the ground.

---

## Step 5: Signaling Completion or Failure

When your mission is done, call `ModeBase::completed()`. PX4 will switch back to
Hold (or whatever is configured as the post-mode behavior):

```cpp
// Mission succeeded
ModeBase::completed(px4_ros2::Result::Success);

// Mission failed (e.g., lost target)
ModeBase::completed(px4_ros2::Result::ModeFailureOther);
```

> **Guard with a flag!** `updateSetpoint` is called every tick. Without a guard,
> `completed()` gets called 50 times per second. Use a boolean to fire it exactly once:

```cpp
case State::Finished: {
    if (!_done) {
        _done = true;
        ModeBase::completed(px4_ros2::Result::Success);
    }
    break;
}
```

---

## Step 6: The `main()` Function — Spin It Up

The entry point uses the `NodeWithMode` template to register your class with PX4
and spin the ROS 2 executor:

```cpp
int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<px4_ros2::NodeWithMode<MyMode>>(
        "MyModeName",  // must match the name in Settings{}
        true           // enable debug output
    ));
    rclcpp::shutdown();
    return 0;
}
```

**Real example:** `PrecisionLand.cpp` lines 491–499

---

## Complete Skeleton

Here is a minimal template you can copy and fill in:

```cpp
// MyMode.hpp
#pragma once
#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/control/setpoint_types/experimental/trajectory.hpp>
#include <px4_ros2/odometry/local_position.hpp>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Core>

inline constexpr char kMyModeName[] = "MyModeCustom";

class MyMode : public px4_ros2::ModeBase
{
public:
    explicit MyMode(rclcpp::Node& node);
    void onActivate()  override;
    void onDeactivate() override;
    void updateSetpoint(float dt_s) override;

private:
    enum class State { Phase1, Phase2, Finished };

    rclcpp::Node& _node;
    std::shared_ptr<px4_ros2::TrajectorySetpointType> _trajectory_setpoint;
    std::shared_ptr<px4_ros2::OdometryLocalPosition>  _vehicle_local_position;

    State _state = State::Phase1;
    Eigen::Vector3f _start_position{};
    float _elapsed = 0.f;
    bool _done = false;
};
```

```cpp
// MyMode.cpp
#include "MyMode.hpp"
#include <px4_ros2/components/node_with_mode.hpp>

MyMode::MyMode(rclcpp::Node& node)
    : ModeBase(node, ModeBase::Settings{kMyModeName})
    , _node(node)
{
    setSkipMessageCompatibilityCheck();
    _trajectory_setpoint     = std::make_shared<px4_ros2::TrajectorySetpointType>(*this);
    _vehicle_local_position  = std::make_shared<px4_ros2::OdometryLocalPosition>(*this);
}

void MyMode::onActivate()
{
    _start_position = _vehicle_local_position->positionNed();
    _state   = State::Phase1;
    _elapsed = 0.f;
    _done    = false;
}

void MyMode::onDeactivate() {}

void MyMode::updateSetpoint(float dt_s)
{
    _elapsed += dt_s;

    switch (_state) {
    case State::Phase1: {
        _trajectory_setpoint->updatePosition(_start_position);
        if (_elapsed >= 5.0f) { _elapsed = 0.f; _state = State::Phase2; }
        break;
    }
    case State::Phase2: {
        // TODO: your logic here
        _state = State::Finished;
        break;
    }
    case State::Finished: {
        if (!_done) { _done = true; ModeBase::completed(px4_ros2::Result::Success); }
        break;
    }
    }
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<px4_ros2::NodeWithMode<MyMode>>(
        kMyModeName, true));
    rclcpp::shutdown();
    return 0;
}
```

---

## Common Pitfalls

| Mistake | Fix |
|---------|-----|
| Calling `completed()` every tick | Guard with a `bool _done` flag |
| Using absolute altitude in position setpoints | Snapshot `_base_position` in `onActivate()` and offset from it |
| Forgetting NED sign | Up = negative Z. `target.z() = base.z() - height_meters` |
| Not resetting integrators in `onDeactivate()` | Zero all accumulated state so the next activation starts clean |
| Mode name mismatch between `Settings{}` and `NodeWithMode<>` | Use a shared `constexpr char kModeName[]` constant |

---

## What's Next

Once your mode works, you can make it fully autonomous by pairing it with a
`ModeExecutorBase`. The executor handles arming and disarming, so the drone can
complete an entire mission with no pilot input.

→ See [Tutorial_ModeExecutorBase.md](./Tutorial_ModeExecutorBase.md)
