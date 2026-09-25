# Tutorial: Autonomous Missions with `ModeExecutorBase`

A `ModeBase` mode becomes active only when a pilot (or GCS) selects it. That's fine
for in-flight use, but sometimes you want the drone to **handle everything by itself**:
power on, arm, take off, run your mission, land, and disarm — with no human input.

That's what `ModeExecutorBase` is for.

---

## What is an Executor?

An executor is a **state machine that orchestrates other modes**. It owns one custom
mode (your `ModeBase` subclass) and coordinates the full mission sequence using
built-in PX4 operations like `arm()`, `takeoff()`, and `disarm()`.

```
Executor state machine:
  Arming → Running (your custom mode) → Disarming
```

The executor activates automatically when the flight controller is ready (with
`ActivateAlways`), so launching the ROS 2 node is all you need to start the mission.

---

## How It Fits Together

```
┌──────────────────────────────────────────────────┐
│ NodeWithModeExecutor<Executor, Mode>             │
│                                                  │
│  ModeExecutorBase (Executor)                     │
│    onActivate() ──→ arm() ──→ scheduleMode() ──→ disarm()
│                                                  │
│  ModeBase (Mode)  ←── controlled by executor ──→ │
│    onActivate()                                  │
│    updateSetpoint()   ← runs at ~50 Hz           │
│    onDeactivate()                                │
└──────────────────────────────────────────────────┘
```

The executor calls **asynchronous methods** that each take a completion callback.
When the operation finishes, the callback fires and you transition to the next state.

---

## Step 1: Define the Executor Class

```cpp
// MyModeAuto.hpp  (executor section)
#include <px4_ros2/components/mode_executor.hpp>

class MyModeExecutor : public px4_ros2::ModeExecutorBase
{
public:
    MyModeExecutor(rclcpp::Node& node, px4_ros2::ModeBase& owned_mode);

    enum class State { Arming, Running, Disarming };

    void onActivate() override;
    void onDeactivate(DeactivateReason reason) override;

private:
    void runState(State state, px4_ros2::Result result);
    rclcpp::Node& _node;
};
```

The `owned_mode` is an instance of your `ModeBase` subclass. The executor
controls when that mode runs via `scheduleMode(ownedMode().id(), callback)`.

---

## Step 2: Constructor — Activation Policy

```cpp
MyModeExecutor::MyModeExecutor(rclcpp::Node& node, px4_ros2::ModeBase& owned_mode)
    : ModeExecutorBase(node,
        ModeExecutorBase::Settings{Settings::Activation::ActivateAlways},
        owned_mode)
    , _node(node)
{
    setSkipMessageCompatibilityCheck();
}
```

`Activation::ActivateAlways` means the executor activates as soon as the flight
controller reports ready. It will immediately start `onActivate()` and begin the
mission sequence.

---

## Step 3: `onActivate()` — Kick Off the Sequence

```cpp
void MyModeExecutor::onActivate()
{
    RCLCPP_INFO(_node.get_logger(), "Executor active — arming");
    runState(State::Arming, px4_ros2::Result::Success);
}
```

`runState` is your own helper that acts as the state machine dispatcher.

---

## Step 4: `runState()` — The Async State Machine

This is the heart of the executor. Each case calls an **asynchronous operation**
and passes a lambda that moves to the next state when it finishes:

```cpp
void MyModeExecutor::runState(State state, px4_ros2::Result result)
{
    // If a previous step failed, stop and log the error
    if (result != px4_ros2::Result::Success) {
        RCLCPP_ERROR(_node.get_logger(), "Step %i failed: %s",
            (int)state, resultToString(result));
        return;
    }

    switch (state) {

    case State::Arming:
        // arm() sends an arm command to PX4.
        // The lambda fires when PX4 confirms the drone is armed.
        arm([this](px4_ros2::Result r) {
            runState(State::Running, r);
        });
        break;

    case State::Running:
        // scheduleMode() tells PX4 to switch to our custom mode.
        // The lambda fires when our mode calls ModeBase::completed().
        RCLCPP_INFO(_node.get_logger(), "Armed — starting mission");
        scheduleMode(ownedMode().id(), [this](px4_ros2::Result r) {
            runState(State::Disarming, r);
        });
        break;

    case State::Disarming:
        // disarm() sends a disarm command to PX4.
        // The lambda fires when the drone is confirmed disarmed.
        RCLCPP_INFO(_node.get_logger(), "Mission done — disarming");
        disarm([this](px4_ros2::Result r) {
            RCLCPP_INFO(_node.get_logger(), "Disarmed. Mission complete.");
        });
        break;
    }
}
```

**Real example:** `PrecisionLandAuto.cpp` — the `PrecisionLandAutoExecutor::runState`
method follows this exact pattern.

### Why Lambdas Instead of Polling?

Each async method (`arm`, `scheduleMode`, `disarm`) launches an operation and returns
immediately. The operation finishes at some point in the future (maybe 2 seconds for
arming, maybe 30 seconds for a mission). The lambda is stored and called when the
result arrives — there's no spinning or sleeping in your code.

This keeps the executor's code linear and readable even though the operations are
asynchronous:

```
arm ──→ (wait) ──→ lambda fires ──→ scheduleMode ──→ (wait) ──→ lambda fires ──→ disarm
```

---

## Step 5: `onDeactivate()` — Handle Interruption

Called if the executor is interrupted (pilot switches mode, failsafe, etc.):

```cpp
void MyModeExecutor::onDeactivate(DeactivateReason reason)
{
    // Usually empty — PX4 handles clean-up for you.
    // Add logging here if you want to know why you were deactivated.
}
```

---

## Step 6: Disarming from the Mode (No Executor)

If you don't use an executor but still need to disarm after landing, publish a
`VehicleCommand` directly. This is what `PrecisionLand.cpp` does:

```cpp
void MyMode::sendDisarm()
{
    px4_msgs::msg::VehicleCommand cmd{};
    cmd.timestamp  = _node.now().nanoseconds() / 1000;
    cmd.command    = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM;
    cmd.param1     = 0.0f;      // 0 = disarm
    cmd.param2     = 21196.0f;  // force-disarm: bypasses safety checks when landed
    cmd.target_system    = 1;
    cmd.target_component = 1;
    cmd.source_system    = 1;
    cmd.source_component = 1;
    cmd.from_external    = true;
    _vehicle_command_pub->publish(cmd);
}
```

Call this once (guarded by a flag) when the mode reaches its `Finished` state and
`_land_detected` is true.

> **Why `param2 = 21196.0f`?**  
> PX4 has safety checks that prevent disarming while a custom mode is holding
> control. `21196` is PX4's force-disarm unlock key that bypasses those checks.
> It's safe to use once the landing detector has confirmed touchdown.

---

## Step 7: The `main()` Function

Use `NodeWithModeExecutor` instead of `NodeWithMode`:

```cpp
#include <px4_ros2/components/node_with_mode.hpp>

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<px4_ros2::NodeWithModeExecutor<
        MyModeExecutor, MyMode>>(
        kMyModeName,  // mode name constant
        true          // debug output
    ));
    rclcpp::shutdown();
    return 0;
}
```

The template takes `<ExecutorClass, ModeClass>` in that order.

**Real example:** `PrecisionLandAuto.cpp` last 8 lines.

---

## Why Not Use `takeoff()` in the Executor?

The `ModeExecutorBase` has a `takeoff(callback, altitude_m)` helper that schedules
PX4's built-in Takeoff mode. However, PX4's Takeoff mode only calls `completed()`
once it reaches its internal `MIS_TAKEOFF_ALT` parameter (typically 2.5m). If you
pass a lower altitude (like 1.25m), the callback may never fire.

**Best practice for this repo:** Skip the executor's `takeoff()` entirely. Your
`ModeBase` mode can handle the climb in its own `OpticalFlowInit → Climbing` states
using trajectory setpoints. This way the flight altitude is controlled by the mode's
`target_height` parameter — not buried in the executor.

```cpp
// ✗ Fragile: callback may not fire if altitude < MIS_TAKEOFF_ALT
takeoff([this](px4_ros2::Result r) { runState(State::Running, r); }, 1.25f);

// ✓ Robust: arm, then hand control straight to your mode
arm([this](px4_ros2::Result r) { runState(State::Running, r); });
// ... then in State::Running:
scheduleMode(ownedMode().id(), callback);
```

---

## Complete Skeleton

```cpp
// MyModeAuto.hpp  (add to your existing mode header)
class MyModeExecutor : public px4_ros2::ModeExecutorBase
{
public:
    MyModeExecutor(rclcpp::Node& node, px4_ros2::ModeBase& owned_mode);
    enum class State { Arming, Running, Disarming };
    void onActivate() override;
    void onDeactivate(DeactivateReason reason) override;
private:
    void runState(State state, px4_ros2::Result result);
    rclcpp::Node& _node;
};
```

```cpp
// MyModeAuto.cpp  (executor implementation + main)
MyModeExecutor::MyModeExecutor(rclcpp::Node& node, px4_ros2::ModeBase& owned_mode)
    : ModeExecutorBase(node,
        ModeExecutorBase::Settings{Settings::Activation::ActivateAlways},
        owned_mode)
    , _node(node)
{
    setSkipMessageCompatibilityCheck();
}

void MyModeExecutor::onActivate()
{
    runState(State::Arming, px4_ros2::Result::Success);
}

void MyModeExecutor::onDeactivate(DeactivateReason reason) {}

void MyModeExecutor::runState(State state, px4_ros2::Result result)
{
    if (result != px4_ros2::Result::Success) {
        RCLCPP_ERROR(_node.get_logger(), "State %i failed: %s",
            (int)state, resultToString(result));
        return;
    }
    switch (state) {
    case State::Arming:
        arm([this](px4_ros2::Result r) { runState(State::Running, r); });
        break;
    case State::Running:
        scheduleMode(ownedMode().id(), [this](px4_ros2::Result r) {
            runState(State::Disarming, r);
        });
        break;
    case State::Disarming:
        disarm([this](px4_ros2::Result r) {
            RCLCPP_INFO(_node.get_logger(), "Complete.");
        });
        break;
    }
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<px4_ros2::NodeWithModeExecutor<
        MyModeExecutor, MyMode>>(kMyModeName, true));
    rclcpp::shutdown();
    return 0;
}
```

---

## Quick Reference

| Operation | Method | When the callback fires |
|-----------|--------|------------------------|
| Arm the drone | `arm(callback)` | PX4 confirms armed |
| Run your custom mode | `scheduleMode(id, callback)` | Your mode calls `ModeBase::completed()` |
| Disarm the drone | `disarm(callback)` | PX4 confirms disarmed |
| Wait for natural disarm | `waitUntilDisarmed(callback)` | PX4 auto-disarms (may take a long time) |

---

## Checklist: ModeBase + Executor Together

- [ ] Mode header: inherits `ModeBase`, has `onActivate`, `onDeactivate`, `updateSetpoint`
- [ ] Mode constructor: creates `TrajectorySetpointType` and `OdometryLocalPosition`
- [ ] Mode `onActivate()`: snapshots position, resets all state
- [ ] Mode `updateSetpoint()`: calls `_trajectory_setpoint` every tick; calls `completed()` (guarded) at the end
- [ ] Executor header: inherits `ModeExecutorBase`, defines `State` enum with at least `Arming`, `Running`, `Disarming`
- [ ] Executor constructor: uses `Activation::ActivateAlways`
- [ ] Executor `runState()`: `arm()` → `scheduleMode()` → `disarm()`
- [ ] `main()`: uses `NodeWithModeExecutor<Executor, Mode>`
- [ ] CMakeLists: separate `add_executable` target for the auto variant
