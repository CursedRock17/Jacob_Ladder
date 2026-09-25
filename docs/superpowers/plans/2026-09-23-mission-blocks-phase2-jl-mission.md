# Mission Blocks Phase 2: `jl_mission` relay mode and executor — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build `jl_mission`, the one C++ PX4 external mode behind every mission file. It relays the Python runner's setpoints after checking them against the spec §5 safety contract, and it owns arm, takeoff and land. `jl_mission_interfaces` defines the takeoff service the runner calls.

**Architecture:**
- **`SetpointGuard`:** validation, velocity clamping and the silence watchdog, as plain C++ functions (Eigen only), unit-tested with gtest.
- **`RelayMode`** (a `px4_ros2::ModeBase`, named from the `mission_name` parameter so it appears in QGC) runs in three phases. In **Idle** it holds position. In **Climb** it holds the takeoff target. In **Relay** it forwards guarded setpoints: 0.5 s of silence means hold, and 5 s means completing with a failure so the executor lands.
- **`MissionExecutor`** serves `/jl/NAME/takeoff` and `/jl/NAME/land` with deferred responses. Takeoff is arm, then PX4's native takeoff, then a Climb phase in our own mode to the exact height. The takeoff request is answered only once the drone is at that height.

**Tech Stack:** C++17, ROS 2 Humble (rclcpp, rosidl), px4_ros2_cpp (px4-ros2-interface-lib 1.6.0 fork), Eigen3, ament_cmake_gtest, PX4 v1.16.0 SITL + Gazebo Harmonic in the `jacob_ladder_sim` container, clang-format 18 (LLVM style, 2-space indent).

**Spec:** `docs/superpowers/specs/2026-09-21-mission-blocks-design.md`. This plan implements phase 2 of section 9: the §5 safety contract, the §2 package layout for `jl_mission` and `jl_mission_interfaces`, and the L1 (C++) row of §8.

## Global Constraints

- Do NOT modify `src/precision_land/`, `src/drogue_flight/` or any other existing flight code. `jl_mission` is new code. Its debug publishers are copies of precision_land's, adapted, not shared.
- **Do not commit.** The maintainer makes all commits. Each task ends with a checkpoint: stop and report the diff. Never run `git add/commit/stash/checkout/reset/restore`.
- Work only on `main`; never add yourself to commit history.
- Safety-contract defaults (spec §5), all ROS parameters: `silence_hold_s` 0.5, `silence_land_s` 5.0, `max_step_m` 5.0, `max_speed` 1.0 m/s.
- Topics and services per mission `NAME`:
  - `/jl/NAME/setpoint` (`px4_msgs/TrajectorySetpoint`, sensor-data QoS)
  - `/jl/NAME/active` (`std_msgs/Bool`, transient-local)
  - `/jl/NAME/state` (`std_msgs/String`, transient-local, re-sent at 1 Hz)
  - `/jl/NAME/takeoff` (`jl_mission_interfaces/srv/Takeoff`)
  - `/jl/NAME/land` (`std_srvs/srv/Trigger`)
  - `/tracking_error` (`geometry_msgs/Vector3Stamped`)
- `mission_name` must be 1–24 characters, the PX4 limit for external mode names.
- C++ builds with `-Wall -Wextra -Wpedantic -Werror -Wno-unused-parameter`, matching the other packages. It is formatted with `clang-format` using the package's `.clang-format` (LLVM, 2-space indent, 80 columns). The code below is already formatted.
- Build and test inside the sim container:
  `docker exec --user user -w /hdd/Documents/Electrical/Matrix_Lab/jacob_drone_ws/src/Jacob_Ladder jacob_ladder_sim bash -c 'source /opt/ros/humble/setup.bash && ...'`
  If the container isn't running, start it with `docker start jacob_ladder_sim`. If it doesn't exist, create it with `./docker/run_sim_container.sh`.

## File Structure

```
src/jl_mission_interfaces/
├── package.xml, CMakeLists.txt     rosidl interface package
└── srv/Takeoff.srv                 float32 height -> bool success, string message
src/jl_mission/
├── package.xml, CMakeLists.txt
├── .clang-format                   LLVM style (copy of example_autonomous_mode's)
├── include/jl_mission/
│   ├── setpoint_guard.hpp          safety contract as pure functions (no ROS)
│   ├── debug_publishers.hpp        StatePublisher(topic), TrackingErrorPublisher
│   └── mission_mode.hpp            RelayMode, MissionExecutor
├── src/
│   ├── setpoint_guard.cpp
│   ├── mission_mode.cpp
│   └── main.cpp                    wait-for-FMU + registration retry
├── launch/jl_mission.launch.py     one instance: mission_name:=NAME
└── test/
    ├── test_setpoint_guard.cpp     gtest, 9 tests
    ├── fake_runner.py              stand-in for the Phase 3 mission_runner
    └── sitl_jl_mission.sh          headless SITL check of the whole contract
Makefile                            + sitl-test target
README.md                           + package table row
docs/superpowers/specs/...design.md + §5 takeoff wording, §11 notes
```

---

### Task 1: `jl_mission_interfaces` (the takeoff service)

**Files:**
- Create: `src/jl_mission_interfaces/srv/Takeoff.srv`, `src/jl_mission_interfaces/package.xml`, `src/jl_mission_interfaces/CMakeLists.txt`

**Interfaces:**
- Consumes: nothing.
- Produces: `jl_mission_interfaces/srv/Takeoff`, with request `float32 height` and response `bool success, string message`. The generated C++ header is `<jl_mission_interfaces/srv/takeoff.hpp>`, the type is `jl_mission_interfaces::srv::Takeoff`, and the Python import is `from jl_mission_interfaces.srv import Takeoff`.

- [ ] **Step 1: Confirm the interface doesn't exist yet (RED)**

Run (inside the container, from the repo root): `ros2 interface show jl_mission_interfaces/srv/Takeoff`
Expected: an error that the package or interface is unknown.

- [ ] **Step 2: Create the package**

`src/jl_mission_interfaces/srv/Takeoff.srv`:

```
# Take off to `height` metres above the takeoff point, then hand over to the mission.
float32 height
---
bool success
string message
```

`src/jl_mission_interfaces/package.xml`:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>jl_mission_interfaces</name>
  <version>0.1.0</version>
  <description>Service definitions between the jl_blocks mission runner and the jl_mission external mode</description>
  <maintainer email="mtglucas1@gmail.com">Lucas Wendland</maintainer>
  <license>BSD-3-Clause</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <buildtool_depend>rosidl_default_generators</buildtool_depend>
  <exec_depend>rosidl_default_runtime</exec_depend>
  <member_of_group>rosidl_interface_packages</member_of_group>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

`src/jl_mission_interfaces/CMakeLists.txt`:

```cmake
cmake_minimum_required(VERSION 3.8)
project(jl_mission_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/Takeoff.srv"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
```

- [ ] **Step 3: Build and verify (GREEN)**

Run: `colcon build --packages-select jl_mission_interfaces && source install/setup.bash && ros2 interface show jl_mission_interfaces/srv/Takeoff`
Expected: the build finishes and the output shows the `.srv` text with `float32 height` above `---`.

- [ ] **Step 4: Checkpoint**

Stop and report the diff (`git status --short`) to the maintainer. Do not commit.

---

### Task 2: `jl_mission` package and the `SetpointGuard` (gtest)

**Files:**
- Create: `src/jl_mission/package.xml`, `src/jl_mission/CMakeLists.txt` (guard-only version below; Task 3 replaces it), `src/jl_mission/.clang-format`, `src/jl_mission/include/jl_mission/setpoint_guard.hpp`, `src/jl_mission/src/setpoint_guard.cpp`
- Test: `src/jl_mission/test/test_setpoint_guard.cpp`

**Interfaces:**
- Consumes: Eigen3 only (no ROS, no PX4).
- Produces (namespace `jl_mission`):
  - `struct GuardLimits { float silence_hold_s{0.5f}; float silence_land_s{5.0f}; float max_step_m{5.0f}; float max_speed{1.0f}; }`
  - `struct Command { std::optional<Eigen::Vector3f> position; std::optional<Eigen::Vector3f> velocity; std::optional<float> yaw; }`
  - `struct Checked { bool ok{false}; Command command; std::string reason; }`
  - `Checked fromPx4(const float position[3], const float velocity[3], float yaw)`
  - `Checked check(const Command&, const Eigen::Vector3f& current, const GuardLimits&)`
  - `enum class Action { Relay, Hold, Land }`, and `Action watchdog(double seconds_since_valid, const GuardLimits&)`
  - A CMake library target `setpoint_guard` that exports the `include/` directory.

- [ ] **Step 1: Write the failing tests**

`src/jl_mission/test/test_setpoint_guard.cpp`:

```cpp
#include "jl_mission/setpoint_guard.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

using jl_mission::Action;
using jl_mission::Command;
using jl_mission::GuardLimits;

namespace {
const float kNan = std::numeric_limits<float>::quiet_NaN();
const float kInf = std::numeric_limits<float>::infinity();
const Eigen::Vector3f kHere{0.f, 0.f, -1.f};
} // namespace

TEST(FromPx4, PositionOnlyWithNanVelocityIsAccepted) {
  const float pos[3] = {1.f, 2.f, -1.5f};
  const float vel[3] = {kNan, kNan, kNan};
  const auto result = jl_mission::fromPx4(pos, vel, kNan);
  ASSERT_TRUE(result.ok);
  EXPECT_TRUE(result.command.position.has_value());
  EXPECT_FALSE(result.command.velocity.has_value());
  EXPECT_FALSE(result.command.yaw.has_value());
}

TEST(FromPx4, PartlyNanVectorIsRejected) {
  const float pos[3] = {1.f, kNan, -1.5f};
  const float vel[3] = {kNan, kNan, kNan};
  const auto result = jl_mission::fromPx4(pos, vel, kNan);
  EXPECT_FALSE(result.ok);
  EXPECT_EQ(result.reason, "position has some axes set and some NaN");
}

TEST(FromPx4, InfinityAnywhereIsRejected) {
  const float pos[3] = {1.f, 2.f, -1.5f};
  const float vel[3] = {kNan, kNan, kNan};
  EXPECT_FALSE(jl_mission::fromPx4(pos, vel, kInf).ok);
  const float bad_pos[3] = {kInf, 0.f, -1.f};
  EXPECT_FALSE(jl_mission::fromPx4(bad_pos, vel, kNan).ok);
}

TEST(Check, CommandWithNeitherPositionNorVelocityIsRejected) {
  Command command;
  command.yaw = 0.f;
  const auto result = jl_mission::check(command, kHere, GuardLimits{});
  EXPECT_FALSE(result.ok);
  EXPECT_EQ(result.reason, "controls neither position nor velocity");
}

TEST(Check, PositionTooFarFromTheVehicleIsRejected) {
  Command command;
  command.position = Eigen::Vector3f{6.f, 0.f, -1.f};
  const auto result = jl_mission::check(command, kHere, GuardLimits{});
  EXPECT_FALSE(result.ok);
  EXPECT_EQ(result.reason, "position is 6.0 m away (limit 5.0 m)");
}

TEST(Check, PositionWithinMaxStepIsAccepted) {
  Command command;
  command.position = Eigen::Vector3f{4.f, 0.f, -1.f};
  EXPECT_TRUE(jl_mission::check(command, kHere, GuardLimits{}).ok);
}

TEST(Check, FastVelocityIsClampedToMaxSpeedKeepingDirection) {
  Command command;
  command.velocity = Eigen::Vector3f{3.f, 4.f, 0.f}; // 5 m/s
  const auto result = jl_mission::check(command, kHere, GuardLimits{});
  ASSERT_TRUE(result.ok);
  EXPECT_NEAR(result.command.velocity->norm(), 1.f, 1e-5f);
  EXPECT_NEAR(result.command.velocity->x(), 0.6f, 1e-5f);
  EXPECT_NEAR(result.command.velocity->y(), 0.8f, 1e-5f);
}

TEST(Check, SlowVelocityIsUnchanged) {
  Command command;
  command.velocity = Eigen::Vector3f{0.3f, 0.f, 0.f};
  const auto result = jl_mission::check(command, kHere, GuardLimits{});
  ASSERT_TRUE(result.ok);
  EXPECT_FLOAT_EQ(result.command.velocity->x(), 0.3f);
}

TEST(Watchdog, RelaysUntilSilenceHoldThenHoldsThenLands) {
  const GuardLimits limits{};
  EXPECT_EQ(jl_mission::watchdog(0.0, limits), Action::Relay);
  EXPECT_EQ(jl_mission::watchdog(0.49, limits), Action::Relay);
  EXPECT_EQ(jl_mission::watchdog(0.5, limits), Action::Hold);
  EXPECT_EQ(jl_mission::watchdog(4.99, limits), Action::Hold);
  EXPECT_EQ(jl_mission::watchdog(5.0, limits), Action::Land);
}
```

`src/jl_mission/include/jl_mission/setpoint_guard.hpp` (the interface under test):

```cpp
#pragma once

// The jl_mission safety contract (spec section 5) as plain functions, so it can
// be unit-tested without ROS or PX4. The relay mode calls these every tick.

#include <Eigen/Core>

#include <optional>
#include <string>

namespace jl_mission {

struct GuardLimits {
  float silence_hold_s{0.5f}; // no valid setpoint this long -> hold position
  float silence_land_s{5.0f}; // still nothing this long -> land
  float max_step_m{5.0f};     // reject positions farther than this from here
  float max_speed{1.0f};      // m/s, velocity setpoints are clamped to this
};

// One setpoint from the runner. An empty field means "not controlled".
struct Command {
  std::optional<Eigen::Vector3f> position; // NED, metres
  std::optional<Eigen::Vector3f> velocity; // NED, m/s
  std::optional<float> yaw;                // radians
};

// The result of checking a Command: ok with a (possibly clamped) command, or
// a reason it was rejected.
struct Checked {
  bool ok{false};
  Command command;
  std::string reason;
};

// Build a Command from PX4-style arrays where NaN means "not controlled".
// A vector with some NaN and some finite axes, or any infinity, is rejected.
Checked fromPx4(const float position[3], const float velocity[3], float yaw);

// Apply the limits: reject a position farther than max_step_m from `current`,
// reject a command that controls neither position nor velocity, and clamp the
// velocity to max_speed.
Checked check(const Command &command, const Eigen::Vector3f &current,
              const GuardLimits &limits);

enum class Action { Relay, Hold, Land };

// What to do given how long it has been since the last valid setpoint.
Action watchdog(double seconds_since_valid, const GuardLimits &limits);

} // namespace jl_mission
```

`src/jl_mission/package.xml` (final; its dependencies cover Task 3 too):

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>jl_mission</name>
  <version>0.1.0</version>
  <description>The one C++ PX4 external mode behind every jl_blocks mission: relays checked setpoints, owns arm/takeoff/land</description>
  <maintainer email="mtglucas1@gmail.com">Lucas Wendland</maintainer>
  <license>BSD-3-Clause</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <buildtool_depend>eigen3_cmake_module</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>px4_ros2_cpp</depend>
  <depend>px4_msgs</depend>
  <depend>std_msgs</depend>
  <depend>std_srvs</depend>
  <depend>geometry_msgs</depend>
  <depend>jl_mission_interfaces</depend>
  <depend>eigen</depend>

  <test_depend>ament_cmake_gtest</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

`src/jl_mission/CMakeLists.txt` (guard-only for now):

```cmake
cmake_minimum_required(VERSION 3.8)
project(jl_mission)

set(CMAKE_CXX_STANDARD 17)
add_compile_options(-Wall -Wextra -Wpedantic -Werror -Wno-unused-parameter)

find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

# The safety contract as plain C++ (Eigen only), so it is unit-tested without ROS
add_library(setpoint_guard src/setpoint_guard.cpp)
target_include_directories(setpoint_guard PUBLIC include ${EIGEN3_INCLUDE_DIR})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_setpoint_guard test/test_setpoint_guard.cpp)
  target_link_libraries(test_setpoint_guard setpoint_guard)
endif()

ament_package()
```

`src/jl_mission/.clang-format`:

```yaml
# This package is formatted with clang-format, not the astyle setup the older
# packages use (see the `format` target in the repository Makefile). The two
# styles disagree about braces and indentation, so keeping this file here scopes
# clang-format to this package and stops the two tools fighting over it.
#
#   clang-format -i src/jl_mission/src/*.cpp src/jl_mission/include/jl_mission/*.hpp src/jl_mission/test/*.cpp
---
BasedOnStyle: LLVM
IndentWidth: 2
ColumnLimit: 80
```

- [ ] **Step 2: Run to verify it fails**

Run: `colcon build --packages-select jl_mission`
Expected: FAIL. CMake reports `Cannot find source file: src/setpoint_guard.cpp`, because the implementation doesn't exist yet.

- [ ] **Step 3: Implement the guard**

`src/jl_mission/src/setpoint_guard.cpp`:

```cpp
#include "jl_mission/setpoint_guard.hpp"

#include <cmath>
#include <cstdio>

namespace jl_mission {

namespace {

// Read one PX4-style vector: all-NaN means "not controlled", all-finite is a
// value, anything else (mixed NaN, or an infinity) is an error.
bool readVector(const float v[3], const char *name,
                std::optional<Eigen::Vector3f> &out, std::string &reason) {
  int nan_count = 0;
  for (int i = 0; i < 3; ++i) {
    if (std::isinf(v[i])) {
      reason = std::string(name) + " contains infinity";
      return false;
    }
    if (std::isnan(v[i])) {
      ++nan_count;
    }
  }
  if (nan_count == 3) {
    out.reset();
    return true;
  }
  if (nan_count > 0) {
    reason = std::string(name) + " has some axes set and some NaN";
    return false;
  }
  out = Eigen::Vector3f{v[0], v[1], v[2]};
  return true;
}

} // namespace

Checked fromPx4(const float position[3], const float velocity[3], float yaw) {
  Checked result;
  if (!readVector(position, "position", result.command.position,
                  result.reason) ||
      !readVector(velocity, "velocity", result.command.velocity,
                  result.reason)) {
    return result;
  }
  if (std::isinf(yaw)) {
    result.reason = "yaw is infinite";
    return result;
  }
  if (!std::isnan(yaw)) {
    result.command.yaw = yaw;
  }
  result.ok = true;
  return result;
}

Checked check(const Command &command, const Eigen::Vector3f &current,
              const GuardLimits &limits) {
  Checked result;
  result.command = command;
  if (!command.position && !command.velocity) {
    result.reason = "controls neither position nor velocity";
    return result;
  }
  if (command.position) {
    const float distance = (*command.position - current).norm();
    if (distance > limits.max_step_m) {
      char text[96];
      std::snprintf(text, sizeof(text),
                    "position is %.1f m away (limit %.1f m)", distance,
                    limits.max_step_m);
      result.reason = text;
      return result;
    }
  }
  if (command.velocity) {
    const float speed = command.velocity->norm();
    if (speed > limits.max_speed) {
      result.command.velocity = *command.velocity * (limits.max_speed / speed);
    }
  }
  result.ok = true;
  return result;
}

Action watchdog(double seconds_since_valid, const GuardLimits &limits) {
  if (seconds_since_valid >= limits.silence_land_s) {
    return Action::Land;
  }
  if (seconds_since_valid >= limits.silence_hold_s) {
    return Action::Hold;
  }
  return Action::Relay;
}

} // namespace jl_mission
```

- [ ] **Step 4: Run to verify it passes**

Run: `colcon build --packages-select jl_mission && build/jl_mission/test_setpoint_guard`
Expected: the build is clean (no warnings, because of `-Werror`), and gtest prints `[  PASSED  ] 9 tests.`

Run (from the host, in the repo): `clang-format --dry-run -Werror src/jl_mission/src/*.cpp src/jl_mission/include/jl_mission/*.hpp src/jl_mission/test/*.cpp`
Expected: no output, exit code 0.

- [ ] **Step 5: Checkpoint**

Stop and report the diff to the maintainer. Do not commit.

---

### Task 3: `RelayMode`, `MissionExecutor`, and the SITL contract check

**Files:**
- Create: `src/jl_mission/include/jl_mission/debug_publishers.hpp`, `src/jl_mission/include/jl_mission/mission_mode.hpp`, `src/jl_mission/src/mission_mode.cpp`, `src/jl_mission/src/main.cpp`, `src/jl_mission/launch/jl_mission.launch.py`
- Modify: `src/jl_mission/CMakeLists.txt` (replace it with the full version below)
- Test: `src/jl_mission/test/fake_runner.py`, `src/jl_mission/test/sitl_jl_mission.sh`

**Interfaces:**
- Consumes:
  - `GuardLimits`, `Command`, `fromPx4`, `check`, `watchdog` (Task 2)
  - `jl_mission_interfaces::srv::Takeoff` (Task 1)
  - px4_ros2: `ModeBase`, `ModeExecutorBase` (`arm`, `takeoff(cb, altitude_amsl)`, `land`, `scheduleMode`, `isInCharge`, `isArmed`, `ownedMode`), `OdometryLocalPosition`, `OdometryGlobalPosition`, `LandDetected`, `TrajectorySetpointType`, `NodeWithModeExecutor`, `waitForFMU`
- Produces:
  - Executable `jl_mission`, parameterised by `mission_name` (1–24 characters) plus the §5 limit parameters, `climb_tolerance_m` (0.1) and `takeoff_timeout_s` (30).
  - The topics and services listed in Global Constraints.
  - `/jl/NAME/state` values:
    - `idle`
    - `waiting for takeoff request`
    - `climbing`
    - `relay`
    - `hold (no valid setpoint)`
    - `landing (no valid setpoint)`
    - `landing`
    - `landed`
    - `land failed`
    - `inactive`
  - Launch file `jl_mission.launch.py` (argument `mission_name`; node name `jl_mission_<lowercase name>`).

Notes for the implementer:
- **Why the Climb phase:** PX4's `takeoff(altitude)` takes an AMSL altitude and reports "complete" within `NAV_MC_ALT_RAD`, 0.8 m by default and on the real drone. The executor therefore passes current AMSL plus height when a global position exists, and NaN otherwise, which makes PX4 use `MIS_TAKEOFF_ALT`. It then climbs the rest of the way in `RelayMode`'s Climb phase, and answers the takeoff request only once `|z − target| ≤ climb_tolerance_m`.
- **Idle phase:** it exists so that arming, which briefly activates the mode on the ground, never starts the silence watchdog. `onActivate()` must not overwrite a Climb target; that bug made the first prototype hover at 0.7 m.
- **Where the subscriptions live:** `ModeExecutorBase` isn't a px4_ros2 `Context`, so the global-position and land-detected subscriptions live in `RelayMode`, and the executor reads them through `altitudeAmsl()` and `landed()`.

- [ ] **Step 1: Write the SITL check first**

`src/jl_mission/test/fake_runner.py` (make it executable):

```python
#!/usr/bin/env python3
"""Stand-in for the Phase 3 mission_runner, used by sitl_jl_mission.sh.

Waits for /jl/NAME/active, asks for a takeoff, then streams a scripted
sequence of setpoints (seconds after the takeoff reply):
   0-8   hold the takeoff position
   8-16  1 m north of it
  16-18  an invalid setpoint (infinite x): must be rejected, so jl_mission holds
  18-    nothing at all: jl_mission holds, then lands 5 s after the last valid one
Prints every /jl/NAME/state change and the takeoff reply.
"""

import math
import sys
import time

import rclpy
from jl_mission_interfaces.srv import Takeoff
from px4_msgs.msg import TrajectorySetpoint, VehicleLocalPosition
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from std_msgs.msg import Bool, String

NAN = float("nan")


class FakeRunner(Node):
    def __init__(self, name, height):
        super().__init__("fake_runner")
        self.height = height
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(
            TrajectorySetpoint, f"/jl/{name}/setpoint", qos_profile_sensor_data
        )
        self.client = self.create_client(Takeoff, f"/jl/{name}/takeoff")
        self.create_subscription(Bool, f"/jl/{name}/active", self.on_active, latched)
        self.create_subscription(String, f"/jl/{name}/state", self.on_state, latched)
        self.create_subscription(
            VehicleLocalPosition,
            "/fmu/out/vehicle_local_position",
            self.on_pos,
            qos_profile_sensor_data,
        )
        self.pos = None
        self.base = None
        self.t0 = None
        self.state = None
        self.asked = False
        self.create_timer(0.02, self.tick)

    def log(self, text):
        print(f"[{time.monotonic():.1f}] {text}", flush=True)

    def on_pos(self, msg):
        self.pos = (msg.x, msg.y, msg.z)

    def on_state(self, msg):
        if msg.data != self.state:
            self.state = msg.data
            self.log(f"STATE {msg.data}")

    def on_active(self, msg):
        if msg.data and not self.asked:
            self.asked = True
            self.log("active: requesting takeoff")
            req = Takeoff.Request()
            req.height = self.height
            self.client.wait_for_service()
            self.client.call_async(req).add_done_callback(self.on_takeoff)

    def on_takeoff(self, future):
        res = future.result()
        z = self.pos[2] if self.pos else NAN
        self.log(f"TAKEOFF success={res.success} message='{res.message}' z={z:.2f}")
        if res.success:
            self.base = self.pos
            self.t0 = time.monotonic()

    def tick(self):
        if self.t0 is None:
            return
        t = time.monotonic() - self.t0
        if t >= 18.0:
            return
        msg = TrajectorySetpoint()
        msg.velocity = [NAN, NAN, NAN]
        msg.yaw = NAN
        x, y, z = self.base
        if t < 8.0:
            msg.position = [x, y, z]
        elif t < 16.0:
            msg.position = [x + 1.0, y, z]
        else:
            msg.position = [math.inf, y, z]
        self.pub.publish(msg)


def main():
    rclpy.init()
    rclpy.spin(FakeRunner(sys.argv[1], float(sys.argv[2])))


if __name__ == "__main__":
    main()
```

`src/jl_mission/test/sitl_jl_mission.sh` (make it executable):

```bash
#!/bin/bash
# SITL check of the jl_mission safety contract (spec section 5), headless:
# takeoff to the exact height, relay, reject an invalid setpoint (hold), and
# land after 5 s of silence. Run inside the sim container after colcon build.
#   src/jl_mission/test/sitl_jl_mission.sh
HERE="$(dirname "$(readlink -f "$0")")"
source "$HERE/../../../jl_env.sh"
source /opt/ros/humble/setup.bash
source "$JL_WS_ROOT/install/setup.bash"
NAME=JlSitlCheck
LOGS=$(mktemp -d)
export GZ_PARTITION=sitl_jl_mission
PX4_BUILD="$JL_PX4_DIR/build/px4_sitl_default"
trap 'kill $(jobs -p) 2>/dev/null; pkill -f "gz sim"; pkill -f "$PX4_BUILD/bin/px4"' EXIT

# Fresh PX4 working dir (default params); -d: no interactive shell
mkdir -p "$LOGS/rootfs" && cp "$PX4_BUILD/rootfs/gz_env.sh" "$LOGS/rootfs/"
HEADLESS=1 PX4_SIM_MODEL=gz_x500 GZ_IP=127.0.0.1 \
  "$PX4_BUILD/bin/px4" -d -w "$LOGS/rootfs" "$PX4_BUILD/etc" > "$LOGS/px4.log" 2>&1 &
MicroXRCEAgent udp4 -p 8888 > "$LOGS/agent.log" 2>&1 &
ros2 run translation_node translation_node_bin > "$LOGS/translation.log" 2>&1 &
px4cmd() { (cd "$LOGS/rootfs" && "$PX4_BUILD/bin/px4-$1" "${@:2}") >> "$LOGS/commander.log" 2>&1; }
for _ in $(seq 120); do grep -q "synchronized with time offset" "$LOGS/px4.log" && break; sleep 1; done
px4cmd param set NAV_DLL_ACT 0  # no GCS in this test

ros2 run jl_mission jl_mission --ros-args -p mission_name:=$NAME > "$LOGS/mode.log" 2>&1 &
for _ in $(seq 60); do grep -q "Registered '$NAME'" "$LOGS/mode.log" && break; sleep 1; done
python3 "$HERE/fake_runner.py" $NAME 1.5 > "$LOGS/runner.log" 2>&1 &
sleep 2
px4cmd commander mode ext1

# The whole script takes about 45 s from selecting the mode to landed
for _ in $(seq 90); do grep -q "STATE landed" "$LOGS/runner.log" && break; sleep 1; done
cat "$LOGS/runner.log"

rc=0
expect() { if grep -q "$1" "$LOGS/runner.log"; then echo "PASS $2"; else echo "FAIL $2"; rc=1; fi; }
expect "TAKEOFF success=True" "takeoff reported success"
z=$(grep -o "TAKEOFF success=True .* z=-[0-9.]*" "$LOGS/runner.log" | grep -o "z=-[0-9.]*" | cut -c4-)
if python3 -c "import sys; sys.exit(0 if abs(float('${z:-0}') - 1.5) <= 0.15 else 1)"; then
  echo "PASS takeoff height ${z} m (want 1.5 +/- 0.15)"; else echo "FAIL takeoff height ${z:-none} m (want 1.5 +/- 0.15)"; rc=1; fi
expect "STATE relay" "relayed the runner's setpoints"
grep -q "Rejected setpoint: position contains infinity" "$LOGS/mode.log" && echo "PASS invalid setpoint rejected" || { echo "FAIL invalid setpoint rejected"; rc=1; }
expect "STATE hold (no valid setpoint)" "held on silence"
expect "STATE landing (no valid setpoint)" "landed after prolonged silence"
expect "STATE landed" "reached the ground"
[ $rc -ne 0 ] && echo "logs: $LOGS"
exit $rc
```

- [ ] **Step 2: Run it to verify it fails**

Run (inside the container, from the repo root, after `source install/setup.bash`): `src/jl_mission/test/sitl_jl_mission.sh`
Expected: FAIL on every line. `ros2 run jl_mission jl_mission` reports "No executable found", so nothing registers and the runner never sees `/jl/JlSitlCheck/active`.

- [ ] **Step 3: Implement the mode, executor and entry point**

`src/jl_mission/include/jl_mission/debug_publishers.hpp`:

```cpp
#pragma once

// Debug topics every jl_mission instance publishes, matching precision_land's
// StatePublisher and TrackingErrorPublisher (copied rather than shared, so the
// flight code in src/precision_land stays untouched).

#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <Eigen/Core>

#include <chrono>
#include <string>

namespace jl_mission {

// Latched state string, re-published at 1 Hz so a bag that starts mid-flight
// still contains it.
class StatePublisher {
public:
  StatePublisher(rclcpp::Node &node, const std::string &topic)
      : _pub(node.create_publisher<std_msgs::msg::String>(
            topic, rclcpp::QoS(1).transient_local())),
        _timer(node.create_wall_timer(std::chrono::seconds(1),
                                      [this] { publish(); })) {}

  void set(const std::string &state) {
    if (state == _state) {
      return;
    }
    _state = state;
    publish();
  }

private:
  void publish() {
    if (_state.empty()) {
      return;
    }
    std_msgs::msg::String msg;
    msg.data = _state;
    _pub->publish(msg);
  }

  std::string _state;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _pub;
  rclcpp::TimerBase::SharedPtr _timer;
};

// (commanded - actual) position on /tracking_error, NED.
class TrackingErrorPublisher {
public:
  explicit TrackingErrorPublisher(rclcpp::Node &node)
      : _node(node),
        _pub(node.create_publisher<geometry_msgs::msg::Vector3Stamped>(
            "/tracking_error", rclcpp::QoS(10))) {}

  void publish(const Eigen::Vector3f &commanded,
               const Eigen::Vector3f &actual) {
    geometry_msgs::msg::Vector3Stamped err;
    err.header.stamp = _node.now();
    err.header.frame_id = "odom";
    err.vector.x = commanded.x() - actual.x();
    err.vector.y = commanded.y() - actual.y();
    err.vector.z = commanded.z() - actual.z();
    _pub->publish(err);
  }

private:
  rclcpp::Node &_node;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr _pub;
};

} // namespace jl_mission
```

`src/jl_mission/include/jl_mission/mission_mode.hpp`:

```cpp
#pragma once

// jl_mission: the one C++ external mode behind every mission file.
//
// RelayMode passes the Python mission_runner's setpoints to PX4, but only after
// the SetpointGuard has checked them. MissionExecutor owns arm, takeoff and
// land, which the runner requests over two services. See spec section 5.

#include "jl_mission/debug_publishers.hpp"
#include "jl_mission/setpoint_guard.hpp"

#include <jl_mission_interfaces/srv/takeoff.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/components/mode_executor.hpp>
#include <px4_ros2/control/setpoint_types/experimental/trajectory.hpp>
#include <px4_ros2/odometry/global_position.hpp>
#include <px4_ros2/odometry/local_position.hpp>
#include <px4_ros2/vehicle_state/land_detected.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <memory>
#include <optional>
#include <string>

namespace jl_mission {

class RelayMode : public px4_ros2::ModeBase {
public:
  explicit RelayMode(rclcpp::Node &node);

  void onActivate() override;
  void onDeactivate() override;
  void updateSetpoint(float dt_s) override;

  // Called by the executor: hold `target` (no watchdog) until startRelay().
  void beginClimb(const Eigen::Vector3f &target);
  // Start relaying the runner's setpoints; the silence watchdog starts now.
  void startRelay();

  const std::string &missionName() const { return _name; }
  StatePublisher &state() { return _state; }
  Eigen::Vector3f position() const { return _local->positionNed(); }
  // Current altitude above mean sea level, if PX4 has a global position.
  std::optional<float> altitudeAmsl() const;
  bool landed() const;

private:
  // Idle: hold where we are (no watchdog) until the executor picks a phase.
  enum class Phase { Idle, Climb, Relay };

  void onSetpoint(const px4_msgs::msg::TrajectorySetpoint &msg);
  void send(const Command &command);

  rclcpp::Node &_node;
  std::string _name;
  GuardLimits _limits;
  StatePublisher _state;
  TrackingErrorPublisher _tracking_error;
  std::shared_ptr<px4_ros2::OdometryLocalPosition> _local;
  std::shared_ptr<px4_ros2::OdometryGlobalPosition> _global;
  std::shared_ptr<px4_ros2::LandDetected> _land_detected;
  std::shared_ptr<px4_ros2::TrajectorySetpointType> _setpoint;
  rclcpp::Subscription<px4_msgs::msg::TrajectorySetpoint>::SharedPtr _sub;

  Phase _phase{Phase::Idle};
  Eigen::Vector3f _hold{Eigen::Vector3f::Zero()};
  std::optional<Command> _last;
  rclcpp::Time _last_valid;
  bool _holding{false};
  bool _land_requested{false};
};

class MissionExecutor : public px4_ros2::ModeExecutorBase {
public:
  MissionExecutor(rclcpp::Node &node, RelayMode &mode);

  void onActivate() override;
  void onDeactivate(DeactivateReason reason) override;

private:
  using TakeoffSrv = jl_mission_interfaces::srv::Takeoff;
  using LandSrv = std_srvs::srv::Trigger;

  void onTakeoff(std::shared_ptr<rmw_request_id_t> header,
                 std::shared_ptr<TakeoffSrv::Request> request);
  void onLand(std::shared_ptr<rmw_request_id_t> header,
              std::shared_ptr<LandSrv::Request> request);
  void finishTakeoff(bool success, const std::string &message);
  void finishLand(bool success, const std::string &message);
  void checkClimb();
  void landNow(const std::string &why);

  rclcpp::Node &_node;
  RelayMode &_mode;
  float _climb_tolerance_m{0.1f};
  float _takeoff_timeout_s{30.f};

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr _active_pub;
  rclcpp::Service<TakeoffSrv>::SharedPtr _takeoff_srv;
  rclcpp::Service<LandSrv>::SharedPtr _land_srv;
  rclcpp::TimerBase::SharedPtr _climb_timer;

  std::shared_ptr<rmw_request_id_t> _takeoff_request;
  std::shared_ptr<rmw_request_id_t> _land_request;
  Eigen::Vector3f _climb_target{Eigen::Vector3f::Zero()};
  rclcpp::Time _takeoff_started;
};

} // namespace jl_mission
```

`src/jl_mission/src/mission_mode.cpp`:

```cpp
#include "jl_mission/mission_mode.hpp"

#include <cmath>

namespace jl_mission {

namespace {

// PX4 limits external mode names to 24 characters.
constexpr size_t kMaxModeNameLength = 24;

// The mode's name has to be known before ModeBase is constructed, so read the
// parameter here, in the initializer list.
std::string declareMissionName(rclcpp::Node &node) {
  const auto name =
      node.declare_parameter<std::string>("mission_name", "JlMission");
  if (name.empty() || name.size() > kMaxModeNameLength) {
    throw std::invalid_argument("mission_name must be 1-24 characters, got '" +
                                name + "'");
  }
  return name;
}

} // namespace

// ── RelayMode ──────────────────────────────────────────────────────────────

RelayMode::RelayMode(rclcpp::Node &node)
    : ModeBase(node, Settings{declareMissionName(node), false}), _node(node),
      _name(node.get_parameter("mission_name").as_string()),
      _state(node, "/jl/" + _name + "/state"), _tracking_error(node),
      _last_valid(node.now()) {
  // The executor skips the check too; the translation node converts messages.
  setSkipMessageCompatibilityCheck();

  _limits.silence_hold_s =
      node.declare_parameter<float>("silence_hold_s", _limits.silence_hold_s);
  _limits.silence_land_s =
      node.declare_parameter<float>("silence_land_s", _limits.silence_land_s);
  _limits.max_step_m =
      node.declare_parameter<float>("max_step_m", _limits.max_step_m);
  _limits.max_speed =
      node.declare_parameter<float>("max_speed", _limits.max_speed);

  _local = std::make_shared<px4_ros2::OdometryLocalPosition>(*this);
  _global = std::make_shared<px4_ros2::OdometryGlobalPosition>(*this);
  _land_detected = std::make_shared<px4_ros2::LandDetected>(*this);
  _setpoint = std::make_shared<px4_ros2::TrajectorySetpointType>(*this);

  _sub = node.create_subscription<px4_msgs::msg::TrajectorySetpoint>(
      "/jl/" + _name + "/setpoint", rclcpp::SensorDataQoS(),
      [this](const px4_msgs::msg::TrajectorySetpoint::SharedPtr msg) {
        onSetpoint(*msg);
      });
  _state.set("idle");
}

std::optional<float> RelayMode::altitudeAmsl() const {
  if (!_global->positionValid()) {
    return std::nullopt;
  }
  return static_cast<float>(_global->position().z());
}

bool RelayMode::landed() const {
  return _land_detected->lastValid() && _land_detected->landed();
}

void RelayMode::onActivate() {
  // A climb target set by the executor must survive activation.
  if (_phase != Phase::Climb) {
    _hold = _local->positionNed();
  }
  _last.reset();
  _last_valid = _node.now();
  _holding = false;
  _land_requested = false;
}

void RelayMode::onDeactivate() {
  _phase = Phase::Idle;
  _state.set("inactive");
}

void RelayMode::beginClimb(const Eigen::Vector3f &target) {
  _phase = Phase::Climb;
  _hold = target;
  _state.set("climbing");
}

void RelayMode::startRelay() {
  _phase = Phase::Relay;
  _last.reset();
  _last_valid = _node.now();
  _holding = false;
  _hold = _local->positionNed();
  _state.set("relay");
}

void RelayMode::onSetpoint(const px4_msgs::msg::TrajectorySetpoint &msg) {
  auto parsed = fromPx4(msg.position.data(), msg.velocity.data(), msg.yaw);
  if (parsed.ok) {
    parsed = check(parsed.command, _local->positionNed(), _limits);
  }
  if (!parsed.ok) {
    // A rejected setpoint counts as silence: the watchdog keeps running.
    RCLCPP_WARN_THROTTLE(_node.get_logger(), *_node.get_clock(), 1000,
                         "Rejected setpoint: %s", parsed.reason.c_str());
    return;
  }
  _last = parsed.command;
  _last_valid = _node.now();
}

void RelayMode::updateSetpoint(float /*dt_s*/) {
  if (_phase != Phase::Relay) {
    send(Command{_hold, std::nullopt, std::nullopt});
    return;
  }

  const double silence = (_node.now() - _last_valid).seconds();
  switch (watchdog(silence, _limits)) {
  case Action::Relay:
    if (_last) {
      _holding = false;
      _state.set("relay");
      send(*_last);
    } else {
      send(Command{_hold, std::nullopt, std::nullopt});
    }
    break;

  case Action::Hold:
    if (!_holding) {
      _holding = true;
      _hold = _local->positionNed();
      _state.set("hold (no valid setpoint)");
      RCLCPP_WARN(_node.get_logger(), "No valid setpoint for %.1f s: holding",
                  silence);
    }
    send(Command{_hold, std::nullopt, std::nullopt});
    break;

  case Action::Land:
    send(Command{_hold, std::nullopt, std::nullopt});
    if (!_land_requested) {
      _land_requested = true;
      _state.set("landing (no valid setpoint)");
      RCLCPP_ERROR(_node.get_logger(),
                   "No valid setpoint for %.1f s: handing over to land",
                   silence);
      // The executor sees this failure and lands.
      completed(px4_ros2::Result::ModeFailureOther);
    }
    break;
  }
}

void RelayMode::send(const Command &command) {
  px4_ros2::TrajectorySetpoint setpoint;
  if (command.position) {
    setpoint.withPosition(*command.position);
  }
  if (command.velocity) {
    setpoint.withVelocity(*command.velocity);
  }
  if (command.yaw) {
    setpoint.withYaw(*command.yaw);
  }
  _setpoint->update(setpoint);
  if (command.position) {
    _tracking_error.publish(*command.position, _local->positionNed());
  }
}

// ── MissionExecutor ────────────────────────────────────────────────────────

MissionExecutor::MissionExecutor(rclcpp::Node &node, RelayMode &mode)
    : ModeExecutorBase(node, Settings{Settings::Activation::ActivateAlways},
                       mode),
      _node(node), _mode(mode) {
  setSkipMessageCompatibilityCheck();

  _climb_tolerance_m =
      node.declare_parameter<float>("climb_tolerance_m", _climb_tolerance_m);
  _takeoff_timeout_s =
      node.declare_parameter<float>("takeoff_timeout_s", _takeoff_timeout_s);

  const std::string prefix = "/jl/" + mode.missionName();
  _active_pub = node.create_publisher<std_msgs::msg::Bool>(
      prefix + "/active", rclcpp::QoS(1).transient_local());
  _takeoff_srv = node.create_service<TakeoffSrv>(
      prefix + "/takeoff",
      [this](std::shared_ptr<rmw_request_id_t> header,
             std::shared_ptr<TakeoffSrv::Request> request) {
        onTakeoff(header, request);
      });
  _land_srv = node.create_service<LandSrv>(
      prefix + "/land", [this](std::shared_ptr<rmw_request_id_t> header,
                               std::shared_ptr<LandSrv::Request> request) {
        onLand(header, request);
      });

  std_msgs::msg::Bool inactive;
  inactive.data = false;
  _active_pub->publish(inactive);
}

void MissionExecutor::onActivate() {
  std_msgs::msg::Bool active;
  active.data = true;
  _active_pub->publish(active);
  _mode.state().set("waiting for takeoff request");
  RCLCPP_INFO(_node.get_logger(), "%s executor — active, waiting for runner",
              _mode.missionName().c_str());
}

void MissionExecutor::onDeactivate(DeactivateReason reason) {
  std_msgs::msg::Bool inactive;
  inactive.data = false;
  _active_pub->publish(inactive);
  _climb_timer.reset();
  const std::string why = reason == DeactivateReason::FailsafeActivated
                              ? "failsafe activated"
                              : "mission deactivated";
  finishTakeoff(false, why);
  finishLand(false, why);
}

void MissionExecutor::onTakeoff(std::shared_ptr<rmw_request_id_t> header,
                                std::shared_ptr<TakeoffSrv::Request> request) {
  auto reply = [this, header](bool success, const std::string &message) {
    TakeoffSrv::Response response;
    response.success = success;
    response.message = message;
    _takeoff_srv->send_response(*header, response);
  };
  if (!isInCharge()) {
    reply(false, "mission is not active; select it in QGC first");
    return;
  }
  if (_takeoff_request) {
    reply(false, "takeoff already in progress");
    return;
  }
  if (!(request->height > 0.f)) {
    reply(false, "height must be a positive number of metres");
    return;
  }
  if (isArmed() && !_mode.landed()) {
    // Already flying (e.g. the pilot switched in mid-air): just take over.
    scheduleMode(ownedMode().id(), [this](px4_ros2::Result result) {
      if (result != px4_ros2::Result::Success) {
        landNow(std::string("relay ended: ") + resultToString(result));
      }
    });
    _mode.startRelay();
    reply(true, "already airborne; relaying");
    return;
  }

  _takeoff_request = header;
  _takeoff_started = _node.now();
  const Eigen::Vector3f start = _mode.position();
  _climb_target = start - Eigen::Vector3f{0.f, 0.f, request->height};

  arm([this, height = request->height](px4_ros2::Result result) {
    if (result != px4_ros2::Result::Success) {
      finishTakeoff(false,
                    std::string("arm failed: ") + resultToString(result));
      return;
    }
    // PX4's takeoff altitude is AMSL. Without a global position, NaN makes
    // PX4 use MIS_TAKEOFF_ALT; the climb below fixes the height either way.
    const auto amsl = _mode.altitudeAmsl();
    const float target_amsl = amsl ? *amsl + height : NAN;
    if (!amsl) {
      RCLCPP_WARN(_node.get_logger(),
                  "No global position: PX4 takes off to MIS_TAKEOFF_ALT, then "
                  "the mission climbs to %.2f m",
                  height);
    }
    takeoff(
        [this](px4_ros2::Result takeoff_result) {
          if (takeoff_result != px4_ros2::Result::Success) {
            finishTakeoff(false, std::string("takeoff failed: ") +
                                     resultToString(takeoff_result));
            return;
          }
          // PX4 calls takeoff complete within NAV_MC_ALT_RAD of the target,
          // so climb the rest of the way in our own mode before handing over.
          _mode.beginClimb(_climb_target);
          scheduleMode(ownedMode().id(), [this](px4_ros2::Result relay_result) {
            if (relay_result != px4_ros2::Result::Success) {
              landNow(std::string("relay ended: ") +
                      resultToString(relay_result));
            }
          });
          _climb_timer = _node.create_wall_timer(std::chrono::milliseconds(100),
                                                 [this] { checkClimb(); });
        },
        target_amsl);
  });
}

void MissionExecutor::checkClimb() {
  if (!_takeoff_request) {
    _climb_timer.reset();
    return;
  }
  const float error = std::abs(_mode.position().z() - _climb_target.z());
  if (error <= _climb_tolerance_m) {
    _climb_timer.reset();
    _mode.startRelay();
    finishTakeoff(true, "at takeoff height; relaying");
    return;
  }
  if ((_node.now() - _takeoff_started).seconds() > _takeoff_timeout_s) {
    _climb_timer.reset();
    finishTakeoff(false, "did not reach takeoff height in time");
    landNow("takeoff timed out");
  }
}

void MissionExecutor::onLand(std::shared_ptr<rmw_request_id_t> header,
                             std::shared_ptr<LandSrv::Request> /*request*/) {
  if (!isInCharge()) {
    LandSrv::Response response;
    response.success = false;
    response.message = "mission is not active";
    _land_srv->send_response(*header, response);
    return;
  }
  _land_request = header;
  landNow("requested by the mission");
}

void MissionExecutor::landNow(const std::string &why) {
  RCLCPP_INFO(_node.get_logger(), "Landing: %s", why.c_str());
  _mode.state().set("landing");
  land([this](px4_ros2::Result result) {
    const bool ok = result == px4_ros2::Result::Success;
    _mode.state().set(ok ? "landed" : "land failed");
    finishLand(ok, ok ? "landed"
                      : std::string("land failed: ") + resultToString(result));
  });
}

void MissionExecutor::finishTakeoff(bool success, const std::string &message) {
  if (!_takeoff_request) {
    return;
  }
  TakeoffSrv::Response response;
  response.success = success;
  response.message = message;
  _takeoff_srv->send_response(*_takeoff_request, response);
  _takeoff_request.reset();
  RCLCPP_INFO(_node.get_logger(), "Takeoff %s: %s", success ? "done" : "failed",
              message.c_str());
}

void MissionExecutor::finishLand(bool success, const std::string &message) {
  if (!_land_request) {
    return;
  }
  LandSrv::Response response;
  response.success = success;
  response.message = message;
  _land_srv->send_response(*_land_request, response);
  _land_request.reset();
}

} // namespace jl_mission
```

`src/jl_mission/src/main.cpp`:

```cpp
// jl_mission entry point. One process per mission file: the mission's name
// (as shown in QGC) comes from the mission_name parameter.
//
// Registration is retried the same way TakeoffHold does it: wait for the FMU,
// then keep retrying, because at boot the DDS link can come up after us.

#include "jl_mission/mission_mode.hpp"

#include <px4_ros2/components/node_with_mode.hpp>
#include <px4_ros2/components/wait_for_fmu.hpp>

#include <chrono>
#include <stdexcept>

int main(int argc, char *argv[]) {
  using namespace std::chrono_literals;

  rclcpp::init(argc, argv);

  // waitForFMU needs a node that is not the mode node: constructing the mode
  // node is what triggers registration.
  {
    auto startup_node = std::make_shared<rclcpp::Node>("jl_mission_startup");
    if (!px4_ros2::waitForFMU(*startup_node, 60s)) {
      RCLCPP_WARN(startup_node->get_logger(),
                  "No FMU heartbeat after 60 s: is the DDS agent running? "
                  "Retrying registration anyway.");
    }
  }

  const auto retry_delay = 2s;
  int exit_code = 0;
  while (rclcpp::ok()) {
    try {
      auto node = std::make_shared<px4_ros2::NodeWithModeExecutor<
          jl_mission::MissionExecutor, jl_mission::RelayMode>>("jl_mission",
                                                               true);
      RCLCPP_INFO(node->get_logger(), "Registered '%s' with PX4",
                  node->getMode().missionName().c_str());
      rclcpp::spin(node);
      break;
    } catch (const std::invalid_argument &e) {
      // A bad parameter will not fix itself: stop instead of retrying.
      RCLCPP_FATAL(rclcpp::get_logger("jl_mission"), "%s", e.what());
      exit_code = 1;
      break;
    } catch (const std::runtime_error &e) {
      RCLCPP_WARN(rclcpp::get_logger("jl_mission"),
                  "Mode registration failed (%s); retrying in %lds", e.what(),
                  static_cast<long>(retry_delay.count()));
      rclcpp::sleep_for(retry_delay);
    }
  }

  rclcpp::shutdown();
  return exit_code;
}
```

`src/jl_mission/launch/jl_mission.launch.py`:

```python
"""Start one jl_mission instance: the mode that appears in QGC as MISSION_NAME.

ros2 launch jl_mission jl_mission.launch.py mission_name:=TrackMovingAruco
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    name = LaunchConfiguration("mission_name")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "mission_name", description="QGC mode name, 1-24 characters"
            ),
            Node(
                package="jl_mission",
                executable="jl_mission",
                # One node per mission, so several missions can be registered at once
                name=PythonExpression(["'jl_mission_' + '", name, "'.lower()"]),
                output="screen",
                parameters=[{"mission_name": name}],
            ),
        ]
    )
```

Replace `src/jl_mission/CMakeLists.txt` with:

```cmake
cmake_minimum_required(VERSION 3.8)
project(jl_mission)

set(CMAKE_CXX_STANDARD 17)
add_compile_options(-Wall -Wextra -Wpedantic -Werror -Wno-unused-parameter)

find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_ros2_cpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(jl_mission_interfaces REQUIRED)

# The safety contract as plain C++ (Eigen only), so it is unit-tested without ROS
add_library(setpoint_guard src/setpoint_guard.cpp)
target_include_directories(setpoint_guard PUBLIC include ${EIGEN3_INCLUDE_DIR})

add_executable(jl_mission src/main.cpp src/mission_mode.cpp)
target_include_directories(jl_mission PRIVATE include)
target_link_libraries(jl_mission setpoint_guard)
ament_target_dependencies(jl_mission rclcpp Eigen3 px4_ros2_cpp px4_msgs std_msgs
  std_srvs geometry_msgs jl_mission_interfaces)

install(TARGETS jl_mission DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_setpoint_guard test/test_setpoint_guard.cpp)
  target_link_libraries(test_setpoint_guard setpoint_guard)
endif()

ament_package()
```

- [ ] **Step 4: Build, then run both checks (GREEN)**

Run: `colcon build --packages-select jl_mission_interfaces jl_mission && build/jl_mission/test_setpoint_guard`
Expected: a clean build, and `[  PASSED  ] 9 tests.`

Run: `docker restart jacob_ladder_sim` (a clean slate for PX4 and Gazebo). Then, inside the container from the repo root: `source install/setup.bash && src/jl_mission/test/sitl_jl_mission.sh`
Expected, after about 60 s, all seven lines pass (the prototype run gave 1.38–1.39 m):

```
PASS takeoff reported success
PASS takeoff height 1.39 m (want 1.5 +/- 0.15)
PASS relayed the runner's setpoints
PASS invalid setpoint rejected
PASS held on silence
PASS landed after prolonged silence
PASS reached the ground
```

In the printed runner log, check the timing: `STATE hold (no valid setpoint)` comes about 16.5 s after the `TAKEOFF success=True` line, and `STATE landing (no valid setpoint)` about 5 s after that.

Run (host): `clang-format --dry-run -Werror src/jl_mission/src/*.cpp src/jl_mission/include/jl_mission/*.hpp src/jl_mission/test/*.cpp` and `.check-venv/bin/ruff check src/jl_mission && .check-venv/bin/ruff format --check src/jl_mission`
Expected: no clang-format output. Ruff: "All checks passed!" and "already formatted".

- [ ] **Step 5: Checkpoint**

Stop and report the diff and the SITL output to the maintainer. Do not commit.

---

### Task 4: `make sitl-test`, docs, and spec updates

**Files:**
- Modify: `Makefile` (a new target, and `.PHONY`), `README.md` (the START HERE package table), `docs/superpowers/specs/2026-09-21-mission-blocks-design.md` (§5 takeoff text, §11 notes)

**Interfaces:**
- Consumes: `src/jl_mission/test/sitl_jl_mission.sh` (Task 3) and the `jacob_ladder_sim` container.
- Produces: `make sitl-test` (the L3 entry point from spec §8).

- [ ] **Step 1: Add the Makefile target**

Append after the `check` recipe:

```make
# L3: fly the jl_mission safety contract in headless SITL (spec section 8).
# Needs the jacob_ladder_sim container (./docker/run_sim_container.sh) with
# jl_mission_interfaces and jl_mission built by colcon inside it.
SIM_CONTAINER ?= jacob_ladder_sim

sitl-test:
	docker restart $(SIM_CONTAINER) > /dev/null
	sleep 3
	docker exec --user user -w $(CURDIR) $(SIM_CONTAINER) bash -c 'source /opt/ros/humble/setup.bash && source install/setup.bash && src/jl_mission/test/sitl_jl_mission.sh'
```

Change the `.PHONY` line to: `.PHONY: all format build clean check sitl-test`

Run: `make sitl-test`
Expected: the same seven PASS lines as Task 3, with exit code 0.

- [ ] **Step 2: README row**

In `README.md`'s START HERE package table, add this row directly below the `example_autonomous_mode` row:

```markdown
| [jl_mission](src/jl_mission) | The one C++ external mode behind every mission file: checks and relays the Python mission runner's setpoints, owns arm/takeoff/land. Checked in SITL with `make sitl-test` |
```

- [ ] **Step 3: Spec updates**

In `docs/superpowers/specs/2026-09-21-mission-blocks-design.md`, section 5, replace the table row that starts `| Native takeoff/arm/land |` with:

```markdown
| Native takeoff/arm/land | only the executor calls PX4's `arm()`, `takeoff()`, `land()`. `takeoff()` gets current AMSL + height (NaN, i.e. `MIS_TAKEOFF_ALT`, without a global position); the relay mode then climbs to exactly the requested height, and the takeoff request is answered only when the vehicle is within `climb_tolerance_m` of it | 0.1 m |
```

In section 11, append:

```markdown
- `jl_mission` needs `mission_name` of at most 24 characters (PX4's external mode name limit). The Phase 1 loader's name rule allows longer names; Phase 3 must add the length check so `jl_blocks check` catches it.
- `jl_mission` has only been flown in SITL. Before its first real flight, check on the drone what `takeoff()` does with no global position (the NaN / `MIS_TAKEOFF_ALT` path) and that the Climb phase reaches the requested height under VIO.
```

- [ ] **Step 4: Checkpoint**

Stop and report the diff to the maintainer. Do not commit.

---

## Maintainer notes (found while planning; not tasks)

- The prototype was verified in `jacob_ladder_sim`: a clean `-Werror` build, 9/9 gtests, and 7/7 SITL checks (twice).
- **SITL assumption:** the SITL check uses PX4's own `gz_x500` in the default world and a fresh PX4 working directory, with `NAV_DLL_ACT 0` because there is no GCS. It needs neither the `x500_dual_cam` model nor any real-drone param file.
- **Real-drone gap:** `jl_mission` has never flown on hardware. The real drone is GPS-denied, so its first flight should confirm the no-global-position takeoff path (see the §11 note added in Task 4).
- **Phase 3 carry-overs:**
  - the executor-action channel (spec §11);
  - the mission-name length check;
  - the deferred Phase 1 minor findings, in `.superpowers/sdd/2026-09-21-mission-blocks-phase1-core/progress.md`.
