# External Modes
------------------
This project uses External Modes for all of its autonomous control. An external mode lets the drone fly autonomously at the flip of a switch on the safety handset, and you can engage it ahead of time. External modes are the standard way to run autonomous algorithms on the ROS 2 / PX4 stack this drone uses.
You build them with the [PX4 ROS2 Interface Library](https://github.com/Auterion/px4-ros2-interface-lib), and they
use ROS 2 nodes to send setpoints to PX4.
They replace the `offboard` mode that many people are familiar with.

This [video](https://www.youtube.com/watch?v=3zRCIsq_MCE) has more information.

#### Benefits
1) They don't require MAVLink
2) There's no limit to the number of setpoint types
3) They integrate more generally with ROS 2, with extended controls and a growing feature set
4) You can use multiple applications to control the vehicle

#### Caveats
The message types have to match between PX4 ([uORB](https://docs.px4.io/main/en/middleware/uorb)) and ROS 2 [interfaces](https://docs.ros.org/en/humble/Concepts/Basic/About-Interfaces.html). You
can get around this by running the message [translation node](https://docs.px4.io/main/en/ros2/px4_ros2_msg_translation_node):
```shell
ros2 run translation_node translation_node_bin
```
That's how this project
handles it. In your custom mode's constructor, add the following line:
```Cpp
setSkipMessageCompatibilityCheck();
```

### File Structure
Following the usual C++ conventions, split each autonomous setup into a header (`.hpp`) file and a source (`.cpp`) file. The mode and executor can live in the same file, so one autonomous behavior should only take two files.

#### Header File Essentials
Header files differ in which packages they include and how they name things, but they generally contain the same pieces:
**Packages + Header Guard**:
Add the header guard and the packages the setup needs to run:
```cpp
#pragma once

#include "StatePublisher.hpp"

#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/components/mode_executor.hpp>
#include <px4_ros2/control/setpoint_types/experimental/trajectory.hpp>
#include <px4_ros2/odometry/local_position.hpp>

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>

#include <Eigen/Core>

#include <string>
```

**Naming**:
Put each autonomous mode in its own namespace so its names don't clash with other packages. Also give it a precise name: QGC shows that name, so it's how you pick the right mode every time. Everything else for the mode **must** live inside the namespace.
```cpp
namespace simple_external {
inline constexpr char kSimpleExternalModeName[] = "SimpleExternal";
inline constexpr bool kSimpleExternalDebugOutput = true;

// ... More Modes + Executors
}
```

### Executors
An executor runs the drone's overall state machine. It also decides which modes (states) run at any given
time, based on whatever conditions you set. This splits the workflow in two, which gives you more control over how the autonomous setup behaves.


**Executor Header File**:



### Modes
A mode is a component that sends one or more setpoints. It can also do tasks
besides flying, such as checking the battery state or reading the local position.

Below is a very simple mode that can change states and registers itself.
The full code is in "general_docs/SimpleExternal.cpp", and you can use it as a
template for your own mission.

*Note*: the `SimpleExternalMode` class doesn't do anything useful on its own, but it creates a mode that PX4 and QGC will recognize. It's a shell containing the functions you'll need.

**Mode Header File**:


**Simple Constructor**:
First, write a constructor that passes what's needed to the `ModeBase` and ROS 2 `Node` classes. It sets up the node, creates the mode's settings, lets messages pass between PX4 and ROS 2, and declares every parameter the drone needs. [Parameters](https://docs.ros.org/en/foxy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Parameters/Understanding-ROS2-Parameters.html) are variables in the ROS ecosystem that are easy to change.
```cpp
SimpleExternalMode::SimpleExternalMode(rclcpp::Node& node)
	: ModeBase(node, ModeBase::Settings{kSimpleExternalModeName})
	, _node(node)
{
  // Since we're using the translation node, we can skip message compatibility and PX4 will convert for us
	setSkipMessageCompatibilityCheck();

  // We want access to the estimated vehicle position along with the ability to go to certain setpoints
	_vehicle_local_position = std::make_shared<px4_ros2::OdometryLocalPosition>(*this);
	_trajectory_setpoint = std::make_shared<px4_ros2::TrajectorySetpointType>(*this);

  // Grab any and all parameters ROS will show available to us
	loadParameters();
}
```
**Load Parameters**:
The mode can load parameters at compile time from our ROS 2 parameter list.
You can add as many parameters as you want to be able to change. Just declare and
get each one.
```cpp
void SimpleExternalMode::loadParameters()
{
  // Declare any ROS 2 parameter for the network and grab any value present
	_node.declare_parameter<float>("some_param_height", 0.1f);

	_node.get_parameter("some_param_height", _some_param_height;
}
```

**Activation Functions**:
Next, add two standard functions: one runs when the external mode is switched on,
the other when it's switched off.
```cpp
void SimpleExternalMode::onActivate()
{
  // Ensure we have a valid starting state
  switchToState(State::Idle)
	RCLCPP_INFO(_node.get_logger(), "External Mode Activated")
}

void SimpleExternalMode::onDeactivate()
{
  // We can go back to idle when all finished
	switchToState(State::Idle);
	RCLCPP_INFO(_node.get_logger(), "External Mode Deactivated")
}
```

**State Machine**:
This is the core of the external mode. Everything goes in the `updateSetpoint` function, which runs once every control loop. The
state machine decides what the drone does (takeoff, landing, searching, hovering, returning to recharge, or anything else). Your autonomous code goes here.
```cpp
void SimpleExternalMode::updateSetpoint(float dt_s)
{
  if (!_active) return;

  _state_elapsed += dt_s;

  switch (_state) {
    case SimpleState::SimpleStart: {
      // First State Logic would go here
      RCLCPP_INFO(_node.get_logger(), "Hello World!");
	}
	break;
  }

}
```

### Linking into the project
This is a C++ colcon project, so each package folder has a `CMakeLists.txt` file. When you finish a mode + executor pair, add it to that file. Your mode's name should appear there 4 times. First, near the top, after the basic CMake setup, add any extra packages you need with:
```cpp
find_package(some_package REQUIRED)
```

Next, add an executable so the mode is easy to launch: name it and list its source file. Then link its dependencies and set its compile features:
```cpp
add_executable(simple_external SimpleExternal.cpp)
ament_target_dependencies(simple_external rclcpp Eigen3 px4_ros2_cpp std_msgs)
target_compile_features(simple_external PUBLIC c_std_99 cxx_std_17)
```

Finally, add the executable to the install section:
```cpp
install(TARGETS
  simple_external
  DESTINATION lib/${PROJECT_NAME}
)
```

### Create Your Own Autonomous Mode (Template)
The [precision_land](https://github.com/CursedRock17/Jacob_Ladder/tree/main/src/precision_land) directory has an example node that takes off autonomously, runs autonomous code that you HAVE to fill in, then lands. If the sections above didn't quite make sense, start from it: it makes running an autonomous mode in PX4 easier.

#### Ensuring Mode is saved
In the MAVLink Console run:
```bash
commander status
```
Your external mode should appear in the output.
