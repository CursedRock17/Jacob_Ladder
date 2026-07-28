# External Modes
------------------
This project takes advantage of External Modes for any and all autonomous control. External modes allow our drone to have autonomously functionality at the switch of a button on the safety handset and can be engaged ahead of time. They are the standard interface for working with autonomous algorithms through the ROS 2 / PX4 Stack that this drone uses.
External modes are created with the [PX4 ROS2 Interface Library](https://github.com/Auterion/px4-ros2-interface-lib), and 
utilize ROS 2 nodes to interact with PX4 setpoints. 
External modes are a superior alternative to `offboard` mode that many people are familiar with.

Fantastic [video](https://www.youtube.com/watch?v=3zRCIsq_MCE) w/more information

#### Benefits
1) They don't require MAVLink
2) There's no limit to number of setpoint types
3) More general integration with ROS 2, extended controls and increasing features
4) You can use multiple applications to control the vehicle

#### Caveats
You have to match the message type between PX4 ([uORB](https://docs.px4.io/main/en/middleware/uorb)) and ROS 2 [interfaces](https://docs.ros.org/en/humble/Concepts/Basic/About-Interfaces.html), this
can be subverted by running the message [translation node](https://docs.px4.io/main/en/ros2/px4_ros2_msg_translation_node) :
```shell
ros2 run translation_node translation_node_bin
```
which is how we handle
it in this project. In your custom mode constructor simply add the following line:
```Cpp
setSkipMessageCompatibilityCheck();
```

### File Structure
As per usual C++ guidelines we're going to split our autonomous set up into a header (`.hpp`) file and source (`.cpp`) file. The mode and executor can live in the same file, therefore for some autonomous behavior you should only end up with two files.

#### Header File Essentials
While each header file may contain different packages and naming practices, they'll generally all have the same things in them:
**Packages + Header Guard**:
Include our header guard and necessaary packages to actually run our setup:
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
Each autonomous mode should use a namespace to differentiate its information from other packages. It should also use a precise name which gets serialized in QGC to correctly identify the mode you're launching each and everytime. All further mode **must** live inside of the namespace
```cpp
namespace simple_external {
inline constexpr char kSimpleExternalModeName[] = "SimpleExternal";
inline constexpr bool kSimpleExternalDebugOutput = true;

// ... More Modes + Executors
}
```

### Executors
Executors handle the overall state machine for the drone. They also register certain modes (states) at any given
time, based on any set structure of conditions. It splits the whole workflow into 2 and allows a better molding of your autonomous setup


**Executor Header File**:



### Modes
A Component that sends setpoints (one or more) and can perform a number of tasks
agnostic to just flying (i.e. check battery state, view local position, etc.)

The following is a super simple mode that can change states and registers itself
The full code is in the "src/jacob_manual/SimpleExternal.cpp", you can build this 
"template" out to fit your ideal mission.

*Note*, the `SimpleExternalMode` class present won't run anything useful, but will create a mode that PX4/QGC will be able to recognize. The class serves as a shell for the necessary functions users will need.

**Mode Header File**:


**Simple Constructor**:
We first need to create a simple constructor which will inherit the necessary parameters from the `ModeBase` and ROS 2 `Node` classes. We'll standardize our node, create settings, allow our messages to be bridged between PX4 and ROS 2, and declare any and all parameters necessary for the drone. [Parameters](https://docs.ros.org/en/foxy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Parameters/Understanding-ROS2-Parameters.html) are essentially easy to control variables in the ROS ecosystem. 
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
We have the ability to load parameters at compile time from our ROS 2 parameters list.
This can feature as many parameters as you want to change at any given time, just declare and
grab each of the parameters you want.
```cpp
void SimpleExternalMode::loadParameters()
{
  // Declare any ROS 2 parameter for the network and grab any value present
	_node.declare_parameter<float>("some_param_height", 0.1f);

	_node.get_parameter("some_param_height", _some_param_height;
}
```

**Activation Functions**:
Next, we neeed some standard functions. One for when the external mode is switched on, 
the other, when the external mode is switched off.
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
This is the guts of the External Mode. You'll put everything into an `updateSetpoint` function which will run once in every portion of the control loop. The
state machine that you'll use will determine what behaviors the drone will exhibit (takeoff, landing, searching, hovering, need to recharge, anything). This is where autonomous code will have to be placed when utilizing the drone.
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
Since we're using `cpp` in our colcon project, we'll use a `CMakeLists.txt` file in each folder. Therefore, when you finish your mode+executor pair, be sure to upload into that CMake file. There should be 4 references to the name of your mode in that file. First, make sure you any packages you desire after basic cmake setup, this will be towards the top, simply add addition external packages with:
```cpp
find_package(some_package REQUIRED)
```

Then, we need to add an executable to make this easy to launch, label your desired file and add in the source file. We'll also need to link dependencies and add compilation features:
```cpp
add_executable(simple_external SimpleExternal.cpp)
ament_target_dependencies(simple_external rclcpp Eigen3 px4_ros2_cpp std_msgs)
target_compile_features(simple_external PUBLIC c_std_99 cxx_std_17)
```

Finally, make sure to install our executables in the installation section:
```cpp
install(TARGETS
  simple_external
  DESTINATION lib/${PROJECT_NAME}
)
```

### Create Your Own Autonomous Mode (Template)
We've provided an example node in the [precision_land](https://github.com/CursedRock17/Jacob_Ladder/tree/main/src/precision_land) directory which will autonomously takeoff, run some autonomous code that you HAVE to fill in, then land. This is a good stub for you if didn't quite understand what was just said and allows an easier time in running autonomous modes in PX4.

#### Ensuring Mode is saved
In the MAVLink Console run:
```bash
commander status
```
The result of the of External Mode channel should show up
