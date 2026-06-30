# External Modes
------------------
This project takes advantage of External Modes for any and all autonomous control.
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

### Executors
Handle the overall state machine for the drone. The register certain modes (states) at any given
time, based on any set structure of conditions.

TODO: Add Explanation for Executors


### Modes
A Component that sends setpoints (one or more) and can perform a number of tasks
agnostic to just flying (i.e. check battery state, view local position, etc.)

The following is a super simple mode that can change states and registers itself
The full code is in the "src/jacob_manual/SimpleExternal.cpp", you can build this 
"template" out to fit your ideal mission.

Simple Constructor:
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

We then need to establish, locally, if the vehicle has landed so we see when our states
have finished activating. We can simply grab this from the corresponding ROS 2 topic.
This is a "callback", callbacks are updated in some interval with respect to a given
subscription.
```cpp
void SimpleExternalMode::vehicleLandDetectedCallback(const px4_msgs::msg::VehicleLandDetected::SharedPtr msg)
{
  // Check the landing state of the vehicle via ROS topic, update locally
	_land_detected = msg->landed;
}
```

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

#### Ensuring Mode is saved
In the MAVLink Console run:
```bash
commander status
```
The result of the of External Mode channel should show up
