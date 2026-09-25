#include "SimpleExternal.hpp"

#include <px4_ros2/components/node_with_mode.hpp>

namespace precision_land
{

SimpleExternalMode::TakeoffLandMode(rclcpp::Node& node)
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

void SimpleExternalMode::loadParameters()
{
  // Declare any ROS 2 parameter for the network and grab any value present
	_node.declare_parameter<float>("some_param_height", 0.1f);

	_node.get_parameter("some_param_height", _some_param_height;
}

void SimpleExternalMode::vehicleLandDetectedCallback(const px4_msgs::msg::VehicleLandDetected::SharedPtr msg)
{
  // Check the landing state of the vehicle via ROS topic, update locally
	_land_detected = msg->landed;
}

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

void SimpleExternalMode::updateSetpoint(float dt_s)
{
  // Track the change in seconds between setpoint updates
	_state_elapsed += dt_s;

	switch (_state) {
    // Have a blank state for our Idle, so we don't miss any conditions
    case State::Idle: {
      break;
    }  // End of Idle State

    case State::Init: {
      // Enter the first state which isn't just idle, just a sort of temporary state to mark change
      RCLCPP_INFO(_node.get_logger(), "We're in the first non-Idle state of our External Mode")
      switchToState(State::Holding);
      break;
    }  // End of Init State

    case State::Holding: {
      if (_state_elapsed >= _hold_duration) {
        RCLCPP_INFO(_node.get_logger(), "Hold complete — descending");
        switchToState(State::Descending);
        break;
      }

      _trajectory_setpoint->updatePosition(_hold_position);
      break;
    }  // End of Holding State

    case State::Finished: {
      Eigen::Vector3f hold = _vehicle_local_position->positionNed();
      _trajectory_setpoint->updatePosition(hold);
      ModeBase::completed(px4_ros2::Result::Success);
      break;
    }  // End Finished State
	}  // End Switch Case Statement
}

void SimpleExternalMode::switchToState(State state)
{
  // No Need to change states if we're already in the desired one
	if (_state == state) {
		return;
	}

  // Let the user know in the logs/terminal that we've switched to a different state
	RCLCPP_INFO(_node.get_logger(), "Switching to %s", stateName(state).c_str());
	_state = state;
}

std::string SimpleExternalMode::stateName(State state) const
{
  // Enum to String sort of function
	switch (state) {
	case State::Idle:
		return "Idle";
	case State::Init:
		return "Init";
	case State::Holding:
		return "Holding";
	case State::Finished:
		return "Finished";
	default:
		return "Unknown";
	}
}

} // namespace precision_land

int main(int argc, char* argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<px4_ros2::NodeWithMode<precision_land::SimpleExternalMode>>(
		precision_land::kSimpleExternalModeName, precision_land::kTakeoffLandDebugOutput));
	rclcpp::shutdown();
	return 0;
}
