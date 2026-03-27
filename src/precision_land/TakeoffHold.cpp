#include "TakeoffHold.hpp"

#include <px4_ros2/components/node_with_mode.hpp>

namespace precision_land
{

// ── Mode: captures current position on activation, holds it indefinitely ──

TakeoffHoldMode::TakeoffHoldMode(rclcpp::Node& node)
	: ModeBase(node, Settings{kTakeoffHoldModeName, false})
	, _node(node)
	, _hold_position(Eigen::Vector3f::Zero())
{
	setSkipMessageCompatibilityCheck();

	_vehicle_local_position = std::make_shared<px4_ros2::OdometryLocalPosition>(*this);
	_trajectory_setpoint = std::make_shared<px4_ros2::TrajectorySetpointType>(*this);
}

void TakeoffHoldMode::onActivate()
{
	_hold_position = _vehicle_local_position->positionNed();
	_position_set = true;
	RCLCPP_INFO(_node.get_logger(), "TakeoffHold mode active — holding [%.2f, %.2f, %.2f]",
		_hold_position.x(), _hold_position.y(), _hold_position.z());
}

void TakeoffHoldMode::onDeactivate()
{
	_position_set = false;
}

void TakeoffHoldMode::updateSetpoint(float dt_s)
{
	if (_position_set) {
		_trajectory_setpoint->updatePosition(_hold_position);
	}
}

// ── Executor: PX4 internal takeoff -> hand off to hold mode ──

TakeoffHoldExecutor::TakeoffHoldExecutor(rclcpp::Node& node, px4_ros2::ModeBase& owned_mode)
	: ModeExecutorBase(node, ModeExecutorBase::Settings{Settings::Activation::ActivateAlways}, owned_mode)
	, _node(node)
{
	setSkipMessageCompatibilityCheck();
	_node.declare_parameter<float>("takeoff_height", 2.5f);
	_node.get_parameter("takeoff_height", _param_takeoff_height);
}

void TakeoffHoldExecutor::onActivate()
{
	RCLCPP_INFO(_node.get_logger(), "TakeoffHold executor — arming and taking off to %.1f m", _param_takeoff_height);
	runState(State::Arming, px4_ros2::Result::Success);
}

void TakeoffHoldExecutor::onDeactivate(DeactivateReason reason)
{
}

void TakeoffHoldExecutor::runState(State state, px4_ros2::Result result)
{
	if (result != px4_ros2::Result::Success) {
		RCLCPP_ERROR(_node.get_logger(), "State %i failed: %s", (int)state,
			resultToString(result));
		return;
	}

	switch (state) {
	case State::Arming:
		arm([this](px4_ros2::Result r) { runState(State::TakingOff, r); });
		break;

	case State::TakingOff:
		takeoff([this](px4_ros2::Result r) { runState(State::Hold, r); },
			_param_takeoff_height);
		break;

	case State::Hold:
		RCLCPP_INFO(_node.get_logger(), "Takeoff complete — holding position");
		scheduleMode(ownedMode().id(), [this](px4_ros2::Result r) {
			RCLCPP_INFO(_node.get_logger(), "Hold mode ended (%s)", resultToString(r));
		});
		break;
	}
}

} // namespace precision_land

int main(int argc, char* argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<px4_ros2::NodeWithModeExecutor<
		precision_land::TakeoffHoldExecutor, precision_land::TakeoffHoldMode>>(
		precision_land::kTakeoffHoldModeName, precision_land::kTakeoffHoldDebugOutput));
	rclcpp::shutdown();
	return 0;
}
