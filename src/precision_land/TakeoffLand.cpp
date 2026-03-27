#include "TakeoffLand.hpp"

#include <px4_ros2/components/node_with_mode.hpp>

using namespace std::chrono_literals;

namespace precision_land
{

// ── Mode: captures current position on activation, holds it ──

TakeoffLandMode::TakeoffLandMode(rclcpp::Node& node)
	: ModeBase(node, Settings{kTakeoffLandModeName, false})
	, _node(node)
	, _hold_position(Eigen::Vector3f::Zero())
{
	setSkipMessageCompatibilityCheck();

	_vehicle_local_position = std::make_shared<px4_ros2::OdometryLocalPosition>(*this);
	_trajectory_setpoint = std::make_shared<px4_ros2::TrajectorySetpointType>(*this);
}

void TakeoffLandMode::onActivate()
{
	_hold_position = _vehicle_local_position->positionNed();
	_position_set = true;
	RCLCPP_INFO(_node.get_logger(), "TakeoffLand mode active — holding [%.2f, %.2f, %.2f]",
		_hold_position.x(), _hold_position.y(), _hold_position.z());
}

void TakeoffLandMode::onDeactivate()
{
	_position_set = false;
}

void TakeoffLandMode::updateSetpoint(float dt_s)
{
	if (_position_set) {
		_trajectory_setpoint->updatePosition(_hold_position);
	}
}

// ── Executor: arm -> takeoff -> hold (timed) -> land -> disarm ──

TakeoffLandExecutor::TakeoffLandExecutor(rclcpp::Node& node, px4_ros2::ModeBase& owned_mode)
	: ModeExecutorBase(node, ModeExecutorBase::Settings{Settings::Activation::ActivateAlways}, owned_mode)
	, _node(node)
{
	setSkipMessageCompatibilityCheck();
	_node.declare_parameter<float>("takeoff_height", 2.5f);
	_node.declare_parameter<float>("hold_duration", 5.0f);
	_node.get_parameter("takeoff_height", _param_takeoff_height);
	_node.get_parameter("hold_duration", _param_hold_duration);
}

void TakeoffLandExecutor::onActivate()
{
	RCLCPP_INFO(_node.get_logger(), "TakeoffLand executor — arming, takeoff to %.1f m, hold %.1f s, then land",
		_param_takeoff_height, _param_hold_duration);
	runState(State::Arming, px4_ros2::Result::Success);
}

void TakeoffLandExecutor::onDeactivate(DeactivateReason reason)
{
	if (_hold_timer) {
		_hold_timer->cancel();
		_hold_timer.reset();
	}
}

void TakeoffLandExecutor::runState(State state, px4_ros2::Result result)
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
		RCLCPP_INFO(_node.get_logger(), "Takeoff complete — holding for %.1f s", _param_hold_duration);
		scheduleMode(ownedMode().id(), [this](px4_ros2::Result r) {
			// Mode ended (either hold timer fired or external deactivation)
		});
		// Start a timer to transition to landing after hold_duration
		_hold_timer = _node.create_wall_timer(
			std::chrono::duration_cast<std::chrono::nanoseconds>(
				std::chrono::duration<float>(_param_hold_duration)),
			[this]() {
				_hold_timer->cancel();
				RCLCPP_INFO(_node.get_logger(), "Hold complete — landing");
				runState(State::Landing, px4_ros2::Result::Success);
			});
		break;

	case State::Landing:
		land([this](px4_ros2::Result r) { runState(State::Disarming, r); });
		break;

	case State::Disarming:
		RCLCPP_INFO(_node.get_logger(), "Landed — waiting for disarm");
		waitUntilDisarmed([this](px4_ros2::Result r) {
			RCLCPP_INFO(_node.get_logger(), "Disarmed — TakeoffLand complete");
		});
		break;
	}
}

} // namespace precision_land

int main(int argc, char* argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<px4_ros2::NodeWithModeExecutor<
		precision_land::TakeoffLandExecutor, precision_land::TakeoffLandMode>>(
		precision_land::kTakeoffLandModeName, precision_land::kTakeoffLandDebugOutput));
	rclcpp::shutdown();
	return 0;
}
