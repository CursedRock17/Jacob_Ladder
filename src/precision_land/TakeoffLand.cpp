#include "TakeoffLand.hpp"

#include <px4_ros2/components/node_with_mode.hpp>

using namespace std::chrono_literals;

namespace precision_land
{

// ── Mode: optical flow init, smooth climb, hold ──

TakeoffLandMode::TakeoffLandMode(rclcpp::Node& node)
	: ModeBase(node, Settings{kTakeoffLandModeName, false})
	, _node(node)
	, _base_position(Eigen::Vector3f::Zero())
	, _hold_position(Eigen::Vector3f::Zero())
{
	setSkipMessageCompatibilityCheck();

	_vehicle_local_position = std::make_shared<px4_ros2::OdometryLocalPosition>(*this);
	_trajectory_setpoint = std::make_shared<px4_ros2::TrajectorySetpointType>(*this);

	_node.declare_parameter<float>("optical_flow_height", 0.1f);
	_node.declare_parameter<float>("optical_flow_hold_time", 3.0f);
	_node.declare_parameter<float>("target_height", 2.5f);
	_node.declare_parameter<float>("climb_rate", 0.3f);
	_node.declare_parameter<float>("delta_position", 0.05f);

	_node.get_parameter("optical_flow_height", _optical_flow_height);
	_node.get_parameter("optical_flow_hold_time", _optical_flow_hold_time);
	_node.get_parameter("target_height", _target_height);
	_node.get_parameter("climb_rate", _climb_rate);
	_node.get_parameter("delta_position", _delta_position);
}

void TakeoffLandMode::onActivate()
{
	_base_position = _vehicle_local_position->positionNed();
	_hold_position = _base_position;
	_state = TakeoffState::OpticalFlowInit;
	_state_elapsed = 0.0f;
	_reached_flow_height = false;
	_active = true;

	_hold_position.z() = _base_position.z() - _optical_flow_height;

	RCLCPP_INFO(_node.get_logger(),
		"TakeoffLand mode active — optical flow init at %.2f m, then climb to %.1f m at %.1f m/s",
		_optical_flow_height, _target_height, _climb_rate);
}

void TakeoffLandMode::onDeactivate()
{
	_active = false;
}

void TakeoffLandMode::updateSetpoint(float dt_s)
{
	if (!_active) return;

	_state_elapsed += dt_s;

	switch (_state) {

	case TakeoffState::OpticalFlowInit: {
		const float current_z = _vehicle_local_position->positionNed().z();
		const float altitude_gained = _base_position.z() - current_z;
		RCLCPP_INFO_THROTTLE(_node.get_logger(), *_node.get_clock(), 2000,
			"[OpticalFlowInit] height: %.2f m | alt gained: %.2f m | target: %.2f m | elapsed: %.1f s",
			-current_z, altitude_gained, _optical_flow_height, _state_elapsed);

		if (!_reached_flow_height
			&& altitude_gained >= (_optical_flow_height - _delta_position)) {
			_reached_flow_height = true;
			_state_elapsed = 0.0f;
			RCLCPP_INFO(_node.get_logger(),
				"Reached optical flow height (%.2f m gained) — holding for %.1f s",
				altitude_gained, _optical_flow_hold_time);
		}

		if (_reached_flow_height && _state_elapsed >= _optical_flow_hold_time) {
			_state = TakeoffState::Climbing;
			_state_elapsed = 0.0f;
			RCLCPP_INFO(_node.get_logger(),
				"Optical flow stabilized — climbing to %.1f m", _target_height);
		}

		_trajectory_setpoint->update(
			px4_ros2::TrajectorySetpoint{}
				.withPosition(_hold_position)
				.withYaw(0.0f)
		);
		break;
	}

	case TakeoffState::Climbing: {
		const float target_z = _base_position.z() - _target_height;
		const float current_z = _vehicle_local_position->positionNed().z();
		RCLCPP_INFO_THROTTLE(_node.get_logger(), *_node.get_clock(), 2000,
			"[Climbing] height: %.2f m | alt gained: %.2f m | target: %.2f m | setpoint_z: %.2f m",
			-current_z, _base_position.z() - current_z, _target_height, _hold_position.z());

		_hold_position.z() -= _climb_rate * dt_s;

		if (_hold_position.z() <= target_z) {
			_hold_position.z() = target_z;
		}

		const float altitude_gained = _base_position.z() - current_z;
		if (altitude_gained >= (_target_height - _delta_position)) {
			_state = TakeoffState::Holding;
			_state_elapsed = 0.0f;
			RCLCPP_INFO(_node.get_logger(),
				"Reached %.1f m (actual: %.2f m) — holding position",
				_target_height, altitude_gained);
		}

		_trajectory_setpoint->update(
			px4_ros2::TrajectorySetpoint{}
				.withPosition(_hold_position)
				.withVelocityZ(_state == TakeoffState::Holding ? 0.0f : -_climb_rate)
				.withYaw(0.0f)
		);
		break;
	}

	case TakeoffState::Holding:
		RCLCPP_INFO_THROTTLE(_node.get_logger(), *_node.get_clock(), 5000,
			"[Holding] height: %.2f m | hold_z: %.2f m",
			-_vehicle_local_position->positionNed().z(), _hold_position.z());
		_trajectory_setpoint->update(
			px4_ros2::TrajectorySetpoint{}
				.withPosition(_hold_position)
				.withYaw(0.0f)
		);
		break;
	}
}

// ── Executor: arm -> takeoff(1.25) -> hold (timed) -> land -> disarm ──

TakeoffLandExecutor::TakeoffLandExecutor(rclcpp::Node& node, px4_ros2::ModeBase& owned_mode)
	: ModeExecutorBase(node, ModeExecutorBase::Settings{Settings::Activation::ActivateAlways}, owned_mode)
	, _node(node)
{
	_node.declare_parameter<float>("hold_duration", 5.0f);
	_node.get_parameter("hold_duration", _param_hold_duration);
}

void TakeoffLandExecutor::onActivate()
{
	RCLCPP_INFO(_node.get_logger(), "TakeoffLand executor — arming, hold %.1f s, then land",
		_param_hold_duration);
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
		RCLCPP_INFO(_node.get_logger(), "Arm complete — takeoff");
		takeoff([this](px4_ros2::Result r) { runState(State::Hold, r); }, 1.25f);
		break;

	case State::Hold:
		RCLCPP_INFO(_node.get_logger(), "Takeoff complete — scheduling mode, landing in %.1f s", _param_hold_duration);
		scheduleMode(ownedMode().id(), [this](px4_ros2::Result r) {
			// Mode ended (either hold timer fired or external deactivation)
		});
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
