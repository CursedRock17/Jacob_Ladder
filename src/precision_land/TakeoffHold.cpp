#include "TakeoffHold.hpp"

#include <px4_ros2/components/node_with_mode.hpp>

namespace precision_land
{

// ── Mode: state-machine takeoff with optical flow init, smooth climb, hold ──

TakeoffHoldMode::TakeoffHoldMode(rclcpp::Node& node)
	: ModeBase(node, Settings{kTakeoffHoldModeName, false})
	, _node(node)
	, _base_position(Eigen::Vector3f::Zero())
	, _hold_position(Eigen::Vector3f::Zero())
{
	setSkipMessageCompatibilityCheck();

	_vehicle_local_position = std::make_shared<px4_ros2::OdometryLocalPosition>(*this);
	_trajectory_setpoint = std::make_shared<px4_ros2::TrajectorySetpointType>(*this);

	// Declare parameters with defaults
	_node.declare_parameter<float>("optical_flow_height", 0.5f);
	_node.declare_parameter<float>("optical_flow_hold_time", 3.0f);
	_node.declare_parameter<float>("target_height", 1.25f);
	_node.declare_parameter<float>("climb_rate", 0.3f);
	_node.declare_parameter<float>("delta_position", 0.25f);

	_node.get_parameter("optical_flow_height", _optical_flow_height);
	_node.get_parameter("optical_flow_hold_time", _optical_flow_hold_time);
	_node.get_parameter("target_height", _target_height);
	_node.get_parameter("climb_rate", _climb_rate);
	_node.get_parameter("delta_position", _delta_position);
}

void TakeoffHoldMode::onActivate()
{
	_base_position = _vehicle_local_position->positionNed();
	_hold_position = _base_position;
	_state = TakeoffState::OpticalFlowInit;
	_state_elapsed = 0.0f;
	_reached_flow_height = false;
	_active = true;

	// Command initial position: optical_flow_height above ground (NED: negative z is up)
	_hold_position.z() = _base_position.z() - _optical_flow_height;

	RCLCPP_INFO(_node.get_logger(),
		"TakeoffHold active — optical flow init at %.2f m, then climb to %.1f m at %.1f m/s",
		_optical_flow_height, _target_height, _climb_rate);
}

void TakeoffHoldMode::onDeactivate()
{
	_active = false;
}

void TakeoffHoldMode::updateSetpoint(float dt_s)
{
	if (!_active) return;

	_state_elapsed += dt_s;

	switch (_state) {

	case TakeoffState::OpticalFlowInit: {
		const float current_z = _vehicle_local_position->positionNed().z();
		// How far above our starting point have we actually climbed (positive = upward)
		const float altitude_gained = _base_position.z() - current_z;
		RCLCPP_INFO_THROTTLE(_node.get_logger(), *_node.get_clock(), 2000,
			"[OpticalFlowInit] height: %.2f m | alt gained: %.2f m | target: %.2f m | elapsed: %.1f s",
			-current_z, altitude_gained, _optical_flow_height, _state_elapsed);

		// Only start the hold timer once we've actually climbed to near the target
		if (!_reached_flow_height
			&& altitude_gained >= (_optical_flow_height - _delta_position)) {
			_reached_flow_height = true;
			_state_elapsed = 0.0f;
			RCLCPP_INFO(_node.get_logger(),
				"Reached optical flow height (%.2f m gained) — holding for %.1f s",
				altitude_gained, _optical_flow_hold_time);
		}

		// After reaching height and holding long enough, transition to climb
		if (_reached_flow_height && _state_elapsed >= _optical_flow_hold_time) {
			_state = TakeoffState::Climbing;
			_state_elapsed = 0.0f;
			RCLCPP_INFO(_node.get_logger(),
				"Optical flow stabilized — climbing to %.1f m", _target_height);
		}

		// Position-only setpoint at optical flow height
		_trajectory_setpoint->update(
			px4_ros2::TrajectorySetpoint{}
				.withPosition(_hold_position)
				.withYaw(0.0f)
		);
		break;
	}

	case TakeoffState::Climbing: {
		// Target z in NED (negative = up)
		const float target_z = _base_position.z() - _target_height;
		const float current_z = _vehicle_local_position->positionNed().z();
		RCLCPP_INFO_THROTTLE(_node.get_logger(), *_node.get_clock(), 2000,
			"[Climbing] height: %.2f m | alt gained: %.2f m | target: %.2f m | setpoint_z: %.2f m",
			-current_z, _base_position.z() - current_z, _target_height, _hold_position.z());

		// Ramp z downward (upward in world) at climb_rate
		_hold_position.z() -= _climb_rate * dt_s;

		// Clamp setpoint to target
		if (_hold_position.z() <= target_z) {
			_hold_position.z() = target_z;
		}

		// Only transition once the drone has actually reached the target height
		const float altitude_gained = _base_position.z() - current_z;
		if (altitude_gained >= (_target_height - _delta_position)) {
			_state = TakeoffState::Holding;
			_state_elapsed = 0.0f;
			RCLCPP_INFO(_node.get_logger(),
				"Reached %.1f m (actual: %.2f m) — holding position",
				_target_height, altitude_gained);
		}

		// Position + velocity feedforward for smooth tracking
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
		// Hold at target height indefinitely
		_trajectory_setpoint->update(
			px4_ros2::TrajectorySetpoint{}
				.withPosition(_hold_position)
				.withYaw(0.0f)
		);
		break;
	}
}

// ── Executor: arms -> hands off to mode (takeoff handled by state machine) ──

TakeoffHoldExecutor::TakeoffHoldExecutor(rclcpp::Node& node, TakeoffHoldMode& owned_mode)
	: ModeExecutorBase(node, ModeExecutorBase::Settings{Settings::Activation::ActivateAlways}, owned_mode)
	, _node(node)
	, _mode(owned_mode)
{
}

void TakeoffHoldExecutor::onActivate()
{
	RCLCPP_INFO(_node.get_logger(), "TakeoffHold executor — arming");
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

	case State::TakingOff: {
		const float takeoff_amsl = _mode.heightToAmsl(_mode.opticalFlowHeight());
		RCLCPP_INFO(_node.get_logger(),
			"Arm complete — takeoff to %.2f m (%.2f m AMSL)",
			_mode.opticalFlowHeight(), takeoff_amsl);
		takeoff([this](px4_ros2::Result r) { runState(State::Hold, r); }, -0.5f);
		break;
	}

	case State::Hold:
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
