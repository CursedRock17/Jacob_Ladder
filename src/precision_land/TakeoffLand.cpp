#include "TakeoffLand.hpp"

#include <px4_ros2/components/node_with_mode.hpp>

using namespace std::chrono_literals;

namespace precision_land
{

// ── Mode: optical flow init, smooth climb, hold ──

TakeoffLandMode::TakeoffLandMode(rclcpp::Node& node)
	: ModeBase(node, Settings{kTakeoffLandModeName, false})
	, _node(node)
	, _state_pub(node)
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
	_node.declare_parameter<float>("hold_duration", 5.0f);
	_node.declare_parameter<float>("descent_rate", 0.3f);
	_node.declare_parameter<float>("land_handoff_height", 0.3f);

	_node.get_parameter("optical_flow_height", _optical_flow_height);
	_node.get_parameter("optical_flow_hold_time", _optical_flow_hold_time);
	_node.get_parameter("target_height", _target_height);
	_node.get_parameter("climb_rate", _climb_rate);
	_node.get_parameter("delta_position", _delta_position);
	_node.get_parameter("hold_duration", _hold_duration);
	_node.get_parameter("descent_rate", _descent_rate);
	_node.get_parameter("land_handoff_height", _land_handoff_height);
}

void TakeoffLandMode::onActivate()
{
	// The executor's PX4 native takeoff() has already lifted us to target height,
	// so just hold the current position for hold_duration, then signal completion
	// (the executor lands on that). OpticalFlowInit/Climbing are no longer used.
	_base_position = _vehicle_local_position->positionNed();
	_hold_position = _base_position;
	_state = TakeoffState::Holding;
	_state_elapsed = 0.0f;
	_reached_flow_height = true;
	_completed_signaled = false;
	_active = true;
	// The mode activates at hover height, so the ground is ~target_height
	// below here (NED: down is +z). Descend to land_handoff_height above it.
	_descent_target_z = _base_position.z() + _target_height - _land_handoff_height;
	_state_pub.set("Hover");

	RCLCPP_INFO(_node.get_logger(),
		"TakeoffLand mode active — holding at [%.2f, %.2f, %.2f] for %.1f s, then land",
		_hold_position.x(), _hold_position.y(), _hold_position.z(), _hold_duration);
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
			"[Holding] height: %.2f m | hold_z: %.2f m | held: %.1f/%.1f s",
			-_vehicle_local_position->positionNed().z(), _hold_position.z(),
			_state_elapsed, _hold_duration);
		_trajectory_setpoint->update(
			px4_ros2::TrajectorySetpoint{}
				.withPosition(_hold_position)
				.withYaw(0.0f)
		);

		// Hovered at target for the full duration — descend before handing
		// off to land(), which drops at MPC_LAND_SPEED (min 0.6 m/s).
		if (_state_elapsed >= _hold_duration) {
			_state = TakeoffState::Descending;
			_state_elapsed = 0.0f;
			_state_pub.set("Descending");
			RCLCPP_INFO(_node.get_logger(),
				"Hover complete (%.1f s at target) — descending at %.1f m/s to %.1f m before land()",
				_hold_duration, _descent_rate, _land_handoff_height);
		}
		break;

	case TakeoffState::Descending: {
		const float current_z = _vehicle_local_position->positionNed().z();
		RCLCPP_INFO_THROTTLE(_node.get_logger(), *_node.get_clock(), 2000,
			"[Descending] height: %.2f m | handoff at: %.2f m",
			-current_z, -_descent_target_z);

		// Ramp the setpoint downward (NED: +z), clamped at the handoff point
		_hold_position.z() += _descent_rate * dt_s;
		if (_hold_position.z() >= _descent_target_z) {
			_hold_position.z() = _descent_target_z;
		}

		const bool at_handoff = current_z >= _descent_target_z - _delta_position;
		_trajectory_setpoint->update(
			px4_ros2::TrajectorySetpoint{}
				.withPosition(_hold_position)
				.withVelocityZ(at_handoff ? 0.0f : _descent_rate)
				.withYaw(0.0f)
		);

		// Reached the handoff height — let the executor's land() finish the
		// last stretch so PX4's land detector owns the touchdown.
		if (!_completed_signaled && at_handoff) {
			_completed_signaled = true;
			_state_pub.set("DescentComplete");
			RCLCPP_INFO(_node.get_logger(),
				"Descent complete (%.2f m) — signaling completion for land()", -current_z);
			ModeBase::completed(px4_ros2::Result::Success);
		}
		break;
	}
	}
}

// ── Executor: arm -> takeoff(1.25) -> hold (timed) -> land -> disarm ──

TakeoffLandExecutor::TakeoffLandExecutor(rclcpp::Node& node, TakeoffLandMode& owned_mode)
	: ModeExecutorBase(node, ModeExecutorBase::Settings{Settings::Activation::ActivateAlways}, owned_mode)
	, _node(node)
	, _mode(owned_mode)
{
  setSkipMessageCompatibilityCheck();
	_node.declare_parameter<float>("safety_timeout", 30.0f);
	_node.get_parameter("safety_timeout", _param_safety_timeout);

	// target_height is declared by the mode (constructed first); reuse it for the
	// native takeoff altitude. Complete when within 0.2 m of target (NED z is neg).
	if (_node.has_parameter("target_height")) {
		_node.get_parameter("target_height", _param_target_height);
	}
	_takeoff_target_z = -(_param_target_height - 0.2f);

	// Detect takeoff completion by altitude — the takeoff() callback does not fire
	// reliably below MIS_TAKEOFF_ALT, so watch local position z directly.
	_local_pos_sub = _node.create_subscription<px4_msgs::msg::VehicleLocalPosition>(
		"/fmu/out/vehicle_local_position", rclcpp::QoS(1).best_effort(),
		[this](const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
			if (_in_takeoff && !_takeoff_complete && msg->z < _takeoff_target_z) {
				_takeoff_complete = true;
				_in_takeoff = false;
				RCLCPP_INFO(_node.get_logger(),
					"Takeoff altitude reached (z=%.2f) — hover then land", msg->z);
				runState(State::Hold, px4_ros2::Result::Success);
			}
		});
}

void TakeoffLandExecutor::onActivate()
{
	RCLCPP_INFO(_node.get_logger(),
		"TakeoffLand executor — arm, PX4 native takeoff to %.1f m, hover, land (safety %.1f s)",
		_param_target_height, _param_safety_timeout);
	_land_triggered = false;
	_in_takeoff = false;
	_takeoff_complete = false;
	_mode.statePublisher().set("Arming");
	runState(State::Arming, px4_ros2::Result::Success);
}

void TakeoffLandExecutor::onDeactivate(DeactivateReason reason)
{
	if (_safety_timer) {
		_safety_timer->cancel();
		_safety_timer.reset();
	}
}

// Land exactly once, whichever fires first: mode completion or the safety timeout
void TakeoffLandExecutor::triggerLanding()
{
	if (_land_triggered) return;
	_land_triggered = true;
	if (_safety_timer) {
		_safety_timer->cancel();
		_safety_timer.reset();
	}
	runState(State::Landing, px4_ros2::Result::Success);
}

void TakeoffLandExecutor::runState(State state, px4_ros2::Result result)
{
	if (result != px4_ros2::Result::Success) {
		RCLCPP_ERROR(_node.get_logger(), "State %i failed: %s", (int)state,
			resultToString(result));
		_mode.statePublisher().set("Failed");
		return;
	}

	switch (state) {
	case State::Arming:
		RCLCPP_INFO(_node.get_logger(), "Arming");
		arm([this](px4_ros2::Result r) { runState(State::TakingOff, r); });
		break;

	case State::TakingOff:
		RCLCPP_INFO(_node.get_logger(), "Armed — PX4 native takeoff to %.1f m", _param_target_height);
		_mode.statePublisher().set("TakingOff");
		_in_takeoff = true;
		_takeoff_complete = false;
		// takeoff() lifts off; the local-position subscriber above detects when the
		// target altitude is reached (callback may not fire at low altitudes).
		takeoff([this](px4_ros2::Result r) {
			if (!_takeoff_complete) {
				_takeoff_complete = true;
				_in_takeoff = false;
				runState(State::Hold, r);
			}
		}, _param_target_height);
		break;

	case State::Hold:
		RCLCPP_INFO(_node.get_logger(), "Takeoff complete — handing off to mode (hover), then land");
		// Land when the mode reports it finished hovering at target.
		scheduleMode(ownedMode().id(), [this](px4_ros2::Result r) {
			RCLCPP_INFO(_node.get_logger(), "Mode ended (%s)", resultToString(r));
			// Only land on a clean completion; a non-success result means the vehicle
			// was already taken over (failsafe / disarm), so there's nothing to land.
			if (r == px4_ros2::Result::Success) {
				triggerLanding();
			}
		});
		// Safety net: land anyway if the climb/hover never completes in time.
		_safety_timer = _node.create_wall_timer(
			std::chrono::duration_cast<std::chrono::nanoseconds>(
				std::chrono::duration<float>(_param_safety_timeout)),
			[this]() {
				RCLCPP_WARN(_node.get_logger(),
					"Mode did not complete within %.1f s — landing (safety)", _param_safety_timeout);
				triggerLanding();
			});
		break;

	case State::Landing:
		_mode.statePublisher().set("Landing");
		land([this](px4_ros2::Result r) { runState(State::Disarming, r); });
		break;

	case State::Disarming:
		RCLCPP_INFO(_node.get_logger(), "Landed — waiting for disarm");
		_mode.statePublisher().set("Landed");
		waitUntilDisarmed([this](px4_ros2::Result r) {
			RCLCPP_INFO(_node.get_logger(), "Disarmed — TakeoffLand complete");
			_mode.statePublisher().set("Disarmed");
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
