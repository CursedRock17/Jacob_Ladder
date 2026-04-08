#include "TakeoffLand.hpp"

#include <px4_ros2/components/node_with_mode.hpp>

namespace precision_land
{

TakeoffLandMode::TakeoffLandMode(rclcpp::Node& node)
	: ModeBase(node, ModeBase::Settings{kTakeoffLandModeName})
	, _node(node)
{
	setSkipMessageCompatibilityCheck();

	_vehicle_local_position = std::make_shared<px4_ros2::OdometryLocalPosition>(*this);
	_trajectory_setpoint = std::make_shared<px4_ros2::TrajectorySetpointType>(*this);

	auto qos = rclcpp::QoS(1).best_effort();
	_vehicle_land_detected_sub = _node.create_subscription<px4_msgs::msg::VehicleLandDetected>(
		"/fmu/out/vehicle_land_detected", qos,
		std::bind(&TakeoffLandMode::vehicleLandDetectedCallback, this, std::placeholders::_1));

	loadParameters();
}

void TakeoffLandMode::loadParameters()
{
	_node.declare_parameter<float>("optical_flow_height", 0.1f);
	_node.declare_parameter<float>("optical_flow_hold_time", 3.0f);
	_node.declare_parameter<float>("target_height", 2.5f);
	_node.declare_parameter<float>("climb_rate", 0.3f);
	_node.declare_parameter<float>("delta_position", 0.05f);
	_node.declare_parameter<float>("hold_duration", 5.0f);
	_node.declare_parameter<float>("descent_vel", 0.5f);

	_node.get_parameter("optical_flow_height", _optical_flow_height);
	_node.get_parameter("optical_flow_hold_time", _optical_flow_hold_time);
	_node.get_parameter("target_height", _target_height);
	_node.get_parameter("climb_rate", _climb_rate);
	_node.get_parameter("delta_position", _delta_position);
	_node.get_parameter("hold_duration", _hold_duration);
	_node.get_parameter("descent_vel", _descent_vel);
}

void TakeoffLandMode::vehicleLandDetectedCallback(const px4_msgs::msg::VehicleLandDetected::SharedPtr msg)
{
	_land_detected = msg->landed;
}

void TakeoffLandMode::onActivate()
{
	_base_position = _vehicle_local_position->positionNed();
	_hold_position = _base_position;
	_hold_position.z() = _base_position.z() - _optical_flow_height;
	_reached_flow_height = false;
	_state_elapsed = 0.0f;
	_land_detected = false;
	switchToState(State::OpticalFlowInit);

	RCLCPP_INFO(_node.get_logger(),
		"TakeoffLand active — optical flow init at %.2f m, then climb to %.1f m, hold %.1f s, then land",
		_optical_flow_height, _target_height, _hold_duration);
}

void TakeoffLandMode::onDeactivate()
{
	switchToState(State::Idle);
}

void TakeoffLandMode::updateSetpoint(float dt_s)
{
	_state_elapsed += dt_s;

	switch (_state) {
	case State::Idle:
		break;

	case State::OpticalFlowInit: {
		const float current_z = _vehicle_local_position->positionNed().z();
		const float altitude_gained = _base_position.z() - current_z;

		if (!_reached_flow_height
			&& altitude_gained >= (_optical_flow_height - _delta_position)) {
			_reached_flow_height = true;
			_state_elapsed = 0.0f;
			RCLCPP_INFO(_node.get_logger(),
				"Reached optical flow height (%.2f m gained) — holding for %.1f s",
				altitude_gained, _optical_flow_hold_time);
		}

		if (_reached_flow_height && _state_elapsed >= _optical_flow_hold_time) {
			_state_elapsed = 0.0f;
			switchToState(State::Climbing);
		}

		_trajectory_setpoint->updatePosition(_hold_position);
		break;
	}

	case State::Climbing: {
      /*
		const float target_z = _base_position.z() - _target_height;
		const float current_z = _vehicle_local_position->positionNed().z();

		_hold_position.z() -= _climb_rate * dt_s;

		if (_hold_position.z() <= target_z) {
			_hold_position.z() = target_z;
		}

		const float altitude_gained = _base_position.z() - current_z;
		if (altitude_gained >= (_target_height - _delta_position)) {
			_state_elapsed = 0.0f;
			RCLCPP_INFO(_node.get_logger(),
				"Reached %.1f m (actual: %.2f m) — holding for %.1f s",
				_target_height, altitude_gained, _hold_duration);
			switchToState(State::Holding);
		}

		_trajectory_setpoint->updatePosition(_hold_position);
      */
		switchToState(State::Holding);
		break;
	}

	case State::Holding: {
		if (_state_elapsed >= _hold_duration) {
			RCLCPP_INFO(_node.get_logger(), "Hold complete — descending");
			switchToState(State::Descending);
			break;
		}

		_trajectory_setpoint->updatePosition(_hold_position);
		break;
	}

	case State::Descending: {
		Eigen::Vector3f velocity(0.f, 0.f, _descent_vel);
		_trajectory_setpoint->update(velocity, std::nullopt, 0.0f);

		if (_land_detected) {
			switchToState(State::Finished);
		}
		break;
	}

	case State::Finished: {
		Eigen::Vector3f hold = _vehicle_local_position->positionNed();
		_trajectory_setpoint->updatePosition(hold);
		ModeBase::completed(px4_ros2::Result::Success);
		break;
	}
	}
}

void TakeoffLandMode::switchToState(State state)
{
	if (_state == state) {
		return;
	}

	RCLCPP_INFO(_node.get_logger(), "Switching to %s", stateName(state).c_str());
	_state = state;
}

std::string TakeoffLandMode::stateName(State state) const
{
	switch (state) {
	case State::Idle:
		return "Idle";
	case State::OpticalFlowInit:
		return "OpticalFlowInit";
	case State::Climbing:
		return "Climbing";
	case State::Holding:
		return "Holding";
	case State::Descending:
		return "Descending";
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
	rclcpp::spin(std::make_shared<px4_ros2::NodeWithMode<precision_land::TakeoffLandMode>>(
		precision_land::kTakeoffLandModeName, precision_land::kTakeoffLandDebugOutput));
	rclcpp::shutdown();
	return 0;
}
