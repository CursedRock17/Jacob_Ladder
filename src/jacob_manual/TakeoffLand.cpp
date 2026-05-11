/**
 * TakeoffLand.cpp — Implementation of the TakeoffLand flight mode
 *
 * State machine flow:
 *   OpticalFlowInit -> Climbing -> Holding -> Descending -> Finished
 *
 * See TakeoffLand.hpp for a description of each state.
 */
#include "TakeoffLand.hpp"

#include <px4_ros2/components/node_with_mode.hpp>

namespace precision_land
{

TakeoffLandMode::TakeoffLandMode(rclcpp::Node& node)
	: ModeBase(node, ModeBase::Settings{kTakeoffLandModeName})
	, _node(node)
{
	// Skip px4_msgs version check (useful during development)
	setSkipMessageCompatibilityCheck();

	// Create PX4 ROS 2 interface objects for position reading and setpoint sending
	_vehicle_local_position = std::make_shared<px4_ros2::OdometryLocalPosition>(*this);
	_trajectory_setpoint = std::make_shared<px4_ros2::TrajectorySetpointType>(*this);

	// Subscribe to PX4's landing detector so we know when we've touched down
	auto qos = rclcpp::QoS(1).best_effort();
	_vehicle_land_detected_sub = _node.create_subscription<px4_msgs::msg::VehicleLandDetected>(
		"/fmu/out/vehicle_land_detected", qos,
		std::bind(&TakeoffLandMode::vehicleLandDetectedCallback, this, std::placeholders::_1));

	// Publish current state on /drone_state for debugging — reliable so each
	// transition is seen by `ros2 topic echo` even under message load
	_drone_state_publisher = _node.create_publisher<std_msgs::msg::String>(
		"/drone_state", rclcpp::QoS(10));

	// Publish commanded-minus-actual position error on /tracking_error so we can
	// see whether PX4 is actually following our position setpoints
	_tracking_error_publisher = _node.create_publisher<geometry_msgs::msg::Vector3Stamped>(
		"/tracking_error", rclcpp::QoS(10));

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
	// Record where the drone is right now as the reference point
	_base_position = _vehicle_local_position->positionNed();
	// Set initial target slightly above base (NED: negative z = up)
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

	// --- Rise to a low height so the optical flow sensor can initialize ---
	case State::OpticalFlowInit: {
		const float current_z = _vehicle_local_position->positionNed().z();
		// In NED, z gets more negative as we go up, so "gained" = base - current
		const float altitude_gained = _base_position.z() - current_z;

		// Check if we've reached the flow height (within tolerance)
		if (!_reached_flow_height
			&& altitude_gained >= (_optical_flow_height - _delta_position)) {
			_reached_flow_height = true;
			_state_elapsed = 0.0f;
			RCLCPP_INFO(_node.get_logger(),
				"Reached optical flow height (%.2f m gained) — holding for %.1f s",
				altitude_gained, _optical_flow_hold_time);
		}

		// After holding long enough at flow height, start climbing
		if (_reached_flow_height && _state_elapsed >= _optical_flow_hold_time) {
			_state_elapsed = 0.0f;
			switchToState(State::Climbing);
		}

		commandPosition(_hold_position);
		break;
	}

	// --- Climb to the target altitude (currently skipped — goes straight to hold) ---
	case State::Climbing: {
		// NOTE: Gradual climb is commented out; currently jumps directly to Holding.
		// Uncomment the block below to enable smooth climbing at _climb_rate m/s.
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

		commandPosition(_hold_position);
		*/
		switchToState(State::Holding);
		break;
	}

	// --- Hold position for _hold_duration seconds, then descend ---
	case State::Holding: {
		if (_state_elapsed >= _hold_duration) {
			RCLCPP_INFO(_node.get_logger(), "Hold complete — descending");
			switchToState(State::Descending);
			break;
		}

		commandPosition(_hold_position);
		break;
	}

	// --- Descend at a constant velocity until PX4 detects landing ---
	case State::Descending: {
		// Current Poistion
    Eigen::Vector3f current_position = _vehicle_local_position->positionNed();
		// Positive z velocity = downward in NED
		Eigen::Vector3f velocity(0.f, 0.f, _descent_vel);
		_trajectory_setpoint->update(velocity, std::nullopt, 0.0f);

		if (_land_detected || current_position.z() >= 0.10f) {
			switchToState(State::Finished);
		}
		break;
	}

	// --- Landed — hold current position and tell PX4 we're done ---
	case State::Finished: {
		Eigen::Vector3f hold = _vehicle_local_position->positionNed();
		commandPosition(hold);
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

	std_msgs::msg::String state_msg;
	state_msg.data = stateName(state);
	_drone_state_publisher->publish(state_msg);

	_state = state;
}

void TakeoffLandMode::commandPosition(const Eigen::Vector3f& pos)
{
	_trajectory_setpoint->updatePosition(pos);

	const auto actual = _vehicle_local_position->positionNed();
	geometry_msgs::msg::Vector3Stamped err;
	err.header.stamp = _node.now();
	err.header.frame_id = "odom";
	err.vector.x = pos.x() - actual.x();
	err.vector.y = pos.y() - actual.y();
	err.vector.z = pos.z() - actual.z();
	_tracking_error_publisher->publish(err);
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

// Entry point — NodeWithMode registers our mode with PX4 and spins the ROS node
int main(int argc, char* argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<px4_ros2::NodeWithMode<precision_land::TakeoffLandMode>>(
		precision_land::kTakeoffLandModeName, precision_land::kTakeoffLandDebugOutput));
	rclcpp::shutdown();
	return 0;
}
