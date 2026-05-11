/**
 * PrecisionLand.cpp — Implementation of the precision landing mode
 *
 * State machine flow:
 *   OpticalFlowInit -> Climbing -> Search -> Approach -> Descend -> Finished
 *
 * Uses a downward-facing camera to find an ArUco tag, fly above it,
 * then descend while tracking with a PI controller.
 */
#include "PrecisionLand.hpp"

#include <px4_ros2/components/node_with_mode.hpp>
#include <px4_ros2/utils/geometry.hpp>

#include <algorithm>
#include <cmath>

namespace precision_land
{

PrecisionLand::PrecisionLand(rclcpp::Node& node)
	: ModeBase(node, ModeBase::Settings{kPrecisionLandModeName})
	, _node(node)
{
	setSkipMessageCompatibilityCheck();

	// PX4 ROS 2 interface objects
	_vehicle_local_position = std::make_shared<px4_ros2::OdometryLocalPosition>(*this);
	_vehicle_attitude = std::make_shared<px4_ros2::OdometryAttitude>(*this);
	_trajectory_setpoint = std::make_shared<px4_ros2::TrajectorySetpointType>(*this);

	auto qos = rclcpp::QoS(1).best_effort();

	// Subscribe to the downward camera's tag detections
	_target_pose_sub = _node.create_subscription<geometry_msgs::msg::PoseStamped>(
		"/target_pose", qos,
		std::bind(&PrecisionLand::targetPoseCallback, this, std::placeholders::_1));

	// Subscribe to PX4's landing detector
	_vehicle_land_detected_sub = _node.create_subscription<px4_msgs::msg::VehicleLandDetected>(
		"/fmu/out/vehicle_land_detected", qos,
		std::bind(&PrecisionLand::vehicleLandDetectedCallback, this, std::placeholders::_1));

	_vehicle_command_pub = _node.create_publisher<px4_msgs::msg::VehicleCommand>(
		"/fmu/in/vehicle_command", rclcpp::QoS(1).best_effort());

	// Publish current state on /drone_state for debugging — reliable so each
	// transition is seen by `ros2 topic echo` even under message load
	_drone_state_publisher = _node.create_publisher<std_msgs::msg::String>(
		"/drone_state", rclcpp::QoS(10));

	// Publish commanded-minus-actual position error on /tracking_error so we can
	// see whether PX4 is actually following our position setpoints
	_tracking_error_publisher = _node.create_publisher<geometry_msgs::msg::Vector3Stamped>(
		"/tracking_error", rclcpp::QoS(10));

	// Rotation from downward camera optical frame to drone body frame
	// Camera optical: Z=down through lens, X=right, Y=forward
	// Drone body:     X=forward, Y=right, Z=down (NED)
	Eigen::Matrix3d down_matrix;
	down_matrix << 0, -1, 0,
			 1, 0, 0,
			 0, 0, 1;
	_down_optical_to_body = Eigen::Quaterniond(down_matrix);

	loadParameters();
}

void PrecisionLand::loadParameters()
{
	_node.declare_parameter<float>("descent_vel", 1.0f);
	_node.declare_parameter<float>("vel_p_gain", 1.5f);
	_node.declare_parameter<float>("vel_i_gain", 0.0f);
	_node.declare_parameter<float>("max_velocity", 3.0f);
	_node.declare_parameter<float>("target_timeout", 3.0f);
	_node.declare_parameter<float>("delta_position", 0.25f);
	_node.declare_parameter<float>("delta_velocity", 0.25f);
	_node.declare_parameter<float>("optical_flow_height", 0.1f);
	_node.declare_parameter<float>("optical_flow_hold_time", 3.0f);
	_node.declare_parameter<float>("target_height", 2.5f);
	_node.declare_parameter<float>("climb_rate", 0.3f);

	_node.get_parameter("descent_vel", _param_descent_vel);
	_node.get_parameter("vel_p_gain", _param_vel_p_gain);
	_node.get_parameter("vel_i_gain", _param_vel_i_gain);
	_node.get_parameter("max_velocity", _param_max_velocity);
	_node.get_parameter("target_timeout", _param_target_timeout);
	_node.get_parameter("delta_position", _param_delta_position);
	_node.get_parameter("delta_velocity", _param_delta_velocity);
	_node.get_parameter("optical_flow_height", _optical_flow_height);
	_node.get_parameter("optical_flow_hold_time", _optical_flow_hold_time);
	_node.get_parameter("target_height", _target_height);
	_node.get_parameter("climb_rate", _climb_rate);
}

void PrecisionLand::targetPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
	// Ignore detections until we've reached search altitude (avoids false triggers during takeoff)
	if (!_search_started) {
		return;
	}

	ArucoTag tag;
	tag.position = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
	tag.orientation = Eigen::Quaterniond(msg->pose.orientation.w, msg->pose.orientation.x,
						     msg->pose.orientation.y, msg->pose.orientation.z).normalized();
	tag.timestamp = _node.now();

	const auto vehicle_position = _vehicle_local_position->positionNed();
	const auto vehicle_velocity = _vehicle_local_position->velocityNed();
	const auto vehicle_attitude = _vehicle_attitude->attitude();

	auto finite3 = [](const auto& vec) {
		for (int i = 0; i < 3; ++i) {
			if (!std::isfinite(vec[i])) {
				return false;
			}
		}
		return true;
	};

	auto finiteQuat = [](const auto& quat) {
		const auto coeffs = quat.coeffs();
		for (int i = 0; i < coeffs.size(); ++i) {
			if (!std::isfinite(coeffs[i])) {
				return false;
			}
		}
		return true;
	};

	if (!finite3(vehicle_position) || !finite3(vehicle_velocity) || !finiteQuat(vehicle_attitude)) {
		return;
	}

	_tag = transformDownTag(tag);
	_tag.timestamp = tag.timestamp;
}

void PrecisionLand::vehicleLandDetectedCallback(const px4_msgs::msg::VehicleLandDetected::SharedPtr msg)
{
	_land_detected = msg->landed;
}

void PrecisionLand::onActivate()
{
	_base_position = _vehicle_local_position->positionNed();
	_hold_position = _base_position;
	_hold_position.z() = _base_position.z() - _optical_flow_height;
	_reached_flow_height = false;
	_state_elapsed = 0.0f;
	_search_started = false;
	_tag = {};
	_target_lost_prev = true;
	_land_detected = false;
	_disarm_sent = false;
	_vel_x_integral = 0.f;
	_vel_y_integral = 0.f;
	switchToState(State::OpticalFlowInit);

	RCLCPP_INFO(_node.get_logger(),
		"PrecisionLand active — optical flow init at %.2f m, then climb to %.1f m",
		_optical_flow_height, _target_height);
}

void PrecisionLand::onDeactivate()
{
	_vel_x_integral = 0.f;
	_vel_y_integral = 0.f;
}

void PrecisionLand::updateSetpoint(float dt_s)
{
	_state_elapsed += dt_s;

	bool target_lost = targetExpired(_node.now());

	if (_state != State::OpticalFlowInit && _state != State::Climbing) {
		if (target_lost && !_target_lost_prev) {
			RCLCPP_INFO(_node.get_logger(), "Target lost while in %s", stateName(_state).c_str());
		} else if (!target_lost && _target_lost_prev) {
			RCLCPP_INFO(_node.get_logger(), "Target acquired");
		}
		_target_lost_prev = target_lost;
	}

	switch (_state) {
	case State::Idle:
		break;

	case State::OpticalFlowInit: {
		const float current_z = _vehicle_local_position->positionNed().z();
		const float altitude_gained = _base_position.z() - current_z;

		if (!_reached_flow_height
			&& altitude_gained >= (_optical_flow_height - _param_delta_position)) {
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

		commandPosition(_hold_position);
		break;
	}

	// --- Climb gradually to search altitude ---
	case State::Climbing: {
		const float target_z = _base_position.z() - _target_height;
		const float current_z = _vehicle_local_position->positionNed().z();

		// Ramp the hold position upward at _climb_rate m/s
		_hold_position.z() -= _climb_rate * dt_s;

		if (_hold_position.z() <= target_z) {
			_hold_position.z() = target_z;
		}

		const float altitude_gained = _base_position.z() - current_z;
		if (altitude_gained >= (_target_height - _param_delta_position)) {
			RCLCPP_INFO(_node.get_logger(),
				"Reached %.1f m (actual: %.2f m) — starting search",
				_target_height, altitude_gained);
			// Build the spiral search pattern and start looking for the tag
			generateSearchWaypoints();
			_search_started = true;
			switchToState(State::Search);
		}

		commandPosition(_hold_position);
		break;
	}

	// --- Fly an expanding spiral pattern looking for the landing tag ---
	case State::Search: {
		// If we see the tag, lock the current altitude and move to approach
		if (_tag.valid() && !target_lost) {
			_approach_altitude = _vehicle_local_position->positionNed().z();
			switchToState(State::Approach);
			break;
		}

		// Otherwise, fly to the next waypoint in the spiral
		auto waypoint_position = _search_waypoints[_search_waypoint_index];
		commandPosition(waypoint_position);

		if (positionReached(waypoint_position)) {
			_search_waypoint_index++;
			// Loop back to start if we've exhausted the pattern
			if (_search_waypoint_index >= static_cast<int>(_search_waypoints.size())) {
				_search_waypoint_index = 0;
			}
		}
		break;
	}

	// --- Fly horizontally to position directly above the tag ---
	case State::Approach: {
		if (target_lost) {
			RCLCPP_INFO(_node.get_logger(), "Target lost during %s", stateName(_state).c_str());
			ModeBase::completed(px4_ros2::Result::ModeFailureOther);
			switchToState(State::Idle);
			return;
		}

		// Target = tag's XY position at our current (locked) altitude
		Eigen::Vector3f target_position(
			static_cast<float>(_tag.position.x()),
			static_cast<float>(_tag.position.y()),
			_approach_altitude);

		commandPosition(target_position);

		if (positionReached(target_position)) {
			switchToState(State::Descend);
		}
		break;
	}

	// --- Descend onto the tag, tracking it with PI control ---
	case State::Descend: {
		if (target_lost) {
			RCLCPP_INFO(_node.get_logger(), "Target lost during %s", stateName(_state).c_str());
			ModeBase::completed(px4_ros2::Result::ModeFailureOther);
			switchToState(State::Idle);
			return;
		}

		// PI controller keeps us centered over the tag while descending
		Eigen::Vector2f vel = calculateVelocitySetpointXY();
		_trajectory_setpoint->update(
			Eigen::Vector3f(vel.x(), vel.y(), _param_descent_vel),
			std::nullopt,
			px4_ros2::quaternionToYaw(_tag.orientation));

		if (_land_detected) {
			switchToState(State::Finished);
		}
		break;
	}

	// --- Landed — disarm and report success ---
	case State::Finished: {
		if (!_disarm_sent) {
			_disarm_sent = true;
			sendDisarm();
			ModeBase::completed(px4_ros2::Result::Success);
		}
		break;
	}
	}
}

PrecisionLand::ArucoTag PrecisionLand::transformDownTag(const ArucoTag& tag) const
{
	// Transform chain: world <- drone <- downward camera <- tag
	ArucoTag world = tag;

	if (!tag.valid()) {
		return world;
	}

	auto vehicle_position = Eigen::Vector3d(_vehicle_local_position->positionNed().cast<double>());
	auto vehicle_orientation = Eigen::Quaterniond(_vehicle_attitude->attitude().cast<double>());

	Eigen::Affine3d drone_transform = Eigen::Translation3d(vehicle_position) * vehicle_orientation;
	Eigen::Affine3d camera_transform = Eigen::Translation3d(0, 0, 0) * _down_optical_to_body;
	Eigen::Affine3d tag_transform = Eigen::Translation3d(tag.position) * tag.orientation;

	Eigen::Affine3d tag_world = drone_transform * camera_transform * tag_transform;
	world.position = tag_world.translation();
	world.orientation = Eigen::Quaterniond(tag_world.rotation()).normalized();

	return world;
}

bool PrecisionLand::targetExpired(const rclcpp::Time& now) const
{
	if (!_tag.valid()) {
		return true;
	}

	return (now - _tag.timestamp).seconds() > _param_target_timeout;
}

bool PrecisionLand::positionReached(const Eigen::Vector3f& target) const
{
	auto position = _vehicle_local_position->positionNed();
	auto velocity = _vehicle_local_position->velocityNed();

	Eigen::Vector3f delta = target - position;

	return (delta.head<2>().norm() < _param_delta_position)
		&& (std::abs(delta.z()) < _param_delta_position)
		&& (velocity.norm() < _param_delta_velocity);
}

Eigen::Vector2f PrecisionLand::calculateVelocitySetpointXY()
{
	// PI controller to keep the drone centered over the tag during descent.
	// Error = how far the drone is from directly above the tag.
	float delta_pos_x = _vehicle_local_position->positionNed().x() - static_cast<float>(_tag.position.x());
	float delta_pos_y = _vehicle_local_position->positionNed().y() - static_cast<float>(_tag.position.y());

	// Accumulate integral (clamped to prevent windup)
	_vel_x_integral += delta_pos_x;
	_vel_y_integral += delta_pos_y;

	float max_integral = _param_max_velocity;
	_vel_x_integral = std::clamp(_vel_x_integral, -max_integral, max_integral);
	_vel_y_integral = std::clamp(_vel_y_integral, -max_integral, max_integral);

	// PI output: negative because we want to move TOWARD the tag (reduce error)
	float vx = -1.f * (delta_pos_x * _param_vel_p_gain + _vel_x_integral * _param_vel_i_gain);
	float vy = -1.f * (delta_pos_y * _param_vel_p_gain + _vel_y_integral * _param_vel_i_gain);

	vx = std::clamp(vx, -_param_max_velocity, _param_max_velocity);
	vy = std::clamp(vy, -_param_max_velocity, _param_max_velocity);

	return Eigen::Vector2f(vx, vy);
}

void PrecisionLand::generateSearchWaypoints()
{
	// Build an expanding spiral search pattern.
	// The drone spirals outward at one altitude, drops down a layer,
	// spirals back inward, drops again, and so on.
	// This covers a wide area while gradually descending.

	double start_x = 0.0;
	double start_y = 0.0;
	double current_z = _vehicle_local_position->positionNed().z();
	auto min_z = -1.0;  // Don't go above this altitude (NED, so -1 = 1m up)

	double max_radius = 2.0;        // Spiral extends to this radius (meters)
	double layer_spacing = 0.5;     // Altitude drop between layers (meters)
	int points_per_layer = 16;      // How many points around each spiral ring
	std::vector<Eigen::Vector3f> waypoints;

	int num_layers = (static_cast<int>((min_z - current_z) / layer_spacing) / 2) < 1 ? 1 : (static_cast<int>((
				 min_z - current_z) / layer_spacing) / 2);

	for (int layer = 0; layer < num_layers; ++layer) {
		std::vector<Eigen::Vector3f> layer_waypoints;
		double radius = 0.0;

		// Spiral outward: radius grows with each point
		for (int point = 0; point < points_per_layer + 1; ++point) {
			double angle = 2.0 * M_PI * point / points_per_layer;
			double x = start_x + radius * cos(angle);
			double y = start_y + radius * sin(angle);
			double z = current_z;

			layer_waypoints.push_back(Eigen::Vector3f(x, y, z));
			radius += max_radius / points_per_layer;
		}

		// Add outward spiral to the path
		waypoints.insert(waypoints.end(), layer_waypoints.begin(), layer_waypoints.end());

		// Drop one layer
		current_z += layer_spacing;

		// Spiral inward at the new altitude (reverse the outward path)
		std::reverse(layer_waypoints.begin(), layer_waypoints.end());
		for (auto& waypoint : layer_waypoints) {
			waypoint.z() = current_z;
		}
		waypoints.insert(waypoints.end(), layer_waypoints.begin(), layer_waypoints.end());

		// Drop another layer for the next outward pass
		current_z += layer_spacing;
	}

	_search_waypoints = waypoints;
}

void PrecisionLand::sendDisarm()
{
	px4_msgs::msg::VehicleCommand cmd{};
	cmd.timestamp = _node.now().nanoseconds() / 1000;
	cmd.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM;
	cmd.param1 = 0.0f;  // 0 = disarm
	cmd.target_system = 1;
	cmd.target_component = 1;
	cmd.source_system = 1;
	cmd.source_component = 1;
	cmd.from_external = true;
	_vehicle_command_pub->publish(cmd);
	RCLCPP_INFO(_node.get_logger(), "Disarm command sent");
}

void PrecisionLand::switchToState(State state)
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

void PrecisionLand::commandPosition(const Eigen::Vector3f& pos)
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

std::string PrecisionLand::stateName(State state) const
{
	switch (state) {
	case State::Idle:
		return "Idle";
	case State::OpticalFlowInit:
		return "OpticalFlowInit";
	case State::Climbing:
		return "Climbing";
	case State::Search:
		return "Search";
	case State::Approach:
		return "Approach";
	case State::Descend:
		return "Descend";
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
	rclcpp::spin(std::make_shared<px4_ros2::NodeWithMode<precision_land::PrecisionLand>>(
		precision_land::kPrecisionLandModeName, precision_land::kPrecisionLandDebugOutput));
	rclcpp::shutdown();
	return 0;
}
