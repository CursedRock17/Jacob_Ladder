/**
 * FrontApproach.cpp — Implementation of the front-camera approach mode
 *
 * State machine flow:
 *   Search -> Approach -> Finished
 *
 * The Approach state uses a PID controller to compute velocity commands
 * that smoothly fly the drone toward the detected ArUco tag.
 */
#include "FrontApproach.hpp"

#include <px4_ros2/components/node_with_mode.hpp>
#include <px4_ros2/utils/geometry.hpp>

#include <algorithm>
#include <cmath>

namespace precision_land
{

FrontApproach::FrontApproach(rclcpp::Node& node)
	: ModeBase(node, ModeBase::Settings{kFrontApproachModeName})
	, _node(node)
{
	setSkipMessageCompatibilityCheck();

	// PX4 ROS 2 interface objects
	_vehicle_local_position = std::make_shared<px4_ros2::OdometryLocalPosition>(*this);
	_vehicle_attitude = std::make_shared<px4_ros2::OdometryAttitude>(*this);
	_trajectory_setpoint = std::make_shared<px4_ros2::TrajectorySetpointType>(*this);

	// Subscribe to the front camera's ArUco tag detections
	auto qos = rclcpp::QoS(1).best_effort();
	_front_target_sub = _node.create_subscription<geometry_msgs::msg::PoseStamped>(
		"/front/target_pose", qos,
		std::bind(&FrontApproach::frontTargetCallback, this, std::placeholders::_1));
	
  // Publish current state on /drone_state for debugging — reliable so each
  // transition is seen by `ros2 topic echo` even under message load
  _drone_state_publisher = _node.create_publisher<std_msgs::msg::String>(
    "/drone_state", rclcpp::QoS(10));

  // Publish commanded-minus-actual position error on /tracking_error so we can
  // see whether PX4 is actually following our position setpoints
  _tracking_error_publisher = _node.create_publisher<geometry_msgs::msg::Vector3Stamped>(
    "/tracking_error", rclcpp::QoS(10));

	// Define the rotation from the front camera's optical frame to the drone's body frame.
	// Camera optical: Z=forward, X=right, Y=down
	// Drone body:     X=forward, Y=right, Z=down (in NED)
	Eigen::Matrix3d front_matrix;
	front_matrix << 
      0, 0, 1,
			1, 0, 0,
			0, 1, 0;
	_front_optical_to_body = Eigen::Quaterniond(front_matrix);

	loadParameters();
}

void FrontApproach::loadParameters()
{
	_node.declare_parameter<float>("front_hold_distance", 1.0f);
	_node.declare_parameter<float>("front_delta_position", 0.25f);
	_node.declare_parameter<float>("front_delta_velocity", 0.25f);
	_node.declare_parameter<float>("front_target_timeout", 3.0f);

	_node.declare_parameter<float>("front_pid_kp", 1.2f);
	_node.declare_parameter<float>("front_pid_ki", 0.0f);
	_node.declare_parameter<float>("front_pid_kd", 0.0f);
	_node.declare_parameter<float>("front_pid_max_velocity", 2.5f);
	_node.declare_parameter<float>("front_pid_integral_limit", 1.5f);

	_node.declare_parameter<float>("front_pid_kp_z", 1.0f);
	_node.declare_parameter<float>("front_pid_max_velocity_z", 1.0f);

	_node.get_parameter("front_hold_distance", _param_hold_distance);
	_node.get_parameter("front_delta_position", _param_delta_position);
	_node.get_parameter("front_delta_velocity", _param_delta_velocity);
	_node.get_parameter("front_target_timeout", _param_target_timeout);

	_node.get_parameter("front_pid_kp", _param_kp_xy);
	_node.get_parameter("front_pid_ki", _param_ki_xy);
	_node.get_parameter("front_pid_kd", _param_kd_xy);
	_node.get_parameter("front_pid_max_velocity", _param_max_velocity_xy);
	_node.get_parameter("front_pid_integral_limit", _param_integral_limit);

	_node.get_parameter("front_pid_kp_z", _param_kp_z);
	_node.get_parameter("front_pid_max_velocity_z", _param_max_velocity_z);
}

void FrontApproach::frontTargetCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
	// Parse the tag detection from the camera
	ArucoTag tag;
	tag.position = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
	tag.orientation = Eigen::Quaterniond(msg->pose.orientation.w, msg->pose.orientation.x,
						     msg->pose.orientation.y, msg->pose.orientation.z).normalized();
	tag.timestamp = _node.now();

	// Safety check: make sure all vehicle state values are valid numbers (not NaN/Inf)
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

	// Transform the tag from camera frame to world (NED) frame
	_front_tag = transformTagToWorld(tag);
	_front_tag.timestamp = tag.timestamp;
}

void FrontApproach::onActivate()
{
	_front_tag = {};
	_target_lost_prev = true;
	_approach_target_set = false;
	resetController();
	switchToState(State::Search);
}

void FrontApproach::onDeactivate()
{
	resetController();
}

void FrontApproach::updateSetpoint(float dt_s)
{
	auto now = _node.now();
	bool target_lost = targetExpired(now);

	if (target_lost && !_target_lost_prev) {
		RCLCPP_INFO(_node.get_logger(), "Front target lost while in %s", stateName(_state).c_str());
	} else if (!target_lost && _target_lost_prev) {
		RCLCPP_INFO(_node.get_logger(), "Front target acquired");
	}

	_target_lost_prev = target_lost;

	switch (_state) {
	case State::Idle:
		break;

	// --- Hover in place until we see a tag ---
	case State::Search: {
		Eigen::Vector3f hold = _vehicle_local_position->positionNed();
		commandPosition(hold);

		if (_front_tag.valid() && !target_lost) {
			switchToState(State::Approach);
		}
		break;
	}

	// --- PID-controlled flight 1.5 m forward ---
	case State::Approach: {
		if (target_lost) {
			switchToState(State::Search);
			break;
		}

		// Latch a fixed target 1.5 m straight ahead in the drone's current
		// heading direction on first tick. The PID then drives toward this
		// fixed world-frame point — without latching, the target would
		// slide forward every tick and the error would never shrink.
		if (!_approach_target_set) {
			constexpr float kForwardDistance = 1.5f;
			Eigen::Vector3f current_pos = _vehicle_local_position->positionNed();
			Eigen::Quaterniond q = _vehicle_attitude->attitude().cast<double>();
			Eigen::Vector3d forward_body = q * Eigen::Vector3d::UnitX();
			Eigen::Vector2f forward_xy(
				static_cast<float>(forward_body.x()),
				static_cast<float>(forward_body.y()));
			if (forward_xy.norm() > 1e-6f) {
				forward_xy.normalize();
			}
			_approach_target = current_pos
				+ Eigen::Vector3f(kForwardDistance * forward_xy.x(),
						  kForwardDistance * forward_xy.y(),
						  0.0f);
			_approach_yaw = std::atan2(forward_xy.y(), forward_xy.x());
			_approach_target_set = true;
		}

		const Eigen::Vector3f target_position = _approach_target;

		// --- PID Controller for XY velocity ---
		Eigen::Vector2d error_xy(
			target_position.x() - _vehicle_local_position->positionNed().x(),
			target_position.y() - _vehicle_local_position->positionNed().y());

		// Integral term (accumulates error over time, clamped to prevent windup)
		_integral_xy += error_xy * dt_s;
		_integral_xy = _integral_xy.cwiseMax(-Eigen::Vector2d::Constant(_param_integral_limit))
						.cwiseMin(Eigen::Vector2d::Constant(_param_integral_limit));

		// Derivative term (rate of change of error)
		Eigen::Vector2d derivative_xy = Eigen::Vector2d::Zero();
		if (_has_prev_error && dt_s > 1e-3f) {
			derivative_xy = (error_xy - _prev_error_xy) / dt_s;
		}

		// PID output = P * error + I * integral + D * derivative
		Eigen::Vector2d vel_xy = _param_kp_xy * error_xy
					      + _param_ki_xy * _integral_xy
					      + _param_kd_xy * derivative_xy;

		// Clamp to max velocity
		double vel_xy_norm = vel_xy.norm();
		if (vel_xy_norm > _param_max_velocity_xy) {
			vel_xy = vel_xy.normalized() * _param_max_velocity_xy;
		}

		// Simple proportional control for altitude (Z axis)
		float error_z = target_position.z() - _vehicle_local_position->positionNed().z();
		float vel_z = _param_kp_z * error_z;
		vel_z = std::clamp(vel_z, -_param_max_velocity_z, _param_max_velocity_z);

		// Send velocity command and hold the latched heading
		Eigen::Vector3f velocity_cmd(static_cast<float>(vel_xy.x()), static_cast<float>(vel_xy.y()), vel_z);
		_trajectory_setpoint->update(velocity_cmd, std::nullopt, _approach_yaw);

		_prev_error_xy = error_xy;
		_has_prev_error = true;

      /*
		RCLCPP_INFO_THROTTLE(_node.get_logger(), *_node.get_clock(), 3000,
	             "Target Position North(X): %f, East(Y): %f, Down(Z): %f", 
               target_position.x(), target_position.y(), target_position.z());
      */
		if (positionReached(target_position)) {
		  hold_pos = _vehicle_local_position->positionNed();
			switchToState(State::Finished);
		}
		break;
	}

	// --- Arrived at hold distance — hold position and report success ---
	case State::Finished: {
		commandPosition(hold_pos);
		ModeBase::completed(px4_ros2::Result::Success);
		break;
	}
	}
}

FrontApproach::ArucoTag FrontApproach::transformTagToWorld(const ArucoTag& tag) const
{
	// Transform a tag detection from camera frame to world (NED) frame.
	// The chain is: world <- drone <- camera <- tag
	//   drone_transform:  places the drone in the world
	//   camera_transform: rotates from camera optical frame to drone body
	//   tag_transform:    the tag's pose as seen by the camera
	ArucoTag world = tag;

	if (!tag.valid()) {
		return world;
	}

	auto vehicle_position = Eigen::Vector3d(_vehicle_local_position->positionNed().cast<double>());
	auto vehicle_orientation = Eigen::Quaterniond(_vehicle_attitude->attitude().cast<double>());

	Eigen::Affine3d drone_transform = Eigen::Translation3d(vehicle_position) * vehicle_orientation;
	Eigen::Affine3d camera_transform = Eigen::Translation3d(0, 0, 0) * _front_optical_to_body;
	Eigen::Affine3d tag_transform = Eigen::Translation3d(tag.position) * tag.orientation;

	Eigen::Affine3d tag_world = drone_transform * camera_transform * tag_transform;

	world.position = tag_world.translation();
	world.orientation = Eigen::Quaterniond(tag_world.rotation()).normalized();
	return world;
}

bool FrontApproach::targetExpired(const rclcpp::Time& now) const
{
	if (!_front_tag.valid()) {
		return true;
	}

	return (now - _front_tag.timestamp).seconds() > _param_target_timeout;
}

bool FrontApproach::positionReached(const Eigen::Vector3f& target) const
{
	// "Reached" = close enough in position AND moving slowly enough
	auto position = _vehicle_local_position->positionNed();
	auto velocity = _vehicle_local_position->velocityNed();

	Eigen::Vector3f delta = target - position;

	return (delta.head<2>().norm() < _param_delta_position)   // XY within threshold
		&& (std::abs(delta.z()) < _param_delta_position)      // Z within threshold
		&& (velocity.norm() < _param_delta_velocity);         // Speed below threshold
}

void FrontApproach::resetController()
{
	_integral_xy.setZero();
	_prev_error_xy.setZero();
	_has_prev_error = false;
}

void FrontApproach::switchToState(State state)
{
	if (_state == state) {
		return;
	}

  // Log the current state so we'll know for the future
	RCLCPP_INFO(_node.get_logger(), "Switching to %s", stateName(state).c_str());

  // Also publish the state name on /drone_state so it's visible via `ros2 topic echo`
  std_msgs::msg::String state_msg;
  state_msg.data = stateName(state);
  _drone_state_publisher->publish(state_msg);

  // List our current position
  Eigen::Vector3f current_pos = _vehicle_local_position->positionNed();
	RCLCPP_INFO(_node.get_logger(), "Current Position on State Change: North(X): %f, East(Y): %f, Down(Z): %f", 
              current_pos.x(), current_pos.y(), current_pos.z());
	_state = state;

	if (state == State::Search) {
		resetController();
	}
	if (state == State::Approach) {
		_approach_target_set = false;
	}
}

void FrontApproach::commandPosition(const Eigen::Vector3f& pos)
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

std::string FrontApproach::stateName(State state) const
{
	switch (state) {
	case State::Idle:
		return "Idle";
	case State::Search:
		return "Search";
	case State::Approach:
		return "Approach";
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
	rclcpp::spin(std::make_shared<px4_ros2::NodeWithMode<precision_land::FrontApproach>>(
		precision_land::kFrontApproachModeName, precision_land::kFrontApproachDebugOutput));
	rclcpp::shutdown();
	return 0;
}
