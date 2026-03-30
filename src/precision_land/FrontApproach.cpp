#include "FrontApproach.hpp"

#include <px4_ros2/components/node_with_mode.hpp>
#include <px4_ros2/utils/geometry.hpp>

#include <algorithm>
#include <cmath>

namespace precision_land
{

FrontApproach::FrontApproach(rclcpp::Node& node)
	: ModeBase(node, ModeBase::Settings{kFrontApproachModeName, false})
	, _node(node)
{
	setSkipMessageCompatibilityCheck();

	_vehicle_local_position = std::make_shared<px4_ros2::OdometryLocalPosition>(*this);
	_vehicle_attitude = std::make_shared<px4_ros2::OdometryAttitude>(*this);
	_trajectory_setpoint = std::make_shared<px4_ros2::TrajectorySetpointType>(*this);

	auto qos = rclcpp::QoS(1).best_effort();
	_front_target_sub = _node.create_subscription<geometry_msgs::msg::PoseStamped>(
		"/front/target_pose", qos,
		std::bind(&FrontApproach::frontTargetCallback, this, std::placeholders::_1));

	Eigen::Matrix3d front_matrix;
	front_matrix << 0, 0, 1,
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

	_front_tag = transformTagToWorld(tag);
	_front_tag.timestamp = tag.timestamp;
}

void FrontApproach::onActivate()
{
	_front_tag = {};
	_target_lost_prev = true;
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

	case State::Search: {
		Eigen::Vector3f hold = _vehicle_local_position->positionNed();
		_trajectory_setpoint->updatePosition(hold);

		if (_front_tag.valid() && !target_lost) {
			switchToState(State::Approach);
		}
		break;
	}

	case State::Approach: {
		if (target_lost) {
			switchToState(State::Search);
			break;
		}

		Eigen::Vector3d vehicle_pos = _vehicle_local_position->positionNed().cast<double>();
		Eigen::Vector3d tag_pos = _front_tag.position;

		Eigen::Vector3d to_tag = tag_pos - vehicle_pos;
		Eigen::Vector2d delta_xy(to_tag.x(), to_tag.y());

		Eigen::Vector2d desired_xy = delta_xy;
		double distance_xy = delta_xy.norm();

		if (distance_xy > 1e-3) {
			double hold = static_cast<double>(_param_hold_distance);
			if (distance_xy > hold) {
				desired_xy = delta_xy - delta_xy.normalized() * hold;
			} else {
				desired_xy = Eigen::Vector2d::Zero();
			}
		}

		Eigen::Vector3f target_position(
			static_cast<float>(vehicle_pos.x() + desired_xy.x()),
			static_cast<float>(vehicle_pos.y() + desired_xy.y()),
			static_cast<float>(tag_pos.z()));

		Eigen::Vector2d error_xy(
			target_position.x() - _vehicle_local_position->positionNed().x(),
			target_position.y() - _vehicle_local_position->positionNed().y());

		_integral_xy += error_xy * dt_s;
		_integral_xy = _integral_xy.cwiseMax(-Eigen::Vector2d::Constant(_param_integral_limit))
						.cwiseMin(Eigen::Vector2d::Constant(_param_integral_limit));

		Eigen::Vector2d derivative_xy = Eigen::Vector2d::Zero();
		if (_has_prev_error && dt_s > 1e-3f) {
			derivative_xy = (error_xy - _prev_error_xy) / dt_s;
		}

		Eigen::Vector2d vel_xy = _param_kp_xy * error_xy
					      + _param_ki_xy * _integral_xy
					      + _param_kd_xy * derivative_xy;

		double vel_xy_norm = vel_xy.norm();
		if (vel_xy_norm > _param_max_velocity_xy) {
			vel_xy = vel_xy.normalized() * _param_max_velocity_xy;
		}

		float error_z = target_position.z() - _vehicle_local_position->positionNed().z();
		float vel_z = _param_kp_z * error_z;
		vel_z = std::clamp(vel_z, -_param_max_velocity_z, _param_max_velocity_z);

		Eigen::Vector3f velocity_cmd(static_cast<float>(vel_xy.x()), static_cast<float>(vel_xy.y()), vel_z);

		float desired_yaw = std::atan2(to_tag.y(), to_tag.x());
		_trajectory_setpoint->update(velocity_cmd, std::nullopt, desired_yaw);

		_prev_error_xy = error_xy;
		_has_prev_error = true;

		if (positionReached(target_position)) {
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

FrontApproach::ArucoTag FrontApproach::transformTagToWorld(const ArucoTag& tag) const
{
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
	auto position = _vehicle_local_position->positionNed();
	auto velocity = _vehicle_local_position->velocityNed();

	Eigen::Vector3f delta = target - position;

	return (delta.head<2>().norm() < _param_delta_position)
		&& (std::abs(delta.z()) < _param_delta_position)
		&& (velocity.norm() < _param_delta_velocity);
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

	RCLCPP_INFO(_node.get_logger(), "Switching to %s", stateName(state).c_str());
	_state = state;

	if (state == State::Search) {
		resetController();
	}
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

// ── Executor: arm -> takeoff -> schedule FrontApproach mode ──

FrontApproachExecutor::FrontApproachExecutor(rclcpp::Node& node, px4_ros2::ModeBase& owned_mode)
	: ModeExecutorBase(node, ModeExecutorBase::Settings{Settings::Activation::ActivateAlways}, owned_mode)
	, _node(node)
{
	setSkipMessageCompatibilityCheck();
	_node.declare_parameter<float>("takeoff_height", 2.5f);
	_node.get_parameter("takeoff_height", _param_takeoff_height);
}

void FrontApproachExecutor::onActivate()
{
	RCLCPP_INFO(_node.get_logger(), "FrontApproach executor — arming and taking off to %.1f m", _param_takeoff_height);
	runState(State::Arming, px4_ros2::Result::Success);
}

void FrontApproachExecutor::onDeactivate(DeactivateReason reason)
{
}

void FrontApproachExecutor::runState(State state, px4_ros2::Result result)
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
		takeoff([this](px4_ros2::Result r) { runState(State::Approaching, r); },
			_param_takeoff_height);
		break;

	case State::Approaching:
		RCLCPP_INFO(_node.get_logger(), "Takeoff complete — starting front approach");
		scheduleMode(ownedMode().id(), [this](px4_ros2::Result r) {
			RCLCPP_INFO(_node.get_logger(), "FrontApproach mode ended (%s)", resultToString(r));
		});
		break;
	}
}

} // namespace precision_land

int main(int argc, char* argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<px4_ros2::NodeWithModeExecutor<
		precision_land::FrontApproachExecutor, precision_land::FrontApproach>>(
		precision_land::kFrontApproachModeName, precision_land::kFrontApproachDebugOutput));
	rclcpp::shutdown();
	return 0;
}
