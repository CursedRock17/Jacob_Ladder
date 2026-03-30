#include "FrontApproachPrecisionLandCombined.hpp"

#include <px4_ros2/components/node_with_mode.hpp>
#include <px4_ros2/utils/geometry.hpp>

#include <algorithm>
#include <cmath>

namespace precision_land
{

FrontApproachPrecisionLandCombined::FrontApproachPrecisionLandCombined(rclcpp::Node& node)
	: ModeBase(node, ModeBase::Settings{kFrontToPrecisionModeName, false})
	, _node(node)
{
	setSkipMessageCompatibilityCheck();

	_vehicle_local_position = std::make_shared<px4_ros2::OdometryLocalPosition>(*this);
	_vehicle_attitude = std::make_shared<px4_ros2::OdometryAttitude>(*this);
	_trajectory_setpoint = std::make_shared<px4_ros2::TrajectorySetpointType>(*this);

	auto qos = rclcpp::QoS(1).best_effort();
	_front_target_sub = _node.create_subscription<geometry_msgs::msg::PoseStamped>(
		"/front/target_pose", qos,
		std::bind(&FrontApproachPrecisionLandCombined::frontTargetCallback, this, std::placeholders::_1));

	_land_detected_sub = _node.create_subscription<px4_msgs::msg::VehicleLandDetected>(
		"/fmu/out/vehicle_land_detected", qos,
		std::bind(&FrontApproachPrecisionLandCombined::landDetectedCallback, this, std::placeholders::_1));

	Eigen::Matrix3d front_matrix;
	front_matrix << 0, 0, 1,
			1, 0, 0,
			0, 1, 0;
	_front_optical_to_body = Eigen::Quaterniond(front_matrix);

	loadParameters();
}

void FrontApproachPrecisionLandCombined::loadParameters()
{
	_node.declare_parameter<float>("front_hold_distance", 1.0f);
	_node.declare_parameter<float>("front_target_timeout", 3.0f);
	_node.declare_parameter<float>("front_pid_kp", 0.8f);
	_node.declare_parameter<float>("front_pid_ki", 0.02f);
	_node.declare_parameter<float>("front_pid_kd", 0.3f);
	_node.declare_parameter<float>("front_pid_max_velocity", 1.0f);
	_node.declare_parameter<float>("front_pid_integral_limit", 0.5f);
	_node.declare_parameter<float>("front_pid_kp_z", 0.6f);
	_node.declare_parameter<float>("front_pid_max_velocity_z", 0.6f);

	_node.declare_parameter<float>("precision_target_timeout", 3.0f);
	_node.declare_parameter<float>("precision_descent_velocity", 0.5f);
	_node.declare_parameter<float>("precision_delta_position", 0.25f);
	_node.declare_parameter<float>("precision_delta_velocity", 0.25f);

	_node.get_parameter("front_hold_distance", _param_front_hold_distance);
	_node.get_parameter("front_target_timeout", _param_front_target_timeout);
	_node.get_parameter("front_pid_kp", _param_front_kp);
	_node.get_parameter("front_pid_ki", _param_front_ki);
	_node.get_parameter("front_pid_kd", _param_front_kd);
	_node.get_parameter("front_pid_max_velocity", _param_front_max_vel);
	_node.get_parameter("front_pid_integral_limit", _param_front_int_limit);
	_node.get_parameter("front_pid_kp_z", _param_front_kp_z);
	_node.get_parameter("front_pid_max_velocity_z", _param_front_max_vel_z);

	_node.get_parameter("precision_target_timeout", _param_precision_target_timeout);
	_node.get_parameter("precision_descent_velocity", _param_precision_descent_vel);
	_node.get_parameter("precision_delta_position", _param_precision_delta_position);
	_node.get_parameter("precision_delta_velocity", _param_precision_delta_velocity);
}

void FrontApproachPrecisionLandCombined::frontTargetCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
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

	_front_tag = transformFrontTag(tag);
	_front_tag.timestamp = tag.timestamp;
}

void FrontApproachPrecisionLandCombined::landDetectedCallback(const px4_msgs::msg::VehicleLandDetected::SharedPtr msg)
{
	_land_detected = msg->landed;
}

void FrontApproachPrecisionLandCombined::onActivate()
{
	_front_tag = {};
	_land_detected = false;
	_front_target_lost_prev = true;
	resetFrontController();
	switchToState(State::FrontSearch);
}

void FrontApproachPrecisionLandCombined::onDeactivate()
{
	resetFrontController();
}

void FrontApproachPrecisionLandCombined::updateSetpoint(float dt_s)
{
	const auto now = _node.now();
	bool front_lost = targetExpired(now, _front_tag);

	if (front_lost && !_front_target_lost_prev) {
		RCLCPP_INFO(_node.get_logger(), "Front target lost while in %s", stateName(_state).c_str());
	} else if (!front_lost && _front_target_lost_prev) {
		RCLCPP_INFO(_node.get_logger(), "Front target acquired");
	}
	_front_target_lost_prev = front_lost;

	switch (_state) {
	case State::Idle:
		break;

	case State::FrontSearch: {
		Eigen::Vector3f hold = _vehicle_local_position->positionNed();
		_trajectory_setpoint->updatePosition(hold);

		if (_front_tag.valid() && !front_lost) {
			switchToState(State::FrontApproach);
		}
		break;
	}

	case State::FrontApproach: {
		if (front_lost) {
			resetFrontController();
			switchToState(State::FrontSearch);
			break;
		}

		Eigen::Vector3d vehicle_pos = _vehicle_local_position->positionNed().cast<double>();
		Eigen::Vector3d tag_pos = _front_tag.position;

		Eigen::Vector3d to_tag = tag_pos - vehicle_pos;
		Eigen::Vector2d delta_xy(to_tag.x(), to_tag.y());

		Eigen::Vector2d desired_xy = delta_xy;
		const double distance_xy = delta_xy.norm();

		if (distance_xy > 1e-3) {
			double hold = static_cast<double>(_param_front_hold_distance);
			desired_xy = delta_xy - delta_xy.normalized() * hold;
		}

		Eigen::Vector3f target_position(
			static_cast<float>(vehicle_pos.x() + desired_xy.x()),
			static_cast<float>(vehicle_pos.y() + desired_xy.y()),
			static_cast<float>(tag_pos.z()));

		Eigen::Vector2d error_xy(
			target_position.x() - _vehicle_local_position->positionNed().x(),
			target_position.y() - _vehicle_local_position->positionNed().y());

		_front_integral_xy += error_xy * dt_s;
		_front_integral_xy = _front_integral_xy.cwiseMax(-Eigen::Vector2d::Constant(_param_front_int_limit))
					.cwiseMin(Eigen::Vector2d::Constant(_param_front_int_limit));

		Eigen::Vector2d derivative_xy = Eigen::Vector2d::Zero();
		if (_front_has_prev_error && dt_s > 1e-3f) {
			derivative_xy = (error_xy - _front_prev_error_xy) / dt_s;
		}

		Eigen::Vector2d vel_xy = _param_front_kp * error_xy
					    + _param_front_ki * _front_integral_xy
					    + _param_front_kd * derivative_xy;

		double vel_norm = vel_xy.norm();
		if (vel_norm > _param_front_max_vel) {
			vel_xy = vel_xy.normalized() * _param_front_max_vel;
		}

		float error_z = target_position.z() - _vehicle_local_position->positionNed().z();
		float vel_z = _param_front_kp_z * error_z;
		vel_z = std::clamp(vel_z, -_param_front_max_vel_z, _param_front_max_vel_z);

		Eigen::Vector3f velocity_cmd(static_cast<float>(vel_xy.x()), static_cast<float>(vel_xy.y()), vel_z);
		float desired_yaw = std::atan2(to_tag.y(), to_tag.x());
		_trajectory_setpoint->update(velocity_cmd, std::nullopt, desired_yaw);

		_front_prev_error_xy = error_xy;
		_front_has_prev_error = true;

		if (positionReached(target_position)) {
			resetFrontController();
			switchToState(State::PrecisionApproach);
		}
		break;
	}

	case State::PrecisionApproach: {
		// Descend straight down — ramp latched Z target each tick
		_precision_target.z() += _param_precision_descent_vel * dt_s;

		_trajectory_setpoint->updatePosition(_precision_target);

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

FrontApproachPrecisionLandCombined::ArucoTag FrontApproachPrecisionLandCombined::transformFrontTag(const ArucoTag& tag) const
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

bool FrontApproachPrecisionLandCombined::targetExpired(const rclcpp::Time& now, const ArucoTag& tag) const
{
	if (!tag.valid()) {
		return true;
	}

	return (now - tag.timestamp).seconds() > _param_front_target_timeout;
}

bool FrontApproachPrecisionLandCombined::positionReached(const Eigen::Vector3f& target) const
{
	auto position = _vehicle_local_position->positionNed();
	auto velocity = _vehicle_local_position->velocityNed();

	Eigen::Vector3f delta = target - position;

	return (delta.head<2>().norm() < _param_precision_delta_position)
		&& (std::abs(delta.z()) < _param_precision_delta_position)
		&& (velocity.norm() < _param_precision_delta_velocity);
}

void FrontApproachPrecisionLandCombined::resetFrontController()
{
	_front_integral_xy.setZero();
	_front_prev_error_xy.setZero();
	_front_has_prev_error = false;
}

void FrontApproachPrecisionLandCombined::switchToState(State state)
{
	if (_state == state) {
		return;
	}

	RCLCPP_INFO(_node.get_logger(), "Switching to %s", stateName(state).c_str());
	_state = state;

	if (state == State::FrontSearch) {
		resetFrontController();
	}

	if (state == State::PrecisionApproach) {
		auto pos = _vehicle_local_position->positionNed();
		_precision_target = pos;
		RCLCPP_INFO(_node.get_logger(), "Descending straight down from [%.2f, %.2f, %.2f]",
			_precision_target.x(), _precision_target.y(), _precision_target.z());
	}
}

std::string FrontApproachPrecisionLandCombined::stateName(State state) const
{
	switch (state) {
	case State::Idle:
		return "Idle";
	case State::FrontSearch:
		return "FrontSearch";
	case State::FrontApproach:
		return "FrontApproach";
	case State::PrecisionApproach:
		return "Descend";
	case State::Finished:
		return "Finished";
	default:
		return "Unknown";
	}
}

// ── Executor: arm -> takeoff -> schedule mode -> wait for disarm ──

FrontApproachPrecisionLandExecutor::FrontApproachPrecisionLandExecutor(rclcpp::Node& node, px4_ros2::ModeBase& owned_mode)
	: ModeExecutorBase(node, ModeExecutorBase::Settings{Settings::Activation::ActivateAlways}, owned_mode)
	, _node(node)
{
	setSkipMessageCompatibilityCheck();
	_node.declare_parameter<float>("takeoff_height", 2.5f);
	_node.get_parameter("takeoff_height", _param_takeoff_height);
}

void FrontApproachPrecisionLandExecutor::onActivate()
{
	RCLCPP_INFO(_node.get_logger(), "FrontToPrecisionLand executor — arming and taking off to %.1f m", _param_takeoff_height);
	runState(State::Arming, px4_ros2::Result::Success);
}

void FrontApproachPrecisionLandExecutor::onDeactivate(DeactivateReason reason)
{
}

void FrontApproachPrecisionLandExecutor::runState(State state, px4_ros2::Result result)
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
		RCLCPP_INFO(_node.get_logger(), "Takeoff complete — starting front approach and land");
		scheduleMode(ownedMode().id(), [this](px4_ros2::Result r) {
			runState(State::Disarming, r);
		});
		break;

	case State::Disarming:
		RCLCPP_INFO(_node.get_logger(), "Landed — waiting for disarm");
		waitUntilDisarmed([this](px4_ros2::Result r) {
			RCLCPP_INFO(_node.get_logger(), "Disarmed — FrontToPrecisionLand complete");
		});
		break;
	}
}

} // namespace precision_land

int main(int argc, char* argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<px4_ros2::NodeWithModeExecutor<
		precision_land::FrontApproachPrecisionLandExecutor, precision_land::FrontApproachPrecisionLandCombined>>(
		precision_land::kFrontToPrecisionModeName, precision_land::kFrontToPrecisionDebugOutput));
	rclcpp::shutdown();
	return 0;
}
