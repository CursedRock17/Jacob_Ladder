#include "PrecisionLandAuto.hpp"

#include <px4_ros2/components/node_with_mode.hpp>
#include <px4_ros2/utils/geometry.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace px4_ros2::literals;

namespace precision_land_auto
{

PrecisionLandAuto::PrecisionLandAuto(rclcpp::Node& node)
	: ModeBase(node, Settings{kPrecisionLandAutoModeName, false})
	, _node(node)
{
	setSkipMessageCompatibilityCheck();

	_trajectory_setpoint = std::make_shared<px4_ros2::TrajectorySetpointType>(*this);

	_vehicle_local_position = std::make_shared<px4_ros2::OdometryLocalPosition>(*this);

	_vehicle_attitude = std::make_shared<px4_ros2::OdometryAttitude>(*this);

	_target_pose_sub = _node.create_subscription<geometry_msgs::msg::PoseStamped>("/target_pose",
			   rclcpp::QoS(1).best_effort(), std::bind(&PrecisionLandAuto::targetPoseCallback, this, std::placeholders::_1));

	_vehicle_land_detected_sub = _node.create_subscription<px4_msgs::msg::VehicleLandDetected>("/fmu/out/vehicle_land_detected",
				     rclcpp::QoS(1).best_effort(), std::bind(&PrecisionLandAuto::vehicleLandDetectedCallback, this, std::placeholders::_1));

	loadParameters();
}

void PrecisionLandAuto::loadParameters()
{
	_node.declare_parameter<float>("descent_vel", 1.0);
	_node.declare_parameter<float>("vel_p_gain", 1.5);
	_node.declare_parameter<float>("vel_i_gain", 0.0);
	_node.declare_parameter<float>("max_velocity", 3.0);
	_node.declare_parameter<float>("target_timeout", 3.0);
	_node.declare_parameter<float>("delta_position", 0.25);
	_node.declare_parameter<float>("delta_velocity", 0.25);
	_node.declare_parameter<float>("optical_flow_height", 0.1f);
	_node.declare_parameter<float>("optical_flow_hold_time", 3.0f);
	_node.declare_parameter<float>("target_height", 2.5f);
	_node.declare_parameter<float>("climb_rate", 0.3f);
	_node.declare_parameter<float>("land_z_tolerance", 0.15f);

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
	_node.get_parameter("land_z_tolerance", _param_land_z_tolerance);

	RCLCPP_INFO(_node.get_logger(), "descent_vel: %f", _param_descent_vel);
	RCLCPP_INFO(_node.get_logger(), "vel_i_gain: %f", _param_vel_i_gain);
}

void PrecisionLandAuto::vehicleLandDetectedCallback(const px4_msgs::msg::VehicleLandDetected::SharedPtr msg)
{
	_land_detected = msg->landed;
}

void PrecisionLandAuto::targetPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
	if (_search_started) {
		auto tag = ArucoTag {
			.position = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z),
			.orientation = Eigen::Quaterniond(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z),
			.timestamp = _node.now(),
		};

		// Save tag position/orientation in NED world frame
		_tag = getTagWorld(tag);
	}

}

PrecisionLandAuto::ArucoTag PrecisionLandAuto::getTagWorld(const ArucoTag& tag)
{
	// Convert from optical to NED
	// Optical: X right, Y down, Z away from lens
	// NED: X forward, Y right, Z away from viewer
	Eigen::Matrix3d R;
	R << 0, -1, 0,
	1, 0, 0,
	0, 0, 1;
	Eigen::Quaterniond quat_NED(R);

	auto vehicle_position = Eigen::Vector3d(_vehicle_local_position->positionNed().cast<double>());
	auto vehicle_orientation = Eigen::Quaterniond(_vehicle_attitude->attitude().cast<double>());

	Eigen::Affine3d drone_transform = Eigen::Translation3d(vehicle_position) * vehicle_orientation;
	Eigen::Affine3d camera_transform = Eigen::Translation3d(0, 0, 0) * quat_NED;
	Eigen::Affine3d tag_transform = Eigen::Translation3d(tag.position) * tag.orientation;
	Eigen::Affine3d tag_world_transform = drone_transform * camera_transform * tag_transform;

	ArucoTag world_tag = {
		.position = tag_world_transform.translation(),
		.orientation = Eigen::Quaterniond(tag_world_transform.rotation()),
		.timestamp = tag.timestamp,
	};

	return world_tag;
}

void PrecisionLandAuto::onActivate()
{
	_base_position = _vehicle_local_position->positionNed();
	_hold_position = _base_position;
	_hold_position.z() = _base_position.z() - _optical_flow_height;
	_reached_flow_height = false;
	_state_elapsed = 0.0f;
	_search_started = false;

	switchToState(State::OpticalFlowInit);

	RCLCPP_INFO(_node.get_logger(),
		"PrecisionLandAuto active — optical flow init at %.2f m, then climb to %.1f m",
		_optical_flow_height, _target_height);
}

void PrecisionLandAuto::onDeactivate()
{
	// No-op
}

void PrecisionLandAuto::updateSetpoint(float dt_s)
{
	_state_elapsed += dt_s;

	// Target tracking for states that need it
	bool target_lost = checkTargetTimeout();

	if (_state != State::OpticalFlowInit && _state != State::Climbing) {
		if (target_lost && !_target_lost_prev) {
			RCLCPP_INFO(_node.get_logger(), "Target lost: State %s", stateName(_state).c_str());
		} else if (!target_lost && _target_lost_prev) {
			RCLCPP_INFO(_node.get_logger(), "Target acquired");
		}
		_target_lost_prev = target_lost;
	}

	// State machine
	switch (_state) {

	case State::OpticalFlowInit: {
		const float current_z = _vehicle_local_position->positionNed().z();
		const float altitude_gained = _base_position.z() - current_z;
		RCLCPP_INFO_THROTTLE(_node.get_logger(), *_node.get_clock(), 2000,
			"[OpticalFlowInit] height: %.2f m | alt gained: %.2f m | target: %.2f m | elapsed: %.1f s",
			-current_z, altitude_gained, _optical_flow_height, _state_elapsed);

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

		_trajectory_setpoint->update(
			px4_ros2::TrajectorySetpoint{}
				.withPosition(_hold_position)
				.withYaw(0.0f)
		);
		break;
	}

	case State::Climbing: {
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
		if (altitude_gained >= (_target_height - _param_delta_position)) {
			RCLCPP_INFO(_node.get_logger(),
				"Reached %.1f m (actual: %.2f m) — starting search",
				_target_height, altitude_gained);
			generateSearchWaypoints();
			_search_started = true;
			switchToState(State::Search);
		}

		_trajectory_setpoint->update(
			px4_ros2::TrajectorySetpoint{}
				.withPosition(_hold_position)
				.withVelocityZ(-_climb_rate)
				.withYaw(0.0f)
		);
		break;
	}

	case State::Idle: {
		// No-op -- just spin
		break;
	}

	case State::Search: {
		RCLCPP_INFO_THROTTLE(_node.get_logger(), *_node.get_clock(), 3000,
			"[Search] waypoint %d/%zu | pos: [%.2f, %.2f, %.2f]",
			_search_waypoint_index, _search_waypoints.size(),
			_vehicle_local_position->positionNed().x(),
			_vehicle_local_position->positionNed().y(),
			_vehicle_local_position->positionNed().z());

		if (!std::isnan(_tag.position.x())) {
			_approach_altitude = _vehicle_local_position->positionNed().z();
			switchToState(State::Approach);
			break;
		}

		auto waypoint_position = _search_waypoints[_search_waypoint_index];

		_trajectory_setpoint->updatePosition(waypoint_position);

		if (positionReached(waypoint_position)) {
			_search_waypoint_index++;

			// If we have searched all waypoints, start over
			if (_search_waypoint_index >= static_cast<int>(_search_waypoints.size())) {
				_search_waypoint_index = 0;
			}
		}

		break;
	}

	case State::Approach: {
		RCLCPP_INFO_THROTTLE(_node.get_logger(), *_node.get_clock(), 2000,
			"[Approach] tag: [%.2f, %.2f, %.2f] | drone: [%.2f, %.2f, %.2f]",
			_tag.position.x(), _tag.position.y(), _tag.position.z(),
			_vehicle_local_position->positionNed().x(),
			_vehicle_local_position->positionNed().y(),
			_vehicle_local_position->positionNed().z());

		if (target_lost) {
			RCLCPP_INFO(_node.get_logger(), "Failed! Target lost during %s", stateName(_state).c_str());
			ModeBase::completed(px4_ros2::Result::ModeFailureOther);
			switchToState(State::Idle);
			return;
		}

		// Approach using position setpoints
		auto target_position = Eigen::Vector3f(_tag.position.x(), _tag.position.y(), _approach_altitude);

		_trajectory_setpoint->updatePosition(target_position);

		if (positionReached(target_position)) {
			switchToState(State::Descend);
		}

		break;
	}

	case State::Descend: {
		RCLCPP_INFO_THROTTLE(_node.get_logger(), *_node.get_clock(), 2000,
			"[Descend] height: %.2f m | vel_z: %.2f m/s | tag: [%.2f, %.2f, %.2f]",
			-_vehicle_local_position->positionNed().z(), _param_descent_vel,
			_tag.position.x(), _tag.position.y(), _tag.position.z());

		if (target_lost) {
			RCLCPP_INFO(_node.get_logger(), "Failed! Target lost during %s", stateName(_state).c_str());
			ModeBase::completed(px4_ros2::Result::ModeFailureOther);
			switchToState(State::Idle);
			return;
		}

		// Descend using velocity setpoints and P velocity controller for XY
		Eigen::Vector2f vel = calculateVelocitySetpointXY();
		_trajectory_setpoint->update(Eigen::Vector3f(vel.x(), vel.y(), _param_descent_vel), std::nullopt,
					     px4_ros2::quaternionToYaw(_tag.orientation));

		const float current_z = _vehicle_local_position->positionNed().z();
		const bool near_ground = current_z >= -_param_land_z_tolerance;

		if (_land_detected || near_ground) {
			RCLCPP_INFO(_node.get_logger(),
				"Landing detected (land_detected=%d, z=%.3f)",
				_land_detected, current_z);
			switchToState(State::Finished);
		}

		break;
	}

	case State::Finished: {
		ModeBase::completed(px4_ros2::Result::Success);
		break;
	}
	} // end switch/case
}

Eigen::Vector2f PrecisionLandAuto::calculateVelocitySetpointXY()
{
	float p_gain = _param_vel_p_gain;
	float i_gain = _param_vel_i_gain;

	// P component
	float delta_pos_x = _vehicle_local_position->positionNed().x() - _tag.position.x();
	float delta_pos_y = _vehicle_local_position->positionNed().y() - _tag.position.y();

	// I component
	_vel_x_integral += delta_pos_x;
	_vel_y_integral += delta_pos_y;
	float max_integral = _param_max_velocity;
	_vel_x_integral = std::clamp(_vel_x_integral, -1.f * max_integral, max_integral);
	_vel_y_integral = std::clamp(_vel_y_integral, -1.f * max_integral, max_integral);

	float Xp = delta_pos_x * p_gain;
	float Xi = _vel_x_integral * i_gain;
	float Yp = delta_pos_y * p_gain;
	float Yi = _vel_y_integral * i_gain;

	// Sum P and I gains
	float vx = -1.f * (Xp + Xi);
	float vy = -1.f * (Yp + Yi);

	// 0.1m/s min vel and 3m/s max vel
	vx = std::clamp(vx, -1.f * _param_max_velocity, _param_max_velocity);
	vy = std::clamp(vy, -1.f * _param_max_velocity, _param_max_velocity);

	return Eigen::Vector2f(vx, vy);
}

bool PrecisionLandAuto::checkTargetTimeout()
{
	if (!_tag.valid()) {
		return true;
	}

	if (_node.now().seconds() - _tag.timestamp.seconds() > _param_target_timeout) {
		return true;
	}

	return false;
}

void PrecisionLandAuto::generateSearchWaypoints()
{
	// Generate spiral search waypoints
	// The search waypoints are generated in the NED frame
	// Parameters for the search pattern
	double start_x = 0.0;
	double start_y = 0.0;
	double current_z = _vehicle_local_position->positionNed().z();
	auto min_z = -1.0;

	double max_radius = 2.0;
	double layer_spacing = 0.5;
	int points_per_layer = 16;
	std::vector<Eigen::Vector3f> waypoints;

	// Generate waypoints
	// Calculate the number of layers needed
	int num_layers = (static_cast<int>((min_z - current_z) / layer_spacing) / 2) < 1 ? 1 : (static_cast<int>((
				 min_z - current_z) / layer_spacing) / 2);

	// Generate waypoints
	for (int layer = 0; layer < num_layers; ++layer) {
		std::vector<Eigen::Vector3f> layer_waypoints;

		// Spiral out to max radius
		double radius = 0.0;

		for (int point = 0; point < points_per_layer + 1; ++point) {
			double angle = 2.0 * M_PI * point / points_per_layer;
			double x = start_x + radius * cos(angle);
			double y = start_y + radius * sin(angle);
			double z = current_z;

			layer_waypoints.push_back(Eigen::Vector3f(x, y, z));
			radius += max_radius / points_per_layer;
		}

		// Push the spiral out waypoints to the main waypoints vector
		waypoints.insert(waypoints.end(), layer_waypoints.begin(), layer_waypoints.end());

		// Decrease the altitude for the inward spiral
		current_z += layer_spacing;

		// Reverse the layer waypoints for spiral in
		std::reverse(layer_waypoints.begin(), layer_waypoints.end());

		// Adjust the z-coordinate for the inward spiral
		for (auto& waypoint : layer_waypoints) {
			waypoint.z() = current_z;
		}

		// Push the reversed waypoints to the main waypoints vector
		waypoints.insert(waypoints.end(), layer_waypoints.begin(), layer_waypoints.end());

		// Decrease the altitude for the next outward spiral
		current_z += layer_spacing;
	}

	_search_waypoints = waypoints;
}

bool PrecisionLandAuto::positionReached(const Eigen::Vector3f& target) const
{
	auto position = _vehicle_local_position->positionNed();
	auto velocity = _vehicle_local_position->velocityNed();

	const auto delta_pos = target - position;
	// NOTE: this does NOT handle a moving target!
	return (delta_pos.norm() < _param_delta_position) && (velocity.norm() < _param_delta_velocity);
}

std::string PrecisionLandAuto::stateName(State state)
{
	switch (state) {
	case State::OpticalFlowInit:
		return "OpticalFlowInit";

	case State::Climbing:
		return "Climbing";

	case State::Idle:
		return "Idle";

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

void PrecisionLandAuto::switchToState(State state)
{
	RCLCPP_INFO(_node.get_logger(), "Switching to %s", stateName(state).c_str());
	_state = state;
}

// ── Executor: arm -> takeoff -> schedule mode -> disarm ──

PrecisionLandAutoExecutor::PrecisionLandAutoExecutor(rclcpp::Node& node, px4_ros2::ModeBase& owned_mode)
	: ModeExecutorBase(node, ModeExecutorBase::Settings{Settings::Activation::ActivateAlways}, owned_mode)
	, _node(node)
{
	// Mode constructor already declared target_height; grab it here for altitude monitoring
	if (_node.has_parameter("target_height")) {
		_node.get_parameter("target_height", _target_height);
	}
	// In NED, z is negative when airborne. Trigger when within 0.2m of target height.
	_takeoff_target_z = -(_target_height - 0.2f);

	// Subscribe to local position so we can detect takeoff completion by altitude,
	// since the PX4 takeoff() callback does not reliably fire at custom altitudes.
	_local_pos_sub = _node.create_subscription<px4_msgs::msg::VehicleLocalPosition>(
		"/fmu/out/vehicle_local_position",
		rclcpp::QoS(1).best_effort(),
		[this](const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
			if (_in_takeoff && !_takeoff_complete && msg->z < _takeoff_target_z) {
				_takeoff_complete = true;
				_in_takeoff = false;
				RCLCPP_INFO(_node.get_logger(),
					"Takeoff altitude reached (z=%.2f) — starting precision land", msg->z);
				runState(State::Approaching, px4_ros2::Result::Success);
			}
		});
}

void PrecisionLandAutoExecutor::onActivate()
{
	if (_mission_complete) {
		RCLCPP_INFO(_node.get_logger(), "Mission already complete — ignoring re-activation");
		return;
	}
	_in_takeoff = false;
	_takeoff_complete = false;
	RCLCPP_INFO(_node.get_logger(), "PrecisionLandAuto executor — arming");
	runState(State::Arming, px4_ros2::Result::Success);
}

void PrecisionLandAutoExecutor::onDeactivate(DeactivateReason reason)
{
	_in_takeoff = false;
}

void PrecisionLandAutoExecutor::runState(State state, px4_ros2::Result result)
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
		RCLCPP_INFO(_node.get_logger(), "Arm complete — taking off to %.1f m", _target_height);
		_in_takeoff = true;
		_takeoff_complete = false;
		// takeoff() gets the drone airborne; position subscriber detects when height is reached
		// because the built-in TAKEOFF callback does not fire at altitudes below MIS_TAKEOFF_ALT
		takeoff([this](px4_ros2::Result r) {
			if (!_takeoff_complete) {
				_takeoff_complete = true;
				_in_takeoff = false;
				runState(State::Approaching, r);
			}
		}, _target_height);
		break;

	case State::Approaching:
		RCLCPP_INFO(_node.get_logger(), "Takeoff complete — starting precision land");
		scheduleMode(ownedMode().id(), [this](px4_ros2::Result r) {
			runState(State::Disarming, r);
		});
		break;

	case State::Disarming:
		RCLCPP_INFO(_node.get_logger(), "Landed — disarming");
		disarm([this](px4_ros2::Result r) {
			_mission_complete = true;
			RCLCPP_INFO(_node.get_logger(), "Disarmed — PrecisionLandAuto complete");
		});
		break;
	}
}

} // namespace precision_land_auto

int main(int argc, char* argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<px4_ros2::NodeWithModeExecutor<
		precision_land_auto::PrecisionLandAutoExecutor, precision_land_auto::PrecisionLandAuto>>(
		precision_land_auto::kPrecisionLandAutoModeName, precision_land_auto::kPrecisionLandAutoDebugOutput));
	rclcpp::shutdown();
	return 0;
}
