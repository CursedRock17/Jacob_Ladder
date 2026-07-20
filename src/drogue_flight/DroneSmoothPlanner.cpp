#include "DroneSmoothPlanner.hpp"

#include <px4_ros2/components/node_with_mode.hpp>

#include <algorithm>
#include <cmath>

namespace drogue_flight
{

// ── Mode ──

DroneSmoothPlanner::DroneSmoothPlanner(rclcpp::Node& node)
	: ModeBase(node, Settings{kDroneSmoothPlannerModeName, false})
	, _node(node)
{
	// Allow running even if some px4_msgs fields differ between versions
	setSkipMessageCompatibilityCheck();

	// PX4 odometry source for current NED position and attitude
	_vehicle_local_position = std::make_shared<px4_ros2::OdometryLocalPosition>(*this);
	_vehicle_attitude = std::make_shared<px4_ros2::OdometryAttitude>(*this);
	// Trajectory setpoint interface — feeds position/velocity/accel to PX4
	_trajectory_setpoint = std::make_shared<px4_ros2::TrajectorySetpointType>(*this);

	// Subscribe to drogue pose coming from the YOLO ranging node
	auto qos = rclcpp::QoS(1).best_effort();
	_drogue_pose_sub = _node.create_subscription<geometry_msgs::msg::PoseStamped>(
		"/tag_detections", qos,
		std::bind(&DroneSmoothPlanner::droguePoseCallback, this, std::placeholders::_1));

	// Path publisher for Foxglove visualization
	_path_viz_pub = _node.create_publisher<nav_msgs::msg::Path>("/trajectory_path_viz", qos);

	loadParameters();
}

void DroneSmoothPlanner::loadParameters()
{
	_node.declare_parameter<float>("takeoff_optical_flow_height", 0.50f);
	_node.declare_parameter<float>("takeoff_optical_flow_hold_time", 3.0f);
	_node.declare_parameter<float>("takeoff_optical_flow_reached_tol", 0.10f);
	_node.declare_parameter<float>("takeoff_height", 1.75f);
	_node.declare_parameter<float>("climb_rate", 0.3f);
	_node.declare_parameter<float>("takeoff_reached_tol", 0.10f);
	_node.declare_parameter<int>("num_waypoints", 10);
	_node.declare_parameter<float>("s_curve_steepness", 4.0f);
	_node.declare_parameter<float>("waypoint_tolerance_m", 0.15f);
	_node.declare_parameter<float>("max_velocity", 0.5f);
	_node.declare_parameter<float>("max_acceleration", 0.35f);
	_node.declare_parameter<float>("replan_threshold", 0.35f);
	_node.declare_parameter<float>("replan_min_interval", 0.5f);
	_node.declare_parameter<float>("camera_pitch_deg", 0.0f);
	_node.declare_parameter<float>("hover_duration", 1.5f);
	_node.declare_parameter<float>("drogue_timeout", 3.0f);

	_node.get_parameter("takeoff_optical_flow_height", _param_takeoff_optical_flow_height);
	_node.get_parameter("takeoff_optical_flow_hold_time", _param_takeoff_optical_flow_hold_time);
	_node.get_parameter("takeoff_optical_flow_reached_tol", _param_takeoff_optical_flow_reached_tol);
	_node.get_parameter("takeoff_height", _param_takeoff_height);
	_node.get_parameter("climb_rate", _param_climb_rate);
	_node.get_parameter("takeoff_reached_tol", _param_takeoff_reached_tol);
	_node.get_parameter("num_waypoints", _param_num_waypoints);
	_node.get_parameter("s_curve_steepness", _param_s_curve_steepness);
	_node.get_parameter("waypoint_tolerance_m", _param_waypoint_tolerance_m);
	_node.get_parameter("max_velocity", _param_max_velocity);
	_node.get_parameter("max_acceleration", _param_max_acceleration);
	_node.get_parameter("replan_threshold", _param_replan_threshold);
	_node.get_parameter("replan_min_interval", _param_replan_min_interval);
	_node.get_parameter("camera_pitch_deg", _param_camera_pitch_deg);
	_node.get_parameter("hover_duration", _param_hover_duration);
	_node.get_parameter("drogue_timeout", _param_drogue_timeout);
}

// Cache the latest drogue position from the YOLO detection pipeline
void DroneSmoothPlanner::droguePoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
	_drogue_pose = Eigen::Vector3f(
		static_cast<float>(msg->pose.position.x),
		static_cast<float>(msg->pose.position.y),
		static_cast<float>(msg->pose.position.z));
	_drogue_valid = true;
	_drogue_timestamp = _node.now();
}

void DroneSmoothPlanner::onActivate()
{
	_path.clear();
	_path_index = 0;
	_drogue_valid = false;
	_last_plan_time = _node.now();
	_last_plan_target_ned = Eigen::Vector3f::Zero();

	// Capture the arm position so both takeoff stages and XY hold are relative to it.
	_base_position = _vehicle_local_position->positionNed();
	_takeoff_setpoint = _base_position;

	// Stage 1 commands a fixed low-altitude position so optical flow can initialize.
	_takeoff_setpoint.z() =
		_base_position.z() - _param_takeoff_optical_flow_height;
	_takeoff_phase = TakeoffPhase::OpticalFlowInit;
	_takeoff_phase_elapsed_s = 0.0f;
	_reached_optical_flow_height = false;

	switchToState(State::Takeoff);
	RCLCPP_INFO(_node.get_logger(),
		"DroneSmoothPlanner activated — optical-flow init at %.2f m for %.1f s, "
		"then climbing to %.2f m at %.2f m/s",
		_param_takeoff_optical_flow_height,
		_param_takeoff_optical_flow_hold_time,
		_param_takeoff_height,
		_param_climb_rate);
}

void DroneSmoothPlanner::onDeactivate()
{
	_path.clear();
	_path_index = 0;
}

// Called by the mode framework at the setpoint rate — drives the state machine
void DroneSmoothPlanner::updateSetpoint(float dt_s)
{
	auto now = _node.now();
	// Consider drogue lost if no detection received within timeout
	bool drogue_expired = !_drogue_valid ||
		(now - _drogue_timestamp).seconds() > _param_drogue_timeout;

	switch (_state) {

	case State::Takeoff: {
		const float current_z = _vehicle_local_position->positionNed().z();
		const float altitude_gained = _base_position.z() - current_z;

		switch (_takeoff_phase) {

		case TakeoffPhase::OpticalFlowInit: {
			// Hold a fixed low-altitude position. The stabilization timer starts only
			// after measured altitude confirms that the vehicle reached this height.
			RCLCPP_INFO_THROTTLE(_node.get_logger(), *_node.get_clock(), 2000,
				"[Takeoff:OpticalFlowInit] gained: %.2f m | target: %.2f m | "
				"hold elapsed: %.1f / %.1f s",
				altitude_gained,
				_param_takeoff_optical_flow_height,
				_takeoff_phase_elapsed_s,
				_param_takeoff_optical_flow_hold_time);

			if (!_reached_optical_flow_height &&
				altitude_gained >=
					(_param_takeoff_optical_flow_height -
					 _param_takeoff_optical_flow_reached_tol)) {
				_reached_optical_flow_height = true;
				_takeoff_phase_elapsed_s = 0.0f;
				RCLCPP_INFO(_node.get_logger(),
					"Reached optical-flow initialization height (actual %.2f m) — "
					"holding for %.1f s",
					altitude_gained,
					_param_takeoff_optical_flow_hold_time);
			}

			if (_reached_optical_flow_height) {
				_takeoff_phase_elapsed_s += dt_s;
			}

			_trajectory_setpoint->update(
				px4_ros2::TrajectorySetpoint{}
					.withPosition(_takeoff_setpoint)
					.withYaw(0.0f));

			if (_reached_optical_flow_height &&
				_takeoff_phase_elapsed_s >=
					_param_takeoff_optical_flow_hold_time) {
				_takeoff_phase = TakeoffPhase::Climbing;
				_takeoff_phase_elapsed_s = 0.0f;
				RCLCPP_INFO(_node.get_logger(),
					"Optical flow stabilized — climbing to %.2f m",
					_param_takeoff_height);
			}
			break;
		}

		case TakeoffPhase::Climbing: {
			// Ramp from the optical-flow hold height to the final takeoff height.
			const float target_z = _base_position.z() - _param_takeoff_height;

			_takeoff_setpoint.z() -= _param_climb_rate * dt_s;
			if (_takeoff_setpoint.z() <= target_z) {
				_takeoff_setpoint.z() = target_z;
			}

			RCLCPP_INFO_THROTTLE(_node.get_logger(), *_node.get_clock(), 2000,
				"[Takeoff:Climbing] gained: %.2f m | target: %.2f m | "
				"setpoint_z: %.2f m",
				altitude_gained,
				_param_takeoff_height,
				_takeoff_setpoint.z());

			const bool reached_final_height =
				altitude_gained >=
					(_param_takeoff_height - _param_takeoff_reached_tol);

			_trajectory_setpoint->update(
				px4_ros2::TrajectorySetpoint{}
					.withPosition(_takeoff_setpoint)
					.withVelocityZ(reached_final_height ? 0.0f : -_param_climb_rate)
					.withYaw(0.0f));

			if (reached_final_height) {
				RCLCPP_INFO(_node.get_logger(),
					"Reached takeoff height (actual %.2f m) — searching for drogue",
					altitude_gained);
				switchToState(State::Search);
			}
			break;
		}
		}
		break;
	}

	case State::Search: {
		RCLCPP_INFO_THROTTLE(_node.get_logger(), *_node.get_clock(), 3000,
			"[Search] holding at [%.2f, %.2f, %.2f] — waiting for drogue detection",
			_vehicle_local_position->positionNed().x(),
			_vehicle_local_position->positionNed().y(),
			_vehicle_local_position->positionNed().z());
		// Hold current position while waiting for drogue detection
		Eigen::Vector3f hold = _vehicle_local_position->positionNed();
		_trajectory_setpoint->updatePosition(hold);

		if (!drogue_expired) {
			RCLCPP_INFO(_node.get_logger(), "Drogue acquired — generating trajectory");
			generateTrajectoryToDrogue();

			if (!_path.empty()) {
				switchToState(State::Approach);
			}
		}
		break;
	}

	case State::Approach: {
		RCLCPP_INFO_THROTTLE(_node.get_logger(), *_node.get_clock(), 2000,
			"[Approach] waypoint %d/%zu | drogue: [%.2f, %.2f, %.2f] | pos: [%.2f, %.2f, %.2f]",
			_path_index, _path.size(),
			_drogue_pose.x(), _drogue_pose.y(), _drogue_pose.z(),
			_vehicle_local_position->positionNed().x(),
			_vehicle_local_position->positionNed().y(),
			_vehicle_local_position->positionNed().z());
		// Fall back to search if detection drops out
		if (drogue_expired) {
			RCLCPP_WARN(_node.get_logger(), "Drogue lost — returning to search");
			switchToState(State::Search);
			break;
		}

		// Replan only when a *fresh* detection has arrived (timestamp advanced past
		// the last plan), the rate limit has elapsed, and the drogue's absolute NED
		// target has actually moved past the deadband. This avoids thrashing on
		// measurement jitter and on our own motion against a stale drogue pose.
		if (_path_index > 0 && !_path.empty()) {
			const bool fresh_detection = _drogue_timestamp > _last_plan_time;
			const bool interval_elapsed =
				(now - _last_plan_time).seconds() >= _param_replan_min_interval;
			Eigen::Vector3f target_ned;

			if (fresh_detection && interval_elapsed && drogueTargetNed(target_ned)) {
				float movement = computeDistance(_last_plan_target_ned, target_ned);
				if (movement > _param_replan_threshold) {
					RCLCPP_INFO(_node.get_logger(), "Drogue moved %.2fm — replanning", movement);
					generateTrajectoryToDrogue();
				}
			}
		}

		// Send the next waypoint's full trajectory setpoint to PX4
		if (_path_index < static_cast<int>(_path.size())) {
			const auto& wp = _path[_path_index];

			// Build a combined position + velocity + acceleration + yaw setpoint
			px4_ros2::TrajectorySetpoint setpoint;
			setpoint.withPosition(wp.position)
				.withVelocity(wp.velocity)
				.withAcceleration(wp.acceleration)
				.withYaw(wp.yaw)
				.withYawRate(wp.yaw_rate);
			_trajectory_setpoint->update(setpoint);

			// Advance to the next waypoint once within tolerance
			Eigen::Vector3f vehicle_pos = _vehicle_local_position->positionNed();
			float distance = computeDistance(vehicle_pos, wp.position);

			if (distance < _param_waypoint_tolerance_m || _path_index == 0) {
				_path_index++;
			}
		} else {
			// All waypoints consumed — check remaining distance to drogue
			float drogue_distance = _drogue_pose.norm();

			if (drogue_distance < _param_waypoint_tolerance_m) {
				RCLCPP_INFO(_node.get_logger(), "Reached drogue — hovering");
				_hover_position = _vehicle_local_position->positionNed();
				_hover_start_time = now;
				switchToState(State::Hover);
			} else {
				// Not close enough, regenerate
				generateTrajectoryToDrogue();
			}
		}
		break;
	}

	case State::Hover: {
		RCLCPP_INFO_THROTTLE(_node.get_logger(), *_node.get_clock(), 2000,
			"[Hover] pos: [%.2f, %.2f, %.2f] | elapsed: %.1f s / %.1f s",
			_vehicle_local_position->positionNed().x(),
			_vehicle_local_position->positionNed().y(),
			_vehicle_local_position->positionNed().z(),
			(now - _hover_start_time).seconds(), _param_hover_duration);
		// Hold position near the drogue for a brief stabilization window
		_trajectory_setpoint->updatePosition(_hover_position);

		if ((now - _hover_start_time).seconds() > _param_hover_duration) {
			RCLCPP_INFO(_node.get_logger(), "Hover complete — finishing");
			switchToState(State::Finished);
		}
		break;
	}

	case State::Finished: {
		// Keep holding while signaling the executor that the mode is done
		_trajectory_setpoint->updatePosition(_hover_position);
		ModeBase::completed(px4_ros2::Result::Success);
		break;
	}
	}
}

// ── Frame transform ──

// The ranging pipeline (pose_estimation_node) publishes the drogue in a camera
// ranging frame: +x = left, +y = up, +z = forward. To command NED setpoints we
// rotate that into NED via the vehicle attitude, matching the mapping the pose
// node itself uses in apply_attitude_correction():
//   ranging (x_left, y_up, z_fwd) -> body FRD (fwd, right, down) = (z, -x, -y)
//   body FRD -> NED via the attitude quaternion (which is body-FRD -> NED)
bool DroneSmoothPlanner::drogueTargetNed(Eigen::Vector3f& target_ned) const
{
	if (!_drogue_valid) {
		return false;
	}

	// Attitude may not have arrived yet — a zero/degenerate quaternion is unusable
	Eigen::Quaternionf q = _vehicle_attitude->attitude();
	if (!std::isfinite(q.w()) || q.norm() < 0.1f) {
		return false;
	}
	q.normalize();

	// Ranging (+x left, +y up, +z forward) -> body FRD, assuming the camera optical
	// axis is body-forward. If the forward cam is mounted with a fixed pitch, tilt
	// the vector by that mount angle (positive camera_pitch_deg = tilted down).
	const Eigen::Vector3f frd_cam(_drogue_pose.z(), -_drogue_pose.x(), -_drogue_pose.y());
	const float pitch_rad = _param_camera_pitch_deg * static_cast<float>(M_PI) / 180.0f;
	const Eigen::Vector3f frd =
		Eigen::AngleAxisf(-pitch_rad, Eigen::Vector3f::UnitY()) * frd_cam;

	const Eigen::Vector3f ned_offset = q * frd;
	target_ned = _vehicle_local_position->positionNed() + ned_offset;
	return true;
}

// ── S-Curve Trajectory Generation ──

// Build a full S-curve trajectory from current position to the drogue
void DroneSmoothPlanner::generateTrajectoryToDrogue()
{
	Eigen::Vector3f vehicle_pos = _vehicle_local_position->positionNed();

	// Transform the camera-frame drogue pose into an absolute NED target. This also
	// guards against a missing drogue pose or an attitude estimate that isn't ready.
	Eigen::Vector3f end_pos;
	if (!drogueTargetNed(end_pos)) {
		RCLCPP_WARN(_node.get_logger(),
			"Cannot generate trajectory — no drogue pose or attitude estimate yet");
		return;
	}

	Eigen::Vector3f start_pos = vehicle_pos;

	// Remember this plan so the Approach replan check can measure real drift
	_last_plan_target_ned = end_pos;
	_last_plan_time = _node.now();

	// Interpolate positions along a tanh S-curve
	auto waypoints = generateSCurveWaypoints(start_pos, end_pos, _param_num_waypoints);

	// Sum segment lengths for the velocity profile scaling
	float total_distance = 0.0f;
	for (size_t i = 0; i + 1 < waypoints.size(); ++i) {
		total_distance += computeDistance(waypoints[i], waypoints[i + 1]);
	}

	// Sigmoid velocity/acceleration profiles scaled to total distance
	float dt = 1.0f / 20.0f;
	std::vector<float> v_profile, a_profile;
	generateSCurveVelocityProfile(total_distance, static_cast<int>(waypoints.size()), dt,
		v_profile, a_profile);

	// Assemble waypoints with position, velocity, acceleration, and yaw
	_path.clear();
	_path.reserve(waypoints.size());

	for (size_t i = 0; i < waypoints.size(); ++i) {
		Waypoint wp;
		wp.position = waypoints[i];

		// Unit direction vector along the path at this segment
		Eigen::Vector3f dir;
		if (i + 1 < waypoints.size()) {
			dir = computeDirection(waypoints[i], waypoints[i + 1]);
		} else if (i > 0) {
			dir = computeDirection(waypoints[i - 1], waypoints[i]);
		} else {
			dir = Eigen::Vector3f::Zero();
		}

		// Scale profile magnitudes by path direction
		wp.velocity = v_profile[i] * dir;
		wp.acceleration = a_profile[i] * dir;

		// Point yaw toward the drogue at every waypoint
		float dx = end_pos.x() - waypoints[i].x();
		float dy = end_pos.y() - waypoints[i].y();
		wp.yaw = std::atan2(dy, dx);
		wp.yaw_rate = 0.0f;

		_path.push_back(wp);
	}

	// Finite-difference yaw rate between consecutive waypoints
	for (size_t i = 0; i < _path.size(); ++i) {
		float next_yaw = _path[(i + 1) % _path.size()].yaw;
		float curr_yaw = _path[i].yaw;
		float diff = next_yaw - curr_yaw;

		// Wrap to [-pi, pi]
		if (diff < -M_PI) diff += 2.0f * M_PI;
		if (diff > M_PI) diff -= 2.0f * M_PI;

		_path[i].yaw_rate = diff / dt;
	}

	// Start tracking from the first waypoint
	_path_index = 0;

	RCLCPP_INFO(_node.get_logger(), "Generated S-curve: %zu points, %.2fm distance",
		_path.size(), total_distance);

	publishPathVisualization();
}

// Interpolate positions along a tanh-based S-curve between start and end
std::vector<Eigen::Vector3f> DroneSmoothPlanner::generateSCurveWaypoints(
	const Eigen::Vector3f& start, const Eigen::Vector3f& end, int num_points)
{
	std::vector<Eigen::Vector3f> waypoints;
	waypoints.reserve(num_points);

	for (int i = 0; i < num_points; ++i) {
		// Normalized parameter [0, 1]
		float t = static_cast<float>(i) / static_cast<float>(num_points - 1);
		// tanh maps linear t to an S-shaped blend factor
		float s = (std::tanh(_param_s_curve_steepness * (t - 0.5f)) + 1.0f) / 2.0f;
		waypoints.push_back(start + s * (end - start));
	}

	return waypoints;
}

// Generate sigmoid-shaped velocity and acceleration profiles for smooth motion
void DroneSmoothPlanner::generateSCurveVelocityProfile(
	float total_distance, int num_samples, float dt,
	std::vector<float>& v_out, std::vector<float>& a_out)
{
	v_out.resize(num_samples);
	a_out.resize(num_samples);

	float T = static_cast<float>(num_samples - 1) * dt;
	if (T <= 0.0f) {
		std::fill(v_out.begin(), v_out.end(), 0.0f);
		std::fill(a_out.begin(), a_out.end(), 0.0f);
		return;
	}

	// Raw sigmoid and its first derivative give velocity and acceleration shapes
	for (int i = 0; i < num_samples; ++i) {
		float ti = static_cast<float>(i) * dt;
		float tau = ti / T;

		float exp_term = std::exp(-10.0f * (tau - 0.5f));
		float s = 1.0f / (1.0f + exp_term);

		// v = ds/dt, a = dv/dt (analytical derivatives of the logistic sigmoid)
		v_out[i] = (10.0f / T) * s * (1.0f - s);
		a_out[i] = (100.0f / (T * T)) * s * (1.0f - s) * (1.0f - 2.0f * s);
	}

	// Trapezoidal integration to find the raw area under the velocity curve
	float integrated = 0.0f;
	for (int i = 0; i + 1 < num_samples; ++i) {
		integrated += (v_out[i] + v_out[i + 1]) / 2.0f * dt;
	}

	// Rescale so the integrated distance matches the actual path length
	if (integrated > 0.0f) {
		float scale = total_distance / integrated;
		for (int i = 0; i < num_samples; ++i) {
			v_out[i] *= scale;
			a_out[i] *= scale;
		}
	}

	// Enforce physical limits on the drone
	for (int i = 0; i < num_samples; ++i) {
		v_out[i] = std::clamp(v_out[i], 0.0f, _param_max_velocity);
		a_out[i] = std::clamp(a_out[i], -_param_max_acceleration, _param_max_acceleration);
	}
}

// ── Helpers ──

// Euclidean distance between two NED positions
float DroneSmoothPlanner::computeDistance(const Eigen::Vector3f& a, const Eigen::Vector3f& b) const
{
	return (b - a).norm();
}

// Unit direction vector from a to b, zero if coincident
Eigen::Vector3f DroneSmoothPlanner::computeDirection(const Eigen::Vector3f& a, const Eigen::Vector3f& b) const
{
	Eigen::Vector3f diff = b - a;
	float mag = diff.norm();
	if (mag < 1e-6f) {
		return Eigen::Vector3f::Zero();
	}
	return diff / mag;
}

// Publish the current S-curve as a nav_msgs::Path for Foxglove
void DroneSmoothPlanner::publishPathVisualization()
{
	if (_path.empty()) return;

	nav_msgs::msg::Path path_msg;
	path_msg.header.stamp = _node.now();
	path_msg.header.frame_id = "map";

	// Convert each waypoint to a PoseStamped with yaw encoded in the quaternion
	for (const auto& wp : _path) {
		geometry_msgs::msg::PoseStamped pose;
		pose.header = path_msg.header;
		pose.pose.position.x = wp.position.x();
		pose.pose.position.y = wp.position.y();
		pose.pose.position.z = wp.position.z();
		pose.pose.orientation.z = std::sin(wp.yaw / 2.0f);
		pose.pose.orientation.w = std::cos(wp.yaw / 2.0f);
		path_msg.poses.push_back(pose);
	}

	_path_viz_pub->publish(path_msg);
	RCLCPP_INFO(_node.get_logger(), "Published path visualization: %zu poses", path_msg.poses.size());
}

void DroneSmoothPlanner::switchToState(State state)
{
	if (_state == state) return;
	RCLCPP_INFO(_node.get_logger(), "State: %s -> %s", stateName(_state).c_str(), stateName(state).c_str());
	_state = state;
}

std::string DroneSmoothPlanner::stateName(State state) const
{
	switch (state) {
	case State::Takeoff:  return "Takeoff";
	case State::Search:   return "Search";
	case State::Approach: return "Approach";
	case State::Hover:    return "Hover";
	case State::Finished: return "Finished";
	default:              return "Unknown";
	}
}

// ── Executor: arm -> schedule DroneSmoothPlanner mode (climb runs in-mode) ──

DroneSmoothPlannerExecutor::DroneSmoothPlannerExecutor(rclcpp::Node& node, px4_ros2::ModeBase& owned_mode)
	: ModeExecutorBase(node, ModeExecutorBase::Settings{Settings::Activation::ActivateAlways}, owned_mode)
	, _node(node)
{
	// Allow running even if some px4_msgs fields differ between versions
	setSkipMessageCompatibilityCheck();
}

void DroneSmoothPlannerExecutor::onActivate()
{
	RCLCPP_INFO(_node.get_logger(), "DroneSmoothPlanner executor — arming");
	runState(State::Arming, px4_ros2::Result::Success);
}

void DroneSmoothPlannerExecutor::onDeactivate(DeactivateReason reason)
{
}

// Sequential state machine driven by PX4 result callbacks: arm -> takeoff -> mode
void DroneSmoothPlannerExecutor::runState(State state, px4_ros2::Result result)
{
	if (result != px4_ros2::Result::Success) {
		RCLCPP_ERROR(_node.get_logger(), "State %i failed: %s", static_cast<int>(state),
			resultToString(result));
		return;
	}

	switch (state) {
	case State::Arming:
		// GPS-denied: skip PX4 auto-takeoff (needs AMSL we don't have) and let the
		// mode's Takeoff state climb via local-NED trajectory setpoints.
		arm([this](px4_ros2::Result r) { runState(State::Approaching, r); });
		break;

	case State::Approaching:
		RCLCPP_INFO(_node.get_logger(), "Armed — handing off to DroneSmoothPlanner mode");
		scheduleMode(ownedMode().id(), [this](px4_ros2::Result r) {
			RCLCPP_INFO(_node.get_logger(), "DroneSmoothPlanner mode ended (%s)", resultToString(r));
		});
		break;
	}
}

} // namespace drogue_flight

// NodeWithModeExecutor wires the executor + mode together and registers with PX4
int main(int argc, char* argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<px4_ros2::NodeWithModeExecutor<
		drogue_flight::DroneSmoothPlannerExecutor, drogue_flight::DroneSmoothPlanner>>(
		drogue_flight::kDroneSmoothPlannerModeName, drogue_flight::kDroneSmoothPlannerDebugOutput));
	rclcpp::shutdown();
	return 0;
}
