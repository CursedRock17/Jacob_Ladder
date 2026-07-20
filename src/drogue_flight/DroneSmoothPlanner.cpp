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
	_node.declare_parameter<float>("drogue_standoff_m", 3.0f);
	_node.declare_parameter<float>("carrot_lead_time", 1.0f);
	_node.declare_parameter<float>("waypoint_tolerance_m", 0.15f);
	_node.declare_parameter<float>("max_velocity", 0.5f);
	_node.declare_parameter<float>("camera_pitch_deg", 0.0f);
	_node.declare_parameter<float>("hover_duration", 1.5f);
	_node.declare_parameter<float>("drogue_timeout", 3.0f);

	_node.get_parameter("takeoff_optical_flow_height", _param_takeoff_optical_flow_height);
	_node.get_parameter("takeoff_optical_flow_hold_time", _param_takeoff_optical_flow_hold_time);
	_node.get_parameter("takeoff_optical_flow_reached_tol", _param_takeoff_optical_flow_reached_tol);
	_node.get_parameter("takeoff_height", _param_takeoff_height);
	_node.get_parameter("climb_rate", _param_climb_rate);
	_node.get_parameter("takeoff_reached_tol", _param_takeoff_reached_tol);
	_node.get_parameter("drogue_standoff_m", _param_drogue_standoff_m);
	_node.get_parameter("carrot_lead_time", _param_carrot_lead_time);
	_node.get_parameter("waypoint_tolerance_m", _param_waypoint_tolerance_m);
	_node.get_parameter("max_velocity", _param_max_velocity);
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
	_drogue_valid = false;

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
			RCLCPP_INFO(_node.get_logger(),
				"Drogue acquired — approaching to %.2f m standoff",
				_param_drogue_standoff_m);
			switchToState(State::Approach);
		}
		break;
	}

	case State::Approach: {
		// Fall back to search if detection drops out
		if (drogue_expired) {
			RCLCPP_WARN(_node.get_logger(), "Drogue lost — returning to search");
			switchToState(State::Search);
			break;
		}

		// Recompute the standoff target from the latest detection every tick. There is
		// no precomputed path to invalidate, so no replan gating is needed — the
		// setpoint simply tracks wherever the drogue currently is.
		Eigen::Vector3f drogue_ned;
		Eigen::Vector3f standoff_ned;
		if (!drogueTargetNed(drogue_ned) || !drogueStandoffNed(standoff_ned)) {
			// Attitude not ready yet — hold rather than command a bogus setpoint
			_trajectory_setpoint->updatePosition(_vehicle_local_position->positionNed());
			break;
		}

		const Eigen::Vector3f pos = _vehicle_local_position->positionNed();
		const Eigen::Vector3f to_standoff = standoff_ned - pos;
		const float distance = to_standoff.norm();

		RCLCPP_INFO_THROTTLE(_node.get_logger(), *_node.get_clock(), 2000,
			"[Approach] %.2f m to standoff | drogue NED: [%.2f, %.2f, %.2f] | "
			"pos: [%.2f, %.2f, %.2f]",
			distance,
			drogue_ned.x(), drogue_ned.y(), drogue_ned.z(),
			pos.x(), pos.y(), pos.z());

		if (distance < _param_waypoint_tolerance_m) {
			RCLCPP_INFO(_node.get_logger(),
				"Reached %.2f m standoff — hovering", _param_drogue_standoff_m);
			_hover_position = pos;
			_hover_start_time = now;
			switchToState(State::Hover);
			break;
		}

		// Carrot setpoint: a single normalized waypoint placed one lead-time ahead of
		// the vehicle along the bearing to the standoff point, never past it. PX4's
		// position controller closes the loop on this; the velocity feed-forward is
		// what actually caps approach speed at max_velocity, since offboard setpoints
		// bypass the mission-mode trajectory shaping and would otherwise be limited
		// only by MPC_XY_VEL_MAX.
		const Eigen::Vector3f direction = to_standoff / distance;
		const float full_lead = _param_max_velocity * _param_carrot_lead_time;
		const float lead = std::min(full_lead, distance);

		// Taper the feed-forward inside the lead distance so the vehicle settles onto
		// the standoff point instead of overshooting it toward the drogue.
		const float speed = (full_lead > 0.0f)
			? _param_max_velocity * std::min(1.0f, distance / full_lead)
			: 0.0f;

		// Yaw tracks the drogue itself, not the standoff point, so the camera stays on
		// target through the whole approach.
		const float yaw = std::atan2(drogue_ned.y() - pos.y(), drogue_ned.x() - pos.x());

		px4_ros2::TrajectorySetpoint setpoint;
		setpoint.withPosition(pos + lead * direction)
			.withVelocity(speed * direction)
			.withYaw(yaw);
		_trajectory_setpoint->update(setpoint);

		publishPathVisualization(pos, standoff_ned, drogue_ned);
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

// Hold drogue_standoff_m of horizontal separation while matching the drogue's
// altitude. Backing off purely in the horizontal plane means the vehicle ends up
// level with the drogue at a fixed lateral gap, rather than short of it along a
// 3D line that could still put it underneath or above the target.
bool DroneSmoothPlanner::drogueStandoffNed(Eigen::Vector3f& standoff_ned) const
{
	Eigen::Vector3f drogue_ned;
	if (!drogueTargetNed(drogue_ned)) {
		return false;
	}

	const Eigen::Vector3f pos = _vehicle_local_position->positionNed();
	const Eigen::Vector2f to_drogue_xy(drogue_ned.x() - pos.x(), drogue_ned.y() - pos.y());
	const float distance_xy = to_drogue_xy.norm();

	// Match the drogue's altitude regardless of horizontal geometry
	standoff_ned.z() = drogue_ned.z();

	if (distance_xy < 1e-3f) {
		// Directly above or below the drogue — no horizontal bearing to back off
		// along, so hold current XY and let the altitude match bring us level.
		standoff_ned.x() = pos.x();
		standoff_ned.y() = pos.y();
		return true;
	}

	// Note: when the drone is already closer than the standoff distance this places
	// the target behind it, so the same setpoint drives a retreat away from the drogue.
	const Eigen::Vector2f bearing = to_drogue_xy / distance_xy;
	standoff_ned.x() = drogue_ned.x() - _param_drogue_standoff_m * bearing.x();
	standoff_ned.y() = drogue_ned.y() - _param_drogue_standoff_m * bearing.y();
	return true;
}

// ── Helpers ──

// Publish vehicle -> standoff -> drogue as a nav_msgs::Path for Foxglove, so the
// standoff gap is visible as the final leg of the line.
void DroneSmoothPlanner::publishPathVisualization(const Eigen::Vector3f& from,
	const Eigen::Vector3f& standoff, const Eigen::Vector3f& drogue)
{
	nav_msgs::msg::Path path_msg;
	path_msg.header.stamp = _node.now();
	path_msg.header.frame_id = "map";

	for (const auto& point : {from, standoff, drogue}) {
		geometry_msgs::msg::PoseStamped pose;
		pose.header = path_msg.header;
		pose.pose.position.x = point.x();
		pose.pose.position.y = point.y();
		pose.pose.position.z = point.z();
		pose.pose.orientation.w = 1.0;
		path_msg.poses.push_back(pose);
	}

	_path_viz_pub->publish(path_msg);
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
