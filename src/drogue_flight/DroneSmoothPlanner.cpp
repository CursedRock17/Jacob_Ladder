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
	_node.declare_parameter<float>("takeoff_height", 1.75f);
	_node.declare_parameter<float>("drogue_standoff_m", 3.0f);
	_node.declare_parameter<float>("carrot_lead_time", 1.0f);
	_node.declare_parameter<float>("final_waypoint_tolerance_m", 0.25f);
	_node.declare_parameter<float>("max_velocity", 0.5f);
	_node.declare_parameter<float>("camera_pitch_deg", 0.0f);
	_node.declare_parameter<float>("drogue_timeout", 3.0f);

	_node.get_parameter("takeoff_height", _param_takeoff_height);
	_node.get_parameter("drogue_standoff_m", _param_drogue_standoff_m);
	_node.get_parameter("carrot_lead_time", _param_carrot_lead_time);
	_node.get_parameter("final_waypoint_tolerance_m", _param_final_waypoint_tolerance_m);
	_node.get_parameter("max_velocity", _param_max_velocity);
	_node.get_parameter("camera_pitch_deg", _param_camera_pitch_deg);
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

	switchToState(State::Search);
	RCLCPP_INFO(_node.get_logger(),
		"DroneSmoothPlanner activated (already airborne) — searching for drogue, "
		"standoff %.2f m",
		_param_drogue_standoff_m);
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

		// Re-derive the standoff every tick. It sits drogue_standoff_m short of the
		// drogue on our OWN approach line, so a straight carrot toward it can never
		// cross the drogue, and it self-corrects: if we drift inside the standoff
		// distance the target lands behind us and the same setpoint commands a retreat.
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

		// Error printed per-axis (N/E/D, +D = standoff is below us) rather than as a
		// magnitude, so a ground test can see which axis is off. Tolerance shown too.
		RCLCPP_INFO_THROTTLE(_node.get_logger(), *_node.get_clock(), 2000,
			"[Approach] error NED: [N %+.2f, E %+.2f, D %+.2f] m (tol +/-%.2f) | "
			"drogue NED: [%.2f, %.2f, %.2f] | pos: [%.2f, %.2f, %.2f]",
			to_standoff.x(), to_standoff.y(), to_standoff.z(),
			_param_final_waypoint_tolerance_m,
			drogue_ned.x(), drogue_ned.y(), drogue_ned.z(),
			pos.x(), pos.y(), pos.z());

		if (withinTolerance(to_standoff, _param_final_waypoint_tolerance_m)) {
			RCLCPP_INFO(_node.get_logger(),
				"Reached %.2f m standoff (error NED [%+.2f, %+.2f, %+.2f] m) — "
				"holding station and tracking drogue",
				_param_drogue_standoff_m,
				to_standoff.x(), to_standoff.y(), to_standoff.z());
			switchToState(State::Hover);
			break;
		}

		// Carrot setpoint: a single waypoint placed one lead-time ahead of the vehicle
		// along the bearing to the standoff point, never past it. PX4's position
		// controller closes the loop; the velocity feed-forward caps approach speed at
		// max_velocity, since offboard setpoints bypass mission-mode trajectory shaping
		// and would otherwise be limited only by MPC_XY_VEL_MAX.
		const Eigen::Vector3f direction = to_standoff / distance;
		const float full_lead = _param_max_velocity * _param_carrot_lead_time;
		const float lead = std::min(full_lead, distance);

		// Taper the feed-forward inside the lead distance so the vehicle settles onto
		// the standoff point instead of overshooting it toward the drogue.
		const float speed = (full_lead > 0.0f)
			? _param_max_velocity * std::min(1.0f, distance / full_lead)
			: 0.0f;

		// Yaw tracks the drogue itself so the camera stays on target during transit.
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
		// Station-keeping, not a timed hold: the mode stays here indefinitely and never
		// completes, so the vehicle holds the standoff until the pilot takes over.
		if (drogue_expired) {
			RCLCPP_WARN(_node.get_logger(), "Drogue lost — returning to search");
			switchToState(State::Search);
			break;
		}

		// The drogue can keep moving, so the standoff is re-derived every tick here too
		// and commanded directly. Small drifts are corrected by PX4's position controller
		// without leaving this state — this is what keeps us tracking a drifting drogue.
		Eigen::Vector3f drogue_ned;
		Eigen::Vector3f standoff_ned;
		if (!drogueTargetNed(drogue_ned) || !drogueStandoffNed(standoff_ned)) {
			_trajectory_setpoint->updatePosition(_vehicle_local_position->positionNed());
			break;
		}

		const Eigen::Vector3f pos = _vehicle_local_position->positionNed();
		const Eigen::Vector3f to_standoff = standoff_ned - pos;

		RCLCPP_INFO_THROTTLE(_node.get_logger(), *_node.get_clock(), 2000,
			"[Hover] error NED: [N %+.2f, E %+.2f, D %+.2f] m (tol +/-%.2f) | "
			"drogue NED: [%.2f, %.2f, %.2f] | pos: [%.2f, %.2f, %.2f]",
			to_standoff.x(), to_standoff.y(), to_standoff.z(),
			_param_final_waypoint_tolerance_m,
			drogue_ned.x(), drogue_ned.y(), drogue_ned.z(),
			pos.x(), pos.y(), pos.z());

		// Hand back to Approach once the drogue has moved far enough that closing the gap
		// deserves the speed-limited carrot. Commanding a distant position setpoint
		// directly would let the controller pick its own (much faster) transit speed,
		// which is exactly what the carrot exists to prevent. The 2x band gives
		// hysteresis so the two states cannot chatter at the tolerance boundary.
		if (!withinTolerance(to_standoff, 2.0f * _param_final_waypoint_tolerance_m)) {
			RCLCPP_INFO(_node.get_logger(),
				"Drogue moved (error NED [%+.2f, %+.2f, %+.2f] m) — re-approaching",
				to_standoff.x(), to_standoff.y(), to_standoff.z());
			switchToState(State::Approach);
			break;
		}

		const float yaw = std::atan2(drogue_ned.y() - pos.y(), drogue_ned.x() - pos.x());
		_trajectory_setpoint->update(
			px4_ros2::TrajectorySetpoint{}
				.withPosition(standoff_ned)
				.withYaw(yaw));

		publishPathVisualization(pos, standoff_ned, drogue_ned);
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

// Compute the standoff point the vehicle holds relative to the drogue.
bool DroneSmoothPlanner::drogueStandoffNed(Eigen::Vector3f& standoff_ned) const
{
	Eigen::Vector3f drogue_ned;
	if (!drogueTargetNed(drogue_ned)) {
		return false;
	}

	// Standoff = drogue_standoff_m back along the drone->drogue bearing, at the
	// drogue's altitude. Because it is measured from our own approach line rather than
	// a fixed heading, the carrot toward it never crosses the drogue, and it self-
	// corrects: drift inside the standoff distance puts the target behind us, so the
	// same setpoint drives a retreat back out to the ring.
	const Eigen::Vector3f pos = _vehicle_local_position->positionNed();
	const Eigen::Vector2f to_drogue_xy(drogue_ned.x() - pos.x(), drogue_ned.y() - pos.y());
	const float distance_xy = to_drogue_xy.norm();

	standoff_ned.z() = drogue_ned.z();

	if (distance_xy < 1e-3f) {
		// Directly above/below the drogue — no horizontal bearing to back off along.
		standoff_ned.x() = pos.x();
		standoff_ned.y() = pos.y();
		return true;
	}

	const Eigen::Vector2f bearing = to_drogue_xy / distance_xy;
	standoff_ned.x() = drogue_ned.x() - _param_drogue_standoff_m * bearing.x();
	standoff_ned.y() = drogue_ned.y() - _param_drogue_standoff_m * bearing.y();
	return true;
}

// ── Helpers ──

bool DroneSmoothPlanner::withinTolerance(const Eigen::Vector3f& error_ned, float band_m) const
{
	return std::abs(error_ned.x()) <= band_m &&
	       std::abs(error_ned.y()) <= band_m &&
	       std::abs(error_ned.z()) <= band_m;
}

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
	case State::Search:   return "Search";
	case State::Approach: return "Approach";
	case State::Hover:    return "Hover";
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

	// takeoff_height is declared by the mode, which is constructed first; reuse it as
	// the native takeoff altitude. Consider takeoff done within 0.2 m (NED z is neg).
	if (_node.has_parameter("takeoff_height")) {
		_node.get_parameter("takeoff_height", _param_takeoff_height);
	}
	_takeoff_target_z = -(_param_takeoff_height - 0.2f);

	// takeoff()'s completion callback does not fire reliably below MIS_TAKEOFF_ALT,
	// so watch local position z directly as well — same workaround as TakeoffLand.
	_local_pos_sub = _node.create_subscription<px4_msgs::msg::VehicleLocalPosition>(
		"/fmu/out/vehicle_local_position", rclcpp::QoS(1).best_effort(),
		[this](const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
			if (_in_takeoff && !_takeoff_complete && msg->z < _takeoff_target_z) {
				_takeoff_complete = true;
				_in_takeoff = false;
				RCLCPP_INFO(_node.get_logger(),
					"Takeoff altitude reached (z=%.2f) — handing off to mode", msg->z);
				runState(State::Approaching, px4_ros2::Result::Success);
			}
		});
}

void DroneSmoothPlannerExecutor::onActivate()
{
	RCLCPP_INFO(_node.get_logger(),
		"DroneSmoothPlanner executor — arm, PX4 native takeoff to %.2f m, then approach",
		_param_takeoff_height);
	_in_takeoff = false;
	_takeoff_complete = false;
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
		RCLCPP_INFO(_node.get_logger(), "Arming");
		arm([this](px4_ros2::Result r) { runState(State::TakingOff, r); });
		break;

	case State::TakingOff:
		RCLCPP_INFO(_node.get_logger(), "Armed — PX4 native takeoff to %.2f m",
			_param_takeoff_height);
		_in_takeoff = true;
		_takeoff_complete = false;
		// Whichever of this callback and the local-position watcher fires first wins.
		takeoff([this](px4_ros2::Result r) {
			if (!_takeoff_complete) {
				_takeoff_complete = true;
				_in_takeoff = false;
				runState(State::Approaching, r);
			}
		}, _param_takeoff_height);
		break;

	case State::Approaching:
		RCLCPP_INFO(_node.get_logger(), "Takeoff complete — handing off to DroneSmoothPlanner mode");
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
