#include "TakeoffHold.hpp"

#include <px4_ros2/components/node_with_mode.hpp>
#include <px4_ros2/components/wait_for_fmu.hpp>

#include <chrono>

namespace precision_land
{

// ── Mode: state-machine takeoff with optical flow init, smooth climb, hold ──

TakeoffHoldMode::TakeoffHoldMode(rclcpp::Node& node)
	: ModeBase(node, Settings{kTakeoffHoldModeName, false})
	, _node(node)
	, _state_pub(node)
	, _tracking_error(node)
	, _base_position(Eigen::Vector3f::Zero())
	, _hold_position(Eigen::Vector3f::Zero())
{
	setSkipMessageCompatibilityCheck();

	_vehicle_local_position = std::make_shared<px4_ros2::OdometryLocalPosition>(*this);
	_trajectory_setpoint = std::make_shared<px4_ros2::TrajectorySetpointType>(*this);

	// Declare parameters with defaults
	_node.declare_parameter<float>("optical_flow_height", 0.5f);
	_node.declare_parameter<float>("optical_flow_hold_time", 3.0f);
	_node.declare_parameter<float>("target_height", 1.25f);
	_node.declare_parameter<float>("climb_rate", 0.3f);
	_node.declare_parameter<float>("delta_position", 0.25f);

	_node.get_parameter("optical_flow_height", _optical_flow_height);
	_node.get_parameter("optical_flow_hold_time", _optical_flow_hold_time);
	_node.get_parameter("target_height", _target_height);
	_node.get_parameter("climb_rate", _climb_rate);
	_node.get_parameter("delta_position", _delta_position);

	// Best-effort: this is exactly the QoS the uXRCE-DDS bridge publishes
	// VehicleOdometry with, and a reliable subscriber would never match it.
	rclcpp::QoS qos(rclcpp::KeepLast(1));
	qos.best_effort().durability_volatile();
	_visual_odometry_sub = _node.create_subscription<px4_msgs::msg::VehicleOdometry>(
		"/fmu/in/vehicle_visual_odometry", qos,
		[this](px4_msgs::msg::VehicleOdometry::UniquePtr /*msg*/) {
			_last_visual_odometry = _node.now();
		});
}

// Runs at ~1 Hz while PX4 polls external arming checks. Every failure reported
// here shows up in QGC's health list and blocks arming, so the messages have
// to name the thing the pilot can actually act on.
void TakeoffHoldMode::checkArmingAndRunConditions(px4_ros2::HealthAndArmingCheckReporter& reporter)
{
	// 3 s: VIO publishes at 30 Hz, so this tolerates a long stall without
	// flapping, and still reports well within a pre-flight check.
	constexpr double kVisualOdometryTimeoutS = 3.0;

	const bool ever_received = _last_visual_odometry.nanoseconds() > 0;
	const double age_s =
		ever_received ? (_node.now() - _last_visual_odometry).seconds() : 0.0;

	// Journal-visible mirror of what QGC is being told. With no shell on the
	// airframe this is the only way to reconstruct, after landing, what the
	// arming check saw at the time.
	RCLCPP_INFO_THROTTLE(
		_node.get_logger(), *_node.get_clock(), 5000,
		"arming check: vio_received=%d vio_age=%.1fs xy_valid=%d z_valid=%d",
		static_cast<int>(ever_received), age_s,
		static_cast<int>(_vehicle_local_position->positionXYValid()),
		static_cast<int>(_vehicle_local_position->positionZValid()));

	if (!ever_received || age_s > kVisualOdometryTimeoutS) {
		// The camera did not enumerate, or the VIO node is down. This is the
		// case that grounded the 2026-09-04 test: vio.service reported
		// "active" while publishing nothing, and PX4 could only say it had no
		// local position.
		/* EVENT
		 */
		reporter.armingCheckFailureExt(
			px4_ros2::events::ID("check_takeoff_hold_no_vio"),
			px4_ros2::events::Log::Error,
			"No VIO: check D435i USB and vio.service");
		return;
	}

	// Vision is arriving but the EKF is not producing a usable position from
	// it -- a PX4-side problem (EKF2_EV_CTRL / EKF2_HGT_REF / EKF2_GPS_CTRL),
	// or simply not converged yet. Distinguishing this from the case above is
	// the entire point: the two look identical from the cockpit otherwise.
	if (!_vehicle_local_position->positionXYValid() ||
		!_vehicle_local_position->positionZValid()) {
		/* EVENT
		 */
		reporter.armingCheckFailureExt(
			px4_ros2::events::ID("check_takeoff_hold_no_local_position"),
			px4_ros2::events::Log::Error,
			"VIO OK but EKF position invalid: check EKF2_EV_CTRL, or wait to converge");
	}
}

void TakeoffHoldMode::onActivate()
{
	_base_position = _vehicle_local_position->positionNed();
	_hold_position = _base_position;
	_state = TakeoffState::OpticalFlowInit;
	_state_elapsed = 0.0f;
	_reached_flow_height = false;
	_active = true;
	_state_pub.set("OpticalFlowInit");

	// Command initial position: optical_flow_height above ground (NED: negative z is up)
	_hold_position.z() = _base_position.z() - _optical_flow_height;

	RCLCPP_INFO(_node.get_logger(),
		"TakeoffHold active — optical flow init at %.2f m, then climb to %.1f m at %.1f m/s",
		_optical_flow_height, _target_height, _climb_rate);
}

void TakeoffHoldMode::onDeactivate()
{
	_active = false;
	_state_pub.set("Deactivated");
}

void TakeoffHoldMode::updateSetpoint(float dt_s)
{
	if (!_active) return;

	_state_elapsed += dt_s;

	switch (_state) {

	case TakeoffState::OpticalFlowInit: {
		const float current_z = _vehicle_local_position->positionNed().z();
		// How far above our starting point have we actually climbed (positive = upward)
		const float altitude_gained = _base_position.z() - current_z;
		RCLCPP_INFO_THROTTLE(_node.get_logger(), *_node.get_clock(), 2000,
			"[OpticalFlowInit] height: %.2f m | alt gained: %.2f m | target: %.2f m | elapsed: %.1f s",
			-current_z, altitude_gained, _optical_flow_height, _state_elapsed);

		// Only start the hold timer once we've actually climbed to near the target
		if (!_reached_flow_height
			&& altitude_gained >= (_optical_flow_height - _delta_position)) {
			_reached_flow_height = true;
			_state_elapsed = 0.0f;
			RCLCPP_INFO(_node.get_logger(),
				"Reached optical flow height (%.2f m gained) — holding for %.1f s",
				altitude_gained, _optical_flow_hold_time);
		}

		// After reaching height and holding long enough, transition to climb
		if (_reached_flow_height && _state_elapsed >= _optical_flow_hold_time) {
			_state = TakeoffState::Climbing;
			_state_elapsed = 0.0f;
			_state_pub.set("Climbing");
			RCLCPP_INFO(_node.get_logger(),
				"Optical flow stabilized — climbing to %.1f m", _target_height);
		}

		// Position-only setpoint at optical flow height
		_trajectory_setpoint->update(
			px4_ros2::TrajectorySetpoint{}
				.withPosition(_hold_position)
				.withYaw(0.0f)
		);
		_tracking_error.publish(_hold_position, _vehicle_local_position->positionNed());
		break;
	}

	case TakeoffState::Climbing: {
		// Target z in NED (negative = up)
		const float target_z = _base_position.z() - _target_height;
		const float current_z = _vehicle_local_position->positionNed().z();
		RCLCPP_INFO_THROTTLE(_node.get_logger(), *_node.get_clock(), 2000,
			"[Climbing] height: %.2f m | alt gained: %.2f m | target: %.2f m | setpoint_z: %.2f m",
			-current_z, _base_position.z() - current_z, _target_height, _hold_position.z());

		// Ramp z downward (upward in world) at climb_rate
		_hold_position.z() -= _climb_rate * dt_s;

		// Clamp setpoint to target
		if (_hold_position.z() <= target_z) {
			_hold_position.z() = target_z;
		}

		// Only transition once the drone has actually reached the target height
		const float altitude_gained = _base_position.z() - current_z;
		if (altitude_gained >= (_target_height - _delta_position)) {
			_state = TakeoffState::Holding;
			_state_elapsed = 0.0f;
			_state_pub.set("Holding");
			RCLCPP_INFO(_node.get_logger(),
				"Reached %.1f m (actual: %.2f m) — holding position",
				_target_height, altitude_gained);
		}

		// Position + velocity feedforward for smooth tracking
		_trajectory_setpoint->update(
			px4_ros2::TrajectorySetpoint{}
				.withPosition(_hold_position)
				.withVelocityZ(_state == TakeoffState::Holding ? 0.0f : -_climb_rate)
				.withYaw(0.0f)
		);
		_tracking_error.publish(_hold_position, _vehicle_local_position->positionNed());
		break;
	}

	case TakeoffState::Holding:
		RCLCPP_INFO_THROTTLE(_node.get_logger(), *_node.get_clock(), 5000,
			"[Holding] height: %.2f m | hold_z: %.2f m",
			-_vehicle_local_position->positionNed().z(), _hold_position.z());
		// Hold at target height indefinitely
		_trajectory_setpoint->update(
			px4_ros2::TrajectorySetpoint{}
				.withPosition(_hold_position)
				.withYaw(0.0f)
		);
		_tracking_error.publish(_hold_position, _vehicle_local_position->positionNed());
		break;
	}
}

// ── Executor: arms -> hands off to mode (takeoff handled by state machine) ──

TakeoffHoldExecutor::TakeoffHoldExecutor(rclcpp::Node& node, TakeoffHoldMode& owned_mode)
	: ModeExecutorBase(node, ModeExecutorBase::Settings{Settings::Activation::ActivateAlways}, owned_mode)
	, _node(node)
	, _mode(owned_mode)
{
	setSkipMessageCompatibilityCheck();
}

void TakeoffHoldExecutor::onActivate()
{
	RCLCPP_INFO(_node.get_logger(), "TakeoffHold executor — arming");
	_mode.statePublisher().set("Arming");
	runState(State::Arming, px4_ros2::Result::Success);
}

void TakeoffHoldExecutor::onDeactivate(DeactivateReason reason)
{
}

void TakeoffHoldExecutor::runState(State state, px4_ros2::Result result)
{
	if (result != px4_ros2::Result::Success) {
		RCLCPP_ERROR(_node.get_logger(), "State %i failed: %s", (int)state,
			resultToString(result));
		_mode.statePublisher().set("Failed");
		return;
	}

	switch (state) {
	case State::Arming:
		// GPS-denied: skip PX4 auto-takeoff (needs AMSL we don't have) and let the
		// mode's Climbing state lift off via local-NED trajectory setpoints.
		arm([this](px4_ros2::Result r) { runState(State::Hold, r); });
		break;

	case State::Hold:
		_mode.statePublisher().set("Armed");
		scheduleMode(ownedMode().id(), [this](px4_ros2::Result r) {
			RCLCPP_INFO(_node.get_logger(), "Hold mode ended (%s)", resultToString(r));
		});
		break;
	}
}

} // namespace precision_land

// Registration is a one-shot request/reply made from the NodeWithModeExecutor
// constructor, which throws if PX4 does not answer within ~25 s. At boot the
// uXRCE-DDS agent has not established its session yet, so the process exited
// and the mode was absent from `commander status` and QGC for the rest of the
// flight even after the link came up. Wait for a real FMU heartbeat first,
// then keep retrying. See DroneSmoothPlanner.cpp for the same treatment.
int main(int argc, char* argv[])
{
	using namespace std::chrono_literals;

	rclcpp::init(argc, argv);

	// waitForFMU needs a node that is not the mode node -- constructing that
	// is what triggers registration.
	{
		auto startup_node = std::make_shared<rclcpp::Node>("takeoff_hold_startup");

		if (!px4_ros2::waitForFMU(*startup_node, 60s)) {
			RCLCPP_WARN(
				startup_node->get_logger(),
				"No FMU heartbeat after 60s -- is dds_agent running and the TELEM2 link up? "
				"Continuing to retry registration anyway.");
		}
	}

	auto retry_delay = 2s;

	while (rclcpp::ok()) {
		try {
			auto node = std::make_shared<px4_ros2::NodeWithModeExecutor<
				precision_land::TakeoffHoldExecutor, precision_land::TakeoffHoldMode>>(
				precision_land::kTakeoffHoldModeName, precision_land::kTakeoffHoldDebugOutput);

			RCLCPP_INFO(
				node->get_logger(), "Registered '%s' with PX4",
				precision_land::kTakeoffHoldModeName);
			rclcpp::spin(node);
			break;

		} catch (const std::runtime_error& e) {
			RCLCPP_WARN(
				rclcpp::get_logger("takeoff_hold"),
				"Mode registration failed (%s); retrying in %lds",
				e.what(), static_cast<long>(retry_delay.count()));
			rclcpp::sleep_for(retry_delay);
		}
	}

	rclcpp::shutdown();
	return 0;
}
