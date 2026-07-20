#pragma once

#include "StatePublisher.hpp"

#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/components/mode_executor.hpp>
#include <px4_ros2/control/setpoint_types/experimental/trajectory.hpp>
#include <px4_ros2/odometry/local_position.hpp>

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>

#include <Eigen/Core>

#include <string>

namespace precision_land
{

inline constexpr char kTakeoffLandModeName[] = "TakeoffLand";
inline constexpr bool kTakeoffLandDebugOutput = true;

// Mode: optical flow init, smooth climb to target, then hold position
class TakeoffLandMode : public px4_ros2::ModeBase
{
public:
	explicit TakeoffLandMode(rclcpp::Node& node);

	void onActivate() override;
	void onDeactivate() override;
	void updateSetpoint(float dt_s) override;

	// Shared with the executor so the whole flight is one state timeline.
	StatePublisher& statePublisher() { return _state_pub; }

private:
	enum class TakeoffState {
		OpticalFlowInit,
		Climbing,
		Holding,
		Descending   // slow setpoint descent to the land() handoff height
	};

	rclcpp::Node& _node;
	StatePublisher _state_pub;

	std::shared_ptr<px4_ros2::OdometryLocalPosition> _vehicle_local_position;
	std::shared_ptr<px4_ros2::TrajectorySetpointType> _trajectory_setpoint;

	Eigen::Vector3f _base_position;
	Eigen::Vector3f _hold_position;
	TakeoffState _state = TakeoffState::OpticalFlowInit;
	bool _active = false;
	bool _reached_flow_height = false;
	bool _completed_signaled = false;  // guards the one-shot completion signal
	float _state_elapsed = 0.0f;

	// Parameters
	float _optical_flow_height = 0.1f;
	float _optical_flow_hold_time = 3.0f;
	float _target_height = 2.5f;
	float _climb_rate = 0.3f;
	float _delta_position = 0.05f;
	float _hold_duration = 5.0f;  // seconds to hover at target before completing
	// PX4's native land() descends at MPC_LAND_SPEED (min 0.6 m/s — hard on a
	// small quad). The mode descends itself at descent_rate down to
	// land_handoff_height above the (estimated) ground, then completes so the
	// executor's land() only covers the final touchdown + land detection.
	float _descent_rate = 0.3f;          // m/s — setpoint descent speed
	float _land_handoff_height = 0.3f;   // meters above ground to hand off to land()
	float _descent_target_z = 0.0f;      // NED z where the descent completes
};

// Executor: arms -> hands off to mode (climb + timed hover) -> lands on completion
class TakeoffLandExecutor : public px4_ros2::ModeExecutorBase
{
public:
	TakeoffLandExecutor(rclcpp::Node& node, TakeoffLandMode& owned_mode);

	enum class State {
		Arming,
		TakingOff,
		Hold,
		Landing,
		Disarming,
	};

	void onActivate() override;
	void onDeactivate(DeactivateReason reason) override;

private:
	void runState(State state, px4_ros2::Result result);
	void triggerLanding();  // land once, whether from mode completion or the safety net

	rclcpp::Node& _node;
	TakeoffLandMode& _mode;
	float _param_safety_timeout = 30.0f;  // force a land if the mode never completes
	bool _land_triggered = false;
	rclcpp::TimerBase::SharedPtr _safety_timer;

	// Holy-Grail native takeoff (from PrecisionLandAuto): PX4 takeoff() lifts off;
	// completion is detected by altitude because the takeoff() callback does not
	// fire reliably at low custom altitudes.
	float _param_target_height = 1.0f;
	float _takeoff_target_z = -0.8f;    // NED z at which takeoff counts as complete
	bool _in_takeoff = false;
	bool _takeoff_complete = false;
	rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr _local_pos_sub;
};

} // namespace precision_land
