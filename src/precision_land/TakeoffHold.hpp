#pragma once

#include "StatePublisher.hpp"

#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/components/mode_executor.hpp>
#include <px4_ros2/control/setpoint_types/experimental/trajectory.hpp>
#include <px4_ros2/odometry/local_position.hpp>

#include <rclcpp/rclcpp.hpp>

#include <Eigen/Core>

#include <string>

namespace precision_land
{

inline constexpr char kTakeoffHoldModeName[] = "TakeoffHold";
inline constexpr bool kTakeoffHoldDebugOutput = true;

// Mode: state-machine takeoff with optical flow init, smooth climb, and hold
class TakeoffHoldMode : public px4_ros2::ModeBase
{
public:
	explicit TakeoffHoldMode(rclcpp::Node& node);

	void onActivate() override;
	void onDeactivate() override;
	void updateSetpoint(float dt_s) override;

	// Shared with the executor so the whole flight is one state timeline.
	StatePublisher& statePublisher() { return _state_pub; }

private:
	enum class TakeoffState {
		OpticalFlowInit,  // Climb to 0.1m, hold for optical flow stabilization
		Climbing,         // Smooth climb to target height
		Holding           // Hold position at target height
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
	float _state_elapsed = 0.0f;

	// Parameters (loaded from node params)
	float _optical_flow_height = 0.5f;     // meters — low hover for optical flow lock
	float _optical_flow_hold_time = 3.0f;  // seconds to hold at optical flow height
	float _target_height = 1.25f;           // meters — final hold altitude
	float _climb_rate = 0.3f;              // m/s — vertical climb speed
	float _delta_position = 0.25f;         // meters — position tolerance for "reached"
};

// Executor: arms -> takeoff to optical flow height -> schedules hold mode
class TakeoffHoldExecutor : public px4_ros2::ModeExecutorBase
{
public:
	TakeoffHoldExecutor(rclcpp::Node& node, TakeoffHoldMode& owned_mode);

	enum class State {
		Arming,
		Hold,
	};

	void onActivate() override;
	void onDeactivate(DeactivateReason reason) override;

private:
	void runState(State state, px4_ros2::Result result);

	rclcpp::Node& _node;
	TakeoffHoldMode& _mode;
};

} // namespace precision_land
