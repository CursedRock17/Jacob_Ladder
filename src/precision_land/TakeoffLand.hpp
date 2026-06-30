#pragma once

#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/components/mode_executor.hpp>
#include <px4_ros2/control/setpoint_types/experimental/trajectory.hpp>
#include <px4_ros2/odometry/local_position.hpp>

#include <rclcpp/rclcpp.hpp>

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

private:
	enum class TakeoffState {
		OpticalFlowInit,
		Climbing,
		Holding
	};

	rclcpp::Node& _node;

	std::shared_ptr<px4_ros2::OdometryLocalPosition> _vehicle_local_position;
	std::shared_ptr<px4_ros2::TrajectorySetpointType> _trajectory_setpoint;

	Eigen::Vector3f _base_position;
	Eigen::Vector3f _hold_position;
	TakeoffState _state = TakeoffState::OpticalFlowInit;
	bool _active = false;
	bool _reached_flow_height = false;
	float _state_elapsed = 0.0f;

	// Parameters
	float _optical_flow_height = 0.1f;
	float _optical_flow_hold_time = 3.0f;
	float _target_height = 2.5f;
	float _climb_rate = 0.3f;
	float _delta_position = 0.05f;
};

// Executor: arms -> takeoff(1.25) -> hold for duration -> land -> disarm
class TakeoffLandExecutor : public px4_ros2::ModeExecutorBase
{
public:
	TakeoffLandExecutor(rclcpp::Node& node, px4_ros2::ModeBase& owned_mode);

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

	rclcpp::Node& _node;
	float _param_hold_duration = 5.0f;
	rclcpp::TimerBase::SharedPtr _hold_timer;
};

} // namespace precision_land
