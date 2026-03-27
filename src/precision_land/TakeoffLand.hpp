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

// Mode: holds position, used between takeoff and land
class TakeoffLandMode : public px4_ros2::ModeBase
{
public:
	explicit TakeoffLandMode(rclcpp::Node& node);

	void onActivate() override;
	void onDeactivate() override;
	void updateSetpoint(float dt_s) override;

private:
	rclcpp::Node& _node;

	std::shared_ptr<px4_ros2::OdometryLocalPosition> _vehicle_local_position;
	std::shared_ptr<px4_ros2::TrajectorySetpointType> _trajectory_setpoint;

	Eigen::Vector3f _hold_position;
	bool _position_set = false;
};

// Executor: arms -> takeoff -> hold for duration -> land -> disarm
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
	float _param_takeoff_height = 2.5f;
	float _param_hold_duration = 5.0f;
	rclcpp::TimerBase::SharedPtr _hold_timer;
};

} // namespace precision_land
