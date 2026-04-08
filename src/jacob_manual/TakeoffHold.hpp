#pragma once

#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/control/setpoint_types/experimental/trajectory.hpp>
#include <px4_ros2/odometry/local_position.hpp>

#include <rclcpp/rclcpp.hpp>

#include <Eigen/Core>

#include <string>

namespace precision_land
{

inline constexpr char kTakeoffHoldModeName[] = "TakeoffHold";
inline constexpr bool kTakeoffHoldDebugOutput = true;

class TakeoffHoldMode : public px4_ros2::ModeBase
{
public:
	explicit TakeoffHoldMode(rclcpp::Node& node);

	void onActivate() override;
	void onDeactivate() override;
	void updateSetpoint(float dt_s) override;

private:
	enum class State {
		Idle,
		OpticalFlowInit,
		Climbing,
		Holding
	};

	void loadParameters();
	void switchToState(State state);
	std::string stateName(State state) const;

private:
	rclcpp::Node& _node;

	std::shared_ptr<px4_ros2::OdometryLocalPosition> _vehicle_local_position;
	std::shared_ptr<px4_ros2::TrajectorySetpointType> _trajectory_setpoint;

	State _state = State::Idle;
	Eigen::Vector3f _base_position{Eigen::Vector3f::Zero()};
	Eigen::Vector3f _hold_position{Eigen::Vector3f::Zero()};
	bool _reached_flow_height = false;
	float _state_elapsed = 0.0f;

	// Parameters
	float _optical_flow_height = 0.5f;
	float _optical_flow_hold_time = 3.0f;
	float _target_height = 1.25f;
	float _climb_rate = 0.3f;
	float _delta_position = 0.25f;
};

} // namespace precision_land
