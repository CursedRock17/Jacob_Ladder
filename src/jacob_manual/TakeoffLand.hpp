#pragma once

#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/control/setpoint_types/experimental/trajectory.hpp>
#include <px4_ros2/odometry/local_position.hpp>

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_land_detected.hpp>

#include <Eigen/Core>

#include <string>

namespace precision_land
{

inline constexpr char kTakeoffLandModeName[] = "TakeoffLand";
inline constexpr bool kTakeoffLandDebugOutput = true;

class TakeoffLandMode : public px4_ros2::ModeBase
{
public:
	explicit TakeoffLandMode(rclcpp::Node& node);

	void onActivate() override;
	void onDeactivate() override;
	void updateSetpoint(float dt_s) override;

private:
	enum class State {
		Idle,
		OpticalFlowInit,
		Climbing,
		Holding,
		Descending,
		Finished
	};

	void loadParameters();
	void vehicleLandDetectedCallback(const px4_msgs::msg::VehicleLandDetected::SharedPtr msg);
	void switchToState(State state);
	std::string stateName(State state) const;

private:
	rclcpp::Node& _node;

	rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr _vehicle_land_detected_sub;

	std::shared_ptr<px4_ros2::OdometryLocalPosition> _vehicle_local_position;
	std::shared_ptr<px4_ros2::TrajectorySetpointType> _trajectory_setpoint;

	State _state = State::Idle;
	Eigen::Vector3f _base_position{Eigen::Vector3f::Zero()};
	Eigen::Vector3f _hold_position{Eigen::Vector3f::Zero()};
	bool _reached_flow_height = false;
	bool _land_detected = false;
	float _state_elapsed = 0.0f;

	// Parameters
	float _optical_flow_height = 0.1f;
	float _optical_flow_hold_time = 3.0f;
	float _target_height = 2.5f;
	float _climb_rate = 0.3f;
	float _delta_position = 0.05f;
	float _hold_duration = 5.0f;
	float _descent_vel = 0.5f;
};

} // namespace precision_land
