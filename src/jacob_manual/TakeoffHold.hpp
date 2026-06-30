/**
 * TakeoffHold.hpp — Takeoff and Hold Flight Mode
 *
 * Similar to TakeoffLand, but without the descent/landing phase.
 * The drone takes off, initializes optical flow, climbs to altitude,
 * and then holds position indefinitely until the mode is deactivated.
 *
 * Useful for testing takeoff behavior or holding a stable hover.
 */
#pragma once

#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/control/setpoint_types/experimental/trajectory.hpp>
#include <px4_ros2/odometry/local_position.hpp>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>

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
	// Flight sequence — no Descending/Finished since we just hold forever
	enum class State {
		Idle,              // Not doing anything
		OpticalFlowInit,   // Low hover for optical flow sensor initialization
		Climbing,          // Rising to the target altitude
		Holding            // Hovering in place indefinitely
	};

	void loadParameters();
	void switchToState(State state);
	std::string stateName(State state) const;

	// Wraps _trajectory_setpoint->updatePosition and publishes the
	// (commanded - actual) position error on /tracking_error
	void commandPosition(const Eigen::Vector3f& pos);

private:
	rclcpp::Node& _node;

	// PX4 ROS 2 interface objects
	std::shared_ptr<px4_ros2::OdometryLocalPosition> _vehicle_local_position;
	std::shared_ptr<px4_ros2::TrajectorySetpointType> _trajectory_setpoint;

	// Publishes the current state name on /drone_state for debugging
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _drone_state_publisher;

	// Publishes commanded-minus-actual position on /tracking_error
	rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr _tracking_error_publisher;

	// State machine tracking
	State _state = State::Idle;
	Eigen::Vector3f _base_position{Eigen::Vector3f::Zero()};   // Where the drone started
	Eigen::Vector3f _hold_position{Eigen::Vector3f::Zero()};   // Current commanded position
	bool _reached_flow_height = false;
	float _state_elapsed = 0.0f;

	// Tunable parameters
	float _optical_flow_height = 0.5f;       // Height for optical flow init (meters)
	float _optical_flow_hold_time = 3.0f;    // How long to hover at flow height (seconds)
	float _target_height = 1.25f;            // Final cruise altitude (meters)
	float _climb_rate = 0.3f;                // Vertical speed during climb (m/s)
	float _delta_position = 0.25f;           // "Close enough" threshold (meters)
};

} // namespace precision_land
