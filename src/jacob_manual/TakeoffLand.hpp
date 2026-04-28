/**
 * TakeoffLand.hpp — Takeoff, Hold, and Land Flight Mode
 *
 * This mode makes the drone:
 *   1. Rise to a low "optical flow" height and pause (so the flow sensor initializes)
 *   2. Climb to the target altitude
 *   3. Hold position for a set duration
 *   4. Descend until landing is detected
 *
 * It registers as a custom PX4 flight mode via the px4_ros2 library.
 */
#pragma once

#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/control/setpoint_types/experimental/trajectory.hpp>
#include <px4_ros2/odometry/local_position.hpp>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
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

	// Called by PX4 when this mode becomes active / inactive
	void onActivate() override;
	void onDeactivate() override;

	// Called every control loop iteration — this is where the state machine lives
	void updateSetpoint(float dt_s) override;

private:
	// Flight sequence as a state machine
	enum class State {
		Idle,              // Not doing anything
		OpticalFlowInit,   // Low hover so the optical flow sensor can lock on
		Climbing,          // Rising to the target altitude
		Holding,           // Hovering in place for a set duration
		Descending,        // Coming back down
		Finished           // Landed — report success to PX4
	};

	void loadParameters();
	void vehicleLandDetectedCallback(const px4_msgs::msg::VehicleLandDetected::SharedPtr msg);
	void switchToState(State state);
	std::string stateName(State state) const;

	// Wraps _trajectory_setpoint->updatePosition and publishes the
	// (commanded - actual) position error on /tracking_error
	void commandPosition(const Eigen::Vector3f& pos);

private:
	rclcpp::Node& _node;

	// Subscription to know when the drone has physically touched down
	rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr _vehicle_land_detected_sub;

	// Publishes the current state name on /drone_state for debugging
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _drone_state_publisher;

	// Publishes commanded-minus-actual position on /tracking_error
	rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr _tracking_error_publisher;

	// PX4 ROS 2 interface objects for reading position and sending commands
	std::shared_ptr<px4_ros2::OdometryLocalPosition> _vehicle_local_position;
	std::shared_ptr<px4_ros2::TrajectorySetpointType> _trajectory_setpoint;

	// State machine tracking
	State _state = State::Idle;
	Eigen::Vector3f _base_position{Eigen::Vector3f::Zero()};   // Where the drone started (NED)
	Eigen::Vector3f _hold_position{Eigen::Vector3f::Zero()};   // Current commanded position
	bool _reached_flow_height = false;
	bool _land_detected = false;
	float _state_elapsed = 0.0f;  // Time spent in the current state (seconds)

	// Tunable parameters (loaded from ROS parameter server)
	float _optical_flow_height = 0.1f;       // Height for optical flow init (meters)
	float _optical_flow_hold_time = 3.0f;    // How long to hover at flow height (seconds)
	float _target_height = 2.5f;             // Final cruise altitude (meters)
	float _climb_rate = 0.3f;                // Vertical speed during climb (m/s)
	float _delta_position = 0.05f;           // "Close enough" threshold (meters)
	float _hold_duration = 5.0f;             // How long to hold at altitude (seconds)
	float _descent_vel = 0.5f;               // Vertical speed during descent (m/s)
};

} // namespace precision_land
