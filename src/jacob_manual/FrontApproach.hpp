/**
 * FrontApproach.hpp — Front Camera Approach Flight Mode
 *
 * This mode uses a forward-facing camera to detect an ArUco tag
 * and fly toward it using a PID controller. The sequence is:
 *   1. Search — hover in place and wait for a tag detection
 *   2. Approach — fly toward the tag, stopping at a hold distance
 *   3. Finished — report success to PX4
 *
 * The tag's camera-frame position is transformed into the world
 * frame so the drone can navigate using global coordinates.
 */
#pragma once

#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/control/setpoint_types/experimental/trajectory.hpp>
#include <px4_ros2/odometry/attitude.hpp>
#include <px4_ros2/odometry/local_position.hpp>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <std_msgs/msg/string.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <limits>
#include <string>

namespace precision_land
{

inline constexpr char kFrontApproachModeName[] = "FrontApproach";
inline constexpr bool kFrontApproachDebugOutput = true;

class FrontApproach : public px4_ros2::ModeBase
{
public:
	explicit FrontApproach(rclcpp::Node& node);

	void onActivate() override;
	void onDeactivate() override;
	void updateSetpoint(float dt_s) override;

private:
	// Holds one ArUco tag detection (position + orientation in 3D space)
	struct ArucoTag {
		Eigen::Vector3d position = Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
		Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
		rclcpp::Time timestamp{};

		// A tag is valid if it has a real timestamp and finite coordinates
		bool valid() const
		{
			return timestamp.nanoseconds() > 0
				&& std::isfinite(position.x())
				&& std::isfinite(position.y())
				&& std::isfinite(position.z());
		}
	};

	enum class State {
		Idle,       // Not doing anything
		Search,     // Hovering, waiting for a tag to appear
		Approach,   // Flying toward the detected tag
		Finished    // Arrived at hold distance — report success
	};

	void loadParameters();
	void frontTargetCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

	// Convert tag position from camera frame to world (NED) frame
	ArucoTag transformTagToWorld(const ArucoTag& tag) const;

	bool targetExpired(const rclcpp::Time& now) const;
	bool positionReached(const Eigen::Vector3f& target) const;

	void resetController();
	void switchToState(State state);
	std::string stateName(State state) const;

	// Wraps _trajectory_setpoint->updatePosition and publishes the
	// (commanded - actual) position error on /tracking_error
	void commandPosition(const Eigen::Vector3f& pos);

private:
	rclcpp::Node& _node;

	// Subscription to the front camera ArUco detector
	rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr _front_target_sub;
  // Publisher for Drone State
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _drone_state_publisher;

	// Publishes commanded-minus-actual position on /tracking_error
	rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr _tracking_error_publisher;

	// PX4 ROS 2 interface objects
	std::shared_ptr<px4_ros2::OdometryLocalPosition> _vehicle_local_position;
	std::shared_ptr<px4_ros2::OdometryAttitude> _vehicle_attitude;
	std::shared_ptr<px4_ros2::TrajectorySetpointType> _trajectory_setpoint;

	State _state = State::Idle;
	ArucoTag _front_tag{};          // Latest tag detection (world frame)
	bool _target_lost_prev = true;  // For logging "acquired" / "lost" transitions
	Eigen::Vector3f hold_pos = Eigen::Vector3f::Zero();

	// Latched target for the Approach state (fly 1.5 m straight forward)
	Eigen::Vector3f _approach_target = Eigen::Vector3f::Zero();
	float _approach_yaw = 0.0f;
	bool _approach_target_set = false;

	// Rotation from the front camera's optical frame to the drone's body frame
	Eigen::Quaterniond _front_optical_to_body;

	// PID controller state for XY velocity commands
	Eigen::Vector2d _integral_xy = Eigen::Vector2d::Zero();
	Eigen::Vector2d _prev_error_xy = Eigen::Vector2d::Zero();
	bool _has_prev_error = false;

	// Tunable parameters
	float _param_hold_distance = 1.0f;      // Stop this far from the tag (meters)
	float _param_delta_position = 0.25f;     // "Close enough" position threshold
	float _param_delta_velocity = 0.25f;     // "Slow enough" velocity threshold
	float _param_target_timeout = 3.0f;      // Seconds before a tag is considered lost

	// PID gains for horizontal (XY) control
	float _param_kp_xy = 1.2f;              // Proportional gain
	float _param_ki_xy = 0.0f;              // Integral gain
	float _param_kd_xy = 0.0f;              // Derivative gain
	float _param_max_velocity_xy = 3.0f;    // Max horizontal speed (m/s)
	float _param_integral_limit = 2.0f;     // Anti-windup clamp

	// Proportional control for vertical (Z) axis
	float _param_kp_z = 1.0f;
	float _param_max_velocity_z = 1.5f;
};

} // namespace precision_land
