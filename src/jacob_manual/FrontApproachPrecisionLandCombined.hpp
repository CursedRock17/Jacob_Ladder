/**
 * FrontApproachPrecisionLandCombined.hpp — Two-Camera Approach + Land Mode
 *
 * Combines the front-camera approach and downward-camera precision landing
 * into a single flight mode. Uses TWO cameras with separate ArUco tags:
 *
 *   1. FrontSearch     — hover, wait for front camera to see a tag
 *   2. FrontApproach   — PID-fly toward the front tag, stop at hold distance
 *   3. PrecisionApproach — (optional) use downward camera to center above tag
 *   4. PrecisionDescend  — descend while tracking with downward camera
 *   5. Finished        — touchdown detected, report success
 *
 * Each camera has its own optical-to-body rotation matrix since they
 * point in different directions (forward vs. down).
 */
#pragma once

#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/control/setpoint_types/experimental/trajectory.hpp>
#include <px4_ros2/odometry/attitude.hpp>
#include <px4_ros2/odometry/local_position.hpp>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <px4_msgs/msg/vehicle_land_detected.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <limits>
#include <string>

namespace precision_land
{

inline constexpr char kFrontToPrecisionModeName[] = "FrontToPrecisionLand";
inline constexpr bool kFrontToPrecisionDebugOutput = true;

class FrontApproachPrecisionLandCombined : public px4_ros2::ModeBase
{
public:
	explicit FrontApproachPrecisionLandCombined(rclcpp::Node& node);

	void onActivate() override;
	void onDeactivate() override;
	void updateSetpoint(float dt_s) override;

private:
	// Holds one ArUco tag detection
	struct ArucoTag {
		Eigen::Vector3d position = Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
		Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
		rclcpp::Time timestamp{};

		bool valid() const
		{
			return timestamp.nanoseconds() > 0
				&& std::isfinite(position.x())
				&& std::isfinite(position.y())
				&& std::isfinite(position.z());
		}
	};

	enum class State {
		Idle,               // Not active
		FrontSearch,        // Hovering, waiting for front camera detection
		FrontApproach,      // PID-flying toward front tag
		PrecisionApproach,  // Centering above target using downward camera
		PrecisionDescend,   // Descending onto the target
		Finished            // Landed
	};

	void loadParameters();

	// Separate callbacks for each camera's detections
	void frontTargetCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
	void downTargetCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
	void landDetectedCallback(const px4_msgs::msg::VehicleLandDetected::SharedPtr msg);

	// Each camera has its own coordinate transform to world frame
	ArucoTag transformFrontTag(const ArucoTag& tag) const;
	ArucoTag transformDownTag(const ArucoTag& tag) const;

	bool targetExpired(const rclcpp::Time& now, const ArucoTag& tag) const;
	bool positionReached(const Eigen::Vector3f& target) const;

	// PI controller for XY tracking during precision descent
	Eigen::Vector2f calculatePrecisionVelocityXY();

	void resetFrontController();
	void switchToState(State state);
	std::string stateName(State state) const;

	// Wraps _trajectory_setpoint->updatePosition and publishes the
	// (commanded - actual) position error on /tracking_error
	void commandPosition(const Eigen::Vector3f& pos);

private:
	rclcpp::Node& _node;

	// Subscriptions — one per camera + landing detector
	rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr _front_target_sub;
	rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr _down_target_sub;
	rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr _land_detected_sub;

	// Publishes the current state name on /drone_state for debugging
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _drone_state_publisher;

	// Publishes commanded-minus-actual position on /tracking_error
	rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr _tracking_error_publisher;

	// PX4 ROS 2 interface objects
	std::shared_ptr<px4_ros2::OdometryLocalPosition> _vehicle_local_position;
	std::shared_ptr<px4_ros2::OdometryAttitude> _vehicle_attitude;
	std::shared_ptr<px4_ros2::TrajectorySetpointType> _trajectory_setpoint;

	// State machine tracking
	State _state = State::Idle;
	ArucoTag _front_tag{};
	ArucoTag _down_tag{};
	bool _front_target_lost_prev = true;
	bool _down_target_lost_prev = true;
	bool _land_detected = false;

	// Camera-to-body rotation for each camera direction
	Eigen::Quaterniond _front_optical_to_body;
	Eigen::Quaterniond _down_optical_to_body;

	// Front approach PID controller state
	Eigen::Vector2d _front_integral_xy = Eigen::Vector2d::Zero();
	Eigen::Vector2d _front_prev_error_xy = Eigen::Vector2d::Zero();
	bool _front_has_prev_error = false;

	// Precision descent PI integrator
	float _precision_integral_x = 0.f;
	float _precision_integral_y = 0.f;

	// Front approach parameters
	float _param_front_hold_distance = 1.0f;    // Stop this far from front tag (meters)
	float _param_front_target_timeout = 3.0f;   // Front tag lost timeout (seconds)
	float _param_front_kp = 0.8f;               // PID proportional
	float _param_front_ki = 0.02f;              // PID integral
	float _param_front_kd = 0.3f;               // PID derivative
	float _param_front_max_vel = 1.0f;          // Max horizontal speed (m/s)
	float _param_front_int_limit = 0.5f;        // Anti-windup clamp
	float _param_front_kp_z = 0.6f;             // Vertical proportional gain
	float _param_front_max_vel_z = 0.6f;        // Max vertical speed (m/s)

	// Precision landing parameters
	float _param_precision_target_timeout = 3.0f;
	float _param_precision_descent_vel = 0.5f;   // Descent speed (m/s)
	float _param_precision_kp = 1.5f;            // Descent tracking P gain
	float _param_precision_ki = 0.0f;            // Descent tracking I gain
	float _param_precision_max_vel = 1.0f;       // Max tracking speed (m/s)
	float _param_precision_delta_position = 0.25f;
	float _param_precision_delta_velocity = 0.25f;
};

} // namespace precision_land
