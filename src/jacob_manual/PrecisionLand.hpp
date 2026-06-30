/**
 * PrecisionLand.hpp — Precision Landing with Downward Camera
 *
 * Full autonomous sequence using a downward-facing camera:
 *   1. OpticalFlowInit — low hover for optical flow sensor
 *   2. Climbing — rise to search altitude
 *   3. Search — fly an expanding spiral pattern looking for the tag
 *   4. Approach — fly horizontally to position above the tag
 *   5. Descend — come straight down while tracking the tag with PI control
 *   6. Finished — touchdown detected, report success
 *
 * Uses the downward camera's ArUco tag detection for the search,
 * approach, and descent phases.
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
#include <px4_msgs/msg/vehicle_command.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <limits>
#include <string>
#include <vector>

namespace precision_land
{

inline constexpr char kPrecisionLandModeName[] = "PrecisionLandCustom";
inline constexpr bool kPrecisionLandDebugOutput = true;

class PrecisionLand : public px4_ros2::ModeBase
{
public:
	explicit PrecisionLand(rclcpp::Node& node);

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
		Idle,              // Not active
		OpticalFlowInit,   // Low hover for optical flow sensor
		Climbing,          // Rising to search altitude
		Search,            // Flying a spiral pattern looking for the tag
		Approach,          // Moving horizontally to position above the tag
		Descend,           // Descending onto the tag with PI tracking
		Finished           // Landed — report success
	};

	void loadParameters();
	void targetPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
	void vehicleLandDetectedCallback(const px4_msgs::msg::VehicleLandDetected::SharedPtr msg);

	// Convert tag from downward camera frame to world (NED) frame
	ArucoTag transformDownTag(const ArucoTag& tag) const;

	bool targetExpired(const rclcpp::Time& now) const;
	bool positionReached(const Eigen::Vector3f& target) const;

	// PI controller for horizontal tracking during descent
	Eigen::Vector2f calculateVelocitySetpointXY();

	// Build an expanding spiral search pattern
	void generateSearchWaypoints();
	void switchToState(State state);
	std::string stateName(State state) const;

	// Wraps _trajectory_setpoint->updatePosition and publishes the
	// (commanded - actual) position error on /tracking_error
	void commandPosition(const Eigen::Vector3f& pos);

private:
	rclcpp::Node& _node;

	// Subscriptions
	rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr _target_pose_sub;
	rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr _vehicle_land_detected_sub;

	// Publishes the current state name on /drone_state for debugging
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _drone_state_publisher;

	// Publishes commanded-minus-actual position on /tracking_error
	rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr _tracking_error_publisher;

	// Sends a disarm VehicleCommand directly to PX4 (no executor needed)
	rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr _vehicle_command_pub;
	void sendDisarm();

	// PX4 ROS 2 interface objects
	std::shared_ptr<px4_ros2::OdometryLocalPosition> _vehicle_local_position;
	std::shared_ptr<px4_ros2::OdometryAttitude> _vehicle_attitude;
	std::shared_ptr<px4_ros2::TrajectorySetpointType> _trajectory_setpoint;

	// State machine tracking
	State _state = State::Idle;
	ArucoTag _tag{};                // Latest tag detection (world frame)
	bool _target_lost_prev = true;
	bool _land_detected = false;
	bool _search_started = false;   // Tag callbacks are ignored until search begins
	bool _disarm_sent = false;      // Guards the one-shot disarm command

	// Rotation from downward camera optical frame to drone body frame
	Eigen::Quaterniond _down_optical_to_body;

	// Takeoff / climb state
	Eigen::Vector3f _base_position{Eigen::Vector3f::Zero()};
	Eigen::Vector3f _hold_position{Eigen::Vector3f::Zero()};
	bool _reached_flow_height = false;
	float _state_elapsed = 0.0f;
	float _approach_altitude = {};  // Altitude locked when tag is found

	// Search pattern waypoints (expanding spiral)
	std::vector<Eigen::Vector3f> _search_waypoints;
	int _search_waypoint_index = 0;

	// PI integrator for XY tracking during descent
	float _vel_x_integral = 0.f;
	float _vel_y_integral = 0.f;

	// Tunable parameters
	float _param_descent_vel = 1.0f;       // Descent speed (m/s, positive = down in NED)
	float _param_vel_p_gain = 1.5f;        // Proportional gain for descent XY tracking
	float _param_vel_i_gain = 0.0f;        // Integral gain for descent XY tracking
	float _param_max_velocity = 3.0f;      // Max horizontal speed (m/s)
	float _param_target_timeout = 3.0f;    // Seconds before tag is considered lost
	float _param_delta_position = 0.25f;   // Position "close enough" threshold
	float _param_delta_velocity = 0.25f;   // Velocity "slow enough" threshold

	float _optical_flow_height = 0.1f;     // Flow init height (meters)
	float _optical_flow_hold_time = 3.0f;  // Hold time at flow height (seconds)
	float _target_height = 2.5f;           // Search altitude (meters)
	float _climb_rate = 0.3f;              // Climb speed (m/s)
	float _param_land_z_tolerance = 0.15f; // Disarm when within this many meters of starting z
};

} // namespace precision_land
