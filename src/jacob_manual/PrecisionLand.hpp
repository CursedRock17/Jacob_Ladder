#pragma once

#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/control/setpoint_types/experimental/trajectory.hpp>
#include <px4_ros2/odometry/attitude.hpp>
#include <px4_ros2/odometry/local_position.hpp>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <px4_msgs/msg/vehicle_land_detected.hpp>

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
		Idle,
		OpticalFlowInit,
		Climbing,
		Search,
		Approach,
		Descend,
		Finished
	};

	void loadParameters();
	void targetPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
	void vehicleLandDetectedCallback(const px4_msgs::msg::VehicleLandDetected::SharedPtr msg);

	ArucoTag transformDownTag(const ArucoTag& tag) const;

	bool targetExpired(const rclcpp::Time& now) const;
	bool positionReached(const Eigen::Vector3f& target) const;
	Eigen::Vector2f calculateVelocitySetpointXY();

	void generateSearchWaypoints();
	void switchToState(State state);
	std::string stateName(State state) const;

private:
	rclcpp::Node& _node;

	rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr _target_pose_sub;
	rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr _vehicle_land_detected_sub;

	std::shared_ptr<px4_ros2::OdometryLocalPosition> _vehicle_local_position;
	std::shared_ptr<px4_ros2::OdometryAttitude> _vehicle_attitude;
	std::shared_ptr<px4_ros2::TrajectorySetpointType> _trajectory_setpoint;

	State _state = State::Idle;
	ArucoTag _tag{};
	bool _target_lost_prev = true;
	bool _land_detected = false;
	bool _search_started = false;

	Eigen::Quaterniond _down_optical_to_body;

	// Optical flow / climb state
	Eigen::Vector3f _base_position{Eigen::Vector3f::Zero()};
	Eigen::Vector3f _hold_position{Eigen::Vector3f::Zero()};
	bool _reached_flow_height = false;
	float _state_elapsed = 0.0f;
	float _approach_altitude = {};

	// Search pattern
	std::vector<Eigen::Vector3f> _search_waypoints;
	int _search_waypoint_index = 0;

	// Descend integrator
	float _vel_x_integral = 0.f;
	float _vel_y_integral = 0.f;

	// Parameters
	float _param_descent_vel = 1.0f;
	float _param_vel_p_gain = 1.5f;
	float _param_vel_i_gain = 0.0f;
	float _param_max_velocity = 3.0f;
	float _param_target_timeout = 3.0f;
	float _param_delta_position = 0.25f;
	float _param_delta_velocity = 0.25f;

	float _optical_flow_height = 0.1f;
	float _optical_flow_hold_time = 3.0f;
	float _target_height = 2.5f;
	float _climb_rate = 0.3f;
};

} // namespace precision_land
