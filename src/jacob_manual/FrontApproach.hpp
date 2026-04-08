#pragma once

#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/control/setpoint_types/experimental/trajectory.hpp>
#include <px4_ros2/odometry/attitude.hpp>
#include <px4_ros2/odometry/local_position.hpp>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

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
		Search,
		Approach,
		Finished
	};

	void loadParameters();
	void frontTargetCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

	ArucoTag transformTagToWorld(const ArucoTag& tag) const;

	bool targetExpired(const rclcpp::Time& now) const;
	bool positionReached(const Eigen::Vector3f& target) const;

	void resetController();
	void switchToState(State state);
	std::string stateName(State state) const;

private:
	rclcpp::Node& _node;

	rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr _front_target_sub;

	std::shared_ptr<px4_ros2::OdometryLocalPosition> _vehicle_local_position;
	std::shared_ptr<px4_ros2::OdometryAttitude> _vehicle_attitude;
	std::shared_ptr<px4_ros2::TrajectorySetpointType> _trajectory_setpoint;

	State _state = State::Idle;
	ArucoTag _front_tag{};
	bool _target_lost_prev = true;

	Eigen::Quaterniond _front_optical_to_body;

	// PID controller state
	Eigen::Vector2d _integral_xy = Eigen::Vector2d::Zero();
	Eigen::Vector2d _prev_error_xy = Eigen::Vector2d::Zero();
	bool _has_prev_error = false;

	// Parameters
	float _param_hold_distance = 1.0f;
	float _param_delta_position = 0.25f;
	float _param_delta_velocity = 0.25f;
	float _param_target_timeout = 3.0f;

	float _param_kp_xy = 1.2f;
	float _param_ki_xy = 0.0f;
	float _param_kd_xy = 0.0f;
	float _param_max_velocity_xy = 3.0f;
	float _param_integral_limit = 2.0f;

	float _param_kp_z = 1.0f;
	float _param_max_velocity_z = 1.5f;
};

} // namespace precision_land
