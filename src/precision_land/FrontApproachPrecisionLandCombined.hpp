#pragma once

#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/components/mode_executor.hpp>
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
		FrontSearch,
		FrontApproach,
		PrecisionApproach,
		Finished
	};

	void loadParameters();

	void frontTargetCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
	void downTargetCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
	void landDetectedCallback(const px4_msgs::msg::VehicleLandDetected::SharedPtr msg);

	ArucoTag transformFrontTag(const ArucoTag& tag) const;
	ArucoTag transformDownTag(const ArucoTag& tag) const;

	bool targetExpired(const rclcpp::Time& now, const ArucoTag& tag) const;
	bool positionReached(const Eigen::Vector3f& target) const;

	void resetFrontController();
	void switchToState(State state);
	std::string stateName(State state) const;

private:
	rclcpp::Node& _node;

	rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr _front_target_sub;
	rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr _down_target_sub;
	rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr _land_detected_sub;

	std::shared_ptr<px4_ros2::OdometryLocalPosition> _vehicle_local_position;
	std::shared_ptr<px4_ros2::OdometryAttitude> _vehicle_attitude;
	std::shared_ptr<px4_ros2::TrajectorySetpointType> _trajectory_setpoint;

	State _state = State::Idle;
	ArucoTag _front_tag{};
	ArucoTag _down_tag{};
	bool _front_target_lost_prev = true;
	bool _down_target_lost_prev = true;
	bool _land_detected = false;

	Eigen::Quaterniond _front_optical_to_body;
	Eigen::Quaterniond _down_optical_to_body;

	// Precision approach latched target
	Eigen::Vector3f _precision_target = Eigen::Vector3f::Zero();

	// Front PID controller state
	Eigen::Vector2d _front_integral_xy = Eigen::Vector2d::Zero();
	Eigen::Vector2d _front_prev_error_xy = Eigen::Vector2d::Zero();
	bool _front_has_prev_error = false;

	// Parameters
	float _param_front_hold_distance = 2.0f;
	float _param_front_target_timeout = 3.0f;
	float _param_front_kp = 0.8f;
	float _param_front_ki = 0.02f;
	float _param_front_kd = 0.3f;
	float _param_front_max_vel = 1.0f;
	float _param_front_int_limit = 0.5f;
	float _param_front_kp_z = 0.6f;
	float _param_front_max_vel_z = 0.6f;

	float _param_precision_target_timeout = 3.0f;
	float _param_precision_descent_vel = 0.5f;
	float _param_precision_delta_position = 0.25f;
	float _param_precision_delta_velocity = 0.25f;
};

// Executor: arms -> PX4 internal takeoff -> schedules FrontApproachPrecisionLand mode
class FrontApproachPrecisionLandExecutor : public px4_ros2::ModeExecutorBase
{
public:
	FrontApproachPrecisionLandExecutor(rclcpp::Node& node, px4_ros2::ModeBase& owned_mode);

	enum class State {
		Arming,
		TakingOff,
		Approaching,
		Disarming,
	};

	void onActivate() override;
	void onDeactivate(DeactivateReason reason) override;

private:
	void runState(State state, px4_ros2::Result result);

	rclcpp::Node& _node;
	float _param_takeoff_height = 2.5f;
};

} // namespace precision_land

