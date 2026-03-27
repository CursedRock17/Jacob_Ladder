#pragma once

#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/components/mode_executor.hpp>
#include <px4_ros2/control/setpoint_types/experimental/trajectory.hpp>
#include <px4_ros2/odometry/attitude.hpp>
#include <px4_ros2/odometry/local_position.hpp>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <limits>
#include <string>

namespace drogue_land
{

inline constexpr char kDrogueApproachModeName[] = "DrogueApproach";
inline constexpr bool kDrogueApproachDebugOutput = true;

// Mode: approaches a drogue/tag target using front camera PID control
class DrogueApproachMode : public px4_ros2::ModeBase
{
public:
	explicit DrogueApproachMode(rclcpp::Node& node);

	void onActivate() override;
	void onDeactivate() override;
	void updateSetpoint(float dt_s) override;

private:
	struct TargetTag {
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
	void targetCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

	TargetTag transformTagToWorld(const TargetTag& tag) const;

	bool targetExpired(const rclcpp::Time& now) const;
	bool positionReached(const Eigen::Vector3f& target) const;

	void resetController();
	void switchToState(State state);
	std::string stateName(State state) const;

private:
	rclcpp::Node& _node;

	rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr _target_sub;

	std::shared_ptr<px4_ros2::OdometryLocalPosition> _vehicle_local_position;
	std::shared_ptr<px4_ros2::OdometryAttitude> _vehicle_attitude;
	std::shared_ptr<px4_ros2::TrajectorySetpointType> _trajectory_setpoint;

	State _state = State::Idle;
	TargetTag _tag{};
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

	float _param_kp_xy = 0.8f;
	float _param_ki_xy = 0.02f;
	float _param_kd_xy = 0.3f;
	float _param_max_velocity_xy = 1.0f;
	float _param_integral_limit = 0.5f;

	float _param_kp_z = 0.6f;
	float _param_max_velocity_z = 0.6f;
};

// Executor: arms -> PX4 internal takeoff -> schedules approach mode -> holds -> lands
class DrogueApproachExecutor : public px4_ros2::ModeExecutorBase
{
public:
	DrogueApproachExecutor(rclcpp::Node& node, px4_ros2::ModeBase& owned_mode);

	enum class State {
		Arming,
		TakingOff,
		Approaching,
		Holding,
		Landing,
		WaitingDisarm,
	};

	void onActivate() override;
	void onDeactivate(DeactivateReason reason) override;

private:
	void runState(State state, px4_ros2::Result result);
	void startHolding(px4_ros2::Result result);
	void startLanding(px4_ros2::Result result);

	rclcpp::Node& _node;
	float _param_takeoff_height = 2.5f;
	float _param_hold_duration = 3.0f;

	rclcpp::TimerBase::SharedPtr _hold_timer;
};

} // namespace drogue_land
