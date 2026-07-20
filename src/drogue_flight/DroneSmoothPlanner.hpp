#pragma once

#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/components/mode_executor.hpp>
#include <px4_ros2/control/setpoint_types/experimental/trajectory.hpp>
#include <px4_ros2/odometry/local_position.hpp>
#include <px4_ros2/odometry/attitude.hpp>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <string>

namespace drogue_flight
{

inline constexpr char kDroneSmoothPlannerModeName[] = "DroneSmoothPlanner";
inline constexpr bool kDroneSmoothPlannerDebugOutput = true;

class DroneSmoothPlanner : public px4_ros2::ModeBase
{
public:
	explicit DroneSmoothPlanner(rclcpp::Node& node);

	void onActivate() override;
	void onDeactivate() override;
	void updateSetpoint(float dt_s) override;

private:
	// Mode state machine: Takeoff -> Search -> Approach -> Hover -> Finished
	enum class State {
		Takeoff,    // Optical-flow init, then climb to takeoff_height while holding XY
		Search,     // Hold position, wait for drogue detection
		Approach,   // Drive toward the standoff point offset from the drogue
		Hover,      // Hold position at the standoff point before completing
		Finished    // Signal success back to executor
	};

	// Two-stage takeoff sub-state: initialize optical flow, then climb to mission height.
	enum class TakeoffPhase {
		OpticalFlowInit,
		Climbing
	};

	void loadParameters();
	void droguePoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

	// Transform the cached drogue pose (ranging frame: +x left, +y up, +z forward)
	// into an absolute NED target using the vehicle attitude. Returns false if no
	// valid drogue pose or attitude is available yet.
	bool drogueTargetNed(Eigen::Vector3f& target_ned) const;

	// Standoff point we actually fly to: the drogue's altitude, held back by
	// drogue_standoff_m of *horizontal* separation along the drone->drogue bearing.
	// This is what keeps the vehicle from flying into the drogue. Returns false for
	// the same reasons drogueTargetNed() does.
	bool drogueStandoffNed(Eigen::Vector3f& standoff_ned) const;

	// Helpers
	void publishPathVisualization(const Eigen::Vector3f& from, const Eigen::Vector3f& standoff,
		const Eigen::Vector3f& drogue);
	void switchToState(State state);
	std::string stateName(State state) const;

private:
	rclcpp::Node& _node;

	// Receives relative drogue position from the YOLO ranging pipeline
	rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr _drogue_pose_sub;
	// Publishes waypoint path to Foxglove for visualization
	rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr _path_viz_pub;

	// PX4 odometry and setpoint interfaces provided by px4_ros2_cpp
	std::shared_ptr<px4_ros2::OdometryLocalPosition> _vehicle_local_position;
	std::shared_ptr<px4_ros2::OdometryAttitude> _vehicle_attitude;
	std::shared_ptr<px4_ros2::TrajectorySetpointType> _trajectory_setpoint;

	State _state = State::Takeoff;

	// Takeoff climb: captured at activation so the climb is relative to arm position
	Eigen::Vector3f _base_position = Eigen::Vector3f::Zero();     // NED pose at activation
	Eigen::Vector3f _takeoff_setpoint = Eigen::Vector3f::Zero();  // ramped climb target (XY held)

	// Two-stage takeoff state
	TakeoffPhase _takeoff_phase = TakeoffPhase::OpticalFlowInit;
	float _takeoff_phase_elapsed_s = 0.0f;
	bool _reached_optical_flow_height = false;

	// Latest drogue pose relative to drone, updated by YOLO pipeline
	Eigen::Vector3f _drogue_pose = Eigen::Vector3f::Zero();
	bool _drogue_valid = false;
	rclcpp::Time _drogue_timestamp{};

	// Hover state
	Eigen::Vector3f _hover_position = Eigen::Vector3f::Zero();
	rclcpp::Time _hover_start_time{};

	// Tunable ROS parameters (adjustable via CLI or launch file)
	float _param_takeoff_optical_flow_height = 0.50f;      // Initial optical-flow height [m]
	float _param_takeoff_optical_flow_hold_time = 3.0f;    // Stabilization hold duration [s]
	float _param_takeoff_optical_flow_reached_tol = 0.10f; // Arrival tolerance for optical-flow height [m]
	float _param_takeoff_height = 1.75f;      // Final climb height above arm position [m]
	float _param_climb_rate = 0.3f;           // Vertical climb speed [m/s]
	float _param_takeoff_reached_tol = 0.10f; // Altitude tolerance to finish climb [m]
	float _param_drogue_standoff_m = 3.0f;     // Horizontal standoff kept from the drogue [m]
	float _param_carrot_lead_time = 1.0f;      // Setpoint lead ahead of the vehicle [s]
	float _param_waypoint_tolerance_m = 0.15f; // Distance to consider the standoff point reached
	float _param_max_velocity = 0.5f;          // Approach speed cap [m/s]
	float _param_camera_pitch_deg = 0.0f;      // Forward cam mount pitch, +ve = tilted down [deg]
	float _param_hover_duration = 1.5f;        // How long to hover at drogue before completing [s]
	float _param_drogue_timeout = 3.0f;        // Max age of drogue pose before considered lost [s]
};

// Executor: manages the arm -> hand-off-to-mode lifecycle (climb runs in the mode)
class DroneSmoothPlannerExecutor : public px4_ros2::ModeExecutorBase
{
public:
	DroneSmoothPlannerExecutor(rclcpp::Node& node, px4_ros2::ModeBase& owned_mode);

	// Sequential executor states driven by PX4 result callbacks
	enum class State {
		Arming,      // Request vehicle arm
		Approaching, // Hand control to DroneSmoothPlanner mode (climb runs in-mode)
	};

	void onActivate() override;
	void onDeactivate(DeactivateReason reason) override;

private:
	void runState(State state, px4_ros2::Result result);

	rclcpp::Node& _node;
};

} // namespace drogue_flight
