#pragma once

#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/components/mode_executor.hpp>
#include <px4_ros2/control/setpoint_types/experimental/trajectory.hpp>
#include <px4_ros2/odometry/local_position.hpp>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>

#include <Eigen/Core>

#include <string>
#include <vector>

namespace drogue_flight
{

inline constexpr char kDroneSmoothPlannerModeName[] = "DroneSmoothPlanner";
inline constexpr bool kDroneSmoothPlannerDebugOutput = true;

// Single point along the S-curve trajectory sent to PX4 each update tick
struct Waypoint {
	Eigen::Vector3f position;
	Eigen::Vector3f velocity;
	Eigen::Vector3f acceleration;
	float yaw;
	float yaw_rate;
};

class DroneSmoothPlanner : public px4_ros2::ModeBase
{
public:
	explicit DroneSmoothPlanner(rclcpp::Node& node);

	void onActivate() override;
	void onDeactivate() override;
	void updateSetpoint(float dt_s) override;

private:
	// Mode state machine: Search -> Approach -> Hover -> Finished
	enum class State {
		Search,     // Hold position, wait for drogue detection
		Approach,   // Follow S-curve waypoints toward drogue
		Hover,      // Hold position near drogue before completing
		Finished    // Signal success back to executor
	};

	void loadParameters();
	void droguePoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

	// S-curve trajectory generation
	void generateTrajectoryToDrogue();
	std::vector<Eigen::Vector3f> generateSCurveWaypoints(
		const Eigen::Vector3f& start, const Eigen::Vector3f& end, int num_points);
	void generateSCurveVelocityProfile(
		float total_distance, int num_samples, float dt,
		std::vector<float>& v_out, std::vector<float>& a_out);

	// Helpers
	float computeDistance(const Eigen::Vector3f& a, const Eigen::Vector3f& b) const;
	Eigen::Vector3f computeDirection(const Eigen::Vector3f& a, const Eigen::Vector3f& b) const;
	void publishPathVisualization();
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
	std::shared_ptr<px4_ros2::TrajectorySetpointType> _trajectory_setpoint;

	State _state = State::Search;
	std::vector<Waypoint> _path;  // Current S-curve trajectory
	int _path_index = 0;          // Next waypoint to track

	// Latest drogue pose relative to drone, updated by YOLO pipeline
	Eigen::Vector3f _drogue_pose = Eigen::Vector3f::Zero();
	bool _drogue_valid = false;
	rclcpp::Time _drogue_timestamp{};

	// Hover state
	Eigen::Vector3f _hover_position = Eigen::Vector3f::Zero();
	rclcpp::Time _hover_start_time{};

	// Tunable ROS parameters (adjustable via CLI or launch file)
	int _param_num_waypoints = 10;            // Number of points along the S-curve
	float _param_s_curve_steepness = 4.0f;    // tanh steepness — higher = sharper transition
	float _param_waypoint_tolerance_m = 0.15f; // Distance to consider a waypoint reached
	float _param_max_velocity = 0.5f;          // Velocity clamp for the profile [m/s]
	float _param_max_acceleration = 0.35f;     // Acceleration clamp for the profile [m/s^2]
	float _param_replan_threshold = 0.10f;     // Replan if drogue moves more than this [m]
	float _param_hover_duration = 1.5f;        // How long to hover at drogue before completing [s]
	float _param_drogue_timeout = 3.0f;        // Max age of drogue pose before considered lost [s]
};

// Executor: manages the arm -> takeoff -> hand-off-to-mode lifecycle
class DroneSmoothPlannerExecutor : public px4_ros2::ModeExecutorBase
{
public:
	DroneSmoothPlannerExecutor(rclcpp::Node& node, px4_ros2::ModeBase& owned_mode);

	// Sequential executor states driven by PX4 result callbacks
	enum class State {
		Arming,      // Request vehicle arm
		TakingOff,   // PX4 internal takeoff to param height
		Approaching, // Hand control to DroneSmoothPlanner mode
	};

	void onActivate() override;
	void onDeactivate(DeactivateReason reason) override;

private:
	void runState(State state, px4_ros2::Result result);

	rclcpp::Node& _node;
	float _param_takeoff_height = 2.5f;
};

} // namespace drogue_flight
