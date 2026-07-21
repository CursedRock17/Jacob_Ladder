#pragma once

#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/components/mode_executor.hpp>
#include <px4_ros2/control/setpoint_types/experimental/trajectory.hpp>
#include <px4_ros2/odometry/local_position.hpp>
#include <px4_ros2/odometry/attitude.hpp>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>

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
	// Mode state machine: Search -> Approach <-> Hover
	// The executor performs a PX4 native takeoff before scheduling this mode, so the
	// vehicle is already airborne on activation and there is no in-mode climb.
	// There is no terminal state either: once the standoff point is reached the mode
	// holds station indefinitely and keeps tracking the drogue, dropping back to
	// Approach if the drogue moves far enough to need a speed-limited transit.
	enum class State {
		Search,     // Hold position, wait for drogue detection
		Approach,   // Drive toward the standoff point in front of the drogue
		Hover       // Station-keep on the (continuously re-tracked) standoff point
	};

	void loadParameters();
	void droguePoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

	// Transform the cached drogue pose (ranging frame: +x left, +y up, +z forward)
	// into an absolute NED target using the vehicle attitude. Returns false if no
	// valid drogue pose or attitude is available yet.
	bool drogueTargetNed(Eigen::Vector3f& target_ned) const;

	// Standoff point we actually fly to: drogue_standoff_m back along the drone->drogue
	// bearing, at the drogue's altitude. Measured from our own approach line, so the
	// path to it never crosses the drogue and it self-corrects on overshoot. Returns
	// false for the same reasons drogueTargetNed() does.
	bool drogueStandoffNed(Eigen::Vector3f& standoff_ned) const;

	// True when every NED component of the error is inside +/- the given band.
	// Checked per-axis rather than on the magnitude so the pass/fail matches the
	// per-axis error vector printed in the approach log.
	bool withinTolerance(const Eigen::Vector3f& error_ned, float band_m) const;

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

	State _state = State::Search;

	// Latest drogue pose relative to drone, updated by YOLO pipeline
	Eigen::Vector3f _drogue_pose = Eigen::Vector3f::Zero();
	bool _drogue_valid = false;
	rclcpp::Time _drogue_timestamp{};

	// Tunable ROS parameters (adjustable via CLI or launch file)
	float _param_takeoff_height = 1.75f;       // PX4 native takeoff altitude [m]
	float _param_drogue_standoff_m = 3.0f;     // Standoff kept back from the drogue [m]
	float _param_carrot_lead_time = 1.0f;      // Setpoint lead ahead of the vehicle [s]
	// Per-axis (+/-) NED band around the standoff point counted as "arrived"
	float _param_final_waypoint_tolerance_m = 0.25f;
	float _param_max_velocity = 0.5f;          // Approach speed cap [m/s]
	float _param_camera_pitch_deg = 0.0f;      // Forward cam mount pitch, +ve = tilted down [deg]
	float _param_drogue_timeout = 3.0f;        // Max age of drogue pose before considered lost [s]
};

// Executor: arm -> PX4 native takeoff -> hand off to the DroneSmoothPlanner mode
class DroneSmoothPlannerExecutor : public px4_ros2::ModeExecutorBase
{
public:
	DroneSmoothPlannerExecutor(rclcpp::Node& node, px4_ros2::ModeBase& owned_mode);

	// Sequential executor states driven by PX4 result callbacks
	enum class State {
		Arming,      // Request vehicle arm
		TakingOff,   // PX4 native takeoff to takeoff_height
		Approaching, // Hand control to DroneSmoothPlanner mode (already airborne)
	};

	void onActivate() override;
	void onDeactivate(DeactivateReason reason) override;

private:
	void runState(State state, px4_ros2::Result result);

	rclcpp::Node& _node;

	// takeoff() completion is detected two ways because its callback does not fire
	// reliably below MIS_TAKEOFF_ALT: this local-position watcher, and the callback
	// itself. Whichever arrives first wins, guarded by _takeoff_complete.
	rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr _local_pos_sub;
	float _param_takeoff_height = 1.75f;
	float _takeoff_target_z = 0.0f;
	bool _in_takeoff = false;
	bool _takeoff_complete = false;
};

} // namespace drogue_flight
