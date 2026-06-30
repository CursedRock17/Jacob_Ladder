/**
 * Visualizer.hpp — RViz / Foxglove Visualization Node
 *
 * Subscribes to PX4 drone state (attitude, position, setpoints) and
 * publishes visualization-friendly messages:
 *   - PoseStamped for the drone's current pose
 *   - Path trails showing where the drone has been and where setpoints were sent
 *   - Arrow markers showing the velocity vector
 *   - TF transforms so RViz can render everything in the correct frame
 *
 * PX4 uses NED (North-East-Down) coordinates, but RViz expects ENU
 * (East-North-Up), so this node handles the conversion.
 */
#pragma once

#include <rclcpp/rclcpp.hpp>

#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <Eigen/Core>

#include <array>
#include <deque>

namespace precision_land
{

class Visualizer : public rclcpp::Node
{
public:
	Visualizer();

private:
	// Callbacks for PX4 data (received in NED frame)
	void vehicleAttitudeCallback(const px4_msgs::msg::VehicleAttitude::SharedPtr msg);
	void vehicleLocalPositionCallback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg);
	void trajectorySetpointCallback(const px4_msgs::msg::TrajectorySetpoint::SharedPtr msg);

	// Main loop — publishes pose, path, velocity marker, and TF at 20 Hz
	void timerCallback();

	// Broadcast map -> odom as identity (static, sent once at startup)
	void publishStaticTransforms();

	// Broadcast map -> base_link from current drone state (sent every loop)
	void publishDynamicTransforms();

	// Helper: build a PoseStamped from position + quaternion
	geometry_msgs::msg::PoseStamped createPoseMsg(
		const std::string& frame_id,
		const Eigen::Vector3d& position,
		const std::array<double, 4>& attitude);

	// Helper: build an arrow Marker for velocity visualization
	visualization_msgs::msg::Marker createArrowMarker(
		int id,
		const Eigen::Vector3d& tail,
		const Eigen::Vector3d& vector);

	// --- Subscribers (PX4 data) ---
	rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr _attitude_sub;
	rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr _local_position_sub;
	rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr _local_position_v1_sub;
	rclcpp::Subscription<px4_msgs::msg::TrajectorySetpoint>::SharedPtr _setpoint_sub;

	// --- Publishers (RViz-friendly messages) ---
	rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr _vehicle_pose_pub;
	rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr _vehicle_vel_pub;
	rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr _vehicle_path_pub;
	rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr _setpoint_path_pub;

	// TF broadcasters for RViz frame tree
	std::unique_ptr<tf2_ros::TransformBroadcaster> _tf_broadcaster;
	std::unique_ptr<tf2_ros::StaticTransformBroadcaster> _static_tf_broadcaster;

	rclcpp::TimerBase::SharedPtr _timer;

	// Drone state — stored in ENU frame (converted from PX4's NED)
	std::array<double, 4> _vehicle_attitude = {1.0, 0.0, 0.0, 0.0}; // quaternion: w, x, y, z
	Eigen::Vector3d _vehicle_position = Eigen::Vector3d::Zero();
	Eigen::Vector3d _vehicle_velocity = Eigen::Vector3d::Zero();
	Eigen::Vector3d _setpoint_position = Eigen::Vector3d::Zero();

	// Path history — rolling buffer for trail visualization
	std::deque<geometry_msgs::msg::PoseStamped> _vehicle_path;
	std::deque<geometry_msgs::msg::PoseStamped> _setpoint_path;
	static constexpr size_t kTrailSize = 1000;  // Max trail length

	// Clear path history after a gap in position updates (< 0 disables)
	double _last_local_pos_update = 0.0;
	double _path_clearing_timeout = -1.0;
};

} // namespace precision_land
