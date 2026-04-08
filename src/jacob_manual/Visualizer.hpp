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
	// PX4 subscription callbacks
	void vehicleAttitudeCallback(const px4_msgs::msg::VehicleAttitude::SharedPtr msg);
	void vehicleLocalPositionCallback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg);
	void trajectorySetpointCallback(const px4_msgs::msg::TrajectorySetpoint::SharedPtr msg);

	// Main loop publishes pose, path, markers, and TF
	void timerCallback();

	// Broadcast map -> odom as identity (static, sent once)
	void publishStaticTransforms();

	// Broadcast map -> base_link from current drone state
	void publishDynamicTransforms();

	// Build a PoseStamped from position + quaternion
	geometry_msgs::msg::PoseStamped createPoseMsg(
		const std::string& frame_id,
		const Eigen::Vector3d& position,
		const std::array<double, 4>& attitude);

	// Build an arrow Marker for velocity visualization
	visualization_msgs::msg::Marker createArrowMarker(
		int id,
		const Eigen::Vector3d& tail,
		const Eigen::Vector3d& vector);

	// Subscribers
	rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr _attitude_sub;
	rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr _local_position_sub;
	rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr _local_position_v1_sub;
	rclcpp::Subscription<px4_msgs::msg::TrajectorySetpoint>::SharedPtr _setpoint_sub;

	// Publishers
	rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr _vehicle_pose_pub;
	rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr _vehicle_vel_pub;
	rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr _vehicle_path_pub;
	rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr _setpoint_path_pub;

	// TF broadcasters
	std::unique_ptr<tf2_ros::TransformBroadcaster> _tf_broadcaster;
	std::unique_ptr<tf2_ros::StaticTransformBroadcaster> _static_tf_broadcaster;

	// Timer
	rclcpp::TimerBase::SharedPtr _timer;

	// Drone state in ENU frame
	std::array<double, 4> _vehicle_attitude = {1.0, 0.0, 0.0, 0.0}; // w, x, y, z
	Eigen::Vector3d _vehicle_position = Eigen::Vector3d::Zero();
	Eigen::Vector3d _vehicle_velocity = Eigen::Vector3d::Zero();
	Eigen::Vector3d _setpoint_position = Eigen::Vector3d::Zero();

	// Path history for trail visualization
	std::deque<geometry_msgs::msg::PoseStamped> _vehicle_path;
	std::deque<geometry_msgs::msg::PoseStamped> _setpoint_path;

	static constexpr size_t kTrailSize = 1000;

	// Path clearing timeout (-1 disables)
	double _last_local_pos_update = 0.0;
	double _path_clearing_timeout = -1.0;
};

} // namespace precision_land
