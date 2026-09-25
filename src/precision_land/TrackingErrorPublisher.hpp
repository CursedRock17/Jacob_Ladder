#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>

#include <Eigen/Core>

namespace precision_land
{

// Publishes (commanded - actual) position on /tracking_error, so a bag shows
// whether PX4 is actually following the position setpoints a mode sends.
// Call publish() right after each position setpoint.
class TrackingErrorPublisher
{
public:
	explicit TrackingErrorPublisher(rclcpp::Node& node)
		: _node(node)
		, _pub(node.create_publisher<geometry_msgs::msg::Vector3Stamped>(
			"/tracking_error", rclcpp::QoS(10)))
	{}

	void publish(const Eigen::Vector3f& commanded, const Eigen::Vector3f& actual)
	{
		geometry_msgs::msg::Vector3Stamped err;
		err.header.stamp = _node.now();
		err.header.frame_id = "odom";  // PX4 local position is NED relative to the EKF origin
		err.vector.x = commanded.x() - actual.x();
		err.vector.y = commanded.y() - actual.y();
		err.vector.z = commanded.z() - actual.z();
		_pub->publish(err);
	}

private:
	rclcpp::Node& _node;
	rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr _pub;
};

} // namespace precision_land
