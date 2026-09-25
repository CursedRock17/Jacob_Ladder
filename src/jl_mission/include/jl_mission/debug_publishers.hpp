#pragma once

// Debug topics every jl_mission instance publishes, matching precision_land's
// StatePublisher and TrackingErrorPublisher (copied rather than shared, so the
// flight code in src/precision_land stays untouched).

#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <Eigen/Core>

#include <chrono>
#include <string>

namespace jl_mission {

// Latched state string, re-published at 1 Hz so a bag that starts mid-flight
// still contains it.
class StatePublisher {
public:
  StatePublisher(rclcpp::Node &node, const std::string &topic)
      : _pub(node.create_publisher<std_msgs::msg::String>(
            topic, rclcpp::QoS(1).transient_local())),
        _timer(node.create_wall_timer(std::chrono::seconds(1),
                                      [this] { publish(); })) {}

  void set(const std::string &state) {
    if (state == _state) {
      return;
    }
    _state = state;
    publish();
  }

private:
  void publish() {
    if (_state.empty()) {
      return;
    }
    std_msgs::msg::String msg;
    msg.data = _state;
    _pub->publish(msg);
  }

  std::string _state;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _pub;
  rclcpp::TimerBase::SharedPtr _timer;
};

// (commanded - actual) position on /tracking_error, NED.
class TrackingErrorPublisher {
public:
  explicit TrackingErrorPublisher(rclcpp::Node &node)
      : _node(node),
        _pub(node.create_publisher<geometry_msgs::msg::Vector3Stamped>(
            "/tracking_error", rclcpp::QoS(10))) {}

  void publish(const Eigen::Vector3f &commanded,
               const Eigen::Vector3f &actual) {
    geometry_msgs::msg::Vector3Stamped err;
    err.header.stamp = _node.now();
    err.header.frame_id = "odom";
    err.vector.x = commanded.x() - actual.x();
    err.vector.y = commanded.y() - actual.y();
    err.vector.z = commanded.z() - actual.z();
    _pub->publish(err);
  }

private:
  rclcpp::Node &_node;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr _pub;
};

} // namespace jl_mission
