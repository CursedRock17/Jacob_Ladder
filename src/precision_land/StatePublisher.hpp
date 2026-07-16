#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <chrono>
#include <string>

namespace precision_land
{

// Publishes the mode/executor state machine as a string on <node>/state so a
// flight can be replayed in Foxglove (State Transitions panel on
// "<node>/state.data") and correlated with the rest of the bag.
//
// The topic is latched (transient_local) for live viewers that connect late,
// and the current state is re-published at 1 Hz so a bag recording that
// starts mid-flight still contains it.
class StatePublisher
{
public:
	explicit StatePublisher(rclcpp::Node& node)
		: _pub(node.create_publisher<std_msgs::msg::String>(
			"~/state", rclcpp::QoS(1).transient_local()))
		, _timer(node.create_wall_timer(
			std::chrono::seconds(1), [this] { publish(); }))
	{}

	void set(const std::string& state)
	{
		_state = state;
		publish();
	}

private:
	void publish()
	{
		if (_state.empty()) return;
		std_msgs::msg::String msg;
		msg.data = _state;
		_pub->publish(msg);
	}

	std::string _state;
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _pub;
	rclcpp::TimerBase::SharedPtr _timer;
};

} // namespace precision_land
