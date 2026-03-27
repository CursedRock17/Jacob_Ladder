#include "BlankMode.hpp"

#include <px4_ros2/components/node_with_mode.hpp>

namespace precision_land
{

BlankMode::BlankMode(rclcpp::Node& node)
	: ModeBase(node, ModeBase::Settings{kBlankModeModeName})
	, _node(node)
	, _hold_position(Eigen::Vector3f::Zero())
{
	setSkipMessageCompatibilityCheck();

	_vehicle_local_position = std::make_shared<px4_ros2::OdometryLocalPosition>(*this);
	_trajectory_setpoint = std::make_shared<px4_ros2::TrajectorySetpointType>(*this);
}

void BlankMode::onActivate()
{
	_hold_position = _vehicle_local_position->positionNed();
	_position_set = true;
	RCLCPP_INFO(_node.get_logger(), "BlankMode activated — holding position [%.2f, %.2f, %.2f]",
		_hold_position.x(), _hold_position.y(), _hold_position.z());
}

void BlankMode::onDeactivate()
{
	_position_set = false;
}

void BlankMode::updateSetpoint(float dt_s)
{
	if (_position_set) {
		_trajectory_setpoint->updatePosition(_hold_position);
	}
}

} // namespace precision_land

int main(int argc, char* argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<px4_ros2::NodeWithMode<precision_land::BlankMode>>(
		precision_land::kBlankModeModeName, precision_land::kBlankModeDebugOutput));
	rclcpp::shutdown();
	return 0;
}
