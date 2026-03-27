#include "Visualizer.hpp"

#include <cmath>
#include <chrono>

using namespace std::chrono_literals;

namespace precision_land
{

Visualizer::Visualizer()
	: Node("visualizer")
{
	auto qos_sub = rclcpp::QoS(1).best_effort().durability_volatile();

	// Subscribe to PX4 attitude (NED frame)
	_attitude_sub = create_subscription<px4_msgs::msg::VehicleAttitude>(
		"fmu/out/vehicle_attitude", qos_sub,
		std::bind(&Visualizer::vehicleAttitudeCallback, this, std::placeholders::_1));

	// Subscribe to PX4 local position (v0 topic)
	_local_position_sub = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
		"fmu/out/vehicle_local_position", qos_sub,
		std::bind(&Visualizer::vehicleLocalPositionCallback, this, std::placeholders::_1));

	// Subscribe to translated local position (v1 topic from translation_node)
	_local_position_v1_sub = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
		"fmu/out/vehicle_local_position_v1", qos_sub,
		std::bind(&Visualizer::vehicleLocalPositionCallback, this, std::placeholders::_1));

	// Subscribe to trajectory setpoints being sent to PX4
	_setpoint_sub = create_subscription<px4_msgs::msg::TrajectorySetpoint>(
		"fmu/in/trajectory_setpoint", qos_sub,
		std::bind(&Visualizer::trajectorySetpointCallback, this, std::placeholders::_1));

	// Pose, path, and velocity marker publishers
	_vehicle_pose_pub = create_publisher<geometry_msgs::msg::PoseStamped>(
		"px4_visualizer/vehicle_pose", 10);
	_vehicle_vel_pub = create_publisher<visualization_msgs::msg::Marker>(
		"px4_visualizer/vehicle_velocity", 10);
	_vehicle_path_pub = create_publisher<nav_msgs::msg::Path>(
		"px4_visualizer/vehicle_path", 10);
	_setpoint_path_pub = create_publisher<nav_msgs::msg::Path>(
		"px4_visualizer/setpoint_path", 10);

	// TF broadcasters for RViz frame tree
	_tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
	_static_tf_broadcaster = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);

	// Publish the static map -> odom identity transform once
	publishStaticTransforms();

	// Path clearing timeout param (-1 disables clearing)
	declare_parameter<double>("path_clearing_timeout", -1.0);
	get_parameter("path_clearing_timeout", _path_clearing_timeout);

	// Main loop at 20 Hz
	_timer = create_wall_timer(50ms, std::bind(&Visualizer::timerCallback, this));
}

void Visualizer::publishStaticTransforms()
{
	// Publish map -> odom as identity so both frames exist in the TF tree
	geometry_msgs::msg::TransformStamped t;
	t.header.stamp = get_clock()->now();
	t.header.frame_id = "map";
	t.child_frame_id = "odom";
	t.transform.translation.x = 0.0;
	t.transform.translation.y = 0.0;
	t.transform.translation.z = 0.0;
	t.transform.rotation.w = 1.0;
	t.transform.rotation.x = 0.0;
	t.transform.rotation.y = 0.0;
	t.transform.rotation.z = 0.0;
	_static_tf_broadcaster->sendTransform(t);
}

void Visualizer::publishDynamicTransforms()
{
	// Broadcast map -> base_link using the drone's current ENU pose
	geometry_msgs::msg::TransformStamped t;
	t.header.stamp = get_clock()->now();
	t.header.frame_id = "map";
	t.child_frame_id = "base_link";
	t.transform.translation.x = _vehicle_position[0];
	t.transform.translation.y = _vehicle_position[1];
	t.transform.translation.z = _vehicle_position[2];
	t.transform.rotation.w = _vehicle_attitude[0];
	t.transform.rotation.x = _vehicle_attitude[1];
	t.transform.rotation.y = _vehicle_attitude[2];
	t.transform.rotation.z = _vehicle_attitude[3];
	_tf_broadcaster->sendTransform(t);
}

void Visualizer::vehicleAttitudeCallback(const px4_msgs::msg::VehicleAttitude::SharedPtr msg)
{
	// Convert NED quaternion (qw, qx, qy, qz) to ENU frame
	double qw = msg->q[0];
	double qx = msg->q[1];
	double qy = msg->q[2];
	double qz = msg->q[3];

	double inv_sqrt2 = 1.0 / std::sqrt(2.0);
	std::array<double, 4> q_enu = {
		inv_sqrt2 * (qw + qz),
		inv_sqrt2 * (qx + qy),
		inv_sqrt2 * (qx - qy),
		inv_sqrt2 * (qw - qz)
	};

	// Normalize the quaternion
	double norm = std::sqrt(q_enu[0]*q_enu[0] + q_enu[1]*q_enu[1] +
				q_enu[2]*q_enu[2] + q_enu[3]*q_enu[3]);
	if (norm > 1e-9) {
		for (auto& q : q_enu) {
			q /= norm;
		}
	}

	_vehicle_attitude = q_enu;
}

void Visualizer::vehicleLocalPositionCallback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
{
	double now_sec = get_clock()->now().nanoseconds() / 1e9;

	// Clear path history if we haven't received a position update in a while
	if (_path_clearing_timeout >= 0.0 &&
		(now_sec - _last_local_pos_update) > _path_clearing_timeout)
	{
		_vehicle_path.clear();
	}
	_last_local_pos_update = now_sec;

	// NED -> ENU: swap x/y, negate z
	_vehicle_position[0] = msg->y;
	_vehicle_position[1] = msg->x;
	_vehicle_position[2] = -msg->z;

	_vehicle_velocity[0] = msg->vy;
	_vehicle_velocity[1] = msg->vx;
	_vehicle_velocity[2] = -msg->vz;
}

void Visualizer::trajectorySetpointCallback(const px4_msgs::msg::TrajectorySetpoint::SharedPtr msg)
{
	// NED -> ENU for setpoint position
	_setpoint_position[0] = msg->position[1];
	_setpoint_position[1] = msg->position[0];
	_setpoint_position[2] = -msg->position[2];
}

void Visualizer::timerCallback()
{
	// Publish drone pose in map frame
	auto vehicle_pose_msg = createPoseMsg("map", _vehicle_position, _vehicle_attitude);
	_vehicle_pose_pub->publish(vehicle_pose_msg);

	// Append to vehicle path trail
	_vehicle_path.push_back(vehicle_pose_msg);
	if (_vehicle_path.size() > kTrailSize) {
		_vehicle_path.pop_front();
	}

	nav_msgs::msg::Path vehicle_path_msg;
	vehicle_path_msg.header = vehicle_pose_msg.header;
	vehicle_path_msg.poses.assign(_vehicle_path.begin(), _vehicle_path.end());
	_vehicle_path_pub->publish(vehicle_path_msg);

	// Append to setpoint path trail
	auto setpoint_pose_msg = createPoseMsg("odom", _vehicle_position, _vehicle_attitude);
	_setpoint_path.push_back(setpoint_pose_msg);
	if (_setpoint_path.size() > kTrailSize) {
		_setpoint_path.pop_front();
	}

	nav_msgs::msg::Path setpoint_path_msg;
	setpoint_path_msg.header = setpoint_pose_msg.header;
	setpoint_path_msg.poses.assign(_setpoint_path.begin(), _setpoint_path.end());
	_setpoint_path_pub->publish(setpoint_path_msg);

	// Publish velocity arrow marker
	auto velocity_msg = createArrowMarker(1, _vehicle_position, _vehicle_velocity);
	_vehicle_vel_pub->publish(velocity_msg);

	// Broadcast map -> base_link TF
	publishDynamicTransforms();
}

geometry_msgs::msg::PoseStamped Visualizer::createPoseMsg(
	const std::string& frame_id,
	const Eigen::Vector3d& position,
	const std::array<double, 4>& attitude)
{
	geometry_msgs::msg::PoseStamped pose_msg;
	pose_msg.header.stamp = get_clock()->now();
	pose_msg.header.frame_id = frame_id;
	pose_msg.pose.orientation.w = attitude[0];
	pose_msg.pose.orientation.x = attitude[1];
	pose_msg.pose.orientation.y = attitude[2];
	pose_msg.pose.orientation.z = attitude[3];
	pose_msg.pose.position.x = position[0];
	pose_msg.pose.position.y = position[1];
	pose_msg.pose.position.z = position[2];
	return pose_msg;
}

visualization_msgs::msg::Marker Visualizer::createArrowMarker(
	int id,
	const Eigen::Vector3d& tail,
	const Eigen::Vector3d& vector)
{
	visualization_msgs::msg::Marker msg;
	msg.action = visualization_msgs::msg::Marker::ADD;
	msg.header.frame_id = "map";
	msg.header.stamp = get_clock()->now();
	msg.ns = "arrow";
	msg.id = id;
	msg.type = visualization_msgs::msg::Marker::ARROW;
	msg.scale.x = 0.1;
	msg.scale.y = 0.2;
	msg.scale.z = 0.0;
	msg.color.r = 0.5f;
	msg.color.g = 0.5f;
	msg.color.b = 0.0f;
	msg.color.a = 1.0f;

	// Scale velocity vector for visual length
	constexpr double dt = 0.3;

	geometry_msgs::msg::Point tail_point;
	tail_point.x = tail[0];
	tail_point.y = tail[1];
	tail_point.z = tail[2];

	geometry_msgs::msg::Point head_point;
	head_point.x = tail[0] + dt * vector[0];
	head_point.y = tail[1] + dt * vector[1];
	head_point.z = tail[2] + dt * vector[2];

	msg.points.push_back(tail_point);
	msg.points.push_back(head_point);

	return msg;
}

} // namespace precision_land

int main(int argc, char* argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<precision_land::Visualizer>());
	rclcpp::shutdown();
	return 0;
}
