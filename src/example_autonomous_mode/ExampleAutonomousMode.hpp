// Example PX4 external mode and executor: take off, hold, descend, and land.
// See README.md for the state diagrams and instructions for copying the example.
#pragma once

#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/components/mode_executor.hpp>
#include <px4_ros2/control/setpoint_types/experimental/trajectory.hpp>
#include <px4_ros2/odometry/local_position.hpp>

#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <px4_msgs/msg/vehicle_land_detected.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <Eigen/Core>

#include <memory>
#include <string>

namespace example_autonomous_mode {

// The name PX4 registers this mode under, as it appears in QGroundControl.
// Must be unique across every mode running against the same autopilot, and
// PX4 caps it at 24 characters (px4_msgs/RegisterExtComponentRequest).
inline constexpr char kExampleAutonomousModeName[] = "ExampleAutonomousMode";
inline constexpr bool kExampleAutonomousModeDebugOutput = true;

class ExampleAutonomousMode : public px4_ros2::ModeBase {
public:
  explicit ExampleAutonomousMode(rclcpp::Node &node);

  // Called when PX4 selects this mode; the executor schedules it after takeoff.
  void onActivate() override;
  // Called when PX4 leaves this mode (completion, takeover, or failsafe).
  void onDeactivate() override;

  // Called by the mode timer; dt_s is the measured interval in seconds.
  void updateSetpoint(float dt_s) override;

private:
  enum class State {
    Idle,       // Before activation and after deactivation; no setpoints
    Holding,    // Hovering in place for a set duration
    Descending, // Controlled descent to the native-landing handoff
    Finished    // Report success to the executor
  };

  void loadParameters();
  void vehicleLandDetectedCallback(
      const px4_msgs::msg::VehicleLandDetected::SharedPtr msg);
  void switchToState(State state);
  std::string stateName(State state) const;

  // Send a position setpoint and publish the optional NED tracking error.
  void commandPosition(const Eigen::Vector3f &pos);

  rclcpp::Node &_node;

  // Optional preflight ground reference and descent stop signal
  rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr
      _vehicle_land_detected_sub;
  // Optional debug topics.
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _drone_state_pub;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr
      _tracking_error_pub;

  std::shared_ptr<px4_ros2::OdometryLocalPosition> _vehicle_local_position;
  std::shared_ptr<px4_ros2::TrajectorySetpointType> _trajectory_setpoint;

  State _state = State::Idle;
  Eigen::Vector3f _hold_position{
      Eigen::Vector3f::Zero()}; // Current commanded position
  bool _land_detected = false;
  bool _ground_z_valid = false;
  float _state_elapsed = 0.0f; // Time spent in the current state (seconds)
  float _ground_z = 0.0f;      // Ground plane observed while landed

  // Defaults, overridden by cfg/example_autonomous_mode_params.yaml.
  float _hold_duration = 7.5f;   // How long to hold at altitude (s)
  float _descent_vel = 0.5f;     // Vertical speed during descent (m/s)
  float _landing_height = 0.10f; // Native-landing handoff height AGL (m)
};

class ExampleAutonomousModeExecutor : public px4_ros2::ModeExecutorBase {
public:
  ExampleAutonomousModeExecutor(rclcpp::Node &node,
                                px4_ros2::ModeBase &owned_mode);
  enum class State {
    Arming,           // Arm the vehicle
    TakingOff,        // PX4-native takeoff to its configured altitude
    RunningMode,      // Schedule the external setpoint mode
    Landing,          // PX4-native final touchdown
    WaitingForDisarm, // Wait for PX4 to report the vehicle disarmed
  };

  void onActivate() override;
  void onDeactivate(DeactivateReason reason) override;

private:
  void runState(State state, px4_ros2::Result result);

  rclcpp::Node &_node;
};

} // namespace example_autonomous_mode
