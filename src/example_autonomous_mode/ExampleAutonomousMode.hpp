/**
 * ExampleAutonomousMode.hpp — a minimal PX4 external mode + executor workflow
 *
 * This is the reference example for writing a new autonomous mode. It is
 * deliberately the simplest thing that still flies end to end:
 *
 *   1. The executor arms and uses PX4's native takeoff to reach a low
 *      "optical flow" height
 *   2. The mode pauses there so the flow sensor can lock, then holds position
 *   3. The mode descends at a controlled rate before the executor hands final
 *      touchdown and disarming back to PX4
 *
 * It registers as a custom PX4 flight mode via the px4_ros2 library, which
 * means it appears in QGroundControl next to the built-in modes. Copy this
 * package as the starting point for a new mode — see README.md for the parts
 * you will want to change first.
 */
#pragma once

#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/components/mode_executor.hpp>
#include <px4_ros2/control/setpoint_types/experimental/trajectory.hpp>
#include <px4_ros2/odometry/local_position.hpp>

#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <px4_msgs/msg/vehicle_land_detected.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <Eigen/Core>

#include <memory>
#include <string>

namespace example_autonomous_mode
{

  // The name PX4 registers this mode under, as it appears in QGroundControl.
  // Must be unique across every mode running against the same autopilot, and
  // PX4 caps it at 24 characters (px4_msgs/RegisterExtComponentRequest).
  inline constexpr char kExampleAutonomousModeName[] = "ExampleAutonomousMode";
  inline constexpr bool kExampleAutonomousModeDebugOutput = true;

  class ExampleAutonomousMode : public px4_ros2::ModeBase
  {
  public:
    explicit ExampleAutonomousMode(rclcpp::Node &node);

    // Called by PX4 when this mode becomes active / inactive
    void onActivate() override;
    void onDeactivate() override;

    // Called every control loop iteration — this is where the state machine lives
    void updateSetpoint(float dt_s) override;

  private:
    // Flight sequence as a state machine
    enum class State
    {
      Idle,                // Not doing anything
      OpticalFlowSettling, // Low hover while optical flow / VIO odometry settles
      Holding,             // Hovering in place for a set duration
      Descending,          // Controlled descent to the native-landing handoff
      Finished             // Report success to the executor
    };

    void loadParameters();
    void vehicleLandDetectedCallback(
        const px4_msgs::msg::VehicleLandDetected::SharedPtr msg);
    void switchToState(State state);
    std::string stateName(State state) const;

    // Wraps _trajectory_setpoint->updatePosition and publishes the
    // (commanded - actual) position error on /tracking_error
    void commandPosition(const Eigen::Vector3f &pos);

    rclcpp::Node &_node;

    // Subscription to know when the drone has physically touched down
    rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr
        _vehicle_land_detected_sub;

    // PX4 ROS 2 interface objects for reading position and sending commands
    std::shared_ptr<px4_ros2::OdometryLocalPosition> _vehicle_local_position;
    std::shared_ptr<px4_ros2::TrajectorySetpointType> _trajectory_setpoint;

    // State machine tracking
    State _state = State::Idle;
    Eigen::Vector3f _base_position{
        Eigen::Vector3f::Zero()}; // Where the drone started (NED)
    Eigen::Vector3f _hold_position{
        Eigen::Vector3f::Zero()}; // Current commanded position
    bool _land_detected = false;
    bool _ground_z_valid = false;
    float _state_elapsed = 0.0f; // Time spent in the current state (seconds)
    float _ground_z = 0.0f;      // Observed or, as a fallback, inferred ground plane

    // Tunable parameters — defaults here are only a fallback; the real values
    // come from cfg/example_autonomous_mode_params.yaml via the launch file
    float _optical_flow_height = 0.25f;   // Height for optical flow init (m)
    float _optical_flow_hold_time = 3.0f; // Hover time at flow height (s)
    float _delta_position = 0.05f;        // Executor takeoff completion tolerance (m)
    float _hold_duration = 7.5f;          // How long to hold at altitude (s)
    float _descent_vel = 0.5f;            // Vertical speed during descent (m/s)
    float _landing_height = 0.10f;        // Native-landing handoff height AGL (m)
  };

  class ExampleAutonomousModeExecutor : public px4_ros2::ModeExecutorBase
  {
  public:
    ExampleAutonomousModeExecutor(rclcpp::Node &node,
                                  px4_ros2::ModeBase &owned_mode);
    enum class State
    {
      Arming,           // Arm the vehicle
      TakingOff,        // PX4-native takeoff to optical_flow_height
      RunningMode,      // Schedule the external setpoint mode
      Landing,          // PX4-native final touchdown
      WaitingForDisarm, // Wait for PX4 to report the vehicle disarmed
    };

    void onActivate() override;
    void onDeactivate(DeactivateReason reason) override;

  private:
    void runState(State state, px4_ros2::Result result);

    rclcpp::Node &_node;

    // takeoff() completion is detected two ways because its callback does not
    // fire reliably below MIS_TAKEOFF_ALT: this local-position watcher, and the
    // callback itself. Whichever arrives first wins, guarded by
    // _takeoff_complete.
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr
        _local_pos_sub;

    float _optical_flow_height = 0.25f;
    float _delta_position = 0.05f;
    float _takeoff_target_z = -0.20f;
    float _latest_local_z = 0.0f;
    bool _have_local_position = false;
    bool _in_takeoff = false;
    bool _takeoff_complete = false;
  };

} // namespace example_autonomous_mode
