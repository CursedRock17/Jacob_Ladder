/**
 * ExampleAutonomousMode.cpp — implementation of the example flight mode
 *
 * State machine flow:
 *   Holding -> Descending -> Finished
 *
 * See ExampleAutonomousMode.hpp for a description of each state, and README.md
 * for how to extend this into a mode of your own.
 *
 * The executor owns arming, native takeoff, native landing, and the disarm wait. The
 * mode owns the continuous setpoints between takeoff and landing. A real mode
 * adds its own states between the hold and controlled descent.
 */

#include "ExampleAutonomousMode.hpp"

#include <px4_ros2/components/node_with_mode.hpp>

#include <memory>

namespace example_autonomous_mode
{

  ExampleAutonomousMode::ExampleAutonomousMode(rclcpp::Node &node)
      : ModeBase(node, ModeBase::Settings{kExampleAutonomousModeName, false}),
        _node(node)
  {
    // This checkout uses PX4 message translation, so both the mode and executor
    // must skip the upstream message-version check during registration.
    setSkipMessageCompatibilityCheck();

    // Create PX4 ROS 2 interface objects for position reading and setpoint
    // sending
    _vehicle_local_position =
        std::make_shared<px4_ros2::OdometryLocalPosition>(*this);
    _trajectory_setpoint =
        std::make_shared<px4_ros2::TrajectorySetpointType>(*this);

    // A landed sample gives the optional controlled descent a ground reference.
    auto qos = rclcpp::QoS(1).best_effort();
    _vehicle_land_detected_sub =
        _node.create_subscription<px4_msgs::msg::VehicleLandDetected>(
            "/fmu/out/vehicle_land_detected", qos,
            std::bind(&ExampleAutonomousMode::vehicleLandDetectedCallback, this,
                      std::placeholders::_1));

    _drone_state_pub = _node.create_publisher<std_msgs::msg::String>(
        "/drone_state", rclcpp::QoS(10));
    _tracking_error_pub =
        _node.create_publisher<geometry_msgs::msg::Vector3Stamped>(
            "/tracking_error", rclcpp::QoS(10));

    loadParameters();
  }

  void ExampleAutonomousMode::loadParameters()
  {
    _node.declare_parameter<float>("hold_duration", _hold_duration);
    _node.declare_parameter<float>("descent_vel", _descent_vel);
    _node.declare_parameter<float>("landing_height", _landing_height);

    _node.get_parameter("hold_duration", _hold_duration);
    _node.get_parameter("descent_vel", _descent_vel);
    _node.get_parameter("landing_height", _landing_height);
  }

  void ExampleAutonomousMode::vehicleLandDetectedCallback(
      const px4_msgs::msg::VehicleLandDetected::SharedPtr msg)
  {
    _land_detected = msg->landed;
    if (msg->landed && _vehicle_local_position->positionZValid())
    {
      _ground_z = _vehicle_local_position->positionNed().z();
      _ground_z_valid = true;
    }
  }

  void ExampleAutonomousMode::onActivate()
  {
    // Native takeoff has completed. Hold the position PX4 reached.
    _hold_position = _vehicle_local_position->positionNed();
    // The optional land-detected sample gives the controlled descent an actual
    // ground reference. Without it, let PX4 handle the whole landing instead.
    if (!_ground_z_valid)
    {
      RCLCPP_WARN(_node.get_logger(),
                  "No preflight ground sample; skipping controlled descent");
    }
    _land_detected = false;
    switchToState(State::Holding);

    RCLCPP_INFO(_node.get_logger(),
                "ExampleAutonomousMode active after takeoff — holding %.1f s",
                _hold_duration);
  }

  void ExampleAutonomousMode::onDeactivate()
  {
    switchToState(State::Idle);
    _ground_z_valid = false;
  }

  void ExampleAutonomousMode::updateSetpoint(float dt_s)
  {
    _state_elapsed += dt_s;

    switch (_state)
    {
    case State::Idle:
      break;

    // Hold, then descend if the ground was observed; otherwise use native land.
    case State::Holding:
    {
      if (_state_elapsed >= _hold_duration)
      {
        commandPosition(_hold_position);
        if (_ground_z_valid)
        {
          RCLCPP_INFO(_node.get_logger(), "Hold complete — descending");
          switchToState(State::Descending);
        }
        else
        {
          switchToState(State::Finished);
        }
        break;
      }

      commandPosition(_hold_position);
      break;
    }

    // Descend to the handoff height, or stop if PX4 already reports landed.
    case State::Descending:
    {
      const Eigen::Vector3f current_position =
          _vehicle_local_position->positionNed();
      // Positive z velocity = downward in NED
      const Eigen::Vector3f velocity(0.f, 0.f, _descent_vel);
      _trajectory_setpoint->update(velocity, std::nullopt, 0.0f);

      // Hand back to the executor shortly above the observed ground plane. PX4's
      // native land() then owns final touchdown and land detection.
      const float landing_handoff_z = _ground_z - _landing_height;
      if (_land_detected || current_position.z() >= landing_handoff_z)
      {
        switchToState(State::Finished);
      }
      break;
    }

    // Keep a setpoint available until the executor switches to native land.
    case State::Finished:
    {
      commandPosition(_vehicle_local_position->positionNed());
      break;
    }
    }
  }

  void ExampleAutonomousMode::switchToState(State state)
  {
    // Keep timing, debug output, and executor completion tied to one transition.
    if (_state == state)
    {
      return;
    }

    RCLCPP_INFO(_node.get_logger(), "Switching to %s", stateName(state).c_str());

    std_msgs::msg::String state_msg;
    state_msg.data = stateName(state);

    _state = state;
    // Every state measures its own dwell time from the moment it is entered, so
    // reset the clock centrally rather than in each transition
    _state_elapsed = 0.0f;
    _drone_state_pub->publish(state_msg);

    // Report the result to PX4 exactly once, on entry, rather than every tick
    if (state == State::Finished)
    {
      ModeBase::completed(px4_ros2::Result::Success);
    }
  }

  void ExampleAutonomousMode::commandPosition(const Eigen::Vector3f &pos)
  {
    // Send the requested NED position to PX4. Calling this every update keeps
    // the position setpoint active while the mode is holding its current state.
    _trajectory_setpoint->updatePosition(pos);

    // Measure the controller's instantaneous tracking error in the same local
    // NED frame as the setpoint: positive values mean the vehicle has not yet
    // reached the commanded coordinate along that axis.
    const auto actual = _vehicle_local_position->positionNed();
    geometry_msgs::msg::Vector3Stamped err;
    err.header.stamp = _node.now();
    err.header.frame_id = "odom";
    err.vector.x = pos.x() - actual.x();
    err.vector.y = pos.y() - actual.y();
    err.vector.z = pos.z() - actual.z();
    _tracking_error_pub->publish(err);
  }

  std::string ExampleAutonomousMode::stateName(State state) const
  {
    // Use the same names in logs and /drone_state for easy cross-checking.
    switch (state)
    {
    case State::Idle:
      return "Idle";
    case State::Holding:
      return "Holding";
    case State::Descending:
      return "Descending";
    case State::Finished:
      return "Finished";
    }

    return "Unknown";
  }

  ExampleAutonomousModeExecutor::ExampleAutonomousModeExecutor(
      rclcpp::Node &node, px4_ros2::ModeBase &owned_mode)
      : ModeExecutorBase(
            node,
            ModeExecutorBase::Settings{Settings::Activation::ActivateAlways},
            owned_mode),
        _node(node)
  {
    // The executor registers separately and needs the same version override.
    setSkipMessageCompatibilityCheck();
  }

  void ExampleAutonomousModeExecutor::onActivate()
  {
    RCLCPP_INFO(_node.get_logger(),
                "Example executor — arm, use PX4 native takeoff, run mode, land");
    runState(State::Arming, px4_ros2::Result::Success);
  }

  void ExampleAutonomousModeExecutor::onDeactivate(DeactivateReason reason)
  {
    RCLCPP_INFO(_node.get_logger(), "Example executor deactivated (%i)",
                static_cast<int>(reason));
  }

  void ExampleAutonomousModeExecutor::runState(State state,
                                               px4_ros2::Result result)
  {
    if (result != px4_ros2::Result::Success)
    {
      RCLCPP_ERROR(_node.get_logger(), "Executor state %i failed: %s",
                   static_cast<int>(state), resultToString(result));
      return;
    }

    switch (state)
    {
    case State::Arming:
      RCLCPP_INFO(_node.get_logger(), "Arming");
      arm([this](px4_ros2::Result r)
          { runState(State::TakingOff, r); });
      break;

    case State::TakingOff:
      RCLCPP_INFO(_node.get_logger(), "Armed — PX4 native takeoff");
      takeoff([this](px4_ros2::Result r)
              { runState(State::RunningMode, r); });
      break;

    case State::RunningMode:
      RCLCPP_INFO(_node.get_logger(),
                  "Takeoff complete — scheduling ExampleAutonomousMode");
      scheduleMode(ownedMode().id(), [this](px4_ros2::Result r)
                   {
      RCLCPP_INFO(_node.get_logger(), "ExampleAutonomousMode ended (%s)",
                  resultToString(r));
      // A non-success result normally means a pilot takeover or failsafe. Do
      // not issue an automatic landing command in that case.
      if (r == px4_ros2::Result::Success) {
        runState(State::Landing, r);
      } });
      break;

    case State::Landing:
      RCLCPP_INFO(_node.get_logger(), "Requesting PX4 native landing");
      land([this](px4_ros2::Result r)
           { runState(State::WaitingForDisarm, r); });
      break;

    case State::WaitingForDisarm:
      RCLCPP_INFO(_node.get_logger(), "Landed — waiting for disarm");
      waitUntilDisarmed([this](px4_ros2::Result r)
                        {
      if (r == px4_ros2::Result::Success) {
        RCLCPP_INFO(_node.get_logger(),
                    "Disarmed — example autonomous workflow complete");
      } else {
        RCLCPP_ERROR(_node.get_logger(), "Disarm wait failed: %s",
                     resultToString(r));
      } });
      break;
    }
  }

} // namespace example_autonomous_mode

// NodeWithModeExecutor constructs, registers, and connects the workflow and
// mode.
int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<px4_ros2::NodeWithModeExecutor<
                   example_autonomous_mode::ExampleAutonomousModeExecutor,
                   example_autonomous_mode::ExampleAutonomousMode>>(
      example_autonomous_mode::kExampleAutonomousModeName,
      example_autonomous_mode::kExampleAutonomousModeDebugOutput));
  rclcpp::shutdown();
  return 0;
}
