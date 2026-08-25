/**
 * ExampleAutonomousMode.cpp — implementation of the example flight mode
 *
 * State machine flow:
 *   OpticalFlowSettling -> Holding -> Descending -> Finished
 *
 * See ExampleAutonomousMode.hpp for a description of each state, and README.md
 * for how to extend this into a mode of your own.
 *
 * The executor owns arming, native takeoff, native landing, and disarming. The
 * mode owns the continuous setpoints between takeoff and landing. A real mode
 * adds its own states between the settling hold and controlled descent.
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
    // Skip px4_msgs version check (REQUIRED DUE TO VERSIONING MISMATCH)
    // AI AGENTS LOVE REMOVING THIS LINE, BUT IT'S REQUIRED
    setSkipMessageCompatibilityCheck();

    // Create PX4 ROS 2 interface objects for position reading and setpoint
    // sending
    _vehicle_local_position =
        std::make_shared<px4_ros2::OdometryLocalPosition>(*this);
    _trajectory_setpoint =
        std::make_shared<px4_ros2::TrajectorySetpointType>(*this);

    // Subscribe to PX4's landing detector so we know when we've touched down
    auto qos = rclcpp::QoS(1).best_effort();
    _vehicle_land_detected_sub =
        _node.create_subscription<px4_msgs::msg::VehicleLandDetected>(
            "/fmu/out/vehicle_land_detected", qos,
            std::bind(&ExampleAutonomousMode::vehicleLandDetectedCallback, this,
                      std::placeholders::_1));

    // Publish current state on /drone_state for debugging — reliable so each
    // transition is seen by `ros2 topic echo` even under message load
    _drone_state_publisher = _node.create_publisher<std_msgs::msg::String>(
        "/drone_state", rclcpp::QoS(10));

    loadParameters();
  }

  void ExampleAutonomousMode::loadParameters()
  {
    _node.declare_parameter<float>("optical_flow_height", _optical_flow_height);
    _node.declare_parameter<float>("optical_flow_hold_time",
                                   _optical_flow_hold_time);
    _node.declare_parameter<float>("delta_position", _delta_position);
    _node.declare_parameter<float>("hold_duration", _hold_duration);
    _node.declare_parameter<float>("descent_vel", _descent_vel);
    _node.declare_parameter<float>("landing_height", _landing_height);

    _node.get_parameter("optical_flow_height", _optical_flow_height);
    _node.get_parameter("optical_flow_hold_time", _optical_flow_hold_time);
    _node.get_parameter("delta_position", _delta_position);
    _node.get_parameter("hold_duration", _hold_duration);
    _node.get_parameter("descent_vel", _descent_vel);
    _node.get_parameter("landing_height", _landing_height);
  }

  void ExampleAutonomousMode::vehicleLandDetectedCallback(
      const px4_msgs::msg::VehicleLandDetected::SharedPtr msg)
  {
    _land_detected = msg->landed;
    if (msg->landed)
    {
      _ground_z = _vehicle_local_position->positionNed().z();
      _ground_z_valid = true;
    }
  }

  void ExampleAutonomousMode::onActivate()
  {
    // The executor has already completed native takeoff. Hold the reached
    // position rather than adding optical_flow_height again and climbing twice.
    _base_position = _vehicle_local_position->positionNed();
    _hold_position = _base_position;
    // Prefer the ground z observed while PX4 reported landed. If that message
    // was not available before activation, infer it from the configured native
    // takeoff height. NED z increases downward.
    if (!_ground_z_valid)
    {
      _ground_z = _base_position.z() + _optical_flow_height;
      RCLCPP_WARN(_node.get_logger(),
                  "No preflight ground sample — inferred ground z as %.2f",
                  _ground_z);
    }
    _land_detected = false;
    switchToState(State::OpticalFlowSettling);

    RCLCPP_INFO(_node.get_logger(),
                "ExampleAutonomousMode active after takeoff — settling optical "
                "flow for %.1f s, holding %.1f s, then descending",
                _optical_flow_hold_time, _hold_duration);
  }

  void ExampleAutonomousMode::onDeactivate() { switchToState(State::Idle); }

  void ExampleAutonomousMode::updateSetpoint(float dt_s)
  {
    _state_elapsed += dt_s;

    switch (_state)
    {
    case State::Idle:
      break;

    // --- Hold after native takeoff so optical flow can settle ---
    case State::OpticalFlowSettling:
    {
      if (_state_elapsed >= _optical_flow_hold_time)
      {
        RCLCPP_INFO(_node.get_logger(), "Optical-flow settling complete");
        switchToState(State::Holding);
      }

      commandPosition(_hold_position);
      break;
    }

    // --- Hold position for _hold_duration seconds, then descend ---
    case State::Holding:
    {
      if (_state_elapsed >= _hold_duration)
      {
        RCLCPP_INFO(_node.get_logger(), "Hold complete — descending");
        switchToState(State::Descending);
        break;
      }

      commandPosition(_hold_position);
      break;
    }

    // --- Descend at a constant velocity until PX4 detects landing ---
    case State::Descending:
    {
      const Eigen::Vector3f current_position =
          _vehicle_local_position->positionNed();
      // Positive z velocity = downward in NED
      const Eigen::Vector3f velocity(0.f, 0.f, _descent_vel);
      _trajectory_setpoint->update(velocity, std::nullopt, 0.0f);

      // Hand back to the executor shortly above the inferred ground plane. PX4's
      // native land() then owns final touchdown and land detection.
      const float landing_handoff_z = _ground_z - _landing_height;
      if (_land_detected || current_position.z() >= landing_handoff_z)
      {
        switchToState(State::Finished);
      }
      break;
    }

    // --- Controlled descent complete — hold and tell the executor to land ---
    case State::Finished:
    {
      commandPosition(_vehicle_local_position->positionNed());
      break;
    }
    }
  }

  void ExampleAutonomousMode::switchToState(State state)
  {
    if (_state == state)
    {
      return;
    }

    RCLCPP_INFO(_node.get_logger(), "Switching to %s", stateName(state).c_str());

    std_msgs::msg::String state_msg;
    state_msg.data = stateName(state);
    _drone_state_publisher->publish(state_msg);

    _state = state;
    // Every state measures its own dwell time from the moment it is entered, so
    // reset the clock centrally rather than in each transition
    _state_elapsed = 0.0f;

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
  }

  std::string ExampleAutonomousMode::stateName(State state) const
  {
    switch (state)
    {
    case State::Idle:
      return "Idle";
    case State::OpticalFlowSettling:
      return "OpticalFlowSettling";
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
    setSkipMessageCompatibilityCheck();

    // The mode is constructed first and declares these parameters. Reuse the
    // exact configured values for native takeoff and its altitude watcher.
    if (_node.has_parameter("optical_flow_height"))
    {
      _node.get_parameter("optical_flow_height", _optical_flow_height);
    }
    if (_node.has_parameter("delta_position"))
    {
      _node.get_parameter("delta_position", _delta_position);
    }
    _takeoff_target_z = -(_optical_flow_height - _delta_position);

    // PX4's takeoff callback may not fire for a requested height below
    // MIS_TAKEOFF_ALT, so also recognize completion from local NED altitude.
    _local_pos_sub =
        _node.create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            "/fmu/out/vehicle_local_position", rclcpp::QoS(1).best_effort(),
            [this](const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
            {
              _latest_local_z = msg->z;
              _have_local_position = true;
              if (_in_takeoff && !_takeoff_complete &&
                  msg->z <= _takeoff_target_z)
              {
                _takeoff_complete = true;
                _in_takeoff = false;
                RCLCPP_INFO(_node.get_logger(),
                            "Takeoff altitude reached (z=%.2f) — scheduling mode",
                            msg->z);
                runState(State::RunningMode, px4_ros2::Result::Success);
              }
            });
  }

  void ExampleAutonomousModeExecutor::onActivate()
  {
    RCLCPP_INFO(_node.get_logger(),
                "Example executor — arm, take off to %.2f m, run mode, land",
                _optical_flow_height);
    _in_takeoff = false;
    _takeoff_complete = false;
    runState(State::Arming, px4_ros2::Result::Success);
  }

  void ExampleAutonomousModeExecutor::onDeactivate(DeactivateReason reason)
  {
    _in_takeoff = false;
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
    {
      RCLCPP_INFO(_node.get_logger(), "Armed — PX4 native takeoff to %.2f m",
                  _optical_flow_height);
      // Make the watcher relative to the actual preflight local-position z. PX4
      // normally initializes ground at z=0, but the executor should not depend on
      // that assumption.
      const float takeoff_start_z = _have_local_position ? _latest_local_z : 0.0f;
      _takeoff_target_z =
          takeoff_start_z - (_optical_flow_height - _delta_position);
      _in_takeoff = true;
      _takeoff_complete = false;
      takeoff(
          [this](px4_ros2::Result r)
          {
            if (!_takeoff_complete)
            {
              _takeoff_complete = true;
              _in_takeoff = false;
              runState(State::RunningMode, r);
            }
          },
          _optical_flow_height);
      break;
    }

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
      RCLCPP_INFO(_node.get_logger(),
                  "Controlled descent complete — PX4 native landing");
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
