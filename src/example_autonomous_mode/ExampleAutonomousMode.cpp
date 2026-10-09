// The mode holds position and optionally descends to a handoff height.
// The executor handles arming, native takeoff, native landing, and the disarm
// wait. See README.md for the state diagrams and setup instructions.

#include "ExampleAutonomousMode.hpp"

#include <px4_ros2/components/node_with_mode.hpp>

#include <memory>

namespace example_autonomous_mode {

ExampleAutonomousMode::ExampleAutonomousMode(rclcpp::Node &node)
    : ModeBase(node, ModeBase::Settings{kExampleAutonomousModeName, false}),
      _node(node) {
  // This checkout uses PX4 message translation, so both the mode and executor
  // must skip the upstream message-version check during registration.
  setSkipMessageCompatibilityCheck();

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

  // Optional debug topics. They can be removed along with their publish calls.
  _drone_state_pub = _node.create_publisher<std_msgs::msg::String>(
      "/drone_state", rclcpp::QoS(10));
  _tracking_error_pub =
      _node.create_publisher<geometry_msgs::msg::Vector3Stamped>(
          "/tracking_error", rclcpp::QoS(10));

  loadParameters();
}

// The launch file loads the parameter YAML; member values are the defaults.
void ExampleAutonomousMode::loadParameters() {
  _node.declare_parameter<float>("hold_duration", _hold_duration);
  _node.declare_parameter<float>("descent_vel", _descent_vel);
  _node.declare_parameter<float>("landing_height", _landing_height);

  _node.get_parameter("hold_duration", _hold_duration);
  _node.get_parameter("descent_vel", _descent_vel);
  _node.get_parameter("landing_height", _landing_height);
}

// Save ground z while landed so descent can stop at the handoff height.
// Without this sample, the executor uses native landing for the whole descent.
void ExampleAutonomousMode::vehicleLandDetectedCallback(
    const px4_msgs::msg::VehicleLandDetected::SharedPtr msg) {
  _land_detected = msg->landed;
  if (msg->landed && _vehicle_local_position->positionZValid()) {
    _ground_z = _vehicle_local_position->positionNed().z();
    _ground_z_valid = true;
  }
}

void ExampleAutonomousMode::onActivate() {
  // Called when PX4 selects the mode, after the executor's native takeoff.
  _hold_position = _vehicle_local_position->positionNed();
  if (!_ground_z_valid) {
    RCLCPP_WARN(_node.get_logger(),
                "No preflight ground sample; skipping controlled descent");
  }
  _land_detected = false;
  switchToState(State::Holding);

  RCLCPP_INFO(_node.get_logger(),
              "ExampleAutonomousMode active after takeoff — holding %.1f s",
              _hold_duration);
}

void ExampleAutonomousMode::onDeactivate() {
  switchToState(State::Idle);
  _ground_z_valid = false;
}

void ExampleAutonomousMode::updateSetpoint(float dt_s) {
  _state_elapsed += dt_s;

  switch (_state) {
  case State::Idle:
    break;

  // Hold, then descend if the ground was observed; otherwise use native land.
  case State::Holding: {
    if (_state_elapsed >= _hold_duration) {
      // Send this tick's setpoint before Finished hands control to the executor.
      // The break below skips the commandPosition() at the end of this case.
      commandPosition(_hold_position);
      if (_ground_z_valid) {
        RCLCPP_INFO(_node.get_logger(), "Hold complete — descending");
        switchToState(State::Descending);
      } else {
        switchToState(State::Finished);
      }
      break;
    }

    commandPosition(_hold_position);
    break;
  }

  // Descend to the handoff height, or stop if PX4 already reports landed.
  case State::Descending: {
    const Eigen::Vector3f current_position =
        _vehicle_local_position->positionNed();
    // Positive z velocity = downward in NED
    const Eigen::Vector3f velocity(0.f, 0.f, _descent_vel);
    // update() sends velocity; commandPosition() uses updatePosition().
    _trajectory_setpoint->update(velocity, std::nullopt, 0.0f);

    // Let native land() handle touchdown once we reach the handoff height.
    const float landing_handoff_z = _ground_z - _landing_height;
    if (_land_detected || current_position.z() >= landing_handoff_z) {
      switchToState(State::Finished);
    }
    break;
  }

  // Keep a setpoint available until the executor switches to native land.
  case State::Finished: {
    commandPosition(_vehicle_local_position->positionNed());
    break;
  }
  }
}

void ExampleAutonomousMode::switchToState(State state) {
  if (_state == state) {
    return;
  }

  RCLCPP_INFO(_node.get_logger(), "Switching to %s", stateName(state).c_str());

  std_msgs::msg::String state_msg;
  state_msg.data = stateName(state);

  _state = state;
  _state_elapsed = 0.0f;
  _drone_state_pub->publish(state_msg);

  // Report the result to PX4 exactly once, on entry, rather than every tick
  if (state == State::Finished) {
    ModeBase::completed(px4_ros2::Result::Success);
  }
}

void ExampleAutonomousMode::commandPosition(const Eigen::Vector3f &pos) {
  _trajectory_setpoint->updatePosition(pos);

  // Commanded minus actual position, in local NED coordinates.
  const auto actual = _vehicle_local_position->positionNed();
  geometry_msgs::msg::Vector3Stamped odom_error;
  odom_error.header.stamp = _node.now();
  odom_error.header.frame_id = "odom";
  odom_error.vector.x = pos.x() - actual.x();
  odom_error.vector.y = pos.y() - actual.y();
  odom_error.vector.z = pos.z() - actual.z();
  _tracking_error_pub->publish(odom_error);
}

std::string ExampleAutonomousMode::stateName(State state) const {
  switch (state) {
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

// ── Executor ──
// PX4 result callbacks advance the sequence: arm -> takeoff -> mode -> land.
// The mode above sends setpoints; the executor requests PX4 operations.

ExampleAutonomousModeExecutor::ExampleAutonomousModeExecutor(
    rclcpp::Node &node, px4_ros2::ModeBase &owned_mode)
    : ModeExecutorBase(
          node,
          ModeExecutorBase::Settings{Settings::Activation::ActivateAlways},
          owned_mode),
      _node(node) {
  // The executor registers separately and needs the same version override.
  setSkipMessageCompatibilityCheck();
}

// Selecting the owned mode activates the executor. ActivateAlways allows
// this while disarmed so the sequence can start with arm().
void ExampleAutonomousModeExecutor::onActivate() {
  RCLCPP_INFO(_node.get_logger(),
              "Example executor — arm, use PX4 native takeoff, run mode, land");
  runState(State::Arming, px4_ros2::Result::Success);
}

void ExampleAutonomousModeExecutor::onDeactivate(DeactivateReason reason) {
  RCLCPP_INFO(_node.get_logger(), "Example executor deactivated (%i)",
              static_cast<int>(reason));
}

void ExampleAutonomousModeExecutor::runState(State state,
                                             px4_ros2::Result result) {
  if (result != px4_ros2::Result::Success) {
    RCLCPP_ERROR(_node.get_logger(), "Executor state %i failed: %s",
                 static_cast<int>(state), resultToString(result));
    return;
  }

  switch (state) {
  case State::Arming:
    RCLCPP_INFO(_node.get_logger(), "Arming");
    arm([this](px4_ros2::Result r) { runState(State::TakingOff, r); });
    break;

  case State::TakingOff:
    RCLCPP_INFO(_node.get_logger(), "Armed — PX4 native takeoff");
    takeoff([this](px4_ros2::Result r) { runState(State::RunningMode, r); });
    break;

  case State::RunningMode:
    RCLCPP_INFO(_node.get_logger(),
                "Takeoff complete — scheduling ExampleAutonomousMode");
    scheduleMode(ownedMode().id(), [this](px4_ros2::Result r) {
      RCLCPP_INFO(_node.get_logger(), "ExampleAutonomousMode ended (%s)",
                  resultToString(r));
      // On takeover or failsafe, leave control with the pilot or PX4.
      if (r == px4_ros2::Result::Success) {
        runState(State::Landing, r);
      }
    });
    break;

  case State::Landing:
    RCLCPP_INFO(_node.get_logger(), "Requesting PX4 native landing");
    land([this](px4_ros2::Result r) { runState(State::WaitingForDisarm, r); });
    break;

  case State::WaitingForDisarm:
    RCLCPP_INFO(_node.get_logger(), "Landed — waiting for disarm");
    waitUntilDisarmed([this](px4_ros2::Result r) {
      if (r == px4_ros2::Result::Success) {
        RCLCPP_INFO(_node.get_logger(),
                    "Disarmed — example autonomous workflow complete");
      } else {
        RCLCPP_ERROR(_node.get_logger(), "Disarm wait failed: %s",
                     resultToString(r));
      }
    });
    break;
  }
}

} // namespace example_autonomous_mode

// Register the mode and its executor in one ROS node.
int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<px4_ros2::NodeWithModeExecutor<
                   example_autonomous_mode::ExampleAutonomousModeExecutor,
                   example_autonomous_mode::ExampleAutonomousMode>>(
      example_autonomous_mode::kExampleAutonomousModeName,
      example_autonomous_mode::kExampleAutonomousModeDebugOutput));
  rclcpp::shutdown();
  return 0;
}
