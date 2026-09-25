#pragma once

// jl_mission: the one C++ external mode behind every mission file.
//
// RelayMode passes the Python mission_runner's setpoints to PX4, but only after
// the SetpointGuard has checked them. MissionExecutor owns arm, takeoff and
// land, which the runner requests over two services. See spec section 5.

#include "jl_mission/debug_publishers.hpp"
#include "jl_mission/setpoint_guard.hpp"

#include <jl_mission_interfaces/srv/takeoff.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/components/mode_executor.hpp>
#include <px4_ros2/control/setpoint_types/experimental/trajectory.hpp>
#include <px4_ros2/odometry/local_position.hpp>
#include <px4_ros2/vehicle_state/land_detected.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace jl_mission {

class RelayMode : public px4_ros2::ModeBase {
public:
  explicit RelayMode(rclcpp::Node &node);

  void onActivate() override;
  void onDeactivate() override;
  void updateSetpoint(float dt_s) override;

  // Called by the executor: hold `target` (no watchdog) until startRelay().
  void beginClimb(const Eigen::Vector3f &target);
  // Start relaying the runner's setpoints; the silence watchdog starts now.
  void startRelay();
  // Back to Idle: no watchdog, hold in place. Clears any stale Climb target
  // or relay history left over from an aborted takeoff or a deactivation.
  void reset();
  // Called by the executor before land(): freezes the hold position at
  // wherever the vehicle is right now and stops relaying for good.
  // Idempotent, so a second call (e.g. the silence watchdog racing an
  // explicit land request) is a no-op.
  void beginLanding();
  // Called exactly once when the silence watchdog decides to land.
  void onSilenceLand(std::function<void()> cb) {
    _on_silence_land = std::move(cb);
  }

  const std::string &missionName() const { return _name; }
  StatePublisher &state() { return _state; }
  Eigen::Vector3f position() const { return _local->positionNed(); }
  Eigen::Vector3f velocity() const { return _local->velocityNed(); }
  // Current altitude above mean sea level, if a recent, valid global-position
  // message has arrived; std::nullopt otherwise (e.g. GPS-denied).
  std::optional<float> altitudeAmsl() const;
  bool landed() const;

private:
  // Idle: hold where we are (no watchdog) until the executor picks a phase.
  enum class Phase { Idle, Climb, Relay };

  void onSetpoint(const px4_msgs::msg::TrajectorySetpoint &msg);
  void onGlobalPosition(const px4_msgs::msg::VehicleGlobalPosition &msg);
  void send(const Command &command);

  rclcpp::Node &_node;
  std::string _name;
  GuardLimits _limits;
  StatePublisher _state;
  TrackingErrorPublisher _tracking_error;
  std::shared_ptr<px4_ros2::OdometryLocalPosition> _local;
  std::shared_ptr<px4_ros2::LandDetected> _land_detected;
  std::shared_ptr<px4_ros2::TrajectorySetpointType> _setpoint;
  rclcpp::Subscription<px4_msgs::msg::TrajectorySetpoint>::SharedPtr _sub;

  // A plain subscription rather than px4_ros2::OdometryGlobalPosition: that
  // class registers global position as a *mode requirement*, which would
  // block this mode from ever activating on the GPS-denied real drone. We
  // only want best-effort AMSL for the takeoff-altitude conversion, and the
  // executor already falls back to NaN (native MIS_TAKEOFF_ALT) when it's
  // unavailable.
  rclcpp::Subscription<px4_msgs::msg::VehicleGlobalPosition>::SharedPtr
      _global_sub;
  std::optional<px4_msgs::msg::VehicleGlobalPosition> _global;
  rclcpp::Time _global_stamp;

  std::function<void()> _on_silence_land;
  // Fires _on_silence_land shortly after publishing "landing (no valid
  // setpoint)": both that state and the executor's subsequent "landing"
  // share one depth-1 latched topic, so calling the callback in the same
  // tick can silently drop the first message before an observer ever reads
  // it. A short deferral gives it a chance to be delivered first.
  rclcpp::TimerBase::SharedPtr _land_defer_timer;

  Phase _phase{Phase::Idle};
  Eigen::Vector3f _hold{Eigen::Vector3f::Zero()};
  std::optional<float> _hold_yaw;
  // Last position setpoint actually sent to PX4, for the F1 speed limiter.
  Eigen::Vector3f _last_sent{Eigen::Vector3f::Zero()};
  std::optional<Command> _last;
  rclcpp::Time _last_valid;
  bool _holding{false};
  bool _land_requested{false};
};

class MissionExecutor : public px4_ros2::ModeExecutorBase {
public:
  MissionExecutor(rclcpp::Node &node, RelayMode &mode);

  void onActivate() override;
  void onDeactivate(DeactivateReason reason) override;

private:
  using TakeoffSrv = jl_mission_interfaces::srv::Takeoff;
  using LandSrv = std_srvs::srv::Trigger;

  void onTakeoff(std::shared_ptr<rmw_request_id_t> header,
                 std::shared_ptr<TakeoffSrv::Request> request);
  void onLand(std::shared_ptr<rmw_request_id_t> header,
              std::shared_ptr<LandSrv::Request> request);
  void finishTakeoff(bool success, const std::string &message);
  void finishLand(bool success, const std::string &message);
  void checkClimb();
  // Idempotent: safe to call while a landing is already underway.
  void landNow(const std::string &why);
  void onScheduledModeResult(px4_ros2::Result result);

  rclcpp::Node &_node;
  RelayMode &_mode;
  float _climb_tolerance_m{0.1f};
  float _takeoff_timeout_s{30.f};

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr _active_pub;
  rclcpp::Service<TakeoffSrv>::SharedPtr _takeoff_srv;
  rclcpp::Service<LandSrv>::SharedPtr _land_srv;
  rclcpp::TimerBase::SharedPtr _climb_timer;

  std::shared_ptr<rmw_request_id_t> _takeoff_request;
  std::vector<std::shared_ptr<rmw_request_id_t>> _land_requests;
  Eigen::Vector3f _climb_target{Eigen::Vector3f::Zero()};
  rclcpp::Time _takeoff_started;
  // When the climb first satisfied the height + vz settle condition; reset
  // whenever either condition fails. Unset while unsettled.
  std::optional<rclcpp::Time> _climb_settled_since;
  bool _landing{false};
  // Set by a Land() that pre-empts a pending takeoff; the arm/takeoff
  // callbacks check it and bail out instead of continuing the sequence.
  bool _takeoff_aborted{false};
};

} // namespace jl_mission
