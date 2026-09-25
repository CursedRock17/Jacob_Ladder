#include "jl_mission/mission_mode.hpp"

#include <cmath>
#include <regex>

namespace jl_mission {

namespace {

// PX4 limits external mode names to 24 characters.
constexpr size_t kMaxModeNameLength = 24;

// The mode's name has to be known before ModeBase is constructed, so read the
// parameter here, in the initializer list.
std::string declareMissionName(rclcpp::Node &node) {
  const auto name =
      node.declare_parameter<std::string>("mission_name", "JlMission");
  if (name.empty() || name.size() > kMaxModeNameLength) {
    throw std::invalid_argument("mission_name must be 1-24 characters, got '" +
                                name + "'");
  }
  static const std::regex kNamePattern("^[A-Za-z][A-Za-z0-9_]{0,23}$");
  if (!std::regex_match(name, kNamePattern)) {
    throw std::invalid_argument(
        "mission_name must start with a letter and contain only letters, "
        "digits and underscores (1-24 characters), got '" +
        name + "'");
  }
  return name;
}

} // namespace

// ── RelayMode ──────────────────────────────────────────────────────────────

RelayMode::RelayMode(rclcpp::Node &node)
    : ModeBase(node, Settings{declareMissionName(node), false}), _node(node),
      _name(node.get_parameter("mission_name").as_string()),
      _state(node, "/jl/" + _name + "/mode_state"), _tracking_error(node),
      _global_stamp(node.now()), _last_valid(node.now()) {
  // The executor skips the check too; the translation node converts messages.
  setSkipMessageCompatibilityCheck();

  _limits.silence_hold_s =
      node.declare_parameter<float>("silence_hold_s", _limits.silence_hold_s);
  _limits.silence_land_s =
      node.declare_parameter<float>("silence_land_s", _limits.silence_land_s);
  _limits.max_step_m =
      node.declare_parameter<float>("max_step_m", _limits.max_step_m);
  _limits.max_speed =
      node.declare_parameter<float>("max_speed", _limits.max_speed);

  if (!(_limits.silence_hold_s > 0.f &&
        _limits.silence_hold_s < _limits.silence_land_s)) {
    throw std::invalid_argument(
        "silence_hold_s must be > 0 and < silence_land_s (got "
        "silence_hold_s=" +
        std::to_string(_limits.silence_hold_s) +
        ", silence_land_s=" + std::to_string(_limits.silence_land_s) + ")");
  }
  if (!(_limits.max_step_m > 0.f)) {
    throw std::invalid_argument("max_step_m must be > 0, got " +
                                std::to_string(_limits.max_step_m));
  }
  if (!(_limits.max_speed > 0.f)) {
    throw std::invalid_argument("max_speed must be > 0, got " +
                                std::to_string(_limits.max_speed));
  }

  _local = std::make_shared<px4_ros2::OdometryLocalPosition>(*this);
  _land_detected = std::make_shared<px4_ros2::LandDetected>(*this);
  _setpoint = std::make_shared<px4_ros2::TrajectorySetpointType>(*this);

  _sub = node.create_subscription<px4_msgs::msg::TrajectorySetpoint>(
      "/jl/" + _name + "/setpoint", rclcpp::SensorDataQoS(),
      [this](const px4_msgs::msg::TrajectorySetpoint::SharedPtr msg) {
        onSetpoint(*msg);
      });
  // See the header comment on _global_sub for why this is a raw subscription
  // rather than px4_ros2::OdometryGlobalPosition.
  _global_sub = node.create_subscription<px4_msgs::msg::VehicleGlobalPosition>(
      "/fmu/out/vehicle_global_position", rclcpp::SensorDataQoS(),
      [this](const px4_msgs::msg::VehicleGlobalPosition::SharedPtr msg) {
        onGlobalPosition(*msg);
      });
  _state.set("idle");
}

std::optional<float> RelayMode::altitudeAmsl() const {
  if (!_global || (_node.now() - _global_stamp).seconds() > 1.0) {
    return std::nullopt;
  }
  if (!_global->alt_valid) {
    return std::nullopt;
  }
  return _global->alt;
}

bool RelayMode::landed() const {
  return _land_detected->lastValid() && _land_detected->landed();
}

void RelayMode::onActivate() {
  // A climb target set by the executor must survive activation.
  if (_phase != Phase::Climb) {
    _hold = _local->positionNed();
    _hold_yaw = _local->heading();
    // Selected while already flying (e.g. QGC mid-air, no runner attached
    // yet): run the silence watchdog immediately instead of hovering with no
    // timeout until a runner shows up.
    if (!landed() && _local->positionXYValid() && _local->positionZValid()) {
      _phase = Phase::Relay;
      _last_sent = _local->positionNed();
    }
  }
  _last.reset();
  _last_valid = _node.now();
  _holding = false;
  _land_requested = false;
}

void RelayMode::onDeactivate() {
  _phase = Phase::Idle;
  _land_defer_timer.reset();
}

void RelayMode::beginClimb(const Eigen::Vector3f &target) {
  _phase = Phase::Climb;
  _hold = target;
  _hold_yaw = _local->heading();
  _state.set("climbing");
}

void RelayMode::startRelay() {
  // Coming from Climb, _hold already holds the climb target (set by
  // beginClimb) rather than wherever the vehicle happens to be right now.
  if (_phase != Phase::Climb) {
    _hold = _local->positionNed();
    _hold_yaw = _local->heading();
  }
  _phase = Phase::Relay;
  _last.reset();
  _last_valid = _node.now();
  _holding = false;
  _last_sent = _local->positionNed();
  _state.set("relay");
}

void RelayMode::reset() {
  _phase = Phase::Idle;
  _last.reset();
  _land_requested = false;
  _holding = false;
  _land_defer_timer.reset();
}

void RelayMode::beginLanding() {
  if (_land_requested) {
    return;
  }
  _hold = _local->positionNed();
  _hold_yaw = _local->heading();
  _land_requested = true;
}

void RelayMode::onSetpoint(const px4_msgs::msg::TrajectorySetpoint &msg) {
  auto parsed = fromPx4(msg.position.data(), msg.velocity.data(), msg.yaw);
  if (parsed.ok) {
    parsed = check(parsed.command, _local->positionNed(), _limits);
  }
  if (!parsed.ok) {
    // A rejected setpoint counts as silence: the watchdog keeps running.
    RCLCPP_WARN_THROTTLE(_node.get_logger(), *_node.get_clock(), 1000,
                         "Rejected setpoint: %s", parsed.reason.c_str());
    return;
  }
  _last = parsed.command;
  _last_valid = _node.now();
}

void RelayMode::onGlobalPosition(
    const px4_msgs::msg::VehicleGlobalPosition &msg) {
  _global = msg;
  _global_stamp = _node.now();
}

void RelayMode::updateSetpoint(float dt_s) {
  if (_phase != Phase::Relay) {
    send(Command{_hold, std::nullopt, _hold_yaw});
    return;
  }

  if (_land_requested) {
    // The silence watchdog has already handed over to landNow(); keep
    // holding instead of racing land() with fresh relay setpoints.
    send(Command{_hold, std::nullopt, _hold_yaw});
    // No position was relayed here, so keep the F1 speed limiter's "from"
    // point current: otherwise a later relay would be limited from wherever
    // the vehicle was when this hold started, not from here.
    _last_sent = _local->positionNed();
    return;
  }

  const double silence = (_node.now() - _last_valid).seconds();
  switch (watchdog(silence, _limits)) {
  case Action::Relay:
    if (_last) {
      _holding = false;
      _state.set("relay");
      Command to_send = *_last;
      if (to_send.position) {
        to_send.position =
            limitStep(*to_send.position, _last_sent, _limits.max_speed, dt_s);
        _last_sent = *to_send.position;
      }
      send(to_send);
      if (!to_send.position) {
        // Velocity-only relay: no position was sent, so keep the F1 speed
        // limiter's "from" point current for the next position setpoint.
        _last_sent = _local->positionNed();
      }
    } else {
      send(Command{_hold, std::nullopt, _hold_yaw});
      // No valid setpoint at all yet: same reasoning as above.
      _last_sent = _local->positionNed();
    }
    break;

  case Action::Hold:
    if (!_holding) {
      _holding = true;
      _hold = _local->positionNed();
      _hold_yaw = _local->heading();
      _state.set("hold (no valid setpoint)");
      RCLCPP_WARN(_node.get_logger(), "No valid setpoint for %.1f s: holding",
                  silence);
    }
    send(Command{_hold, std::nullopt, _hold_yaw});
    // No position was relayed here either: keep the F1 speed limiter's
    // "from" point current.
    _last_sent = _local->positionNed();
    break;

  case Action::Land:
    send(Command{_hold, std::nullopt, _hold_yaw});
    if (!_land_requested) {
      _land_requested = true;
      _state.set("landing (no valid setpoint)");
      RCLCPP_ERROR(_node.get_logger(),
                   "No valid setpoint for %.1f s: handing over to land",
                   silence);
      // The watchdog no longer calls completed(): an airborne-activated
      // mode is never a scheduled mode, so completed() would have no
      // effect there. The executor lands directly instead, deferred by one
      // tick so "landing (no valid setpoint)" isn't clobbered on the wire by
      // the executor's own "landing" state right behind it (see the header
      // comment on _land_defer_timer).
      _land_defer_timer =
          _node.create_wall_timer(std::chrono::milliseconds(50), [this] {
            _land_defer_timer->cancel();
            if (_on_silence_land) {
              _on_silence_land();
            }
          });
    }
    break;
  }
}

void RelayMode::send(const Command &command) {
  px4_ros2::TrajectorySetpoint setpoint;
  if (command.position) {
    setpoint.withPosition(*command.position);
  }
  if (command.velocity) {
    setpoint.withVelocity(*command.velocity);
  }
  if (command.yaw) {
    setpoint.withYaw(*command.yaw);
  }
  _setpoint->update(setpoint);
  if (command.position) {
    _tracking_error.publish(*command.position, _local->positionNed());
  }
}

// ── MissionExecutor ────────────────────────────────────────────────────────

MissionExecutor::MissionExecutor(rclcpp::Node &node, RelayMode &mode)
    : ModeExecutorBase(node, Settings{Settings::Activation::ActivateAlways},
                       mode),
      _node(node), _mode(mode) {
  setSkipMessageCompatibilityCheck();

  _climb_tolerance_m =
      node.declare_parameter<float>("climb_tolerance_m", _climb_tolerance_m);
  _takeoff_timeout_s =
      node.declare_parameter<float>("takeoff_timeout_s", _takeoff_timeout_s);

  if (!(_climb_tolerance_m > 0.f)) {
    throw std::invalid_argument("climb_tolerance_m must be > 0, got " +
                                std::to_string(_climb_tolerance_m));
  }
  if (!(_takeoff_timeout_s > 0.f)) {
    throw std::invalid_argument("takeoff_timeout_s must be > 0, got " +
                                std::to_string(_takeoff_timeout_s));
  }

  _mode.onSilenceLand([this] { landNow("no valid setpoint"); });

  const std::string prefix = "/jl/" + mode.missionName();
  _active_pub = node.create_publisher<std_msgs::msg::Bool>(
      prefix + "/active", rclcpp::QoS(1).transient_local());
  _takeoff_srv = node.create_service<TakeoffSrv>(
      prefix + "/takeoff",
      [this](std::shared_ptr<rmw_request_id_t> header,
             std::shared_ptr<TakeoffSrv::Request> request) {
        onTakeoff(header, request);
      });
  _land_srv = node.create_service<LandSrv>(
      prefix + "/land", [this](std::shared_ptr<rmw_request_id_t> header,
                               std::shared_ptr<LandSrv::Request> request) {
        onLand(header, request);
      });

  std_msgs::msg::Bool inactive;
  inactive.data = false;
  _active_pub->publish(inactive);
}

void MissionExecutor::onActivate() {
  std_msgs::msg::Bool active;
  active.data = true;
  _active_pub->publish(active);
  _mode.state().set("waiting for takeoff request");
  RCLCPP_INFO(_node.get_logger(), "%s executor — active, waiting for runner",
              _mode.missionName().c_str());
}

void MissionExecutor::onDeactivate(DeactivateReason reason) {
  std_msgs::msg::Bool inactive;
  inactive.data = false;
  _active_pub->publish(inactive);
  _climb_timer.reset();
  _mode.reset();
  _mode.state().set("inactive");
  const std::string why = reason == DeactivateReason::FailsafeActivated
                              ? "failsafe activated"
                              : "mission deactivated";
  RCLCPP_INFO(_node.get_logger(), "%s executor — deactivated (%s)",
              _mode.missionName().c_str(), why.c_str());
  _takeoff_aborted = true;
  _landing = false;
  finishTakeoff(false, why);
  finishLand(false, why);
}

void MissionExecutor::onScheduledModeResult(px4_ros2::Result result) {
  // Our own cancellation (e.g. a subsequent land()/scheduleMode() call, or
  // the executor losing charge) must not re-enter landNow(): the mode
  // library asserts on a re-entrant land() while one is already in flight.
  if (result == px4_ros2::Result::Deactivated || !isInCharge()) {
    return;
  }
  if (result != px4_ros2::Result::Success) {
    landNow(std::string("relay ended: ") + resultToString(result));
  }
}

void MissionExecutor::onTakeoff(std::shared_ptr<rmw_request_id_t> header,
                                std::shared_ptr<TakeoffSrv::Request> request) {
  auto reply = [this, header](bool success, const std::string &message) {
    TakeoffSrv::Response response;
    response.success = success;
    response.message = message;
    _takeoff_srv->send_response(*header, response);
  };
  if (!isInCharge()) {
    reply(false, "mission is not active; select it in QGC first");
    return;
  }
  if (_landing) {
    reply(false, "landing in progress");
    return;
  }
  if (_takeoff_request) {
    reply(false, "takeoff already in progress");
    return;
  }
  if (!(request->height > 0.f)) {
    reply(false, "height must be a positive number of metres");
    return;
  }
  if (isArmed() && !_mode.landed()) {
    // Already flying (e.g. the pilot switched in mid-air): just take over.
    scheduleMode(ownedMode().id(), [this](px4_ros2::Result result) {
      onScheduledModeResult(result);
    });
    if (isInCharge() && !_landing) {
      _mode.startRelay();
      reply(true, "already airborne; relaying");
    } else {
      reply(false, "could not take over: mission no longer in charge");
    }
    return;
  }

  // Clear any stale Climb phase or relay history left over from a previous,
  // aborted takeoff before starting a new one.
  _mode.reset();
  _takeoff_aborted = false;
  _takeoff_request = header;
  _takeoff_started = _node.now();
  const Eigen::Vector3f start = _mode.position();
  _climb_target = start - Eigen::Vector3f{0.f, 0.f, request->height};

  arm([this, height = request->height](px4_ros2::Result result) {
    if (_takeoff_aborted) {
      return;
    }
    if (result != px4_ros2::Result::Success) {
      finishTakeoff(false,
                    std::string("arm failed: ") + resultToString(result));
      return;
    }
    // PX4's takeoff altitude is AMSL. Without a global position, NaN makes
    // PX4 use MIS_TAKEOFF_ALT; the climb below fixes the height either way.
    const auto amsl = _mode.altitudeAmsl();
    const float target_amsl = amsl ? *amsl + height : NAN;
    if (!amsl) {
      RCLCPP_WARN(_node.get_logger(),
                  "No global position: PX4 takes off to MIS_TAKEOFF_ALT, then "
                  "the mission climbs to %.2f m",
                  height);
    }
    takeoff(
        [this](px4_ros2::Result takeoff_result) {
          if (_takeoff_aborted) {
            return;
          }
          if (takeoff_result != px4_ros2::Result::Success) {
            finishTakeoff(false, std::string("takeoff failed: ") +
                                     resultToString(takeoff_result));
            return;
          }
          // PX4 calls takeoff complete within NAV_MC_ALT_RAD of the target,
          // so climb the rest of the way in our own mode before handing over.
          _mode.beginClimb(_climb_target);
          scheduleMode(ownedMode().id(), [this](px4_ros2::Result relay_result) {
            onScheduledModeResult(relay_result);
          });
          if (isInCharge() && !_landing) {
            _climb_settled_since.reset();
            _climb_timer = _node.create_wall_timer(
                std::chrono::milliseconds(100), [this] { checkClimb(); });
          } else {
            finishTakeoff(false, "could not start the climb");
          }
        },
        target_amsl);
  });
}

void MissionExecutor::checkClimb() {
  if (!_takeoff_request) {
    _climb_timer.reset();
    _climb_settled_since.reset();
    return;
  }
  const float error = std::abs(_mode.position().z() - _climb_target.z());
  const float vz = std::abs(_mode.velocity().z());
  if (error <= _climb_tolerance_m && vz < 0.1f) {
    if (!_climb_settled_since) {
      _climb_settled_since = _node.now();
    } else if ((_node.now() - *_climb_settled_since).seconds() >= 1.0) {
      _climb_timer.reset();
      _climb_settled_since.reset();
      _mode.startRelay();
      finishTakeoff(true, "at takeoff height; relaying");
      return;
    }
  } else {
    _climb_settled_since.reset();
  }
  if ((_node.now() - _takeoff_started).seconds() > _takeoff_timeout_s) {
    _climb_timer.reset();
    _climb_settled_since.reset();
    finishTakeoff(false, "did not reach takeoff height in time");
    landNow("takeoff timed out");
  }
}

void MissionExecutor::onLand(std::shared_ptr<rmw_request_id_t> header,
                             std::shared_ptr<LandSrv::Request> /*request*/) {
  if (!isInCharge()) {
    LandSrv::Response response;
    response.success = false;
    response.message = "mission is not active";
    _land_srv->send_response(*header, response);
    return;
  }
  _land_requests.push_back(header);
  if (_takeoff_request) {
    finishTakeoff(false, "cancelled by land");
    _climb_timer.reset();
    _takeoff_aborted = true;
  }
  if (!isArmed()) {
    // Never left the ground (or already back down): nothing to land.
    finishLand(true, "not flying");
    return;
  }
  landNow("requested by the mission");
}

void MissionExecutor::landNow(const std::string &why) {
  if (!isInCharge()) {
    // The pilot has taken over: never fight them for control.
    return;
  }
  if (_landing) {
    // A landing is already underway (e.g. the silence watchdog beat this
    // caller to it); the pending request(s) are answered when it finishes.
    // Never call land() twice: the second call would re-enter mid-land.
    return;
  }
  _landing = true;
  RCLCPP_INFO(_node.get_logger(), "Landing: %s", why.c_str());
  // Stop relaying for good before handing off to PX4's land(): otherwise the
  // relay branch in updateSetpoint keeps publishing fresh "relay" setpoints
  // (and state) right up until land() actually takes effect.
  _mode.beginLanding();
  _mode.state().set("landing");
  land([this](px4_ros2::Result result) {
    _landing = false;
    if (result == px4_ros2::Result::Deactivated) {
      // Our own cancellation (e.g. the executor deactivated mid-land):
      // onDeactivate() already answers pending requests.
      return;
    }
    const bool ok = result == px4_ros2::Result::Success;
    _mode.state().set(ok ? "landed" : "land failed");
    finishLand(ok, ok ? "landed"
                      : std::string("land failed: ") + resultToString(result));
  });
}

void MissionExecutor::finishTakeoff(bool success, const std::string &message) {
  if (!_takeoff_request) {
    return;
  }
  TakeoffSrv::Response response;
  response.success = success;
  response.message = message;
  _takeoff_srv->send_response(*_takeoff_request, response);
  _takeoff_request.reset();
  RCLCPP_INFO(_node.get_logger(), "Takeoff %s: %s", success ? "done" : "failed",
              message.c_str());
}

void MissionExecutor::finishLand(bool success, const std::string &message) {
  for (const auto &header : _land_requests) {
    LandSrv::Response response;
    response.success = success;
    response.message = message;
    _land_srv->send_response(*header, response);
  }
  _land_requests.clear();
}

} // namespace jl_mission
