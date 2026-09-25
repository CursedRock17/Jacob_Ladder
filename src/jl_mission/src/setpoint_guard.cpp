#include "jl_mission/setpoint_guard.hpp"

#include <cmath>
#include <cstdio>

namespace jl_mission {

namespace {

// Read one PX4-style vector: all-NaN means "not controlled", all-finite is a
// value, anything else (mixed NaN, or an infinity) is an error.
bool readVector(const float v[3], const char *name,
                std::optional<Eigen::Vector3f> &out, std::string &reason) {
  int nan_count = 0;
  for (int i = 0; i < 3; ++i) {
    if (std::isinf(v[i])) {
      reason = std::string(name) + " contains infinity";
      return false;
    }
    if (std::isnan(v[i])) {
      ++nan_count;
    }
  }
  if (nan_count == 3) {
    out.reset();
    return true;
  }
  if (nan_count > 0) {
    reason = std::string(name) + " has some axes set and some NaN";
    return false;
  }
  out = Eigen::Vector3f{v[0], v[1], v[2]};
  return true;
}

} // namespace

Checked fromPx4(const float position[3], const float velocity[3], float yaw) {
  Checked result;
  if (!readVector(position, "position", result.command.position,
                  result.reason) ||
      !readVector(velocity, "velocity", result.command.velocity,
                  result.reason)) {
    return result;
  }
  if (std::isinf(yaw)) {
    result.reason = "yaw is infinite";
    return result;
  }
  if (!std::isnan(yaw)) {
    result.command.yaw = yaw;
  }
  result.ok = true;
  return result;
}

Checked check(const Command &command, const Eigen::Vector3f &current,
              const GuardLimits &limits) {
  Checked result;
  result.command = command;
  if (!command.position && !command.velocity) {
    result.reason = "controls neither position nor velocity";
    return result;
  }
  if ((command.position && !command.position->allFinite()) ||
      (command.velocity && !command.velocity->allFinite())) {
    result.reason = "position or velocity is not finite";
    return result;
  }
  if (command.yaw && !std::isfinite(*command.yaw)) {
    result.reason = "yaw is not finite";
    return result;
  }
  if (command.position) {
    const float distance = (*command.position - current).norm();
    if (distance > limits.max_step_m) {
      char text[96];
      std::snprintf(text, sizeof(text),
                    "position is %.1f m away (limit %.1f m)", distance,
                    limits.max_step_m);
      result.reason = text;
      return result;
    }
  }
  if (command.velocity) {
    const float speed = command.velocity->norm();
    if (speed > limits.max_speed) {
      result.command.velocity = *command.velocity * (limits.max_speed / speed);
    }
  }
  result.ok = true;
  return result;
}

Eigen::Vector3f limitStep(const Eigen::Vector3f &requested,
                          const Eigen::Vector3f &last_sent, float max_speed,
                          float dt_s) {
  if (dt_s <= 0.f) {
    return last_sent;
  }
  const Eigen::Vector3f delta = requested - last_sent;
  const float distance = delta.norm();
  const float max_step = max_speed * dt_s;
  if (distance <= max_step) {
    return requested;
  }
  return last_sent + delta * (max_step / distance);
}

Action watchdog(double seconds_since_valid, const GuardLimits &limits) {
  if (seconds_since_valid >= limits.silence_land_s) {
    return Action::Land;
  }
  if (seconds_since_valid >= limits.silence_hold_s) {
    return Action::Hold;
  }
  return Action::Relay;
}

} // namespace jl_mission
