#pragma once

// The jl_mission safety contract (spec section 5) as plain functions, so it can
// be unit-tested without ROS or PX4. The relay mode calls these every tick.

#include <Eigen/Core>

#include <optional>
#include <string>

namespace jl_mission {

struct GuardLimits {
  float silence_hold_s{0.5f}; // no valid setpoint this long -> hold position
  float silence_land_s{5.0f}; // still nothing this long -> land
  float max_step_m{5.0f};     // reject positions farther than this from here
  float max_speed{1.0f};      // m/s, velocity setpoints are clamped to this
};

// One setpoint from the runner. An empty field means "not controlled".
struct Command {
  std::optional<Eigen::Vector3f> position; // NED, metres
  std::optional<Eigen::Vector3f> velocity; // NED, m/s
  std::optional<float> yaw;                // radians
};

// The result of checking a Command: ok with a (possibly clamped) command, or
// a reason it was rejected.
struct Checked {
  bool ok{false};
  Command command;
  std::string reason;
};

// Build a Command from PX4-style arrays where NaN means "not controlled".
// A vector with some NaN and some finite axes, or any infinity, is rejected.
Checked fromPx4(const float position[3], const float velocity[3], float yaw);

// Apply the limits: reject a command that controls neither position nor
// velocity or has a non-finite value, reject a position farther than
// max_step_m from `current`, and clamp the velocity to max_speed.
Checked check(const Command &command, const Eigen::Vector3f &current,
              const GuardLimits &limits);

enum class Action { Relay, Hold, Land };

// What to do given how long it has been since the last valid setpoint.
Action watchdog(double seconds_since_valid, const GuardLimits &limits);

// Speed-limit a position setpoint: returns `requested` if it is within
// `max_speed * dt_s` of `last_sent`, otherwise the point that far along the
// line from `last_sent` toward `requested`. dt_s <= 0 returns `last_sent`.
Eigen::Vector3f limitStep(const Eigen::Vector3f &requested,
                          const Eigen::Vector3f &last_sent, float max_speed,
                          float dt_s);

} // namespace jl_mission
