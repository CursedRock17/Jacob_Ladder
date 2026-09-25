#include "jl_mission/setpoint_guard.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

using jl_mission::Action;
using jl_mission::Command;
using jl_mission::GuardLimits;

namespace {
const float kNan = std::numeric_limits<float>::quiet_NaN();
const float kInf = std::numeric_limits<float>::infinity();
const Eigen::Vector3f kHere{0.f, 0.f, -1.f};
} // namespace

TEST(FromPx4, PositionOnlyWithNanVelocityIsAccepted) {
  const float pos[3] = {1.f, 2.f, -1.5f};
  const float vel[3] = {kNan, kNan, kNan};
  const auto result = jl_mission::fromPx4(pos, vel, kNan);
  ASSERT_TRUE(result.ok);
  EXPECT_TRUE(result.command.position.has_value());
  EXPECT_FALSE(result.command.velocity.has_value());
  EXPECT_FALSE(result.command.yaw.has_value());
}

TEST(FromPx4, PartlyNanVectorIsRejected) {
  const float pos[3] = {1.f, kNan, -1.5f};
  const float vel[3] = {kNan, kNan, kNan};
  const auto result = jl_mission::fromPx4(pos, vel, kNan);
  EXPECT_FALSE(result.ok);
  EXPECT_EQ(result.reason, "position has some axes set and some NaN");
}

TEST(FromPx4, InfinityAnywhereIsRejected) {
  const float pos[3] = {1.f, 2.f, -1.5f};
  const float vel[3] = {kNan, kNan, kNan};
  EXPECT_FALSE(jl_mission::fromPx4(pos, vel, kInf).ok);
  const float bad_pos[3] = {kInf, 0.f, -1.f};
  EXPECT_FALSE(jl_mission::fromPx4(bad_pos, vel, kNan).ok);
}

TEST(Check, CommandWithNeitherPositionNorVelocityIsRejected) {
  Command command;
  command.yaw = 0.f;
  const auto result = jl_mission::check(command, kHere, GuardLimits{});
  EXPECT_FALSE(result.ok);
  EXPECT_EQ(result.reason, "controls neither position nor velocity");
}

TEST(Check, NonFiniteValuesAreRejected) {
  Command nan_position;
  nan_position.position = Eigen::Vector3f{kNan, 0.f, -1.f};
  EXPECT_FALSE(jl_mission::check(nan_position, kHere, GuardLimits{}).ok);
  Command inf_velocity;
  inf_velocity.velocity = Eigen::Vector3f{kInf, 0.f, 0.f};
  EXPECT_FALSE(jl_mission::check(inf_velocity, kHere, GuardLimits{}).ok);
  Command nan_yaw;
  nan_yaw.position = Eigen::Vector3f{0.f, 0.f, -1.f};
  nan_yaw.yaw = kNan;
  const auto result = jl_mission::check(nan_yaw, kHere, GuardLimits{});
  EXPECT_FALSE(result.ok);
  EXPECT_EQ(result.reason, "yaw is not finite");
}

TEST(Check, PositionTooFarFromTheVehicleIsRejected) {
  Command command;
  command.position = Eigen::Vector3f{6.f, 0.f, -1.f};
  const auto result = jl_mission::check(command, kHere, GuardLimits{});
  EXPECT_FALSE(result.ok);
  EXPECT_EQ(result.reason, "position is 6.0 m away (limit 5.0 m)");
}

TEST(Check, PositionWithinMaxStepIsAccepted) {
  Command command;
  command.position = Eigen::Vector3f{4.f, 0.f, -1.f};
  EXPECT_TRUE(jl_mission::check(command, kHere, GuardLimits{}).ok);
}

TEST(Check, FastVelocityIsClampedToMaxSpeedKeepingDirection) {
  Command command;
  command.velocity = Eigen::Vector3f{3.f, 4.f, 0.f}; // 5 m/s
  const auto result = jl_mission::check(command, kHere, GuardLimits{});
  ASSERT_TRUE(result.ok);
  EXPECT_NEAR(result.command.velocity->norm(), 1.f, 1e-5f);
  EXPECT_NEAR(result.command.velocity->x(), 0.6f, 1e-5f);
  EXPECT_NEAR(result.command.velocity->y(), 0.8f, 1e-5f);
}

TEST(Check, SlowVelocityIsUnchanged) {
  Command command;
  command.velocity = Eigen::Vector3f{0.3f, 0.f, 0.f};
  const auto result = jl_mission::check(command, kHere, GuardLimits{});
  ASSERT_TRUE(result.ok);
  EXPECT_FLOAT_EQ(result.command.velocity->x(), 0.3f);
}

TEST(Watchdog, RelaysUntilSilenceHoldThenHoldsThenLands) {
  const GuardLimits limits{};
  EXPECT_EQ(jl_mission::watchdog(0.0, limits), Action::Relay);
  EXPECT_EQ(jl_mission::watchdog(0.49, limits), Action::Relay);
  EXPECT_EQ(jl_mission::watchdog(0.5, limits), Action::Hold);
  EXPECT_EQ(jl_mission::watchdog(4.99, limits), Action::Hold);
  EXPECT_EQ(jl_mission::watchdog(5.0, limits), Action::Land);
}

TEST(Check, VelocityOnlyCommandPasses) {
  Command command;
  command.velocity = Eigen::Vector3f{0.2f, 0.f, 0.f};
  EXPECT_TRUE(jl_mission::check(command, kHere, GuardLimits{}).ok);
}

TEST(Check, PositionAndVelocityCommandPasses) {
  Command command;
  command.position = Eigen::Vector3f{1.f, 0.f, -1.f};
  command.velocity = Eigen::Vector3f{0.2f, 0.f, 0.f};
  EXPECT_TRUE(jl_mission::check(command, kHere, GuardLimits{}).ok);
}

TEST(Check, PositionExactlyMaxStepAwayPasses) {
  Command command;
  command.position = Eigen::Vector3f{5.f, 0.f, -1.f};
  EXPECT_TRUE(jl_mission::check(command, kHere, GuardLimits{}).ok);
}

TEST(Check, CustomGuardLimitsClampVelocity) {
  GuardLimits limits;
  limits.max_speed = 0.5f;
  Command command;
  command.velocity = Eigen::Vector3f{1.f, 0.f, 0.f};
  const auto result = jl_mission::check(command, kHere, limits);
  ASSERT_TRUE(result.ok);
  EXPECT_NEAR(result.command.velocity->norm(), 0.5f, 1e-5f);
}

TEST(LimitStep, SmallMoveIsUnchanged) {
  const Eigen::Vector3f last_sent{0.f, 0.f, -1.f};
  const Eigen::Vector3f requested{0.f, 0.01f, -1.f};
  const auto result = jl_mission::limitStep(requested, last_sent, 1.0f, 0.02f);
  EXPECT_NEAR((result - requested).norm(), 0.f, 1e-6f);
}

TEST(LimitStep, LargeJumpMovesOneStepTowardTheTarget) {
  const Eigen::Vector3f last_sent{0.f, 0.f, -1.f};
  const Eigen::Vector3f requested{4.f, 0.f, -1.f};
  const auto result = jl_mission::limitStep(requested, last_sent, 1.0f, 0.02f);
  const Eigen::Vector3f expected{0.02f, 0.f, -1.f};
  EXPECT_NEAR((result - expected).norm(), 0.f, 1e-6f);
}

TEST(LimitStep, ZeroDtReturnsLastSent) {
  const Eigen::Vector3f last_sent{0.f, 0.f, -1.f};
  const Eigen::Vector3f requested{4.f, 0.f, -1.f};
  const auto result = jl_mission::limitStep(requested, last_sent, 1.0f, 0.f);
  EXPECT_NEAR((result - last_sent).norm(), 0.f, 1e-6f);
}
