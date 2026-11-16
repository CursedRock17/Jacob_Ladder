// jl_mission entry point. One process per mission: the mission's name
// (as shown in QGC) comes from the mission_name parameter.
//
// Registration is retried the same way TakeoffHold does it: wait for the FMU,
// then keep retrying, because at boot the DDS link can come up after us.

#include "jl_mission/mission_mode.hpp"

#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_ros2/components/node_with_mode.hpp>
#include <px4_ros2/components/wait_for_fmu.hpp>
#include <rclcpp/exceptions/exceptions.hpp>

#include <chrono>
#include <stdexcept>
#include <string>

int main(int argc, char *argv[]) {
  using namespace std::chrono_literals;

  rclcpp::init(argc, argv);

  // waitForFMU needs a node that is not the mode node: constructing the mode
  // node is what triggers registration.
  {
    auto startup_node = std::make_shared<rclcpp::Node>("jl_mission_startup");
    if (!px4_ros2::waitForFMU(*startup_node, 60s)) {
      RCLCPP_WARN(startup_node->get_logger(),
                  "No FMU heartbeat after 60 s: is the DDS agent running? "
                  "Retrying registration anyway.");
    }
  }

  // A lost reply leaves PX4 holding a ghost registration of this name, which
  // pushes us into another slot and then times us out ("no request received
  // from FMU"). PX4 drops the ghost once it stops answering, so registering
  // again recovers. Up to kMaxAttempts in a row; a registration that held for
  // kHealthyFor counts as a success and resets the count.
  const auto retry_delay = 2s;
  constexpr int kMaxAttempts = 5;
  constexpr auto kHealthyFor = 60s;
  int failures = 0;
  int exit_code = 0;

  // PX4 remembers which slot each mode name had (COM_MODEn_HASH) and gives it
  // back on the next registration -- unless that name is still registered.
  // Then the newcomer lands in the first free slot (typically ext1) and the
  // stale registration lingers until PX4 finds it unresponsive (3 missed
  // arming-check rounds, ~1 s, and only while disarmed and not selected).
  // With mode_slot set, a registration in any other slot is dropped and
  // retried after kSlotRetryDelay, by which time PX4 has removed the stale
  // one and hands back the remembered slot. After kMaxSlotRetries we keep
  // whatever slot we got rather than leave the mission unflyable.
  constexpr auto kSlotRetryDelay = 3s;
  constexpr int kMaxSlotRetries = 3;
  int slot_retries = 0;
  while (rclcpp::ok()) {
    const auto attempt_start = std::chrono::steady_clock::now();
    try {
      auto node = std::make_shared<px4_ros2::NodeWithModeExecutor<
          jl_mission::MissionExecutor, jl_mission::RelayMode>>("jl_mission",
                                                               true);
      // Which external-mode slot PX4 gave us (`commander mode extN`). Not
      // always 1: when PX4 answers a registration late, px4-ros2 resends it,
      // PX4 registers the name twice, and the slot we listen on may be the
      // second one.
      const int slot = node->getMode().id() -
                       px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_EXTERNAL1 +
                       1;
      RCLCPP_INFO(node->get_logger(), "Registered '%s' with PX4 (mode ext%d)",
                  node->getMode().missionName().c_str(), slot);
      const auto wanted_slot =
          node->declare_parameter<int64_t>("mode_slot", 0); // 0 = any
      if (wanted_slot > 0 && slot != wanted_slot) {
        if (slot_retries < kMaxSlotRetries) {
          ++slot_retries;
          RCLCPP_WARN(node->get_logger(),
                      "Wanted ext%ld, got ext%d (PX4 still holds an earlier "
                      "registration of this name); re-registering in %lds "
                      "(%d of %d)",
                      static_cast<long>(wanted_slot), slot,
                      static_cast<long>(kSlotRetryDelay.count()), slot_retries,
                      kMaxSlotRetries);
          node.reset(); // unregisters from PX4
          rclcpp::sleep_for(kSlotRetryDelay);
          continue;
        }
        RCLCPP_WARN(node->get_logger(),
                    "Still ext%d instead of ext%ld after %d re-registrations; "
                    "keeping ext%d",
                    slot, static_cast<long>(wanted_slot), kMaxSlotRetries, slot);
      }
      rclcpp::spin(node);
      break;
    } catch (const rclcpp::exceptions::InvalidParameterTypeException &e) {
      // A parameter of the wrong type (e.g. an int where a float is
      // expected) will not fix itself either: stop instead of retrying.
      RCLCPP_FATAL(rclcpp::get_logger("jl_mission"),
                   "%s (numeric parameters must be floats, e.g. 1.0 not 1)",
                   e.what());
      exit_code = 1;
      break;
    } catch (const std::invalid_argument &e) {
      // A bad parameter will not fix itself: stop instead of retrying.
      RCLCPP_FATAL(rclcpp::get_logger("jl_mission"), "%s", e.what());
      exit_code = 1;
      break;
    } catch (const std::runtime_error &e) {
      if (std::chrono::steady_clock::now() - attempt_start >= kHealthyFor) {
        failures = 0;
      }
      if (++failures >= kMaxAttempts) {
        RCLCPP_FATAL(rclcpp::get_logger("jl_mission"),
                     "Mode registration failed (%s); giving up after %d "
                     "attempts in a row",
                     e.what(), failures);
        exit_code = 1;
        break;
      }
      RCLCPP_WARN(rclcpp::get_logger("jl_mission"),
                  "Mode registration failed (%s); retrying in %lds "
                  "(attempt %d of %d)",
                  e.what(), static_cast<long>(retry_delay.count()),
                  failures + 1, kMaxAttempts);
      // px4-ros2's arming-check watchdog shuts rclcpp down before throwing,
      // which would end this loop: bring it back up to try again. Any other
      // shutdown is Ctrl+C or the launch stopping us, so respect it.
      if (!rclcpp::ok()) {
        if (std::string(e.what()).find("no request received from FMU") ==
            std::string::npos) {
          break;
        }
        rclcpp::init(argc, argv);
      }
      rclcpp::sleep_for(retry_delay);
    }
  }

  rclcpp::shutdown();
  return exit_code;
}
