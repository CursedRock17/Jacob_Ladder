// jl_mission entry point. One process per mission file: the mission's name
// (as shown in QGC) comes from the mission_name parameter.
//
// Registration is retried the same way TakeoffHold does it: wait for the FMU,
// then keep retrying, because at boot the DDS link can come up after us.

#include "jl_mission/mission_mode.hpp"

#include <px4_ros2/components/node_with_mode.hpp>
#include <px4_ros2/components/wait_for_fmu.hpp>
#include <rclcpp/exceptions/exceptions.hpp>

#include <chrono>
#include <stdexcept>

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

  const auto retry_delay = 2s;
  int exit_code = 0;
  while (rclcpp::ok()) {
    try {
      auto node = std::make_shared<px4_ros2::NodeWithModeExecutor<
          jl_mission::MissionExecutor, jl_mission::RelayMode>>("jl_mission",
                                                               true);
      RCLCPP_INFO(node->get_logger(), "Registered '%s' with PX4",
                  node->getMode().missionName().c_str());
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
      RCLCPP_WARN(rclcpp::get_logger("jl_mission"),
                  "Mode registration failed (%s); retrying in %lds", e.what(),
                  static_cast<long>(retry_delay.count()));
      rclcpp::sleep_for(retry_delay);
    }
  }

  rclcpp::shutdown();
  return exit_code;
}
