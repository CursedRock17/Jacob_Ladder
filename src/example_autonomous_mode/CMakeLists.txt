cmake_minimum_required(VERSION 3.5)
project(example_autonomous_mode)

set(CMAKE_CXX_STANDARD 20)

add_compile_options(-Wall -Wextra -Wpedantic -Werror -Wno-unused-parameter)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(px4_ros2_cpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(${Eigen3_INCLUDE_DIRS})

# Minimal external mode: low hover for flow init, hold, then land
add_executable(example_autonomous_mode
  ExampleAutonomousMode.cpp
)

ament_target_dependencies(example_autonomous_mode
  rclcpp
  Eigen3
  px4_ros2_cpp
  px4_msgs
  geometry_msgs
  std_msgs
)

# Install executables
install(TARGETS
  example_autonomous_mode
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

# Install launch and config folders
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY cfg
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
