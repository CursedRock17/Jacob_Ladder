cmake_minimum_required(VERSION 3.8)
project(jl_mission)

set(CMAKE_CXX_STANDARD 17)
add_compile_options(-Wall -Wextra -Wpedantic -Werror -Wno-unused-parameter)

find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_ros2_cpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(jl_mission_interfaces REQUIRED)

# The safety contract as plain C++ (Eigen only), so it is unit-tested without ROS
add_library(setpoint_guard src/setpoint_guard.cpp)
target_include_directories(setpoint_guard PUBLIC include)
# Eigen is an external dependency; its GCC/NEON warnings must not become
# errors under this package's -Werror policy.
target_include_directories(setpoint_guard SYSTEM PUBLIC ${EIGEN3_INCLUDE_DIR})

add_executable(jl_mission src/main.cpp src/mission_mode.cpp)
target_include_directories(jl_mission PRIVATE include)
target_link_libraries(jl_mission setpoint_guard)
ament_target_dependencies(jl_mission rclcpp Eigen3 px4_ros2_cpp px4_msgs std_msgs
  std_srvs geometry_msgs jl_mission_interfaces)

install(TARGETS jl_mission DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_setpoint_guard test/test_setpoint_guard.cpp)
  target_link_libraries(test_setpoint_guard setpoint_guard)
endif()

ament_package()
