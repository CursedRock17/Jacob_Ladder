cmake_minimum_required(VERSION 3.5)
project(drogue_flight)

set(CMAKE_CXX_STANDARD 20)

add_compile_options(-Wall -Wextra -Wpedantic -Werror -Wno-unused-parameter)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(px4_ros2_cpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

include_directories(${Eigen3_INCLUDE_DIRS})

# S-curve trajectory planner with External mode
add_executable(drone_smooth_planner
  DroneSmoothPlanner.cpp
)
ament_target_dependencies(drone_smooth_planner rclcpp Eigen3 px4_ros2_cpp px4_msgs geometry_msgs nav_msgs)
target_compile_features(drone_smooth_planner PUBLIC c_std_99 cxx_std_17)

install(TARGETS
  drone_smooth_planner
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
