cmake_minimum_required(VERSION 3.5)
project(precision_land)

set(CMAKE_CXX_STANDARD 20)

add_compile_options(-Wall -Wextra -Wpedantic -Werror -Wno-unused-parameter)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(px4_ros2_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)

include_directories(${Eigen3_INCLUDE_DIRS})

# Stupid Simple External Mode that does Almost Nothing
add_executable(blank_mode
  BlankMode.cpp
)
ament_target_dependencies(blank_mode rclcpp Eigen3 px4_ros2_cpp geometry_msgs tf2_ros tf2)
target_compile_features(blank_mode PUBLIC c_std_99 cxx_std_17)

# Original static platform version
add_executable(precision_land
    PrecisionLand.cpp
)
ament_target_dependencies(precision_land rclcpp Eigen3 px4_ros2_cpp geometry_msgs tf2_ros tf2)
target_compile_features(precision_land PUBLIC c_std_99 cxx_std_17)

# Add Visualizer for ROS topics
add_executable(visualizer
    Visualizer.cpp
)
ament_target_dependencies(visualizer rclcpp Eigen3 px4_ros2_cpp geometry_msgs visualization_msgs tf2_ros tf2 nav_msgs)
target_compile_features(visualizer PUBLIC c_std_99 cxx_std_17)

# Change to This Mode: It will automatically arm, take off, hover, and land
add_executable(takeoff_land
    TakeoffLand.cpp
)
ament_target_dependencies(takeoff_land rclcpp Eigen3 px4_ros2_cpp geometry_msgs tf2_ros tf2)
target_compile_features(takeoff_land PUBLIC c_std_99 cxx_std_17)

# Change to This Mode: It will automatically arm, take off and hover
add_executable(takeoff_hold
    TakeoffHold.cpp
)
ament_target_dependencies(takeoff_hold rclcpp Eigen3 px4_ros2_cpp geometry_msgs tf2_ros tf2)
target_compile_features(takeoff_hold PUBLIC c_std_99 cxx_std_17)

# Front camera approach mission
add_executable(front_approach
    FrontApproach.cpp
)
ament_target_dependencies(front_approach rclcpp Eigen3 px4_ros2_cpp geometry_msgs tf2_ros tf2)
target_compile_features(front_approach PUBLIC c_std_99 cxx_std_17)

# Combined front approach + precision land mission
add_executable(front_to_precision_land
    FrontApproachPrecisionLandCombined.cpp
)
ament_target_dependencies(front_to_precision_land rclcpp Eigen3 px4_ros2_cpp geometry_msgs tf2_ros tf2 px4_msgs)
target_compile_features(front_to_precision_land PUBLIC c_std_99 cxx_std_17)

# Install both executables
install(TARGETS
  blank_mode
  precision_land
  takeoff_land
  takeoff_hold
  front_approach
  front_to_precision_land
  visualizer
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
