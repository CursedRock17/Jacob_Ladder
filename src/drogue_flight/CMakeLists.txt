cmake_minimum_required(VERSION 3.5)
project(voxl_offboard_ship_land)

set(CMAKE_CXX_STANDARD 20)

add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(px4_ros2_cpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(${Eigen3_INCLUDE_DIRS})

add_executable(drogue_approach
    DrogueApproach.cpp
)
ament_target_dependencies(drogue_approach rclcpp Eigen3 px4_ros2_cpp px4_msgs geometry_msgs)
target_compile_features(drogue_approach PUBLIC c_std_99 cxx_std_17)

install(TARGETS
  drogue_approach
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY cfg
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
