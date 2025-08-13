cmake_minimum_required(VERSION 3.5)
project(active_perception)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 14)
add_compile_options(-Ofast -Wall -Wextra -Wpedantic)

find_package(plan_env REQUIRED)
find_package(bspline REQUIRED)
find_package(path_searching REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

add_library(active_perception
  src/traj_visibility.cpp 
  src/heading_planner.cpp
  src/frontier_finder.cpp
  src/graph_node.cpp
  src/perception_utils.cpp
)

target_include_directories(active_perception
PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(active_perception
  plan_env
  bspline
  path_searching
  rclcpp
  std_msgs
  sensor_msgs
  visualization_msgs
  cv_bridge
  Eigen3
  PCL
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS active_perception
  EXPORT export_active_perception
  DESTINATION lib
)

ament_export_targets(export_active_perception HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  std_msgs
  sensor_msgs
  visualization_msgs
  cv_bridge
  plan_env
  bspline
  path_searching
  Eigen3
)
ament_package()