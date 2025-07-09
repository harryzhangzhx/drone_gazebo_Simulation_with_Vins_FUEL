cmake_minimum_required(VERSION 3.5)
project(active_perception)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")

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
PRIVATE
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

target_link_libraries(active_perception
  ${PCL_LIBRARIES}
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
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS active_perception
  EXPORT export_active_perception
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


# ament_export_include_directories(include)
ament_export_targets(export_active_perception HAS_LIBRARY_TARGET)
# TODO check which packages are exposed to downstream packages
ament_export_dependencies(
  rclcpp
  std_msgs
  sensor_msgs
  visualization_msgs
  cv_bridge
  plan_env
  bspline
  path_searching
)

ament_package()