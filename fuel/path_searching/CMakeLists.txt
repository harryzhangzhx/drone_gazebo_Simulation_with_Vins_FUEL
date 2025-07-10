cmake_minimum_required(VERSION 3.5)
project(path_searching)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")

find_package(plan_env REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(Eigen3 REQUIRED)


add_library(path_searching SHARED
  src/kinodynamic_astar.cpp
  src/astar2.cpp
  src/topo_prm.cpp
)

ament_target_dependencies(path_searching
  plan_env  
  rclcpp
  std_msgs
  visualization_msgs
  cv_bridge
)

target_include_directories(path_searching
PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
PRIVATE
    ${EIGEN3_INCLUDE_DIRS}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS path_searching
  EXPORT export_path_searching
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Export package
ament_export_targets(export_path_searching HAS_LIBRARY_TARGET)
# TODO check which packages are exposed to downstream packages
ament_export_dependencies(
  plan_env  
  rclcpp
  std_msgs
  visualization_msgs
  cv_bridge
)
# ament_export_include_directories(include)

ament_package()