cmake_minimum_required(VERSION 3.5)
project(plan_manage)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")

find_package(quadrotor_msgs REQUIRED)
find_package(poly_traj REQUIRED)
find_package(plan_env REQUIRED)
find_package(path_searching REQUIRED)
find_package(bspline REQUIRED)
find_package(bspline_opt REQUIRED)
find_package(traj_utils REQUIRED)
find_package(active_perception REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(NLopt REQUIRED)


add_library(plan_manage 
  src/planner_manager.cpp
)


target_include_directories(plan_manage
PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
PRIVATE
    ${EIGEN3_INCLUDE_DIRS}
)

ament_target_dependencies(plan_manage
  poly_traj
  plan_env
  path_searching
  bspline
  bspline_opt
  traj_utils
  active_perception
  rclcpp
  std_msgs
)

add_executable(traj_server 
  src/traj_server.cpp
)


target_include_directories(traj_server
PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
PRIVATE
    ${EIGEN3_INCLUDE_DIRS}
)

target_link_libraries(traj_server
  -ldw
)  

ament_target_dependencies(traj_server
  poly_traj
  plan_env
  path_searching
  bspline
  bspline_opt
  traj_utils
  active_perception
  rclcpp
  std_msgs
  quadrotor_msgs
)


install(TARGETS plan_manage
    EXPORT export_plan_manage
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
)

install(TARGETS traj_server
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_targets(export_plan_manage HAS_LIBRARY_TARGET)
# TODO check which packages are exposed to downstream packages
ament_export_dependencies(
  poly_traj
  plan_env
  path_searching
  bspline
  bspline_opt
  traj_utils
  active_perception
  rclcpp
  std_msgs
)
# ament_export_include_directories(include)

ament_package()
