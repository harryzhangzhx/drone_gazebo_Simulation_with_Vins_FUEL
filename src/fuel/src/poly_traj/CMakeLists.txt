cmake_minimum_required(VERSION 3.14.4)
project(poly_traj)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(active_perception REQUIRED)


add_library(poly_traj 
    src/polynomial_traj.cpp 
)

target_include_directories(poly_traj
PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
PRIVATE
    ${EIGEN3_INCLUDE_DIRS}
)

ament_target_dependencies(poly_traj
    rclcpp
    std_msgs
    nav_msgs
    visualization_msgs
    active_perception
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS poly_traj
    EXPORT export_poly_traj
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

ament_export_targets(export_poly_traj HAS_LIBRARY_TARGET)
# ament_export_include_directories(include)
# TODO check which packages are exposed to downstream packages
ament_export_dependencies(    
    rclcpp
    std_msgs
    nav_msgs
    visualization_msgs
    active_perception
)

ament_package()