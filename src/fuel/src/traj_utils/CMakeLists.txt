cmake_minimum_required(VERSION 3.5)
project(traj_utils)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")

find_package(bspline REQUIRED)
find_package(path_searching REQUIRED)
find_package(poly_traj REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(geometry_msgs REQUIRED)


add_library(traj_utils
    src/planning_visualization.cpp 
)

target_include_directories(traj_utils
PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
PRIVATE
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)

target_link_libraries(traj_utils
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
)  

ament_target_dependencies(traj_utils
    bspline
    path_searching
    poly_traj  
    rclcpp
    std_msgs
    cv_bridge
    pcl_conversions
    visualization_msgs
    geometry_msgs
)

add_executable(process_msg 
    src/process_msg.cpp
)

target_include_directories(process_msg 
PRIVATE
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

target_link_libraries(process_msg
  ${PCL_LIBRARIES}
)

ament_target_dependencies(process_msg
    bspline
    path_searching
    poly_traj  
    rclcpp
    std_msgs
    cv_bridge
    pcl_conversions
    sensor_msgs
    visualization_msgs
    geometry_msgs
  )
  
install(TARGETS traj_utils
    EXPORT export_traj_utils
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
)

install(TARGETS process_msg
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_targets(export_traj_utils HAS_LIBRARY_TARGET)
# TODO check which packages are exposed to downstream packages
ament_export_dependencies(
    bspline
    path_searching
    poly_traj  
    rclcpp
    std_msgs
    cv_bridge
    pcl_conversions
    geometry_msgs
)
# ament_export_include_directories(include)

ament_package()
