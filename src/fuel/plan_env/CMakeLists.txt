cmake_minimum_required(VERSION 3.5)
project(plan_env)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)


add_library(plan_env SHARED
    src/sdf_map.cpp
    src/obj_predictor.cpp
    src/edt_environment.cpp
    src/raycast.cpp
    src/map_ros.cpp
)

target_include_directories(plan_env
PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
PRIVATE
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)

target_link_libraries(plan_env
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
)

ament_target_dependencies(plan_env
    rclcpp
    cv_bridge
    std_msgs
    visualization_msgs
    sensor_msgs
    geometry_msgs
    nav_msgs
    pcl_conversions
)

add_executable(obj_generator
    src/obj_generator.cpp
)

target_include_directories(obj_generator 
PRIVATE
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

target_link_libraries(obj_generator
  plan_env
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
)

ament_target_dependencies(obj_generator  
    rclcpp
    cv_bridge
    std_msgs
    visualization_msgs
    sensor_msgs
    geometry_msgs
    nav_msgs
    pcl_conversions
)

install(TARGETS plan_env
  EXPORT export_plan_env
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS obj_generator
    DESTINATION lib/${PROJECT_NAME}
)

install(
    DIRECTORY include/
    DESTINATION include
)

ament_export_targets(export_plan_env HAS_LIBRARY_TARGET)
# TODO check which packages are exposed to downstream packages
ament_export_dependencies(
    rclcpp
    cv_bridge
    std_msgs
    visualization_msgs
    sensor_msgs
    geometry_msgs
    nav_msgs
    pcl_conversions
)
# ament_export_include_directories(include)

ament_package()
