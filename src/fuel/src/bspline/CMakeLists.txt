cmake_minimum_required(VERSION 3.5)
project(bspline)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")


find_package(plan_env REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)


rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Bspline.msg"
  "msg/NextPosAndYaw.msg"
  DEPENDENCIES 
    std_msgs 
    geometry_msgs
    builtin_interfaces
)
  
add_library(bspline_lib SHARED
  src/non_uniform_bspline.cpp
)

target_include_directories(bspline_lib 
PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
PRIVATE
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

target_link_libraries(bspline_lib
  ${PCL_LIBRARIES}
)

ament_target_dependencies(bspline_lib
  plan_env  
  rclcpp
  std_msgs
  visualization_msgs
  geometry_msgs
)

rosidl_target_interfaces(bspline_lib
  ${PROJECT_NAME} "rosidl_typesupport_cpp")


install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS bspline_lib
  EXPORT export_bspline_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Export
# ament_export_include_directories(include)
ament_export_targets(export_bspline_lib HAS_LIBRARY_TARGET)
# TODO check which packages are exposed to downstream packages
ament_export_dependencies(
  plan_env  
  rclcpp
  std_msgs
  visualization_msgs
  geometry_msgs
  rosidl_default_runtime
)

ament_package()