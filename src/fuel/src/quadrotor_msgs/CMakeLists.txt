#-cmake_minimum_required(VERSION 2.4.6)
#-include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

#-rosbuild_init()

#set the default path for built executables to the "bin" directory
#-set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
#-set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#-\rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

#-rosbuild_add_library(encode_msgs src/encode_msgs.cpp)
#-rosbuild_add_link_flags(encode_msgs -Wl,--as-needed)

#list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
#find_package(Eigen3 REQUIRED)

#include_directories(${EIGEN3_INCLUDE_DIR})
#-rosbuild_add_library(decode_msgs src/decode_msgs.cpp)
#-rosbuild_add_link_flags(decode_msgs -Wl,--as-needed)

#------------------------------------------------------------------
cmake_minimum_required(VERSION 3.5)
project(quadrotor_msgs)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rosidl_default_generators REQUIRED)


set(msg_files
  "msg/AuxCommand.msg"
  "msg/Corrections.msg"
  "msg/Gains.msg"
  "msg/OutputData.msg"
  "msg/PositionCommand.msg"
  "msg/PPROutputData.msg"
  "msg/Serial.msg"
  "msg/SO3Command.msg"
  "msg/StatusData.msg"
  "msg/TRPYCommand.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES
    geometry_msgs
    nav_msgs
    std_msgs
)

# # Include directories
# include_directories(
#   include
#   ${EIGEN3_INCLUDE_DIRS}
# )

# # Libraries
# add_library(decode_msgs src/decode_msgs.cpp)
# ament_target_dependencies(decode_msgs
#   rclcpp
#   geometry_msgs
#   nav_msgs
#   std_msgs
# )

# add_library(encode_msgs src/encode_msgs.cpp)
# ament_target_dependencies(encode_msgs
#   rclcpp
#   geometry_msgs
#   nav_msgs
#   std_msgs
# )

# # Install headers
# install(DIRECTORY include/ 
# DESTINATION include)

# # Install libraries
# install(TARGETS decode_msgs
#   EXPORT export_decode_msgs
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin
# )

# install(TARGETS encode_msgs
#   EXPORT export_encode_msgs
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin
# )

# # Export package info
# ament_export_include_directories(include)
# ament_export_targets(export_decode_msgs export_encode_msgs HAS_LIBRARY_TARGET)
# ament_export_dependencies(
#   rclcpp
#   geometry_msgs
#   nav_msgs
#   std_msgs
# )


ament_package()