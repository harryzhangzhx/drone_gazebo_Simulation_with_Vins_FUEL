# cmake_minimum_required(VERSION 2.8.3)
# project(waypoint_generator)

# set(CMAKE_VERBOSE_MAKEFILE "true")
# include(CheckCXXCompilerFlag)
# CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
# CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
# if(COMPILER_SUPPORTS_CXX11)
#     set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
# elseif(COMPILER_SUPPORTS_CXX0X)
#     set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
# else()
#         message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
# endif()

# set(ADDITIONAL_CXX_FLAG "-Wall -O3 -march=native")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${ADDITIONAL_CXX_FLAG}")

# find_package(catkin REQUIRED COMPONENTS
#   roscpp
#   tf
#   nav_msgs
# )
# catkin_package()

# include_directories(include)
# include_directories(
#   ${catkin_INCLUDE_DIRS}
# )

# add_executable(waypoint_generator src/waypoint_generator.cpp)

# target_link_libraries(waypoint_generator
#    ${catkin_LIBRARIES}
# )
cmake_minimum_required(VERSION 3.5)
project(waypoint_generator)

# Set C++ standard to 14 (ROS 2 default)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Additional compiler flags
set(ADDITIONAL_CXX_FLAG "-Wall -O3 -march=native")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${ADDITIONAL_CXX_FLAG}")

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)


# Include directories
include_directories(include)

# Create executable
add_executable(waypoint_generator src/waypoint_generator.cpp)

# Link dependencies
ament_target_dependencies(waypoint_generator
  rclcpp
  tf2_ros
  nav_msgs
  tf2_geometry_msgs
)

# Install the executable
install(TARGETS waypoint_generator
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
