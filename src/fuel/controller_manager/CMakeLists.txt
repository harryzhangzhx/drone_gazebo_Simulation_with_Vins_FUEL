cmake_minimum_required(VERSION 3.5)
project(controller_manager)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(utilities REQUIRED)
find_package(dsm REQUIRED)
find_package(path_planner REQUIRED)
find_package(controller REQUIRED)
find_package(governor REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(quadrotor_msgs REQUIRED)

add_subdirectory(implement)
add_subdirectory(interface)
add_subdirectory(example)

ament_package()
