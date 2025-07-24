cmake_minimum_required(VERSION 3.8)
project(dsm)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(utilities REQUIRED)

add_subdirectory(implement)
add_subdirectory(interface)
add_subdirectory(example)

ament_export_targets(export_dsm HAS_LIBRARY_TARGET)
ament_package()
