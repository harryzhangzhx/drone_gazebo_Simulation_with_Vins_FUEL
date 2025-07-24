cmake_minimum_required(VERSION 3.8)
project(governor)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(utilities REQUIRED)
find_package(dsm REQUIRED)

add_subdirectory(implement)
add_subdirectory(interface)
add_subdirectory(example)

ament_export_targets(export_governor HAS_LIBRARY_TARGET)
# ament_export_include_directories(interface/include)

ament_package()
