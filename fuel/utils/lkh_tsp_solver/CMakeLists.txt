cmake_minimum_required(VERSION 3.14)
project(lkh_tsp_solver)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")

# Required dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# Include directories
include_directories(
  include
)

# Source files
file(GLOB SRCS src/*.c)

# Build the library
add_library(lkh_tsp_solver SHARED
  ${SRCS}
  src/lkh_interface.cpp
)
target_link_libraries(lkh_tsp_solver m)  # -lm

# Build the standalone executable
add_executable(lkh_tsp
  ${SRCS}
)
target_link_libraries(lkh_tsp lkh_tsp_solver m)

# Install targets
install(TARGETS lkh_tsp_solver lkh_tsp
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include)

# Package export
ament_export_include_directories(include)
ament_export_libraries(lkh_tsp_solver)
ament_export_dependencies(rclcpp)

ament_package()

