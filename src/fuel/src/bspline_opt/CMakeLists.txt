cmake_minimum_required(VERSION 3.5)
project(bspline_opt)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")

find_package(plan_env REQUIRED)
find_package(active_perception REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(NLopt REQUIRED)


add_library(bspline_opt 
    src/bspline_optimizer.cpp
)

target_include_directories(bspline_opt
PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
PRIVATE
    ${EIGEN3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${NLOPT_INCLUDE_DIR}
)

ament_target_dependencies(bspline_opt
  rclcpp
  std_msgs
  plan_env
  active_perception
)

target_link_libraries(bspline_opt
  ${PCL_LIBRARIES}
  ${NLOPT_LIBRARIES}
  /usr/local/lib/libnlopt.so
)

install(TARGETS bspline_opt
  EXPORT export_bspline_opt
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

# Export package
ament_export_targets(export_bspline_opt HAS_LIBRARY_TARGET)
# TODO check which packages are exposed to downstream packages
ament_export_dependencies(
    rclcpp
    std_msgs
    plan_env
    active_perception
)
# ament_export_include_directories(include)

ament_package()

