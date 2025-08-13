add_library(path_planner_algorithms SHARED
  src/astar2.cpp
  src/kinodynamic_astar.cpp
  src/potential_field.cpp
  src/topo_prm.cpp
  src/rrtstar.cpp
  src/KDTree.cpp
)

target_include_directories(path_planner_algorithms
PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
PRIVATE
  ${EIGEN3_INCLUDE_DIR}
)
ament_target_dependencies(path_planner_algorithms
    rclcpp
    utilities
)