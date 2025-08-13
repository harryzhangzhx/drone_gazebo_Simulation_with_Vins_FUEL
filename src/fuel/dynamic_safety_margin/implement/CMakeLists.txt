add_library(dsm_algorithms SHARED
  src/dsm.cpp
)

ament_target_dependencies(dsm_algorithms
  utilities
  Eigen3
)

target_include_directories(dsm_algorithms
PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
