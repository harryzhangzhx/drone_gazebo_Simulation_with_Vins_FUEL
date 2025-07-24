add_library(governor_algorithms SHARED
  src/erg.cpp
  src/pathfg.cpp
)

target_include_directories(governor_algorithms
PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
PRIVATE
  ${EIGEN3_INCLUDE_DIR}
)

ament_target_dependencies(governor_algorithms
  utilities
  dsm
)