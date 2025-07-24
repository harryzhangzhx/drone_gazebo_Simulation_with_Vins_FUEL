
add_library(controller_algorithms SHARED 
    src/lqr.cpp
    src/mpc.cpp
)

target_include_directories(controller_algorithms
PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
PRIVATE
    ${EIGEN3_INCLUDE_DIR}
)

ament_target_dependencies(controller_algorithms
  utilities
)