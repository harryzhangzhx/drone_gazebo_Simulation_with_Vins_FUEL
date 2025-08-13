add_library(controller SHARED src/controller_api.cpp)

target_link_libraries(controller
  controller_algorithms # internal target
)

ament_target_dependencies(controller
  utilities
  Eigen3
)

target_include_directories(controller
PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS
  controller
  controller_algorithms
  EXPORT export_controller
  DESTINATION lib
)
