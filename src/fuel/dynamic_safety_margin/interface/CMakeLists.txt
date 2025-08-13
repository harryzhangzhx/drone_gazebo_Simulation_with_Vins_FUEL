add_library(dsm SHARED
  src/dsm_api.cpp
)

ament_target_dependencies(dsm
  utilities
  Eigen3
)

target_link_libraries(dsm
  dsm_algorithms # internal target
)

target_include_directories(dsm
PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS
  dsm
  dsm_algorithms
  EXPORT export_dsm
  DESTINATION lib
)
