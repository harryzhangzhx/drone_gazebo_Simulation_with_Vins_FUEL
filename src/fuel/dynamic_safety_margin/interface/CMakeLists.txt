add_library(dsm SHARED src/dsm_api.cpp)

target_include_directories(dsm
PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/../implement/include
  ${EIGEN3_INCLUDE_DIR}
)

target_link_libraries(dsm
  dsm_algorithms # internal target
  Eigen3::Eigen
)

ament_target_dependencies(dsm
  utilities
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS
  dsm
  dsm_algorithms
  EXPORT export_dsm
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)