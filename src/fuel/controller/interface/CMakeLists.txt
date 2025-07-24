add_library(controller SHARED src/controller_api.cpp)

target_link_libraries(controller
  controller_algorithms # internal target
)

ament_target_dependencies(controller
  utilities
)

target_include_directories(controller
PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/../implement/include
  ${EIGEN3_INCLUDE_DIR}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS
  controller
  controller_algorithms
  EXPORT export_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
