add_library(governor SHARED src/governor_api.cpp)

target_link_libraries(governor
  governor_algorithms # internal target
)

ament_target_dependencies(governor
  utilities
)

target_include_directories(governor
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
  governor
  governor_algorithms
  EXPORT export_governor
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)