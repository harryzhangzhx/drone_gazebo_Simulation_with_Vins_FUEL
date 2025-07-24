add_library(path_planner SHARED src/path_planner_api.cpp)

target_link_libraries(path_planner
  path_planner_algorithms # internal target
)

ament_target_dependencies(path_planner
  utilities
)

target_include_directories(path_planner
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
  path_planner
  path_planner_algorithms
  EXPORT export_path_planner
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
