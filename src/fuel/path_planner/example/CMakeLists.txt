add_executable(path_planner_example path_planner_example.cpp)

target_include_directories(path_planner_example
  PRIVATE
    ../interface/include
)

target_link_libraries(path_planner_example path_planner)

install(
    TARGETS path_planner_example
    DESTINATION bin
)
