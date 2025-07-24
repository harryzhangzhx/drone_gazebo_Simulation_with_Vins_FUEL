add_library(controller_manager_implement SHARED
  src/controller_manager.cpp
)

message(WARNING ${Eigen3_INCLUDE_DIRS})
message(WARNING hello)

target_include_directories(controller_manager_implement
  PRIVATE
    include
    ../interface/include
)

ament_target_dependencies(controller_manager_implement
  rclcpp
  std_msgs
  geometry_msgs
  quadrotor_msgs
  path_planner
  utilities
  controller
)

target_link_libraries(controller_manager_implement
  Eigen3::Eigen
)

install(TARGETS controller_manager_implement
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
