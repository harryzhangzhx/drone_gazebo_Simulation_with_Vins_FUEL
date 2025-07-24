add_executable(controller_manager_example controller_manager_example.cpp)

ament_target_dependencies(controller_manager_example
  rclcpp
  std_msgs
  geometry_msgs
)

install(TARGETS controller_manager_example
  DESTINATION bin
)
