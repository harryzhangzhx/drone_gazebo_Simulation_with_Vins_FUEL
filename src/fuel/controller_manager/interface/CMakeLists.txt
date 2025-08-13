add_executable(controller_manager
  controller_manager_main.cpp  
  src/topic_manager.cpp
  src/service_manager.cpp
)

target_include_directories(controller_manager
PUBLIC
  include
PRIVATE
  ../implement/include
  ${EIGEN3_INCLUDE_DIR}
)

target_link_libraries(controller_manager
  # PRIVATE
    controller_manager_implement
)

ament_target_dependencies(controller_manager
  rclcpp
  std_msgs
  geometry_msgs
  quadrotor_msgs
  path_planner
  controller
  governor
  utilities
)

install(TARGETS controller_manager
  DESTINATION lib/${PROJECT_NAME})
