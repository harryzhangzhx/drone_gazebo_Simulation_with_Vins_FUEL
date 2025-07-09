# cmake_minimum_required(VERSION 2.8.3)
# project(exploration_manager)

# set(CMAKE_BUILD_TYPE "Release")
# #set(CMAKE_CXX_FLAGS "-std=c++11")
# set(CMAKE_CXX_STANDARD 14)
# set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")
# cmake_minimum_required(VERSION 3.14.4)
# find_package(Eigen3 REQUIRED)
# find_package(PCL 1.7 REQUIRED)

# #find_package(catkin REQUIRED COMPONENTS
# # roscpp
# #  std_msgs
# # geometry_msgs
# #  quadrotor_msgs
# #  plan_env
# #  path_searching
# #  bspline
# #  bspline_opt
# #  traj_utils
# #  active_perception
# #  lkh_tsp_solver
# #  plan_manage
# #  cv_bridge
# #  tf
# #  tf2
# #)



# find_package(ament_cmake REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(std_msgs REQUIRED)
# find_package(geometry_msgs REQUIRED)
# find_package(quadrotor_msgs REQUIRED)
# find_package(plan_env REQUIRED)
# find_package(path_searching REQUIRED)
# find_package(bspline REQUIRED)
# find_package(bspline_opt REQUIRED)
# find_package(traj_utils REQUIRED)
# find_package(active_perception)
# find_package(lkh_tsp_solver)
# find_package(plan_manage REQUIRED)
# find_package(cv_bridge REQUIRED)
# find_package(tf REQUIRED)
# find_package(tf2 REQUIRED)







# #catkin_package(
# # INCLUDE_DIRS include
# # LIBRARIES exploration_manager
# # CATKIN_DEPENDS plan_env path_searching bspline bspline_opt active_perception traj_utils lkh_tsp_solver plan_manage
# ##  DEPENDS system_lib
# #)




# #include_directories(
# #  include
# #  SYSTEM
# #  ${catkin_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/include
# #  ${EIGEN3_INCLUDE_DIR}
# #  ${PCL_INCLUDE_DIRS}
# #)



# add_executable(exploration_node
#   src/exploration_node.cpp 
#   src/fast_exploration_fsm.cpp
#   src/fast_exploration_manager.cpp
#   )
# target_link_libraries(exploration_node 
#   ${catkin_LIBRARIES}
#   # -ldw
#   )

# # add_library(exploration_manager
# #   src/fast_exploration_manager.cpp
# # )
# # target_link_libraries(exploration_manager
# #   ${catkin_LIBRARIES}
# # )

# add_executable(write_tsp test/write_tsp.cpp)
# target_link_libraries(write_tsp
#   ${catkin_LIBRARIES} 
# )

# add_executable(fuel_nav src/fuel_nav.cpp)
# target_link_libraries(fuel_nav
#   ${catkin_LIBRARIES} 
# )

# add_executable(load_obj test/load_obj.cpp)
# target_link_libraries(load_obj
#   ${catkin_LIBRARIES} ${PCL_LIBRARIES}
# )

# target_include_directories()??

# ament_package()

cmake_minimum_required(VERSION 3.5)
project(exploration_manager)

# Set C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

# Find ROS 2 packages
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(NLopt REQUIRED)


# Find custom dependencies (ensure these are ported to ROS2)
find_package(quadrotor_msgs REQUIRED)
find_package(plan_env REQUIRED)
find_package(path_searching REQUIRED)
find_package(bspline REQUIRED)
find_package(bspline_opt REQUIRED)
find_package(traj_utils REQUIRED)
find_package(active_perception REQUIRED)
find_package(lkh_tsp_solver REQUIRED)
#find_package(waypoint_generator REQUIRED)
find_package(plan_manage REQUIRED)

# Include directories
include_directories(
  include
  ${Eigen3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  
)

# Declare executables
add_executable(exploration_node
  src/exploration_node.cpp 
  src/fast_exploration_fsm.cpp
  src/fast_exploration_manager.cpp
)
ament_target_dependencies(exploration_node
  rclcpp
  std_msgs
  geometry_msgs
  quadrotor_msgs
  plan_env
  path_searching
  bspline
  bspline_opt
  traj_utils
  active_perception
  lkh_tsp_solver
  #waypoint_generator
  plan_manage
  cv_bridge
  tf2
  tf2_ros
)
target_link_libraries(exploration_node ${PCL_LIBRARIES})

# add_executable(write_tsp test/write_tsp.cpp)
# ament_target_dependencies(write_tsp rclcpp std_msgs)

# add_executable(fuel_nav src/fuel_nav.cpp)
# ament_target_dependencies(fuel_nav rclcpp std_msgs)

# add_executable(load_obj test/load_obj.cpp)
# ament_target_dependencies(load_obj rclcpp)
# target_link_libraries(load_obj ${PCL_LIBRARIES})

# Install targets
install(TARGETS
  exploration_node
  # write_tsp
  # fuel_nav
  # load_obj
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
DESTINATION share/${PROJECT_NAME})

# Export dependencies
ament_export_dependencies(
  rclcpp
  std_msgs
  geometry_msgs
  quadrotor_msgs
  plan_env
  path_searching
  bspline
  bspline_opt
  traj_utils
  active_perception
  lkh_tsp_solver
  #waypoint_generator
  plan_manage
  cv_bridge
  tf2
  tf2_ros
)

# Finalize
ament_package()
