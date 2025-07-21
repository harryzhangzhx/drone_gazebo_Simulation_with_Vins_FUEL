cmake_minimum_required(VERSION 3.8)
project(px4_odometry_converter)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(px4_ros_com REQUIRED)

add_executable(odom_converter_node src/odom_converter_node.cpp)

ament_target_dependencies(odom_converter_node
  rclcpp
  px4_msgs
  nav_msgs
  px4_ros_com
  Eigen3
)

install(TARGETS
  odom_converter_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()