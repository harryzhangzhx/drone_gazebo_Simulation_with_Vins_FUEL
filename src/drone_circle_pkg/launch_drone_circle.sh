#!/bin/bash

# Terminal 0: Prepare Gazebo (uncomment the below line if Gazebo won't reopen)
# gnome-terminal -- bash -c "pkill -f gz; exec bash"

# Terminal 1: PX4 SITL
gnome-terminal -- bash -c "export DISPLAY=:0; sleep 2; cd ~/PX4-Autopilot && PX4_GZ_WORLD=baylands make px4_sitl gz_x500_depth; exec bash" # To set a different sim environment, change PX4_GZ_WORLD=$simulation environment$, currently set to baylands. Drone model is set to gz_x500_depth, output are depth images and RGB images

# Terminal 2: MicroXRCEAgent
gnome-terminal -- bash -c "MicroXRCEAgent udp4 -p 8888; exec bash"

# Terminal 3: QGroundControl
gnome-terminal -- bash -c "~/QGroundControl-x86_64.AppImage; exec bash"

# Terminal 4: ROS 2 Node
gnome-terminal -- bash -c "cd ~/drone_gazebo_Simulation_with_Vins_FUEL && source install/setup.bash && source ./install/setup.bash && sleep 30s && ros2 run drone_circle_pkg drone_circle_node; exec bash" # can modify the file "drone_circle_node.cpp" to switch between hovering motion and circling motion.

# Terminal 5: ROS 2 Parameter Bridge
gnome-terminal -- bash -c "sleep 31s && ros2 run ros_gz_bridge parameter_bridge \
/world/baylands/model/x500_depth_0/link/camera_link/sensor/IMX214/image@sensor_msgs/msg/Image@ignition.msgs.Image \
/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo \
/depth_camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked \
/depth_camera@sensor_msgs/msg/Image@gz.msgs.Image \
/world/baylands/model/x500_depth_0/link/base_link/sensor/imu_sensor/imu@sensor_msgs/msg/Imu[gz.msgs.IMU; exec bash" # publish ros topic from gazebo

# Terminal 6: Odometry Converter Node
gnome-terminal -- bash -c "ros2 run px4_odometry_converter odom_converter_node; exec bash" # convert px4 odometry message to ros message