This repository contains a simple example of a px4 sitl communicating with ros2 to make the drone fly around in a circle automatically

Prerequisites:

1. have px4_sitl installed, following the steps outlined here (Ubuntu): https://docs.px4.io/main/en/dev_setup/dev_env_linux_ubuntu.html
2. install the QGC, as outlined here: https://docs.px4.io/main/en/dev_setup/qgc_daily_build.html
3. install the uXFRCE-messaging system according to the steps outlined here: https://docs.px4.io/main/en/middleware/uxrce_dds.html#micro-xrce-dds-agent-installation



Steps to run this package:

1. Clone this repo into the a folder named drone_circle_pkg in the src directory of your ros2 workspace
2. clone the px4 msgs repository into the src directory and follow the steps outlined here: https://docs.px4.io/main/en/ros2/user_guide.html
3. cd .. into the root of the ros2 workspace, and run rosdep install -i --from-path src --rosdistro humble -y and then colcon build
4. Open a seperate terminal, and execute the bash script launch_drone_circle in the terminal (make sure that this file is marked as executable). Before running, make sure the bash script is going into the right directories and executing the right package.
6. Watch as the drone flies in a circle

In order to spawn in the world file, follow theses steps:

1. move the singleHouseWorld.sdf file to world folder in the Px4-Autopilot folder, the exact path is: /DIRECTORY_CONTAINING_PX4/PX4-Autopilot/Tools/simulation/gz/worlds
2. open the singleHouseWorld.sdf file, and edit the uri pathways to where the singleHouse.dae file is located on your system, where the comments say to change the file path.
3. if needed, change the scale of the model by editing the three numbers in the scale tag of the house model in the singleHouseWorld.sdf file.
4. edit the launch_drone_circle.sh file and uncomment the singleHouseWorld make gz command, and comment out the previous make gz command.
5. run the launch_drone_circle.sh file as before


Things added by Harry:\
Receiving messages from Gazebo and px4.

Install all the things mentioned above: 

Install gazebo ros2 bridge:\
sudo apt install ros-humble-ros-gzharmonic

Main launch file: /ros2_project/src/drone_circle_pkg.sh

1. gnome-terminal -- bash -c "export DISPLAY=:0; sleep 2; cd ~/PX4-Autopilot && PX4_GZ_WORLD=baylands make px4_sitl gz_x500_depth; exec bash": To set a different sim environment, change PX4_GZ_WORLD=$simulation environment$, currently set to baylands. Drone model is set to gz_x500_depth, output are depth images and RGB images
2. gnome-terminal -- bash -c "cd ~/drone_circle && source install/setup.bash && source ./install/setup.bash && sleep 30s && ros2 run drone_circle_pkg drone_circle_node; exec bash": launching the drone, instead of circles now it just hovers.
3. gnome-terminal -- bash -c "sleep 31s && ros2 run ros_gz_bridge parameter_bridge \
/world/baylands/model/x500_depth_0/link/camera_link/sensor/IMX214/image@sensor_msgs/msg/Image@ignition.msgs.Image \
/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo \
/depth_camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked \
/depth_camera@sensor_msgs/msg/Image@gz.msgs.Image \
/world/baylands/model/x500_depth_0/link/base_link/sensor/imu_sensor/imu@sensor_msgs/msg/Imu[gz.msgs.IMU; exec bash": publish ros topic from gazebo
4. gnome-terminal -- bash -c "ros2 run px4_odometry_converter odom_converter_node; exec bash": convert px4 odometry message to ros message

To run VINS:\
ros2 run vins vins_node /$ros_project_folder$/src/VINS-Fusion-ROS2/config/gazebo_x500/realsense_stereo_imu_config.yaml \
For easier debugging, the odometry for FUEL is passed directly from px4. VINS can be ignored for now

To run FUEL:
ros2 launch exploration_manager exploration_combined_2.launch.py\
Currently I have debugged it so that the nodes are subscribed to the correct topics, and all nodes can initialize.\
Odometry topic: /odom\
sensor_pose_topic: /pose_rotated (This topic is important, FUEL won't run without this from past experience. rotated [ 0.5, -0.5, 0.5, -0.5 ] from the odometry topic)\
depth topic: /depth_camera, cloud topic: /depth_camera/points (just set one of these, either one is good and set the other to some random topic name)

List of ROS2 topics that can be used:
/depth_camera: depth camera from drone \
/depth_camera/points: depth camera converted to pointcloud2 format\
/odom: odometry from px4, converted by odom_converter_node\
/pose_rotated: rotated odometry in the format of Posestamp, for sensor_pose_topic in FUEL\
/world/baylands/model/x500_depth_0/link/base_link/sensor/imu_sensor/imu: IMU data\
/world/baylands/model/x500_depth_0/link/camera_link/sensor/IMX214/image: RGB image (if need to change the scene, subscribers to these two topics need to be changed accordingly)


