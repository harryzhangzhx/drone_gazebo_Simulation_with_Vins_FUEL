This repository contains a simple example of a px4 sitl communicating with ros2 to make the drone fly around in a circle automatically

Prerequisites:

1. have px4_sitl installed, following the steps outlined here (Ubuntu): https://docs.px4.io/main/en/dev_setup/dev_env_linux_ubuntu.html and https://docs.px4.io/main/en/ros2/user_guide.html
2. install the QGC, as outlined here: https://docs.px4.io/main/en/dev_setup/qgc_daily_build.html
3. install the uXFRCE-messaging system according to the steps outlined here: https://docs.px4.io/main/en/middleware/uxrce_dds.html#micro-xrce-dds-agent-installation


Steps to run this package:

1. Clone the repo in your ros2 workspace using: git clone --recursive https://github.com/harryzhangzhx/drone_gazebo_Simulation_with_Vins_FUEL.git
2. cd into the cloned folder 
3. run rosdep install -i --from-path src --rosdistro humble -y
4. colcon build
5. source install/setup.bash
6. Open a seperate terminal, and execute the bash script launch_drone_circle in the terminal (make sure that this file is marked as executable). Before running, make sure the bash script is going into the right directories and executing the right package.
7. Watch as the drone flies in a circle


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

Run the main launch file: launch_drone_circle.sh

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
