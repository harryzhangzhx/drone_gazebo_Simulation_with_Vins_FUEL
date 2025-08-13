# !/bin/bash

# # --- PARAMETERS (If relative paths don't work, replace the below with absolute paths) ---
# # the top-level folder of this project:
# export SETUP_PATH="../../install/setup.bash"
# # command to execute the QGroundContorl AppImage:
# export Q_GROUND_CONTROL_CMD="../../../QGroundControl-x86_64.AppImage"
# # the following parameter should be "MicroXRCEAgent" if built from source and
# # "micro-xrce-dds-agent" if installed from the snap package
# export MICRO_XRCE_DDS_AGENT_CMD="MicroXRCEAgent"
# # location of the PX4_Autopilot folder:
# export PX4_AUTOPILOT_DIR="../../../PX4-Autopilot"

# Check the path
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
export SETUP_PATH="$(realpath -m "$SCRIPT_DIR/../../install/setup.bash")"
export Q_GROUND_CONTROL_CMD="$(realpath -m "$SCRIPT_DIR/../../../QGroundControl-x86_64.AppImage")"
export PX4_AUTOPILOT_DIR="$(realpath -m "$SCRIPT_DIR/../../../PX4-Autopilot")"
export MICRO_XRCE_DDS_AGENT_CMD="${MICRO_XRCE_DDS_AGENT_CMD:-$(command -v micro-xrce-dds-agent || command -v MicroXRCEAgent || echo MicroXRCEAgent)}"

# export ROS_DOMAIN_ID=0

# Terminal 0: main terminal
# kill previous Gazebo
source $SETUP_PATH
pkill -f gz

# Terminal 1: PX4 SITL
# To set a different sim environment, change PX4_GZ_WORLD=$simulation environment$, currently set to baylands.
# Drone model is set to gz_x500_depth, output are depth images and RGB images
gnome-terminal -- bash -c "source $SETUP_PATH; sleep 2; cd $PX4_AUTOPILOT_DIR && PX4_GZ_WORLD=baylands make px4_sitl gz_x500_depth; exec bash"

# Terminal 2: MicroXRCEAgent
gnome-terminal -- bash -c "$MICRO_XRCE_DDS_AGENT_CMD udp4 -p 8888; exec bash"

# Terminal 3: QGroundControl
gnome-terminal -- bash -c "$Q_GROUND_CONTROL_CMD; exec bash"

# # Terminal 4: ROS 2 Node
# # modify the file "drone_circle_node.cpp" to switch between hovering motion and circling motion
# gnome-terminal -- bash -c "source $SETUP_PATH && sleep 30s && ros2 run drone_circle_pkg drone_circle_node; exec bash"

# Terminal 5: ROS 2 Parameter Bridge
gnome-terminal -- bash -c "source $SETUP_PATH && sleep 31s && ros2 run ros_gz_bridge parameter_bridge \
/world/baylands/model/x500_depth_0/link/camera_link/sensor/IMX214/image@sensor_msgs/msg/Image@ignition.msgs.Image \
/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo \
/depth_camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked \
/depth_camera@sensor_msgs/msg/Image@gz.msgs.Image \
/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock \
/world/baylands/model/x500_depth_0/link/base_link/sensor/imu_sensor/imu@sensor_msgs/msg/Imu[gz.msgs.IMU; exec bash"

# Terminal 6: Odometry Converter Node
# convert px4 odometry message to ros message
gnome-terminal -- bash -c "source $SETUP_PATH && ros2 run px4_odometry_converter odom_converter_node; exec bash"
