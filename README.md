This repository contains a simple example of a PX4 software-in-the-loop (SITL) communicating with ROS 2 to make the drone fly around in a circle automatically.


# Prerequisites

1. Install `px4_sitl` and the Micro XRCE-DDS Agent following the steps outlined in “[ROS 2 User Guide](https://docs.px4.io/main/en/ros2/user_guide.html).”
2. Install QGroundControl, as outlined in “[QGroundControl Daily Build](https://docs.px4.io/main/en/dev_setup/qgc_daily_build.html).”


# Steps to run this package

## Basic running

1. Clone the repo in your ROS 2 workspace using

    ```sh
    git clone --recursive https://github.com/harryzhangzhx/drone_gazebo_Simulation_with_Vins_FUEL.git
    ```

2. `cd` into the cloned folder.

3. Run
    ```sh
    rosdep install -i --from-path src --rosdistro humble -y
    ```
    to install dependencies.

4. Run `colcon build`.

5. Run `source install/setup.bash`. You can add this to your `.bashrc` file so that it will be ran every time you open a new terminal:

    ```sh
    echo "source <cloned-folder>/install/setup.bash" >> ~/.bashrc
    ```

6. Enter into the containing directory of `exploration_combined_2.launch.py` and execute `ros2 launch exploration_combined_2.launch.py`.

    > **Attention:** *Before running, make sure the all paths in the files of `launch_drone_circle.sh` and `exploration_combined_2.launch.py` are correct. Search "Check the path" comment to find the location of the paths. Update the paths if the dependencies are installed in different locations. Also, if `libcasadi` cannot be found, create a symbolic link or copy of the library in `/usr/lib` (it is normally installed under `/usr/local`).*

7. Watch as the drone flies.
    
   > **Attention:** *If the drone doesn't fly, there are two possibilities. First, the z cooridnate command is upside down (PX4's z axis is pointing down); Second, the PX4 is not armed (triggered), use the command `commander mode offboard` and `commander arm` in the terminal running PX4 to trigger the PX4.*

## Spawning the world file

1. Move the `singleHouseWorld.sdf` file to world folder in the `PX4-Autopilot` folder, i.e., `PX4-Autopilot/Tools/simulation/gz/worlds`.

2. Open the `singleHouseWorld.sdf` file with a text editor, and edit the URI pathways to where the `singleHouse.dae` file is located on your system, where the comments say to change the file path.

3. If needed, change the scale of the model by editing the three numbers in the scale tag of the house model in the `singleHouseWorld.sdf` file.

4. Edit the `launch_drone_circle.sh` file and uncomment the `singleHouseWorld make gz` command, and comment out the previous `make gz` command.

5. Run the `launch_drone_circle.sh` file as before.

## Receiving messages from Gazebo and PX4

1. Install all the things mentioned above.

2. Install Gazebo-ROS 2 bridge:

    ```sh
    sudo apt install ros-humble-ros-gzharmonic
    sudo apt install ros-humble-ros-gz-bridge
    ```

3. Run `launch_drone_circle.sh` as usual.

## Running VINS

```sh
ros2 run vins vins_node <ROS-workspace>/src/VINS-Fusion-ROS2/config/gazebo_x500/realsense_stereo_imu_config.yaml
```

For easier debugging, the odometry for FUEL is passed directly from PX4. VINS can be ignored for now.

## Running FUEL

```sh
ros2 launch exploration_manager exploration_combined_2.launch.py
```

Currently I have debugged it so that the nodes are subscribed to the correct topics, and all nodes can initialize.

# List of ROS 2 topics that can be used

- `/depth_camera`: depth camera from drone.

- `/depth_camera/points`: depth camera converted to `pointcloud2` format.

- `/odom`: odometry from PX4, converted by `odom_converter_node`.

- `/pose_rotated`: rotated odometry in the format of Posestamp, for `sensor_pose_topic` in FUEL.

- `/world/baylands/model/x500_depth_0/link/base_link/sensor/imu_sensor/imu`: IMU data.

- `/world/baylands/model/x500_depth_0/link/camera_link/sensor/IMX214/image`: RGB image. If it is needed to change the scene, subscribers to these two topics need to be changed accordingly.

- `/odom`: odometry topic.

- `/pose_rotated`: `sensor_pose_topic`. This topic is important as FUEL won't run without this from past experience. This is rotated by $[0.5, -0.5, 0.5, -0.5]$ from the odometry topic.

- `/depth_camera`: depth topic.

- `/depth_camera/points`: cloud topic (just set one of these, either one is good and set the other to some random topic name).

# Updates July 20th

- The PX4 odometry converter now publishes to a ROS 2 topic in ROS coordinates.

- A clock topic is outputted from Gazebo.

    > **Note:** *make sure that all ROS time follows this timestamp.*

- The FUEL pipeline is mostly debugged. To trigger the FUEL algorithm, use the same method as the original repo by clicking a navigation goal on RViz. 