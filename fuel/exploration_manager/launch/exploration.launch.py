from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    # Launch arguments (equivalent to <let> and <arg>)
    map_size_x = LaunchConfiguration('map_size_x', default='8.0')
    map_size_y = LaunchConfiguration('map_size_y', default='24.0')
    map_size_z = LaunchConfiguration('map_size_z', default='5.0')
    init_x = LaunchConfiguration('init_x', default='0.0')
    init_y = LaunchConfiguration('init_y', default='0.0')
    init_z = LaunchConfiguration('init_z', default='1.0')
    odom_topic = LaunchConfiguration('odom_topic', default='/odom')

    # Camera intrinsics and other parameters
    cx = LaunchConfiguration('cx', default='321.04638671875')
    cy = LaunchConfiguration('cy', default='243.44969177246094')
    fx = LaunchConfiguration('fx', default='387.229248046875')
    fy = LaunchConfiguration('fy', default='387.229248046875')
    max_vel = LaunchConfiguration('max_vel', default='0.4')
    max_acc = LaunchConfiguration('max_acc', default='0.4')

    # Find package share directories
    from ament_index_python.packages import get_package_share_directory
    exploration_manager_share = get_package_share_directory('exploration_manager')
    plan_manage_share = get_package_share_directory('plan_manage')

    # Include algorithm.launch.py (you must port algorithm.xml to algorithm.launch.py for full compatibility)
    algorithm_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(exploration_manager_share, 'launch', 'algorithm.launch.py')
        ),
        launch_arguments={
            'map_size_x_': map_size_x,
            'map_size_y_': map_size_y,
            'map_size_z_': map_size_z,
            'box_min_x': '-4.0',
            'box_min_y': '-12.0',
            'box_min_z': '-0.8',
            'box_max_x': '4.0',
            'box_max_y': '12.0',
            'box_max_z': '2.0',
            'odometry_topic': odom_topic,
            'sensor_pose_topic': odom_topic,
            'depth_topic': '/world/baylands/model/x500_depth_0/link/camera_link/sensor/IMX214/image',
            'cloud_topic': 'nouse1',
            'cx': cx,
            'cy': cy,
            'fx': fx,
            'fy': fy,
            'max_vel': max_vel,
            'max_acc': max_acc
        }.items()
    )

    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument('map_size_x', default_value='8.0'),
        DeclareLaunchArgument('map_size_y', default_value='24.0'),
        DeclareLaunchArgument('map_size_z', default_value='5.0'),
        DeclareLaunchArgument('init_x', default_value='0.0'),
        DeclareLaunchArgument('init_y', default_value='0.0'),
        DeclareLaunchArgument('init_z', default_value='1.0'),
        DeclareLaunchArgument('odom_topic', default_value='/odom'),

        # Camera intrinsics and velocity/acceleration
        DeclareLaunchArgument('cx', default_value='321.04638671875'),
        DeclareLaunchArgument('cy', default_value='243.44969177246094'),
        DeclareLaunchArgument('fx', default_value='387.229248046875'),
        DeclareLaunchArgument('fy', default_value='387.229248046875'),
        DeclareLaunchArgument('max_vel', default_value='0.4'),
        DeclareLaunchArgument('max_acc', default_value='0.4'),

        # Include the algorithm launch file
        algorithm_launch,

        # traj_server node
        Node(
            package='plan_manage',
            executable='traj_server',
            name='traj_server',
            output='screen',
            remappings=[
                ('/odom_world', odom_topic)
            ],
            parameters=[{
                'traj_server/time_forward': 1.5,
                'traj_server/pub_traj_id': 4,
                'traj_server/init_x': init_x,
                'traj_server/init_y': init_y,
                'traj_server/init_z': init_z,
                'perception_utils/top_angle': 0.56125,
                'perception_utils/left_angle': 0.69222,
                'perception_utils/right_angle': 0.68901,
                'perception_utils/max_dist': 4.5,
                'perception_utils/vis_dist': 1.0,
                'manager/max_jerk': 4.0,
                # 'fsm/thresh_replan1': 0.5,
                # 'fsm/thresh_replan2': 0.5,
                # 'fsm/thresh_replan3': 1.5,
                # 'fsm/replan_time': 0.2,
            }]
        ),

        # waypoint_generator node
        Node(
            package='waypoint_generator',
            executable='waypoint_generator',
            name='waypoint_generator',
            output='screen',
            remappings=[
                ('odom', odom_topic),
                ('goal', '/move_base_simple/goal'),
                ('traj_start_trigger', '/traj_start_trigger')
            ],
            parameters=[{
                'waypoint_type': 'point'
            }]
        ),

        # rviz2 node
        Node(
            package='rviz2',
            executable='rviz2',
            name='rvizvisualisation',
            output='log',
            arguments=['-d', os.path.join(plan_manage_share, 'config', 'traj.rviz')]
        ),

        # static_transform_publisher node
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_53',
            arguments=['0', '0', '0', '0', '0', '0', '1', 'world', 'navigation']
        ),
    ])