from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def launch_setup(context, *args, **kwargs):
    def lc(name): return LaunchConfiguration(name).perform(context)

    exploration_manager_share = get_package_share_directory('exploration_manager')
    plan_manage_share = get_package_share_directory('plan_manage')

    params = {
        'map_ros/cx': float(lc('cx')),
        'map_ros/cy': float(lc('cy')),
        'map_ros/fx': float(lc('fx')),
        'map_ros/fy': float(lc('fy')),
        'map_ros/depth_filter_maxdist': 5.0,
        'map_ros/depth_filter_mindist': 0.2,
        'map_ros/depth_filter_margin': 2,
        'map_ros/k_depth_scaling_factor': 1000.0,
        'map_ros/skip_pixel': 2,
        'map_ros/esdf_slice_height': 0.3,
        'map_ros/visualization_truncate_height': 10.09,
        'map_ros/visualization_truncate_low': -2.0,
        'map_ros/show_occ_time': False,
        'map_ros/show_esdf_time': False,
        'map_ros/show_all_map': True,
        'map_ros/frame_id': 'world',

        'sdf_map/resolution': 0.15,
        'sdf_map/map_size_x': float(lc('map_size_x_')),
        'sdf_map/map_size_y': float(lc('map_size_y_')),
        'sdf_map/map_size_z': float(lc('map_size_z_')),
        'sdf_map/obstacles_inflation': 0.15,
        'sdf_map/local_bound_inflate': 0.2,
        'sdf_map/local_map_mvarin': 50,
        'sdf_map/ground_height': -1.0,
        'sdf_map/default_dist': 0.0,
        'sdf_map/p_hit': 0.65,
        'sdf_map/p_miss': 0.35,
        'sdf_map/p_min': 0.12,
        'sdf_map/p_max': 0.90,
        'sdf_map/p_occ': 0.80,
        'sdf_map/min_ray_length': 0.5,
        'sdf_map/max_ray_length': 4.5,
        'sdf_map/virtual_ceil_height': -10.0,
        'sdf_map/optimistic': False,
        'sdf_map/signed_dist': False,
        'sdf_map/box_min_x': float(lc('box_min_x')),
        'sdf_map/box_min_y': float(lc('box_min_y')),
        'sdf_map/box_min_z': float(lc('box_min_z')),
        'sdf_map/box_max_x': float(lc('box_max_x')),
        'sdf_map/box_max_y': float(lc('box_max_y')),
        'sdf_map/box_max_z': float(lc('box_max_z')),

        'exploration/refine_local': True,
        'exploration/refined_num': 7,
        'exploration/refined_radius': 5.0,
        'exploration/max_decay': 0.8,
        'exploration/top_view_num': 15,
        'exploration/vm': float(lc('max_vel')),
        'exploration/am': float(lc('max_acc')),
        'exploration/yd': 3.1415926 * 60 / 180.0,
        'exploration/ydd': 3.1415926 * 90 / 180.0,
        'exploration/w_dir': 1.5,
        'exploration/tsp_dir': "/home/harry/drone_circle/src/fuel/src/utils/lkh_tsp_solver/resource",
        'exploration/relax_time': 1.0,
        'cmu_exploration': False,
        'frontier/cluster_min': 100,
        'frontier/cluster_size_xy': 2.0,
        'frontier/cluster_size_z': 10.0,
        'frontier/min_candidate_dist': 0.75,
        'frontier/min_candidate_clearance': 0.21,
        'frontier/candidate_dphi': 3.1415926 * 15 / 180.0,
        'frontier/candidate_rnum': 3,
        'frontier/candidate_rmin': 1.5,
        'frontier/candidate_rmax': 2.5,
        'frontier/down_sample': 3,
        'frontier/min_visib_num': 15,
        'frontier/min_view_finish_fraction': 0.2,

        'perception_utils/top_angle': 0.56125,
        'perception_utils/left_angle': 0.69222,
        'perception_utils/right_angle': 0.68901,
        'perception_utils/max_dist': 4.5,
        'perception_utils/vis_dist': 1.0,

        'heading_planner/yaw_diff': 3.1415926 * 30 / 180.0,
        'heading_planner/half_vert_num': 5,
        'heading_planner/lambda1': 2.0,
        'heading_planner/lambda2': 1.0,
        'heading_planner/max_yaw_rate': 3.1415926 * 20 / 180.0,
        'heading_planner/w': 20000.0,
        'heading_planner/weight_type': 1,

        'manager/max_vel': float(lc('max_vel')),
        'manager/max_acc': float(lc('max_acc')),
        'manager/max_jerk': 4.0,
        'manager/dynamic_environment': 0,
        'manager/local_segment_length': 6.0,
        'manager/clearance_threshold': 0.2,
        'manager/control_points_distance': 0.35,
        'manager/use_geometric_path': True,
        'manager/use_kinodynamic_path': True,
        'manager/use_topo_path': True,
        'manager/use_optimization': True,
        'manager/use_active_perception': True,
        'manager/min_time': True,
        'manager/bspline_degree': 3,

        'search/max_tau': 0.8,
        'search/init_max_tau': 1.0,
        'search/max_vel': float(lc('max_vel')),
        'search/vel_margin': 0.25,
        'search/max_acc': float(lc('max_acc')),
        'search/w_time': 10.0,
        'search/horizon': 5.0,
        'search/lambda_heu': 10.0,
        'search/resolution_astar': 0.025,
        'search/time_resolution': 0.8,
        'search/margin': 0.2,
        'search/allocate_num': 100000,
        'search/check_num': 10,
        'search/optimistic': False,

        'astar/lambda_heu': 10000.0,
        'astar/resolution_astar': 0.2,
        'astar/allocate_num': 1000000,
        'astar/max_search_time': 0.001,

        'optimization/ld_smooth': 20.0,
        'optimization/ld_dist': 10.0,
        'optimization/ld_feasi': 2.0,
        'optimization/ld_start': 100.0,
        'optimization/ld_end': 0.5,
        'optimization/ld_guide': 1.5,
        'optimization/ld_waypt': 0.3,
        'optimization/ld_view': 0.0,
        'optimization/ld_time': 1.0,
        'optimization/dist0': 0.7,
        'optimization/max_vel': float(lc('max_vel')),
        'optimization/max_acc': float(lc('max_acc')),
        'optimization/algorithm1': 15,
        'optimization/algorithm2': 11,
        'optimization/max_iteration_num1': 2,
        'optimization/max_iteration_num2': 2000,
        'optimization/max_iteration_num3': 200,
        'optimization/max_iteration_num4': 200,
        'optimization/max_iteration_time1': 0.0001,
        'optimization/max_iteration_time2': 0.005,
        'optimization/max_iteration_time3': 0.003,
        'optimization/max_iteration_time4': 0.003,
        'bspline/limit_vel': float(lc('max_vel')),
        'bspline/limit_acc': float(lc('max_acc')),
        'bspline/limit_ratio': 1.1,

        'fsm/thresh_replan1': 0.5,
        'fsm/thresh_replan2': 0.5,
        'fsm/thresh_replan3': 1.5,
        'fsm/replan_time': 0.2,

        'cmu_exploration': False
    }

    return [
        Node(
            package='exploration_manager',
            executable='exploration_node',
            name='exploration_node',
            output='screen',
            remappings=[
                ('/odom_world', lc('odometry_topic')),
                ('/map_ros/pose', lc('sensor_pose_topic')),
                ('/map_ros/depth', lc('depth_topic')),
                ('/map_ros/cloud', lc('cloud_topic')),
            ],
            parameters=[params]
        ),
        Node(
            package='plan_manage',
            executable='traj_server',
            name='traj_server',
            output='screen',
            remappings=[
                ('/odom_world', lc('odometry_topic'))
            ],
            parameters=[{
                'traj_server/time_forward': 1.5,
                'traj_server/pub_traj_id': 4,
                'traj_server/init_x': float(lc('init_x')),
                'traj_server/init_y': float(lc('init_y')),
                'traj_server/init_z': float(lc('init_z')),
                'perception_utils/top_angle': 0.56125,
                'perception_utils/left_angle': 0.69222,
                'perception_utils/right_angle': 0.68901,
                'perception_utils/max_dist': 4.5,
                'perception_utils/vis_dist': 1.0,
            }]
        ),
        Node(
            package='waypoint_generator',
            executable='waypoint_generator',
            name='waypoint_generator',
            output='screen',
            remappings=[
                ('odom', lc('odometry_topic')),
                ('goal', '/move_base_simple/goal'),
                ('traj_start_trigger', '/traj_start_trigger')
            ],
            parameters=[{'waypoint_type': 'point'}]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rvizvisualisation',
            output='log',
            arguments=['-d', '/home/harry/drone_circle/src/fuel/src/exploration_manager/config/ros2_fuel.rviz']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_world_odom',
            output='screen',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--roll', '0', '--pitch', '0', '--yaw', '0',
                '--frame-id', 'world',
                '--child-frame-id', 'navigation'
            ]
        )
    ]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('map_size_x_', default_value='8.0'),
        DeclareLaunchArgument('map_size_y_', default_value='24.0'),
        DeclareLaunchArgument('map_size_z_', default_value='5.0'),
        DeclareLaunchArgument('box_min_x', default_value='-4.0'),
        DeclareLaunchArgument('box_min_y', default_value='-12.0'),
        DeclareLaunchArgument('box_min_z', default_value='-0.8'),
        DeclareLaunchArgument('box_max_x', default_value='4.0'),
        DeclareLaunchArgument('box_max_y', default_value='12.0'),
        DeclareLaunchArgument('box_max_z', default_value='2.0'),
        DeclareLaunchArgument('odometry_topic', default_value='/odom'),
        DeclareLaunchArgument('sensor_pose_topic', default_value='/odom'),
        DeclareLaunchArgument('depth_topic', default_value='/depth_camera'),
        DeclareLaunchArgument('cloud_topic', default_value='/depth_camera/points1'),
        DeclareLaunchArgument('cx', default_value='321.04638671875'),
        DeclareLaunchArgument('cy', default_value='243.44969177246094'),
        DeclareLaunchArgument('fx', default_value='387.229248046875'),
        DeclareLaunchArgument('fy', default_value='387.229248046875'),
        DeclareLaunchArgument('max_vel', default_value='0.4'),
        DeclareLaunchArgument('max_acc', default_value='0.4'),
        DeclareLaunchArgument('init_x', default_value='0.0'),
        DeclareLaunchArgument('init_y', default_value='0.0'),
        DeclareLaunchArgument('init_z', default_value='1.0'),
        OpaqueFunction(function=launch_setup)
    ])