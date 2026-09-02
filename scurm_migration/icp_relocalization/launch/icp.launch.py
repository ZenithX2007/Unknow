import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def default_prior_map():
    map_path = os.path.join(
        os.path.expanduser('~'),
        'SCURM_SentryNavigation',
        'sentry_bringup',
        'maps',
        'GlobalMap.pcd',
    )
    return map_path if os.path.exists(map_path) else ''


def generate_launch_description():
    map_path = LaunchConfiguration('map_path')
    map_frame_id = LaunchConfiguration('map_frame_id')
    odom_frame_id = LaunchConfiguration('odom_frame_id')
    initial_x = LaunchConfiguration('initial_x')
    initial_y = LaunchConfiguration('initial_y')
    initial_z = LaunchConfiguration('initial_z')
    initial_a = LaunchConfiguration('initial_a')
    pcl_type = LaunchConfiguration('pcl_type')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map_path',
            default_value=default_prior_map(),
            description='PCD prior map used by ICP relocalization.',
        ),
        DeclareLaunchArgument('map_frame_id', default_value='map'),
        DeclareLaunchArgument('odom_frame_id', default_value='odom'),
        DeclareLaunchArgument('initial_x', default_value='0.0'),
        DeclareLaunchArgument('initial_y', default_value='0.0'),
        DeclareLaunchArgument('initial_z', default_value='0.0'),
        DeclareLaunchArgument('initial_a', default_value='0.0'),
        DeclareLaunchArgument(
            'pcl_type',
            default_value='livox',
            description='Use "livox" for /livox/lidar CustomMsg, anything else for PointCloud2.',
        ),
        Node(
            package='icp_relocalization',
            executable='transform_publisher',
            name='transform_publisher',
            output='screen',
            parameters=[
                {'map_frame_id': map_frame_id},
                {'odom_frame_id': odom_frame_id},
            ],
        ),
        Node(
            package='icp_relocalization',
            executable='icp_node',
            name='icp_node',
            output='screen',
            parameters=[
                {'initial_x': ParameterValue(initial_x, value_type=float)},
                {'initial_y': ParameterValue(initial_y, value_type=float)},
                {'initial_z': ParameterValue(initial_z, value_type=float)},
                {'initial_a': ParameterValue(initial_a, value_type=float)},
                {'map_voxel_leaf_size': 0.5},
                {'cloud_voxel_leaf_size': 0.3},
                {'map_frame_id': map_frame_id},
                {'solver_max_iter': 75},
                {'max_correspondence_distance': 0.1},
                {'RANSAC_outlier_rejection_threshold': 1.0},
                {'map_path': map_path},
                {'fitness_score_thre': 0.1},
                {'converged_count_thre': 50},
                {'pcl_type': pcl_type},
            ],
        )
    ])
