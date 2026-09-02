from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_slam_params = PathJoinSubstitution([
        FindPackageShare('sweeper_integration'),
        'config',
        'gen0_slam.yaml',
    ])
    default_scan_params = PathJoinSubstitution([
        FindPackageShare('sweeper_integration'),
        'config',
        'pointcloud_to_laserscan.yaml',
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock from Gazebo.',
        ),
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=default_slam_params,
            description='slam_toolbox parameter file.',
        ),
        DeclareLaunchArgument(
            'scan_params_file',
            default_value=default_scan_params,
            description='pointcloud_to_laserscan parameter file.',
        ),
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            output='screen',
            parameters=[
                LaunchConfiguration('scan_params_file'),
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
            ],
            remappings=[
                ('cloud_in', '/gen0_model/front3d/lidar/points'),
                ('scan', '/scan'),
            ],
        ),
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                LaunchConfiguration('slam_params_file'),
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
            ],
        ),
    ])
