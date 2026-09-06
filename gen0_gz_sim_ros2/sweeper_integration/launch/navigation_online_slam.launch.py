import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    nav2_launch_dir = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'launch',
    )
    default_nav2_params = PathJoinSubstitution([
        FindPackageShare('sweeper_integration'),
        'config',
        'nav2_online_slam.yaml',
    ])
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

    use_sim_time = LaunchConfiguration('use_sim_time')
    nav2_params_file = LaunchConfiguration('nav2_params_file')
    slam_params_file = LaunchConfiguration('slam_params_file')
    scan_params_file = LaunchConfiguration('scan_params_file')

    nav2_lifecycle_nodes = [
        'controller_server',
        'smoother_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
        'velocity_smoother',
    ]

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use Gazebo simulation time.',
        ),
        DeclareLaunchArgument(
            'nav2_params_file',
            default_value=default_nav2_params,
            description='Nav2 parameters for online SLAM navigation.',
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
                scan_params_file,
                {'use_sim_time': use_sim_time},
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
                slam_params_file,
                {'use_sim_time': use_sim_time},
            ],
        ),
        TimerAction(
            period=2.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(nav2_launch_dir, 'navigation_launch.py')
                    ),
                    launch_arguments={
                        'use_sim_time': use_sim_time,
                        'autostart': 'false',
                        'params_file': nav2_params_file,
                        'use_composition': 'False',
                    }.items(),
                ),
            ],
        ),
        TimerAction(
            period=8.0,
            actions=[
                Node(
                    package='sweeper_integration',
                    executable='nav2_lifecycle_bringup',
                    name='nav2_lifecycle_bringup',
                    output='screen',
                    parameters=[{
                        'use_sim_time': False,
                        'node_names': nav2_lifecycle_nodes,
                        'map_topic': '/map',
                        'map_wait_timeout': 90.0,
                        'service_wait_timeout': 90.0,
                        'transition_timeout': 60.0,
                        'retry_delay': 1.0,
                    }],
                ),
            ],
        ),
    ])
