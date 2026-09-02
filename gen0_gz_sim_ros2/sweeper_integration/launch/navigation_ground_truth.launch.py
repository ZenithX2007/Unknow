import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    nav2_launch_dir = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'launch',
    )
    default_params = PathJoinSubstitution([
        FindPackageShare('sweeper_integration'),
        'config',
        'nav2_ground_truth.yaml',
    ])
    default_map = PathJoinSubstitution([
        EnvironmentVariable('HOME'),
        'gen0_maps',
        'san_roundabout_slam.yaml',
    ])

    map_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_to_odom_x = LaunchConfiguration('map_to_odom_x')
    map_to_odom_y = LaunchConfiguration('map_to_odom_y')
    map_to_odom_yaw = LaunchConfiguration('map_to_odom_yaw')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=default_map,
            description='Full path to the occupancy map YAML file.',
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Nav2 parameters tuned for Gen0 ground-truth odometry.',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use Gazebo simulation time.',
        ),
        DeclareLaunchArgument(
            'map_to_odom_x',
            default_value='0.0',
            description='Static map -> odom x offset for ground-truth navigation.',
        ),
        DeclareLaunchArgument(
            'map_to_odom_y',
            default_value='0.0',
            description='Static map -> odom y offset for ground-truth navigation.',
        ),
        DeclareLaunchArgument(
            'map_to_odom_yaw',
            default_value='0.0',
            description='Static map -> odom yaw offset for ground-truth navigation.',
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom_tf',
            arguments=[
                '--x', map_to_odom_x,
                '--y', map_to_odom_y,
                '--z', '0',
                '--roll', '0',
                '--pitch', '0',
                '--yaw', map_to_odom_yaw,
                '--frame-id', 'map',
                '--child-frame-id', 'odom',
            ],
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[
                params_file,
                {
                    'use_sim_time': use_sim_time,
                    'yaml_filename': map_file,
                },
            ],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': ['map_server'],
            }],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_launch_dir, 'navigation_launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'autostart': 'true',
                'params_file': params_file,
                'use_composition': 'False',
            }.items(),
        ),
    ])
