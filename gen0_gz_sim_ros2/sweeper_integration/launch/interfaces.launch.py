from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def static_transform(
        name, parent_frame, child_frame, x, y, z,
        roll=0.0, pitch=0.0, yaw=0.0):
    return Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name=name,
        arguments=[
            '--x', str(x), '--y', str(y), '--z', str(z),
            '--roll', str(roll), '--pitch', str(pitch), '--yaw', str(yaw),
            '--frame-id', parent_frame,
            '--child-frame-id', child_frame,
        ],
        parameters=[{'use_sim_time': True}],
    )


def generate_launch_description():
    config = PathJoinSubstitution([
        FindPackageShare('sweeper_integration'),
        'config',
        'interfaces.yaml',
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'ground_truth_odometry', default_value='true',
            choices=['true', 'false']),
        Node(
            package='sweeper_integration',
            executable='ground_truth_odometry',
            name='ground_truth_odometry',
            output='screen',
            parameters=[config, {'use_sim_time': True}],
            condition=IfCondition(LaunchConfiguration('ground_truth_odometry')),
        ),
        static_transform(
            'base_footprint_to_base_link',
            'base_footprint', 'base_link', 0.0, 0.0, 0.0),
        static_transform(
            'front_left_lidar_tf',
            'base_link', 'fl_2d_lidar_link', 2.45, 1.05, 1.05),
        static_transform(
            'front_right_lidar_tf',
            'base_link', 'fr_2d_lidar_link', 2.45, -1.05, 1.05),
        static_transform(
            'front_3d_lidar_tf',
            'base_link', 'front_3d_lidar_link', 1.9, 0.0, 1.9),
        static_transform(
            'front_camera_tf',
            'base_link', 'gen0_model/front_camera_link/front_camera',
            1.9, 0.0, 2.0, 0.0, 0.849, 0.0),
        static_transform(
            'driver_camera_tf',
            'base_link', 'gen0_model/driver_camera_link/driver_camera',
            2.05, 0.0, 2.25, 0.0, 0.10, 0.0),
        static_transform(
            'imu_tf',
            'base_link', 'gen0_model/imu_link/imu_sensor', 0.0, 0.0, 0.0),
        static_transform(
            'gps_tf',
            'base_link', 'gps_link', -0.25, 0.0, 3.0),
    ])
