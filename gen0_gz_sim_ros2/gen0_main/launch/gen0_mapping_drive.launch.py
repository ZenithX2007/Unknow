from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    enabled = LaunchConfiguration("enabled")
    start_vehicle_interface = LaunchConfiguration("start_vehicle_interface")
    drive_speed = LaunchConfiguration("drive_speed")
    stop_distance = LaunchConfiguration("stop_distance")
    slow_distance = LaunchConfiguration("slow_distance")
    safety_source = LaunchConfiguration("safety_source")
    front3d_min_z = LaunchConfiguration("front3d_min_z")
    front3d_max_z = LaunchConfiguration("front3d_max_z")
    front3d_half_width = LaunchConfiguration("front3d_half_width")
    front3d_timeout = LaunchConfiguration("front3d_timeout")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "enabled",
                default_value="false",
                description="Set true to start low-speed mapping motion.",
            ),
            DeclareLaunchArgument(
                "start_vehicle_interface",
                default_value="true",
                description="Start cmdvel_to_vehicle if no other vehicle interface is running.",
            ),
            DeclareLaunchArgument(
                "drive_speed",
                default_value="0.35",
                description="Forward speed for mapping drive in m/s.",
            ),
            DeclareLaunchArgument(
                "stop_distance",
                default_value="2.5",
                description="Stop if a front lidar obstacle is closer than this distance.",
            ),
            DeclareLaunchArgument(
                "slow_distance",
                default_value="5.0",
                description="Begin slowing down when an obstacle is closer than this distance.",
            ),
            DeclareLaunchArgument(
                "safety_source",
                default_value="front3d",
                description="Safety source: front3d or scan.",
            ),
            DeclareLaunchArgument(
                "front3d_min_z",
                default_value="-1.2",
                description="Minimum z in front_3d_lidar_link frame for obstacle checks.",
            ),
            DeclareLaunchArgument(
                "front3d_max_z",
                default_value="1.5",
                description="Maximum z in front_3d_lidar_link frame for obstacle checks.",
            ),
            DeclareLaunchArgument(
                "front3d_half_width",
                default_value="1.8",
                description="Half width of the front safety corridor in meters.",
            ),
            DeclareLaunchArgument(
                "front3d_timeout",
                default_value="2.0",
                description="Maximum age of front 3D point cloud before stopping.",
            ),
            Node(
                package="gen0_interface",
                executable="cmdvel_to_vehicle",
                name="vehicle_movement_interface",
                output="screen",
                condition=IfCondition(start_vehicle_interface),
                parameters=[
                    {
                        "cmd_vel_topic": "/control/cmd_vel",
                        "cmd_vel_timeout": 0.5,
                    }
                ],
            ),
            Node(
                package="gen0_interface",
                executable="mapping_drive",
                name="gen0_mapping_drive",
                output="screen",
                parameters=[
                    {
                        "enabled": enabled,
                        "drive_speed": drive_speed,
                        "stop_distance": stop_distance,
                        "slow_distance": slow_distance,
                        "safety_source": safety_source,
                        "front3d_min_z": front3d_min_z,
                        "front3d_max_z": front3d_max_z,
                        "front3d_half_width": front3d_half_width,
                        "front3d_timeout": front3d_timeout,
                    }
                ],
            ),
        ]
    )
