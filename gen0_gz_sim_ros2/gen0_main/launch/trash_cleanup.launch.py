from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "world",
                default_value="my_map",
                description="World scenario namespace for trash placement.",
            ),
            DeclareLaunchArgument(
                "trash_scenario",
                default_value="small_trash",
                description="Trash scenario JSON filename without extension.",
            ),
            DeclareLaunchArgument(
                "gazebo_world_name",
                default_value="default",
                description="Gazebo world name used by the remove service.",
            ),
            DeclareLaunchArgument(
                "odom_topic",
                default_value="/odom",
                description="Vehicle odometry topic used for trash cleanup.",
            ),
            DeclareLaunchArgument(
                "cleanup_radius",
                default_value="0.90",
                description="Remove a trash model when vehicle xy distance is within this radius.",
            ),
            DeclareLaunchArgument(
                "partition",
                default_value="gen0_scurm_demo",
                description="Gazebo transport partition used by the running simulation.",
            ),
            SetEnvironmentVariable("IGN_PARTITION", LaunchConfiguration("partition")),
            SetEnvironmentVariable("GZ_PARTITION", LaunchConfiguration("partition")),
            Node(
                package="gen0_main",
                executable="trash_cleanup_node",
                name="gen0_trash_cleanup",
                output="screen",
                parameters=[
                    {
                        "world": LaunchConfiguration("world"),
                        "trash_scenario": LaunchConfiguration("trash_scenario"),
                        "gazebo_world_name": LaunchConfiguration("gazebo_world_name"),
                        "odom_topic": LaunchConfiguration("odom_topic"),
                        "cleanup_radius": LaunchConfiguration("cleanup_radius"),
                    }
                ],
            ),
        ]
    )
