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
                "vehicle_length",
                default_value="3.50",
                description="Vehicle top-down rectangular footprint length in meters.",
            ),
            DeclareLaunchArgument(
                "vehicle_width",
                default_value="1.80",
                description="Vehicle top-down rectangular footprint width in meters.",
            ),
            DeclareLaunchArgument(
                "vehicle_center_offset_x",
                default_value="0.0",
                description="Forward x offset from odom pose to vehicle footprint center.",
            ),
            DeclareLaunchArgument(
                "vehicle_center_offset_y",
                default_value="0.0",
                description="Left y offset from odom pose to vehicle footprint center.",
            ),
            DeclareLaunchArgument(
                "coverage_margin",
                default_value="0.0",
                description="Inset applied to the vehicle footprint before coverage testing.",
            ),
            DeclareLaunchArgument(
                "use_mesh_visual_center",
                default_value="true",
                description="Use the exported mesh visual center instead of the raw model pose.",
            ),
            DeclareLaunchArgument(
                "debug_item",
                default_value="",
                description="Trash item name to log coverage diagnostics for.",
            ),
            DeclareLaunchArgument(
                "debug_period",
                default_value="1.0",
                description="Seconds between debug logs for debug_item.",
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
                        "vehicle_length": LaunchConfiguration("vehicle_length"),
                        "vehicle_width": LaunchConfiguration("vehicle_width"),
                        "vehicle_center_offset_x": LaunchConfiguration(
                            "vehicle_center_offset_x"
                        ),
                        "vehicle_center_offset_y": LaunchConfiguration(
                            "vehicle_center_offset_y"
                        ),
                        "coverage_margin": LaunchConfiguration("coverage_margin"),
                        "use_mesh_visual_center": LaunchConfiguration(
                            "use_mesh_visual_center"
                        ),
                        "debug_item": LaunchConfiguration("debug_item"),
                        "debug_period": LaunchConfiguration("debug_period"),
                    }
                ],
            ),
        ]
    )
