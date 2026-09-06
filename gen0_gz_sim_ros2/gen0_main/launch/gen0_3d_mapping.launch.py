import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("gen0_main")
    default_params = os.path.join(package_share, "config", "gen0_3d_mapper.yaml")
    default_world_obj = os.path.join(
        package_share, "worlds", "san_roundabout", "san_roundabout.obj"
    )

    params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    simulated_lidar = LaunchConfiguration("simulated_lidar")
    front3d_source_topic = LaunchConfiguration("front3d_source_topic")
    simulated_lidar_output_topic = LaunchConfiguration("simulated_lidar_output_topic")
    world_obj_path = LaunchConfiguration("world_obj_path")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="Parameter file for gen0_3d_mapper",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use Gazebo simulation clock",
            ),
            DeclareLaunchArgument(
                "simulated_lidar",
                default_value="true",
                description="Publish a world-mesh lidar point cloud for mapping.",
            ),
            DeclareLaunchArgument(
                "front3d_source_topic",
                default_value="/gen0_mapping/simulated_front3d/lidar/points",
                description="PointCloud2 topic consumed by gen0_3d_mapper.",
            ),
            DeclareLaunchArgument(
                "simulated_lidar_output_topic",
                default_value="/gen0_mapping/simulated_front3d/lidar/points",
                description="Output topic for the OBJ-based simulated LiDAR fallback.",
            ),
            DeclareLaunchArgument(
                "world_obj_path",
                default_value=default_world_obj,
                description="World OBJ used by the simulated lidar source.",
            ),
            Node(
                package="gen0_main",
                executable="simulated_world_lidar",
                name="gen0_simulated_world_lidar",
                output="screen",
                condition=IfCondition(simulated_lidar),
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "world_obj_path": world_obj_path,
                        "output_topic": simulated_lidar_output_topic,
                        "pose_topic": "/gen0_model/links/poses",
                        "pose_index": 15,
                        "frame_id": "front_3d_lidar_link",
                        "scan_rate": 10.0,
                        "max_points": 24000,
                        "world_voxel_size": 0.25,
                        "lidar_xyz_in_base": [1.9, 0.0, 1.9],
                        "min_range": 0.6,
                        "max_range": 80.0,
                        "vertical_min_angle": -1.2,
                        "vertical_max_angle": 0.8,
                        "priority_sampling_enabled": True,
                        "priority_range": 12.0,
                        "priority_min_base_z": -2.5,
                        "priority_max_base_z": 2.8,
                        "self_filter_enabled": True,
                        "self_filter_min_xyz": [-2.8, -1.4, -0.4],
                        "self_filter_max_xyz": [2.8, 1.4, 2.9],
                    }
                ],
            ),
            Node(
                package="gen0_main",
                executable="gen0_3d_mapper",
                name="gen0_3d_mapper",
                output="screen",
                parameters=[
                    params_file,
                    {
                        "use_sim_time": use_sim_time,
                        "cloud_topic": front3d_source_topic,
                    },
                ],
            ),
        ]
    )
