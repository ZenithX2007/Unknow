import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = get_package_share_directory("gen0_main")
    default_params = os.path.join(package_share, "config", "fast_lio_gen0.yaml")
    default_rviz = os.path.join(package_share, "config", "gen0_3d_mapping.rviz")
    default_costmap_params = os.path.join(
        package_share, "config", "scurm_local_costmap_gen0.yaml"
    )
    default_world_obj = os.path.join(
        package_share, "worlds", "san_roundabout", "san_roundabout.obj"
    )
    rviz_launch = os.path.join(package_share, "launch", "gen0_3d_rviz.launch.py")

    params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    odom_output_topic = LaunchConfiguration("odom_output_topic")
    rviz = LaunchConfiguration("rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    terrain_analysis = LaunchConfiguration("terrain_analysis")
    local_costmap = LaunchConfiguration("local_costmap")
    costmap_params_file = LaunchConfiguration("costmap_params_file")
    simulated_lidar = LaunchConfiguration("simulated_lidar")
    world_obj_path = LaunchConfiguration("world_obj_path")
    simulated_lidar_max_points = LaunchConfiguration("simulated_lidar_max_points")
    simulated_lidar_max_range = LaunchConfiguration("simulated_lidar_max_range")
    simulated_lidar_horizontal_min_angle = LaunchConfiguration(
        "simulated_lidar_horizontal_min_angle"
    )
    simulated_lidar_horizontal_max_angle = LaunchConfiguration(
        "simulated_lidar_horizontal_max_angle"
    )
    simulated_lidar_vertical_min_angle = LaunchConfiguration(
        "simulated_lidar_vertical_min_angle"
    )
    simulated_lidar_vertical_max_angle = LaunchConfiguration(
        "simulated_lidar_vertical_max_angle"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="FAST_LIO parameter file adapted for the gen0 Gazebo sensors.",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use Gazebo simulation clock.",
            ),
            DeclareLaunchArgument(
                "odom_output_topic",
                default_value="/gen0_mapping/fast_lio/odom",
                description="FAST_LIO odometry output topic. Use /odom only when ground-truth odom is disabled.",
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="false",
                description="Open RViz with 3D mapping displays.",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=default_rviz,
                description="RViz config for 3D mapping verification.",
            ),
            DeclareLaunchArgument(
                "terrain_analysis",
                default_value="true",
                description="Run SCURM terrain_analysis on FAST_LIO2 registered scans.",
            ),
            DeclareLaunchArgument(
                "local_costmap",
                default_value="true",
                description="Run SCURM costmap_intensity local costmap from terrain_map.",
            ),
            DeclareLaunchArgument(
                "costmap_params_file",
                default_value=default_costmap_params,
                description="SCURM local costmap parameters adapted for gen0.",
            ),
            DeclareLaunchArgument(
                "simulated_lidar",
                default_value="true",
                description="Publish a world-mesh lidar point cloud when Gazebo GPU lidar has no valid depth returns.",
            ),
            DeclareLaunchArgument(
                "world_obj_path",
                default_value=default_world_obj,
                description="World OBJ used by the simulated lidar fallback.",
            ),
            DeclareLaunchArgument(
                "simulated_lidar_max_points",
                default_value="8000",
                description="Maximum points per simulated world-lidar scan.",
            ),
            DeclareLaunchArgument(
                "simulated_lidar_max_range",
                default_value="25.0",
                description="Maximum range for the simulated world-lidar fallback.",
            ),
            DeclareLaunchArgument(
                "simulated_lidar_horizontal_min_angle",
                default_value="-1.5707",
                description="Minimum horizontal angle for simulated world-lidar points.",
            ),
            DeclareLaunchArgument(
                "simulated_lidar_horizontal_max_angle",
                default_value="1.5707",
                description="Maximum horizontal angle for simulated world-lidar points.",
            ),
            DeclareLaunchArgument(
                "simulated_lidar_vertical_min_angle",
                default_value="-0.3926991",
                description="Minimum vertical angle for simulated world-lidar points.",
            ),
            DeclareLaunchArgument(
                "simulated_lidar_vertical_max_angle",
                default_value="0.3926991",
                description="Maximum vertical angle for simulated world-lidar points.",
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
                        "output_topic": "/gen0_model/front3d/lidar/points",
                        "pose_topic": "/gen0_model/links/poses",
                        "pose_index": 15,
                        "frame_id": "front_3d_lidar_link",
                        "scan_rate": 10.0,
                        "max_points": ParameterValue(
                            simulated_lidar_max_points, value_type=int
                        ),
                        "world_voxel_size": 0.25,
                        "lidar_xyz_in_base": [1.9, 0.0, 1.9],
                        "min_range": 0.6,
                        "max_range": ParameterValue(
                            simulated_lidar_max_range, value_type=float
                        ),
                        "horizontal_min_angle": ParameterValue(
                            simulated_lidar_horizontal_min_angle, value_type=float
                        ),
                        "horizontal_max_angle": ParameterValue(
                            simulated_lidar_horizontal_max_angle, value_type=float
                        ),
                        "vertical_min_angle": ParameterValue(
                            simulated_lidar_vertical_min_angle, value_type=float
                        ),
                        "vertical_max_angle": ParameterValue(
                            simulated_lidar_vertical_max_angle, value_type=float
                        ),
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
                executable="gazebo_livox_adapter",
                name="gen0_gazebo_livox_adapter",
                output="screen",
                parameters=[
                    {
                        "input_topic": "/gen0_model/front3d/lidar/points",
                        "output_topic": "/livox/lidar",
                        "scan_rate": 10.0,
                        "line_count": 64,
                        "vertical_min_angle": -1.2,
                        "vertical_max_angle": 0.8,
                        "max_points": 20000,
                        "min_range": 0.5,
                        "max_range": 80.0,
                        "self_filter_enabled": True,
                        "lidar_xyz_in_base": [1.9, 0.0, 1.9],
                        "self_filter_min_xyz": [-2.8, -1.4, -0.4],
                        "self_filter_max_xyz": [2.8, 1.4, 2.9],
                    }
                ],
            ),
            Node(
                package="fast_lio",
                executable="fastlio_mapping",
                name="gen0_fast_lio",
                output="screen",
                parameters=[params_file, {"use_sim_time": use_sim_time}],
                remappings=[
                    ("/Odometry", odom_output_topic),
                    ("/path", "/gen0_mapping/fast_lio/path"),
                    ("/cloud_registered", "/gen0_mapping/cloud_registered"),
                    ("/cloud_registered_body", "/gen0_mapping/cloud_registered_body"),
                    ("/cloud_effected", "/gen0_mapping/cloud_effected"),
                    ("/Laser_map", "/gen0_mapping/fast_lio_map"),
                    ("/ikd_tree", "/gen0_mapping/ikd_tree"),
                    ("/map_save", "/gen0_mapping/save_fast_lio_map"),
                ],
            ),
            Node(
                package="terrain_analysis",
                executable="terrainAnalysis",
                name="gen0_scurm_terrain_analysis",
                output="screen",
                condition=IfCondition(terrain_analysis),
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "map_frame": "odom",
                        "scanVoxelSize": 0.08,
                        "decayTime": 0.5,
                        "noDecayDis": 0.0,
                        "clearingDis": 0.0,
                        "useSorting": True,
                        "quantileZ": 0.4,
                        "considerDrop": False,
                        "limitGroundLift": False,
                        "maxGroundLift": 0.25,
                        "clearDyObs": True,
                        "minDyObsDis": 0.1,
                        "minDyObsAngle": 0.0,
                        "minDyObsRelZ": -0.5,
                        "minDyObsVFOV": -16.0,
                        "maxDyObsVFOV": 16.0,
                        "minDyObsPointNum": 10,
                        "noDataObstacle": False,
                        "noDataBlockSkipNum": 0,
                        "minBlockPointNum": 1,
                        # Gen0 footprint from gen0_model.sdf, with margin for wheels/body.
                        "vehicleHeight": 2.8,
                        "sensorOffsetX": 0.0,
                        "sensorOffsetY": 0.0,
                        "vehicleLength": 5.8,
                        "vehicleWidth": 3.0,
                        "voxelPointUpdateThre": 20,
                        "voxelTimeUpdateThre": 0.5,
                        "minRelZ": -2.5,
                        "maxRelZ": 2.8,
                        "disRatioZ": 0.2,
                    }
                ],
                remappings=[
                    ("/state_estimation", odom_output_topic),
                    ("/registered_scan", "/gen0_mapping/cloud_registered"),
                    ("/terrain_map", "/gen0_mapping/terrain_map"),
                ],
            ),
            TimerAction(
                period=4.0,
                actions=[
                    Node(
                        package="nav2_costmap_2d",
                        executable="nav2_costmap_2d",
                        namespace="costmap",
                        name="costmap",
                        output="screen",
                        condition=IfCondition(local_costmap),
                        parameters=[costmap_params_file, {"use_sim_time": use_sim_time}],
                    ),
                    Node(
                        package="nav2_lifecycle_manager",
                        executable="lifecycle_manager",
                        name="lifecycle_manager_costmap",
                        output="screen",
                        condition=IfCondition(local_costmap),
                        parameters=[
                            {"use_sim_time": use_sim_time},
                            {"autostart": True},
                            {"node_names": ["/costmap/costmap"]},
                        ],
                    ),
                ],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(rviz_launch),
                condition=IfCondition(rviz),
                launch_arguments={
                    "rviz": rviz,
                    "rviz_config": rviz_config,
                    "use_sim_time": use_sim_time,
                }.items(),
            ),
        ]
    )
