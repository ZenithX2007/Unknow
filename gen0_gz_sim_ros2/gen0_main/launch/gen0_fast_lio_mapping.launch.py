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


def default_prior_map():
    map_path = os.path.join(
        os.path.expanduser("~"),
        "SCURM_SentryNavigation",
        "sentry_bringup",
        "maps",
        "GlobalMap.pcd",
    )
    return map_path if os.path.exists(map_path) else ""


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
    front3d_source_topic = LaunchConfiguration("front3d_source_topic")
    simulated_lidar_output_topic = LaunchConfiguration("simulated_lidar_output_topic")
    fast_lio_map_file_path = LaunchConfiguration("fast_lio_map_file_path")
    fast_lio_pcd_save = LaunchConfiguration("fast_lio_pcd_save")
    fast_lio_pcd_save_interval = LaunchConfiguration("fast_lio_pcd_save_interval")
    rviz = LaunchConfiguration("rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    terrain_analysis = LaunchConfiguration("terrain_analysis")
    terrain_analysis_ext = LaunchConfiguration("terrain_analysis_ext")
    projected_map = LaunchConfiguration("projected_map")
    local_costmap = LaunchConfiguration("local_costmap")
    costmap_params_file = LaunchConfiguration("costmap_params_file")
    relocalization = LaunchConfiguration("relocalization")
    prior_map_path = LaunchConfiguration("prior_map_path")
    relocalization_initial_x = LaunchConfiguration("relocalization_initial_x")
    relocalization_initial_y = LaunchConfiguration("relocalization_initial_y")
    relocalization_initial_z = LaunchConfiguration("relocalization_initial_z")
    relocalization_initial_a = LaunchConfiguration("relocalization_initial_a")
    relocalization_fitness_score_threshold = LaunchConfiguration(
        "relocalization_fitness_score_threshold"
    )
    relocalization_converged_count_threshold = LaunchConfiguration(
        "relocalization_converged_count_threshold"
    )
    relocalization_max_correspondence_distance = LaunchConfiguration(
        "relocalization_max_correspondence_distance"
    )
    relocalization_input_cloud_to_base_x = LaunchConfiguration(
        "relocalization_input_cloud_to_base_x"
    )
    relocalization_input_cloud_to_base_y = LaunchConfiguration(
        "relocalization_input_cloud_to_base_y"
    )
    relocalization_input_cloud_to_base_z = LaunchConfiguration(
        "relocalization_input_cloud_to_base_z"
    )
    relocalization_legacy_livox_roll_180 = LaunchConfiguration(
        "relocalization_legacy_livox_roll_180"
    )
    simulated_lidar = LaunchConfiguration("simulated_lidar")
    world_obj_path = LaunchConfiguration("world_obj_path")
    simulated_lidar_max_points = LaunchConfiguration("simulated_lidar_max_points")
    simulated_lidar_max_range = LaunchConfiguration("simulated_lidar_max_range")
    simulated_lidar_world_voxel_size = LaunchConfiguration(
        "simulated_lidar_world_voxel_size"
    )
    simulated_lidar_surface_sampling = LaunchConfiguration(
        "simulated_lidar_surface_sampling"
    )
    simulated_lidar_surface_samples = LaunchConfiguration(
        "simulated_lidar_surface_samples"
    )
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
                "front3d_source_topic",
                default_value="/gen0_mapping/simulated_front3d/lidar/points",
                description="PointCloud2 topic used by the Livox adapter, RViz raw preview, and mapping-drive safety.",
            ),
            DeclareLaunchArgument(
                "simulated_lidar_output_topic",
                default_value="/gen0_mapping/simulated_front3d/lidar/points",
                description="Output topic for the OBJ-based simulated LiDAR fallback.",
            ),
            DeclareLaunchArgument(
                "fast_lio_map_file_path",
                default_value="/tmp/gen0_fast_lio_map.pcd",
                description="PCD output path used by FAST-LIO when map saving is enabled.",
            ),
            DeclareLaunchArgument(
                "fast_lio_pcd_save",
                default_value="false",
                choices=["true", "false"],
                description="Enable FAST-LIO PCD saving and the /gen0_mapping/save_fast_lio_map service.",
            ),
            DeclareLaunchArgument(
                "fast_lio_pcd_save_interval",
                default_value="-1",
                description="FAST-LIO PCD save interval; -1 stores all scans in one file.",
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
                "terrain_analysis_ext",
                default_value="true",
                description="Run SCURM terrain_analysis_ext for the projected 2D map pipeline.",
            ),
            DeclareLaunchArgument(
                "projected_map",
                default_value="true",
                description="Publish /projected_map and /projected_costmap from SCURM terrain points.",
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
                "relocalization",
                default_value="false",
                choices=["true", "false"],
                description="Start SCURM ICP relocalization and run FAST-LIO against a prior PCD map.",
            ),
            DeclareLaunchArgument(
                "prior_map_path",
                default_value=default_prior_map(),
                description="PCD prior map used by FAST-LIO and ICP relocalization.",
            ),
            DeclareLaunchArgument(
                "relocalization_initial_x",
                default_value="0.0",
                description="Initial ICP guess x in the prior-map frame.",
            ),
            DeclareLaunchArgument(
                "relocalization_initial_y",
                default_value="0.0",
                description="Initial ICP guess y in the prior-map frame.",
            ),
            DeclareLaunchArgument(
                "relocalization_initial_z",
                default_value="0.0",
                description="Initial ICP guess z in the prior-map frame.",
            ),
            DeclareLaunchArgument(
                "relocalization_initial_a",
                default_value="0.0",
                description="Initial ICP yaw guess in radians.",
            ),
            DeclareLaunchArgument(
                "relocalization_fitness_score_threshold",
                default_value="1.0",
                description="ICP fitness-score threshold. Lower is stricter.",
            ),
            DeclareLaunchArgument(
                "relocalization_converged_count_threshold",
                default_value="3",
                description="Number of consecutive low-error ICP scans required before publishing /icp_result.",
            ),
            DeclareLaunchArgument(
                "relocalization_max_correspondence_distance",
                default_value="2.0",
                description="Maximum ICP correspondence distance in meters.",
            ),
            DeclareLaunchArgument(
                "relocalization_input_cloud_to_base_x",
                default_value="1.9",
                description="LiDAR input-cloud x offset in base_link before ICP.",
            ),
            DeclareLaunchArgument(
                "relocalization_input_cloud_to_base_y",
                default_value="0.0",
                description="LiDAR input-cloud y offset in base_link before ICP.",
            ),
            DeclareLaunchArgument(
                "relocalization_input_cloud_to_base_z",
                default_value="1.9",
                description="LiDAR input-cloud z offset in base_link before ICP.",
            ),
            DeclareLaunchArgument(
                "relocalization_legacy_livox_roll_180",
                default_value="false",
                choices=["true", "false"],
                description="Apply SCURM's original 180-degree roll correction before ICP.",
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
                default_value="16000",
                description="Maximum points per simulated world-lidar scan.",
            ),
            DeclareLaunchArgument(
                "simulated_lidar_max_range",
                default_value="25.0",
                description="Maximum range for the simulated world-lidar fallback.",
            ),
            DeclareLaunchArgument(
                "simulated_lidar_world_voxel_size",
                default_value="0.10",
                description="Voxel size for the loaded world mesh point source.",
            ),
            DeclareLaunchArgument(
                "simulated_lidar_surface_sampling",
                default_value="true",
                choices=["true", "false"],
                description="Sample OBJ faces in addition to raw vertices for denser road/boundary points.",
            ),
            DeclareLaunchArgument(
                "simulated_lidar_surface_samples",
                default_value="500000",
                description="Number of deterministic OBJ surface samples added before voxel filtering.",
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
                        "output_topic": simulated_lidar_output_topic,
                        "pose_topic": "/gen0_model/links/poses",
                        "pose_index": 15,
                        "frame_id": "front_3d_lidar_link",
                        "scan_rate": 10.0,
                        "max_points": ParameterValue(
                            simulated_lidar_max_points, value_type=int
                        ),
                        "world_voxel_size": ParameterValue(
                            simulated_lidar_world_voxel_size, value_type=float
                        ),
                        "surface_sampling_enabled": ParameterValue(
                            simulated_lidar_surface_sampling, value_type=bool
                        ),
                        "surface_sample_count": ParameterValue(
                            simulated_lidar_surface_samples, value_type=int
                        ),
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
                        "input_topic": front3d_source_topic,
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
                package="icp_relocalization",
                executable="transform_publisher",
                name="gen0_icp_transform_publisher",
                output="screen",
                condition=IfCondition(relocalization),
                parameters=[
                    {
                        "map_frame_id": "map",
                        "odom_frame_id": "odom",
                    }
                ],
            ),
            Node(
                package="icp_relocalization",
                executable="icp_node",
                name="gen0_icp_relocalization",
                output="screen",
                condition=IfCondition(relocalization),
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "initial_x": ParameterValue(
                            relocalization_initial_x, value_type=float
                        ),
                        "initial_y": ParameterValue(
                            relocalization_initial_y, value_type=float
                        ),
                        "initial_z": ParameterValue(
                            relocalization_initial_z, value_type=float
                        ),
                        "initial_a": ParameterValue(
                            relocalization_initial_a, value_type=float
                        ),
                        "map_voxel_leaf_size": 0.5,
                        "cloud_voxel_leaf_size": 0.3,
                        "map_frame_id": "map",
                        "solver_max_iter": 75,
                        "max_correspondence_distance": ParameterValue(
                            relocalization_max_correspondence_distance, value_type=float
                        ),
                        "RANSAC_outlier_rejection_threshold": 1.0,
                        "map_path": prior_map_path,
                        "fitness_score_thre": ParameterValue(
                            relocalization_fitness_score_threshold, value_type=float
                        ),
                        "converged_count_thre": ParameterValue(
                            relocalization_converged_count_threshold, value_type=int
                        ),
                        "pcl_type": "livox",
                        "input_cloud_to_base_x": ParameterValue(
                            relocalization_input_cloud_to_base_x, value_type=float
                        ),
                        "input_cloud_to_base_y": ParameterValue(
                            relocalization_input_cloud_to_base_y, value_type=float
                        ),
                        "input_cloud_to_base_z": ParameterValue(
                            relocalization_input_cloud_to_base_z, value_type=float
                        ),
                        "legacy_livox_roll_180": ParameterValue(
                            relocalization_legacy_livox_roll_180, value_type=bool
                        ),
                        "update_initial_guess_on_high_error": False,
                    }
                ],
            ),
            Node(
                package="fast_lio",
                executable="fastlio_mapping",
                name="gen0_fast_lio",
                output="screen",
                parameters=[
                    params_file,
                    {
                        "use_sim_time": use_sim_time,
                        "locate_in_prior_map": ParameterValue(
                            relocalization, value_type=bool
                        ),
                        "prior_map_path": prior_map_path,
                        "map_file_path": fast_lio_map_file_path,
                        "pcd_save.pcd_save_en": ParameterValue(
                            fast_lio_pcd_save, value_type=bool
                        ),
                        "pcd_save.interval": ParameterValue(
                            fast_lio_pcd_save_interval, value_type=int
                        ),
                    },
                ],
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
                        "vehicleHeight": 1.5,
                        "sensorOffsetX": 0.0,
                        "sensorOffsetY": 0.0,
                        "vehicleLength": 4.4,
                        "vehicleWidth": 2.2,
                        "voxelPointUpdateThre": 100,
                        "voxelTimeUpdateThre": 2.0,
                        "minRelZ": -2.5,
                        "maxRelZ": 2.5,
                        "disRatioZ": 0.2,
                    }
                ],
                remappings=[
                    ("/state_estimation", odom_output_topic),
                    ("/registered_scan", "/gen0_mapping/cloud_registered"),
                    ("/terrain_map", "/gen0_mapping/terrain_map"),
                ],
            ),
            Node(
                package="terrain_analysis_ext",
                executable="terrainAnalysisExt",
                name="gen0_scurm_terrain_analysis_ext",
                output="screen",
                condition=IfCondition(terrain_analysis_ext),
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "map_frame": "odom",
                        "scanVoxelSize": 0.08,
                        "decayTime": 0.5,
                        "noDecayDis": 0.0,
                        "clearingDis": 30.0,
                        "useSorting": True,
                        "quantileZ": 0.25,
                        "limitGroundLift": False,
                        "maxGroundLift": 0.25,
                        "vehicleHeight": 1.5,
                        "sensorOffsetX": 0.0,
                        "sensorOffsetY": 0.0,
                        "vehicleLength": 4.4,
                        "vehicleWidth": 2.2,
                        "voxelPointUpdateThre": 100,
                        "voxelTimeUpdateThre": 2.0,
                        "lowerBoundZ": -2.5,
                        "upperBoundZ": 1.0,
                        "disRatioZ": 0.1,
                        "checkTerrainConn": False,
                        "terrainUnderVehicle": -0.75,
                        "terrainConnThre": 0.5,
                        "ceilingFilteringThre": 2.0,
                        "localTerrainMapRadius": 0.0,
                    }
                ],
                remappings=[
                    ("/state_estimation", odom_output_topic),
                    ("/registered_scan", "/gen0_mapping/cloud_registered"),
                    ("/terrain_map", "/gen0_mapping/terrain_map"),
                    ("/terrain_map_ext", "/gen0_mapping/terrain_map_ext"),
                    ("/cloud_clearing", "/gen0_mapping/cloud_clearing"),
                ],
            ),
            Node(
                package="gen0_main",
                executable="projected_terrain_map",
                name="gen0_projected_terrain_map",
                output="screen",
                condition=IfCondition(projected_map),
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "input_topic": "/gen0_mapping/terrain_map_ext",
                        "odom_topic": odom_output_topic,
                        "map_topic": "/projected_map",
                        "costmap_topic": "/projected_costmap",
                        "frame_id": "odom",
                        "resolution": 0.10,
                        "publish_period": 1.0,
                        "free_intensity_threshold": 0.10,
                        "occupied_intensity_threshold": 0.15,
                        "occupied_cost_intensity": 0.45,
                        "hit_log_odds": 0.85,
                        "miss_log_odds": 0.20,
                        "occupied_log_odds_threshold": 0.0,
                        "free_log_odds_threshold": -0.2,
                        "mark_low_intensity_free": True,
                        "ground_clears_occupied": False,
                        "occupied_padding_radius": 0.0,
                        "filter_speckles": False,
                        "raytrace_free_space": True,
                        "raytrace_clears_occupied": True,
                        "occupied_clear_log_odds_threshold": 1.7,
                        "raytrace_max_range": 22.0,
                        "max_raytrace_cells_per_update": 5000,
                        "robot_clear_radius": 0.8,
                        "robot_clear_length": 4.4,
                        "robot_clear_width": 2.2,
                        "robot_clear_margin": 0.1,
                        "inflation_radius": 0.7,
                        "inflation_cost_scaling": 3.0,
                    }
                ],
            ),
            TimerAction(
                period=8.0,
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
                            {"bond_timeout": 0.0},
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
                    "raw_front3d_input_topic": front3d_source_topic,
                }.items(),
            ),
        ]
    )
