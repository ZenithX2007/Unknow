import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PythonExpression
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
    default_trash_fusion_params = os.path.join(
        package_share, "config", "trash_fusion_detection.yaml"
    )
    default_world_obj = os.path.join(
        package_share, "worlds", "san_roundabout", "san_roundabout.obj"
    )
    rviz_launch = os.path.join(package_share, "launch", "gen0_3d_rviz.launch.py")

    params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    odom_output_topic = LaunchConfiguration("odom_output_topic")
    fast_lio_send_odom_base_tf = LaunchConfiguration("fast_lio_send_odom_base_tf")
    fast_lio_sensor_frame_id = LaunchConfiguration("fast_lio_sensor_frame_id")
    stable_sim_odom = LaunchConfiguration("stable_sim_odom")
    stable_odom_input_topic = LaunchConfiguration("stable_odom_input_topic")
    stable_odom_output_topic = LaunchConfiguration("stable_odom_output_topic")
    stable_registered_scan_topic = LaunchConfiguration("stable_registered_scan_topic")
    scurm_odom_topic = LaunchConfiguration("scurm_odom_topic")
    scurm_registered_scan_topic = LaunchConfiguration("scurm_registered_scan_topic")
    front3d_source_topic = LaunchConfiguration("front3d_source_topic")
    simulated_lidar_output_topic = LaunchConfiguration("simulated_lidar_output_topic")
    simulated_lidar_filtered_output_topic = LaunchConfiguration(
        "simulated_lidar_filtered_output_topic"
    )
    registered_scan_input_topic = LaunchConfiguration("registered_scan_input_topic")
    registered_scan_odom_topic = LaunchConfiguration("registered_scan_odom_topic")
    fast_lio_map_file_path = LaunchConfiguration("fast_lio_map_file_path")
    fast_lio_pcd_save = LaunchConfiguration("fast_lio_pcd_save")
    fast_lio_pcd_save_interval = LaunchConfiguration("fast_lio_pcd_save_interval")
    rviz = LaunchConfiguration("rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    rviz_render_env = LaunchConfiguration("rviz_render_env")
    terrain_analysis = LaunchConfiguration("terrain_analysis")
    terrain_analysis_ext = LaunchConfiguration("terrain_analysis_ext")
    projected_map = LaunchConfiguration("projected_map")
    projected_map_backend = LaunchConfiguration("projected_map_backend")
    projected_map_reference_odom_topic = LaunchConfiguration(
        "projected_map_reference_odom_topic"
    )
    projected_map_max_reference_odom_error = LaunchConfiguration(
        "projected_map_max_reference_odom_error"
    )
    projected_map_max_reference_yaw_error = LaunchConfiguration(
        "projected_map_max_reference_yaw_error"
    )
    projected_map_reference_odom_timeout = LaunchConfiguration(
        "projected_map_reference_odom_timeout"
    )
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
    simulated_lidar_add_obstacle_columns = LaunchConfiguration(
        "simulated_lidar_add_obstacle_columns"
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
    dynamic_actor_topics = LaunchConfiguration("dynamic_actor_topics")
    dynamic_vehicle_topics = LaunchConfiguration("dynamic_vehicle_topics")
    actors_scenario_path = LaunchConfiguration("actors_scenario_path")
    actor_costmap = LaunchConfiguration("actor_costmap")
    actor_obstacle_topic = LaunchConfiguration("actor_obstacle_topic")
    actor_obstacle_frame = LaunchConfiguration("actor_obstacle_frame")
    actor_world_sdf_path = LaunchConfiguration("actor_world_sdf_path")
    actor_world_vehicle_name = LaunchConfiguration("actor_world_vehicle_name")
    actor_world_to_output = LaunchConfiguration("actor_world_to_output")
    actor_output_origin_xy = LaunchConfiguration("actor_output_origin_xy")
    actor_collision_monitor = LaunchConfiguration("actor_collision_monitor")
    actor_collision_event_topic = LaunchConfiguration("actor_collision_event_topic")
    actor_collision_near_margin = LaunchConfiguration("actor_collision_near_margin")
    actor_collision_vehicle_length = LaunchConfiguration(
        "actor_collision_vehicle_length"
    )
    actor_collision_vehicle_width = LaunchConfiguration(
        "actor_collision_vehicle_width"
    )
    trash_scenario_path = LaunchConfiguration("trash_scenario_path")
    trash_fusion_detection = LaunchConfiguration("trash_fusion_detection")
    trash_fusion_params_file = LaunchConfiguration("trash_fusion_params_file")
    trash_fusion_model_path = LaunchConfiguration("trash_fusion_model_path")
    trash_fusion_output_frame = LaunchConfiguration("trash_fusion_output_frame")
    trash_fusion_pointcloud_topic = LaunchConfiguration(
        "trash_fusion_pointcloud_topic"
    )
    trash_fusion_image_topic = LaunchConfiguration("trash_fusion_image_topic")
    trash_fusion_camera_info_topic = LaunchConfiguration(
        "trash_fusion_camera_info_topic"
    )
    scurm_octomap_projected_map = IfCondition(
        PythonExpression(
            [
                "'",
                projected_map,
                "' == 'true' and '",
                projected_map_backend,
                "' == 'octomap'",
            ]
        )
    )
    python_projected_map = IfCondition(
        PythonExpression(
            [
                "'",
                projected_map,
                "' == 'true' and '",
                projected_map_backend,
                "' == 'python'",
            ]
        )
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
                "fast_lio_send_odom_base_tf",
                default_value="true",
                choices=["true", "false"],
                description="Allow FAST_LIO to publish its odom TF.",
            ),
            DeclareLaunchArgument(
                "fast_lio_sensor_frame_id",
                default_value="base_link",
                description="FAST_LIO odometry child frame. Use a non-nav frame when another source owns odom->base_link.",
            ),
            DeclareLaunchArgument(
                "stable_sim_odom",
                default_value="false",
                choices=["true", "false"],
                description="Publish normalized Gazebo odom and a matching registered scan for simulation-stable SCURM/Nav2.",
            ),
            DeclareLaunchArgument(
                "stable_odom_input_topic",
                default_value="/odom",
                description="Raw odometry topic normalized for simulation-stable SCURM/Nav2.",
            ),
            DeclareLaunchArgument(
                "stable_odom_output_topic",
                default_value="/gen0_mapping/stable_odom",
                description="Normalized odometry topic used by stable simulation mapping.",
            ),
            DeclareLaunchArgument(
                "stable_registered_scan_topic",
                default_value="/gen0_mapping/stable_registered_scan",
                description="PointCloud2 scan transformed into the stable odom frame.",
            ),
            DeclareLaunchArgument(
                "scurm_odom_topic",
                default_value="/gen0_mapping/fast_lio/odom",
                description="Odometry topic consumed by SCURM terrain analysis and projected map.",
            ),
            DeclareLaunchArgument(
                "scurm_registered_scan_topic",
                default_value="/gen0_mapping/cloud_registered",
                description="Registered scan topic consumed by SCURM terrain analysis.",
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
                "simulated_lidar_filtered_output_topic",
                default_value="/gen0_mapping/simulated_front3d/lidar/points_no_trash",
                description="Simulated LiDAR output with generated trash points removed.",
            ),
            DeclareLaunchArgument(
                "registered_scan_input_topic",
                default_value="/gen0_mapping/simulated_front3d/lidar/points",
                description="PointCloud2 input used by odom_registered_scan.",
            ),
            DeclareLaunchArgument(
                "registered_scan_odom_topic",
                default_value="/gen0_mapping/stable_odom",
                description="Odometry topic used to register the SCURM input scan.",
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
                "rviz_render_env",
                default_value="software",
                choices=["auto", "software", "passthrough"],
                description="RViz OpenGL environment. auto/software use llvmpipe; passthrough keeps the host GL path.",
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
                description="Publish the online projected occupancy map used by SCURM/Nav2.",
            ),
            DeclareLaunchArgument(
                "projected_map_backend",
                default_value="python",
                choices=["octomap", "python"],
                description="Projected map backend. python is the SCURM-style terrain projection; octomap is the optional 3D fallback.",
            ),
            DeclareLaunchArgument(
                "projected_map_reference_odom_topic",
                default_value="",
                description="Reference odometry for projected-map integration health checks; empty disables the check.",
            ),
            DeclareLaunchArgument(
                "projected_map_max_reference_odom_error",
                default_value="0.0",
                description="Freeze projected-map integration when odom differs from the reference by more than this many meters; 0 disables.",
            ),
            DeclareLaunchArgument(
                "projected_map_max_reference_yaw_error",
                default_value="0.0",
                description="Freeze projected-map integration when odom yaw differs from the reference by more than this many radians; 0 disables.",
            ),
            DeclareLaunchArgument(
                "projected_map_reference_odom_timeout",
                default_value="2.0",
                description="Maximum age in seconds for projected-map reference odometry.",
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
                default_value="48000",
                description="Maximum points per simulated world-lidar scan.",
            ),
            DeclareLaunchArgument(
                "simulated_lidar_max_range",
                default_value="35.0",
                description="Maximum range for the simulated world-lidar fallback.",
            ),
            DeclareLaunchArgument(
                "simulated_lidar_world_voxel_size",
                default_value="0.08",
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
                default_value="1000000",
                description="Number of deterministic OBJ surface samples added before voxel filtering.",
            ),
            DeclareLaunchArgument(
                "simulated_lidar_add_obstacle_columns",
                default_value="false",
                choices=["true", "false"],
                description="Add synthetic vertical obstacle columns from the mesh. Disabled by default to match SCURM's real surface-only LiDAR input.",
            ),
            DeclareLaunchArgument(
                "simulated_lidar_horizontal_min_angle",
                default_value="-3.14159",
                description="Minimum horizontal angle for simulated world-lidar points.",
            ),
            DeclareLaunchArgument(
                "simulated_lidar_horizontal_max_angle",
                default_value="3.14159",
                description="Maximum horizontal angle for simulated world-lidar points.",
            ),
            DeclareLaunchArgument(
                "simulated_lidar_vertical_min_angle",
                default_value="-0.55",
                description="Minimum vertical angle for simulated world-lidar points.",
            ),
            DeclareLaunchArgument(
                "simulated_lidar_vertical_max_angle",
                default_value="0.55",
                description="Maximum vertical angle for simulated world-lidar points.",
            ),
            DeclareLaunchArgument(
                "dynamic_actor_topics",
                default_value="",
                description="Comma-separated actor PoseStamped topics added to the simulated lidar fallback.",
            ),
            DeclareLaunchArgument(
                "dynamic_vehicle_topics",
                default_value="/car/car_008/pose,/car/car_009/pose",
                description="Comma-separated vehicle PoseStamped topics added to the simulated lidar fallback.",
            ),
            DeclareLaunchArgument(
                "actors_scenario_path",
                default_value="",
                description="Actor scenario SDF used as a fallback source for dynamic costmap obstacles.",
            ),
            DeclareLaunchArgument(
                "actor_costmap",
                default_value="false",
                choices=["true", "false"],
                description="Publish moving actor obstacles as an optional debug PointCloud2 source.",
            ),
            DeclareLaunchArgument(
                "actor_obstacle_topic",
                default_value="/gen0_mapping/actor_obstacles",
                description="PointCloud2 topic containing current moving actor obstacles for local costmaps.",
            ),
            DeclareLaunchArgument(
                "actor_obstacle_frame",
                default_value="odom",
                description="Frame id used by actor_obstacle_topic. Use map for relocalized prior-map runs.",
            ),
            DeclareLaunchArgument(
                "actor_collision_monitor",
                default_value="true",
                choices=["true", "false"],
                description="Run passive vehicle-vs-actor collision validation without altering actor motion.",
            ),
            DeclareLaunchArgument(
                "actor_collision_event_topic",
                default_value="/gen0_validation/actor_collision_events",
                description="String topic publishing passive actor collision/near-miss validation events.",
            ),
            DeclareLaunchArgument(
                "actor_collision_near_margin",
                default_value="0.75",
                description="Distance in meters outside actor radius used for near-miss validation events.",
            ),
            DeclareLaunchArgument(
                "actor_collision_vehicle_length",
                default_value="4.0",
                description="Vehicle rectangle length in meters used by actor collision validation.",
            ),
            DeclareLaunchArgument(
                "actor_collision_vehicle_width",
                default_value="2.0",
                description="Vehicle rectangle width in meters used by actor collision validation.",
            ),
            DeclareLaunchArgument(
                "actor_world_sdf_path",
                default_value="",
                description="World SDF used to align actor world coordinates to the costmap frame.",
            ),
            DeclareLaunchArgument(
                "actor_world_vehicle_name",
                default_value="gen0_model",
                description="Vehicle model name whose world pose anchors actor coordinate conversion.",
            ),
            DeclareLaunchArgument(
                "actor_world_to_output",
                default_value="true",
                choices=["true", "false"],
                description="Convert Gazebo actor world coordinates to the relocalized/normalized output frame.",
            ),
            DeclareLaunchArgument(
                "actor_output_origin_xy",
                default_value="0.0,0.0",
                description="XY origin of the vehicle in actor_obstacle_frame after world-to-output conversion.",
            ),
            DeclareLaunchArgument(
                "trash_scenario_path",
                default_value="",
                description="Optional trash JSON scenario added to the simulated lidar fallback.",
            ),
            DeclareLaunchArgument(
                "trash_fusion_detection",
                default_value="false",
                choices=["true", "false"],
                description="Run YOLO + point-cloud trash fusion as a perception-only RViz/debug output.",
            ),
            DeclareLaunchArgument(
                "trash_fusion_params_file",
                default_value=default_trash_fusion_params,
                description="Trash fusion detector parameter file.",
            ),
            DeclareLaunchArgument(
                "trash_fusion_model_path",
                default_value="/home/zjxue2007/Unknow/best.pt",
                description="YOLO model weights used by the trash fusion detector.",
            ),
            DeclareLaunchArgument(
                "trash_fusion_output_frame",
                default_value="map",
                description="Frame used for published trash detections and RViz markers.",
            ),
            DeclareLaunchArgument(
                "trash_fusion_pointcloud_topic",
                default_value="/gen0_mapping/simulated_front3d/lidar/points",
                description="Full PointCloud2 topic used for trash localization.",
            ),
            DeclareLaunchArgument(
                "trash_fusion_image_topic",
                default_value="/gen0_model/front_camera",
                description="Camera image topic used by YOLO trash detection.",
            ),
            DeclareLaunchArgument(
                "trash_fusion_camera_info_topic",
                default_value="/gen0_model/camera_info",
                description="CameraInfo topic used for 3D-to-2D projection.",
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
                        "filtered_output_topic": simulated_lidar_filtered_output_topic,
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
                        "add_obstacle_columns": ParameterValue(
                            simulated_lidar_add_obstacle_columns, value_type=bool
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
                        "dynamic_actor_topics": dynamic_actor_topics,
                        "dynamic_vehicle_topics": dynamic_vehicle_topics,
                        "trash_scenario_path": trash_scenario_path,
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
                        "line_count": 16,
                        "vertical_min_angle": ParameterValue(
                            simulated_lidar_vertical_min_angle, value_type=float
                        ),
                        "vertical_max_angle": ParameterValue(
                            simulated_lidar_vertical_max_angle, value_type=float
                        ),
                        "max_points": ParameterValue(
                            simulated_lidar_max_points, value_type=int
                        ),
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
                package="gen0_main",
                executable="trash_fusion_detector",
                name="gen0_trash_fusion_detector",
                output="screen",
                condition=IfCondition(trash_fusion_detection),
                parameters=[
                    trash_fusion_params_file,
                    {
                        "use_sim_time": use_sim_time,
                        "image_topic": trash_fusion_image_topic,
                        "camera_info_topic": trash_fusion_camera_info_topic,
                        "pointcloud_topic": trash_fusion_pointcloud_topic,
                        "model_path": trash_fusion_model_path,
                        "output_frame": trash_fusion_output_frame,
                    },
                ],
            ),
            Node(
                package="gen0_main",
                executable="stable_odom",
                name="gen0_stable_odom",
                output="screen",
                condition=IfCondition(stable_sim_odom),
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "input_topic": stable_odom_input_topic,
                        "output_topic": stable_odom_output_topic,
                        "odom_frame_id": "odom",
                        "base_frame_id": "base_link",
                        "publish_tf": True,
                        "compute_twist_from_pose": True,
                    }
                ],
            ),
            Node(
                package="gen0_main",
                executable="odom_registered_scan",
                name="gen0_odom_registered_scan",
                output="screen",
                condition=IfCondition(
                    PythonExpression(
                        [
                            "'",
                            stable_sim_odom,
                            "' == 'true' or '",
                            simulated_lidar,
                            "' == 'true'",
                        ]
                    )
                ),
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "input_topic": registered_scan_input_topic,
                        "odom_topic": registered_scan_odom_topic,
                        "output_topic": stable_registered_scan_topic,
                        "output_frame": "odom",
                        "lidar_xyz_in_base": [1.9, 0.0, 1.9],
                        "max_odom_age": 1.0,
                        "max_points": ParameterValue(
                            simulated_lidar_max_points, value_type=int
                        ),
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
                        "use_sim_time": use_sim_time,
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
                        "map_min_z": -1.2,
                        "map_max_z": 30.0,
                        "prior_map_publish_voxel_leaf_size": 1.0,
                        "max_published_prior_map_points": 120000,
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
                        "prior_map_min_z": -1.2,
                        "prior_map_max_z": 30.0,
                        "map_file_path": fast_lio_map_file_path,
                        "pcd_save.pcd_save_en": ParameterValue(
                            fast_lio_pcd_save, value_type=bool
                        ),
                        "pcd_save.interval": ParameterValue(
                            fast_lio_pcd_save_interval, value_type=int
                        ),
                        "publish.effect_map_en": ParameterValue(
                            PythonExpression(["'", relocalization, "' == 'false'"]),
                            value_type=bool,
                        ),
                        "publish.map_en": ParameterValue(
                            PythonExpression(["'", relocalization, "' == 'false'"]),
                            value_type=bool,
                        ),
                        "publish.ikd_tree_en": ParameterValue(
                            PythonExpression(["'", relocalization, "' == 'false'"]),
                            value_type=bool,
                        ),
                        "publish.scan_publish_en": True,
                        "publish.dense_publish_en": True,
                        "publish.scan_bodyframe_pub_en": True,
                        "common.send_odom_base_tf": ParameterValue(
                            fast_lio_send_odom_base_tf, value_type=bool
                        ),
                        "common.sensor_frame_id": fast_lio_sensor_frame_id,
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
                    ("/state_estimation", scurm_odom_topic),
                    ("/registered_scan", scurm_registered_scan_topic),
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
                        "scanVoxelSize": 0.1,
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
                    ("/state_estimation", scurm_odom_topic),
                    ("/registered_scan", scurm_registered_scan_topic),
                    ("/terrain_map", "/gen0_mapping/terrain_map"),
                    ("/terrain_map_ext", "/gen0_mapping/terrain_map_ext"),
                    ("/cloud_clearing", "/gen0_mapping/cloud_clearing"),
                ],
            ),
            Node(
                package="gen0_main",
                executable="actor_obstacle_costmap",
                name="gen0_actor_obstacle_costmap",
                output="screen",
                condition=IfCondition(actor_costmap),
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "actors_scenario_path": actors_scenario_path,
                        "actor_pose_topics": dynamic_actor_topics,
                        "output_topic": actor_obstacle_topic,
                        "frame_id": actor_obstacle_frame,
                        "world_sdf_path": actor_world_sdf_path,
                        "world_vehicle_name": actor_world_vehicle_name,
                        "transform_world_to_output": ParameterValue(
                            actor_world_to_output, value_type=bool
                        ),
                        "output_origin_xy": ParameterValue(
                            actor_output_origin_xy, value_type=str
                        ),
                        "publish_rate": 10.0,
                        "live_pose_timeout": 1.0,
                        "actor_radius": 0.45,
                        "actor_z_min": 0.15,
                        "actor_z_max": 1.45,
                        "actor_mark_intensity": 0.6,
                        "actor_clear_intensity": 0.0,
                        "actor_radial_samples": 24,
                        "actor_height_samples": 4,
                    }
                ],
            ),
            Node(
                package="gen0_main",
                executable="actor_collision_monitor",
                name="gen0_actor_collision_monitor",
                output="screen",
                condition=IfCondition(actor_collision_monitor),
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "actor_pose_topics": dynamic_actor_topics,
                        "vehicle_pose_topic": "/gen0_model/links/poses",
                        "vehicle_pose_index": 15,
                        "event_topic": actor_collision_event_topic,
                        "watchdog_rate": 20.0,
                        "actor_pose_timeout": 1.0,
                        "vehicle_pose_timeout": 1.0,
                        "vehicle_length": ParameterValue(
                            actor_collision_vehicle_length, value_type=float
                        ),
                        "vehicle_width": ParameterValue(
                            actor_collision_vehicle_width, value_type=float
                        ),
                        "vehicle_padding": 0.05,
                        "actor_radius": 0.45,
                        "near_margin": ParameterValue(
                            actor_collision_near_margin, value_type=float
                        ),
                        "collision_margin": 0.05,
                    }
                ],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="gen0_scurm_map_to_odom",
                output="screen",
                condition=scurm_octomap_projected_map,
                arguments=[
                    "--frame-id",
                    "map",
                    "--child-frame-id",
                    "odom",
                    "--x",
                    "0.0",
                    "--y",
                    "0.0",
                    "--z",
                    "0.0",
                    "--qx",
                    "0.0",
                    "--qy",
                    "0.0",
                    "--qz",
                    "0.0",
                    "--qw",
                    "1.0",
                ],
            ),
            Node(
                package="terrain_analysis",
                executable="exchangeField",
                name="gen0_scurm_exchange_field",
                output="screen",
                condition=scurm_octomap_projected_map,
                remappings=[
                    ("/input_topic", "/gen0_mapping/terrain_map_ext"),
                    ("/output_topic", "/gen0_mapping/terrain_map_ext_exchanged"),
                ],
            ),
            Node(
                package="sensor_scan_generation",
                executable="sensorScanGeneration",
                name="gen0_scurm_sensor_scan_generation",
                output="screen",
                condition=scurm_octomap_projected_map,
                remappings=[
                    ("/state_estimation", scurm_odom_topic),
                    ("/registered_scan", "/gen0_mapping/terrain_map_ext_exchanged"),
                    ("/sensor_scan", "/gen0_mapping/terrain_map_at_scan"),
                    (
                        "/state_estimation_at_scan",
                        "/gen0_mapping/state_estimation_at_scan",
                    ),
                ],
            ),
            Node(
                package="octomap_server",
                executable="octomap_server_node",
                name="gen0_scurm_octomap_server",
                output="screen",
                condition=scurm_octomap_projected_map,
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "frame_id": "map",
                        "base_frame_id": "sensor_at_scan",
                        "point_cloud_min_z": 0.15,
                        "filter_speckles": True,
                        "filter_ground_plane": False,
                        "resolution": 0.1,
                        "latch": True,
                    }
                ],
                remappings=[
                    ("/cloud_in", "/gen0_mapping/terrain_map_at_scan"),
                ],
            ),
            Node(
                package="gen0_main",
                executable="projected_terrain_map",
                name="gen0_projected_terrain_map",
                output="screen",
                condition=python_projected_map,
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "input_topic": "/gen0_mapping/terrain_map_ext",
                        "odom_topic": scurm_odom_topic,
                        "map_topic": "/projected_map",
                        "costmap_topic": "/projected_costmap",
                        "frame_id": "odom",
                        "resolution": 0.10,
                        "publish_period": 1.0,
                        "accumulate_history": True,
                        "free_intensity_threshold": 0.04,
                        "occupied_intensity_threshold": 0.08,
                        "occupied_cost_intensity": 0.25,
                        "hit_log_odds": 0.85,
                        "miss_log_odds": 0.35,
                        "occupied_log_odds_threshold": 1.35,
                        "free_log_odds_threshold": -0.2,
                        "mark_low_intensity_free": True,
                        "ground_clears_occupied": True,
                        "occupied_padding_radius": 0.0,
                        "filter_speckles": True,
                        "min_occupied_component_cells": 6,
                        "min_occupied_component_span_cells": 4,
                        "occupied_gap_bridge_cells": 0,
                        "reference_odom_topic": projected_map_reference_odom_topic,
                        "max_reference_odom_error": ParameterValue(
                            projected_map_max_reference_odom_error,
                            value_type=float,
                        ),
                        "max_reference_yaw_error": ParameterValue(
                            projected_map_max_reference_yaw_error,
                            value_type=float,
                        ),
                        "reference_odom_timeout": ParameterValue(
                            projected_map_reference_odom_timeout,
                            value_type=float,
                        ),
                        "raytrace_free_space": True,
                        "raytrace_clears_occupied": True,
                        "occupied_clear_log_odds_threshold": 1.70,
                        "raytrace_max_range": 22.0,
                        "max_raytrace_cells_per_update": 5000,
                        "robot_clear_radius": 0.0,
                        "robot_clear_length": 0.0,
                        "robot_clear_width": 0.0,
                        "robot_clear_margin": 0.0,
                        "publish_empty_until_data": True,
                        "inflation_radius": 0.5,
                        "inflation_cost_scaling": 5.0,
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
                    "rviz_render_env": rviz_render_env,
                    "use_sim_time": use_sim_time,
                    "raw_front3d_input_topic": front3d_source_topic,
                    "registered_preview_input_topic": scurm_registered_scan_topic,
                }.items(),
            ),
        ]
    )
