import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.actions import UnsetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def default_prior_map():
    maps_dir = (
        Path.home()
        / "Unknow"
        / "gen0_gz_sim_ros2"
        / "gen0_main"
        / "maps"
    )
    candidates = [
        maps_dir / "prior_map.pcd",
        maps_dir / "GlobalMap.pcd",
    ]
    if maps_dir.is_dir():
        candidates.extend(
            sorted(
                maps_dir.glob("recovered_fast_lio_*.pcd"),
                key=lambda path: path.stat().st_mtime,
                reverse=True,
            )
        )
    candidates.append(
        Path.home()
        / "SCURM_SentryNavigation"
        / "sentry_bringup"
        / "maps"
        / "GlobalMap.pcd"
    )

    for map_path in candidates:
        if map_path.is_file():
            return str(map_path)
    return ""


def generate_launch_description():
    package_share = get_package_share_directory("gen0_main")
    spawn_launch = os.path.join(package_share, "launch", "spawn.launch.py")
    fast_lio_launch = os.path.join(
        package_share, "launch", "gen0_fast_lio_mapping.launch.py"
    )
    default_rviz = os.path.join(
        package_share, "config", "gen0_relocalization_loam_livox.rviz"
    )
    default_bridge_file = os.path.join(
        package_share, "config", "bridge_no_gz_odom_tf.yaml"
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    spawn = LaunchConfiguration("spawn")
    start_gazebo = LaunchConfiguration("start_gazebo")
    gazebo_gui = LaunchConfiguration("gazebo_gui")
    gui = LaunchConfiguration("gui")
    world = LaunchConfiguration("world")
    partition = LaunchConfiguration("partition")
    bridge_file = LaunchConfiguration("bridge_file")
    rviz = LaunchConfiguration("rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    rviz_render_env = LaunchConfiguration("rviz_render_env")
    render_engine = LaunchConfiguration("render_engine")
    d3d12_adapter = LaunchConfiguration("d3d12_adapter")
    render_env = LaunchConfiguration("render_env")
    gui_visual_mode = LaunchConfiguration("gui_visual_mode")
    prior_map_path = LaunchConfiguration("prior_map_path")
    initial_x = LaunchConfiguration("initial_x")
    initial_y = LaunchConfiguration("initial_y")
    initial_z = LaunchConfiguration("initial_z")
    initial_a = LaunchConfiguration("initial_a")
    fitness_score_threshold = LaunchConfiguration("fitness_score_threshold")
    converged_count_threshold = LaunchConfiguration("converged_count_threshold")
    max_correspondence_distance = LaunchConfiguration("max_correspondence_distance")
    input_cloud_to_base_x = LaunchConfiguration("input_cloud_to_base_x")
    input_cloud_to_base_y = LaunchConfiguration("input_cloud_to_base_y")
    input_cloud_to_base_z = LaunchConfiguration("input_cloud_to_base_z")
    legacy_livox_roll_180 = LaunchConfiguration("legacy_livox_roll_180")
    terrain_analysis = LaunchConfiguration("terrain_analysis")
    terrain_analysis_ext = LaunchConfiguration("terrain_analysis_ext")
    projected_map = LaunchConfiguration("projected_map")
    local_costmap = LaunchConfiguration("local_costmap")
    rviz_software = IfCondition(
        PythonExpression(["'", rviz_render_env, "' != 'passthrough'"])
    )
    rviz_passthrough = IfCondition(
        PythonExpression(["'", rviz_render_env, "' == 'passthrough'"])
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use Gazebo simulation clock.",
            ),
            DeclareLaunchArgument(
                "spawn",
                default_value="true",
                choices=["true", "false"],
                description="Start the Gen0 Gazebo bridge and robot TF, matching SCURM's all-in-one relocalization launch.",
            ),
            DeclareLaunchArgument(
                "start_gazebo",
                default_value="true",
                choices=["true", "false"],
                description="Start Gazebo from spawn.launch.py. Set false when Gazebo is already running.",
            ),
            DeclareLaunchArgument(
                "gazebo_gui",
                default_value="true",
                choices=["true", "false"],
                description="Open Gazebo GUI when spawn is enabled.",
            ),
            DeclareLaunchArgument(
                "gui",
                default_value="",
                choices=["", "true", "false"],
                description="Alias for gazebo_gui. Leave empty to use gazebo_gui.",
            ),
            DeclareLaunchArgument(
                "world",
                default_value="my_map",
                description="Gen0 Gazebo world used by the relocalization sensor source.",
            ),
            DeclareLaunchArgument(
                "partition",
                default_value="gen0_relocalization",
                description="Gazebo transport partition shared by Gazebo and ros_gz_bridge.",
            ),
            DeclareLaunchArgument(
                "bridge_file",
                default_value=default_bridge_file,
                description="ros_gz_bridge YAML. The default keeps FAST-LIO in charge of odom -> base_link.",
            ),
            DeclareLaunchArgument(
                "render_engine",
                default_value="ogre",
                choices=["ogre", "ogre2"],
                description="Gazebo rendering backend used by the relocalization world.",
            ),
            DeclareLaunchArgument(
                "d3d12_adapter",
                default_value="NVIDIA",
                description="Optional WSL D3D12 adapter name, matching the working 3D SLAM shell script.",
            ),
            DeclareLaunchArgument(
                "render_env",
                default_value="software",
                choices=["auto", "unset", "software", "passthrough"],
                description="Gazebo rendering environment. Use software for the verified WSL relocalization path.",
            ),
            DeclareLaunchArgument(
                "gui_visual_mode",
                default_value="full",
                choices=["light", "full"],
                description="Use the full textured world visual by default.",
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="true",
                choices=["true", "false"],
                description="Open the Gen0 relocalization RViz view.",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=default_rviz,
                description="RViz config adapted from SCURM loam_livox.rviz.",
            ),
            DeclareLaunchArgument(
                "rviz_render_env",
                default_value="software",
                choices=["auto", "software", "passthrough"],
                description="RViz OpenGL environment. auto/software use llvmpipe; passthrough keeps the host GL path.",
            ),
            DeclareLaunchArgument(
                "prior_map_path",
                default_value=default_prior_map(),
                description="PCD prior map used by ICP and FAST-LIO relocalization.",
            ),
            DeclareLaunchArgument(
                "initial_x",
                default_value="0.0",
                description="Initial ICP guess x in the prior-map frame.",
            ),
            DeclareLaunchArgument(
                "initial_y",
                default_value="0.0",
                description="Initial ICP guess y in the prior-map frame.",
            ),
            DeclareLaunchArgument(
                "initial_z",
                default_value="0.0",
                description="Initial ICP guess z in the prior-map frame.",
            ),
            DeclareLaunchArgument(
                "initial_a",
                default_value="0.0",
                description="Initial ICP yaw guess in radians.",
            ),
            DeclareLaunchArgument(
                "fitness_score_threshold",
                default_value="1.0",
                description="ICP fitness-score threshold. Lower is stricter.",
            ),
            DeclareLaunchArgument(
                "converged_count_threshold",
                default_value="3",
                description="Consecutive low-error ICP scans required before publishing /icp_result.",
            ),
            DeclareLaunchArgument(
                "max_correspondence_distance",
                default_value="2.0",
                description="Maximum ICP correspondence distance in meters.",
            ),
            DeclareLaunchArgument(
                "input_cloud_to_base_x",
                default_value="1.9",
                description="Gen0 front 3D LiDAR x offset in base_link.",
            ),
            DeclareLaunchArgument(
                "input_cloud_to_base_y",
                default_value="0.0",
                description="Gen0 front 3D LiDAR y offset in base_link.",
            ),
            DeclareLaunchArgument(
                "input_cloud_to_base_z",
                default_value="1.9",
                description="Gen0 front 3D LiDAR z offset in base_link.",
            ),
            DeclareLaunchArgument(
                "legacy_livox_roll_180",
                default_value="false",
                choices=["true", "false"],
                description="Keep false for Gen0; SCURM's original Livox frame used true.",
            ),
            DeclareLaunchArgument(
                "terrain_analysis",
                default_value="false",
                choices=["true", "false"],
                description="Run SCURM terrain analysis during relocalization.",
            ),
            DeclareLaunchArgument(
                "terrain_analysis_ext",
                default_value="false",
                choices=["true", "false"],
                description="Run SCURM extended terrain analysis during relocalization.",
            ),
            DeclareLaunchArgument(
                "projected_map",
                default_value="false",
                choices=["true", "false"],
                description="Publish projected 2D map during relocalization.",
            ),
            DeclareLaunchArgument(
                "local_costmap",
                default_value="false",
                choices=["true", "false"],
                description="Run the standalone SCURM local costmap during relocalization.",
            ),
            GroupAction(
                scoped=True,
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(spawn_launch),
                        condition=IfCondition(spawn),
                        launch_arguments={
                            "use_sim_time": use_sim_time,
                            "world": world,
                            "gazebo_gui": gazebo_gui,
                            "gui": gui,
                            "start_gazebo": start_gazebo,
                            "partition": partition,
                            "rviz": "false",
                            "ground_truth_localization": "false",
                            "static_odom_base": "false",
                            "bridge_file": bridge_file,
                            "render_engine": render_engine,
                            "d3d12_adapter": d3d12_adapter,
                            "render_env": render_env,
                            "gui_visual_mode": gui_visual_mode,
                        }.items(),
                    ),
                ],
            ),
            GroupAction(
                scoped=True,
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(fast_lio_launch),
                        launch_arguments={
                            "use_sim_time": use_sim_time,
                            "rviz": "false",
                            "terrain_analysis": terrain_analysis,
                            "terrain_analysis_ext": terrain_analysis_ext,
                            "projected_map": projected_map,
                            "local_costmap": local_costmap,
                            "relocalization": "true",
                            "prior_map_path": prior_map_path,
                            "relocalization_initial_x": initial_x,
                            "relocalization_initial_y": initial_y,
                            "relocalization_initial_z": initial_z,
                            "relocalization_initial_a": initial_a,
                            "relocalization_fitness_score_threshold": fitness_score_threshold,
                            "relocalization_converged_count_threshold": converged_count_threshold,
                            "relocalization_max_correspondence_distance": max_correspondence_distance,
                            "relocalization_input_cloud_to_base_x": input_cloud_to_base_x,
                            "relocalization_input_cloud_to_base_y": input_cloud_to_base_y,
                            "relocalization_input_cloud_to_base_z": input_cloud_to_base_z,
                            "relocalization_legacy_livox_roll_180": legacy_livox_roll_180,
                        }.items(),
                    ),
                ],
            ),
            UnsetEnvironmentVariable(
                "LIBGL_ALWAYS_SOFTWARE",
                condition=rviz_passthrough,
            ),
            UnsetEnvironmentVariable(
                "MESA_LOADER_DRIVER_OVERRIDE",
                condition=rviz_passthrough,
            ),
            UnsetEnvironmentVariable(
                "QT_XCB_GL_INTEGRATION",
                condition=rviz_passthrough,
            ),
            SetEnvironmentVariable(
                "LIBGL_ALWAYS_SOFTWARE",
                "1",
                condition=rviz_software,
            ),
            SetEnvironmentVariable(
                "MESA_LOADER_DRIVER_OVERRIDE",
                "llvmpipe",
                condition=rviz_software,
            ),
            SetEnvironmentVariable(
                "QT_XCB_GL_INTEGRATION",
                "none",
                condition=rviz_software,
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", rviz_config],
                condition=IfCondition(rviz),
                parameters=[{"use_sim_time": use_sim_time}],
                output="screen",
            ),
        ]
    )
