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


def terrain_analysis_node(use_sim_time, odom_topic):
    return Node(
        package="terrain_analysis",
        executable="terrainAnalysis",
        name="gen0_scurm_terrain_analysis",
        output="screen",
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
            ("/state_estimation", odom_topic),
            ("/registered_scan", "/gen0_mapping/cloud_registered"),
            ("/terrain_map", "/gen0_mapping/terrain_map"),
        ],
    )


def generate_launch_description():
    package_share = get_package_share_directory("gen0_main")
    default_rviz = os.path.join(package_share, "config", "gen0_3d_mapping.rviz")
    default_costmap_params = os.path.join(
        package_share, "config", "scurm_local_costmap_gen0.yaml"
    )
    rviz_launch = os.path.join(package_share, "launch", "gen0_3d_rviz.launch.py")

    use_sim_time = LaunchConfiguration("use_sim_time")
    odom_topic = LaunchConfiguration("odom_topic")
    rviz = LaunchConfiguration("rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    costmap_params_file = LaunchConfiguration("costmap_params_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use Gazebo simulation clock.",
            ),
            DeclareLaunchArgument(
                "odom_topic",
                default_value="/gen0_mapping/fast_lio/odom",
                description="Odometry topic used by SCURM terrain_analysis.",
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="true",
                description="Open RViz with SCURM terrain and local costmap displays.",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=default_rviz,
                description="RViz config for SCURM-style mapping verification.",
            ),
            DeclareLaunchArgument(
                "costmap_params_file",
                default_value=default_costmap_params,
                description="SCURM local costmap parameters adapted for gen0.",
            ),
            terrain_analysis_node(use_sim_time, odom_topic),
            TimerAction(
                period=2.0,
                actions=[
                    Node(
                        package="nav2_costmap_2d",
                        executable="nav2_costmap_2d",
                        namespace="costmap",
                        name="costmap",
                        output="screen",
                        parameters=[costmap_params_file, {"use_sim_time": use_sim_time}],
                    ),
                    Node(
                        package="nav2_lifecycle_manager",
                        executable="lifecycle_manager",
                        name="lifecycle_manager_costmap",
                        output="screen",
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
