import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import SetEnvironmentVariable
from launch.actions import UnsetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def preview_node(
    name,
    input_topic,
    output_topic,
    max_points,
    voxel_size,
    pre_sample_factor,
    color_mode,
    publish_period,
    condition,
    min_range=0.0,
    max_range=0.0,
):
    return Node(
        package="gen0_main",
        executable="pointcloud_preview",
        name=name,
        output="screen",
        condition=condition,
        parameters=[
            {
                "input_topic": input_topic,
                "output_topic": output_topic,
                "max_points": max_points,
                "voxel_size": voxel_size,
                "pre_sample_factor": pre_sample_factor,
                "min_range": min_range,
                "max_range": max_range,
                "color_mode": color_mode,
                "publish_period": publish_period,
            }
        ],
    )


def accumulator_node(
    name,
    input_topic,
    output_topic,
    max_points,
    voxel_size,
    pre_sample_factor,
    color_mode,
    publish_period,
    condition,
):
    return Node(
        package="gen0_main",
        executable="pointcloud_accumulator_preview",
        name=name,
        output="screen",
        condition=condition,
        parameters=[
            {
                "input_topic": input_topic,
                "output_topic": output_topic,
                "max_points": max_points,
                "voxel_size": voxel_size,
                "pre_sample_factor": pre_sample_factor,
                "color_mode": color_mode,
                "publish_period": publish_period,
            }
        ],
    )


def generate_launch_description():
    package_share = get_package_share_directory("gen0_main")
    default_rviz = os.path.join(package_share, "config", "gen0_3d_mapping.rviz")

    rviz = LaunchConfiguration("rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    rviz_render_env = LaunchConfiguration("rviz_render_env")
    use_sim_time = LaunchConfiguration("use_sim_time")
    raw_front3d_input_topic = LaunchConfiguration("raw_front3d_input_topic")
    registered_preview_input_topic = LaunchConfiguration("registered_preview_input_topic")
    condition = IfCondition(rviz)
    rviz_software = IfCondition(
        PythonExpression(["'", rviz_render_env, "' != 'passthrough'"])
    )
    rviz_passthrough = IfCondition(
        PythonExpression(["'", rviz_render_env, "' == 'passthrough'"])
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "rviz",
                default_value="true",
                description="Open RViz with downsampled 3D mapping displays.",
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
                "use_sim_time",
                default_value="true",
                description="Use Gazebo simulation clock.",
            ),
            DeclareLaunchArgument(
                "raw_front3d_input_topic",
                default_value="/gen0_model/front3d/lidar/points",
                description="Raw front 3D point-cloud topic to preview in RViz.",
            ),
            DeclareLaunchArgument(
                "registered_preview_input_topic",
                default_value="/gen0_mapping/cloud_registered",
                description="Registered point-cloud topic to preview and accumulate in RViz.",
            ),
            preview_node(
                "raw_front3d_preview",
                raw_front3d_input_topic,
                "/gen0_mapping/rviz/raw_front3d",
                30000,
                0.12,
                8,
                "range",
                0.5,
                condition,
                min_range=0.5,
                max_range=80.0,
            ),
            preview_node(
                "cloud_registered_preview",
                registered_preview_input_topic,
                "/gen0_mapping/rviz/cloud_registered",
                50000,
                0.12,
                8,
                "z",
                0.5,
                condition,
            ),
            preview_node(
                "terrain_map_preview",
                "/gen0_mapping/terrain_map",
                "/gen0_mapping/rviz/terrain_map",
                180000,
                0.08,
                4,
                "terrain",
                0.2,
                condition,
            ),
            preview_node(
                "terrain_map_ext_preview",
                "/gen0_mapping/terrain_map_ext",
                "/gen0_mapping/rviz/terrain_map_ext",
                320000,
                0.06,
                2,
                "terrain",
                0.5,
                condition,
            ),
            accumulator_node(
                "fast_lio_map_preview",
                registered_preview_input_topic,
                "/gen0_mapping/rviz/fast_lio_map",
                800000,
                0.18,
                2,
                "z",
                1.0,
                condition,
            ),
            UnsetEnvironmentVariable(
                "LIBGL_ALWAYS_SOFTWARE", condition=rviz_passthrough
            ),
            UnsetEnvironmentVariable(
                "MESA_LOADER_DRIVER_OVERRIDE", condition=rviz_passthrough
            ),
            UnsetEnvironmentVariable(
                "QT_XCB_GL_INTEGRATION", condition=rviz_passthrough
            ),
            SetEnvironmentVariable(
                "LIBGL_ALWAYS_SOFTWARE", "1", condition=rviz_software
            ),
            SetEnvironmentVariable(
                "MESA_LOADER_DRIVER_OVERRIDE", "llvmpipe", condition=rviz_software
            ),
            SetEnvironmentVariable(
                "QT_XCB_GL_INTEGRATION", "none", condition=rviz_software
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="gen0_mapping_rviz",
                arguments=["-d", rviz_config],
                condition=condition,
                parameters=[{"use_sim_time": use_sim_time}],
            ),
        ]
    )
