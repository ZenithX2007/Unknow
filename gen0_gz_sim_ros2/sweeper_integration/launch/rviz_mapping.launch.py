from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_config = PathJoinSubstitution([
        FindPackageShare('sweeper_integration'),
        'rviz',
        'mapping_map_only.rviz',
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'rviz_config',
            default_value=default_config,
            description='RViz config file for lightweight mapping display.',
        ),
        SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '1'),
        SetEnvironmentVariable('MESA_LOADER_DRIVER_OVERRIDE', 'llvmpipe'),
        SetEnvironmentVariable('GALLIUM_DRIVER', 'llvmpipe'),
        SetEnvironmentVariable('MESA_GL_VERSION_OVERRIDE', '3.3COMPAT'),
        SetEnvironmentVariable('QT_XCB_GL_INTEGRATION', 'none'),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_mapping',
            output='screen',
            arguments=['-d', LaunchConfiguration('rviz_config')],
            parameters=[{'use_sim_time': True}],
        ),
    ])
