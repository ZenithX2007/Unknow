import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    sweeper_share = get_package_share_directory('sweeper_integration')
    llm_share = get_package_share_directory('gen0_llm_agent')

    manual_launch = os.path.join(
        sweeper_share, 'launch', 'web_control_manual.launch.py')
    navigation_launch = os.path.join(
        sweeper_share, 'launch', 'navigation_online_slam.launch.py')
    llm_launch = os.path.join(
        llm_share, 'launch', 'llm_agent.launch.py')

    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        # AutoDL opens terminals in Conda base (Python 3.12), while ROS Humble
        # rclpy is built for Ubuntu's Python 3.10. Keep every ROS child on the
        # system interpreter even when the parent shell has Conda activated.
        SetEnvironmentVariable(
            'PATH', '/usr/bin:/bin:/usr/sbin:/sbin'),
        DeclareLaunchArgument('world', default_value='my_map'),
        DeclareLaunchArgument(
            'gazebo_gui', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument(
            'render_engine', default_value='ogre2', choices=['ogre', 'ogre2']),
        DeclareLaunchArgument(
            'render_env', default_value='unset',
            choices=['auto', 'unset', 'software', 'passthrough']),
        DeclareLaunchArgument('rosbridge_port', default_value='9090'),
        DeclareLaunchArgument('web_port', default_value='8000'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('llm_provider', default_value='mock'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(manual_launch),
            launch_arguments={
                'world': LaunchConfiguration('world'),
                'gazebo_gui': LaunchConfiguration('gazebo_gui'),
                'render_engine': LaunchConfiguration('render_engine'),
                'render_env': LaunchConfiguration('render_env'),
                'rosbridge_port': LaunchConfiguration('rosbridge_port'),
                'web_port': LaunchConfiguration('web_port'),
            }.items(),
        ),
        TimerAction(
            period=7.0,
            actions=[IncludeLaunchDescription(
                PythonLaunchDescriptionSource(navigation_launch),
                launch_arguments={'use_sim_time': use_sim_time}.items(),
            )],
        ),
        TimerAction(
            period=9.0,
            actions=[IncludeLaunchDescription(
                PythonLaunchDescriptionSource(llm_launch),
                launch_arguments={
                    'provider': LaunchConfiguration('llm_provider'),
                    'use_sim_time': use_sim_time,
                }.items(),
            )],
        ),
    ])
