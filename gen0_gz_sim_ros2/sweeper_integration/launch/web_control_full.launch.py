import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource


def generate_launch_description():
    gen0_main_share = get_package_share_directory('gen0_main')
    sweeper_share = get_package_share_directory('sweeper_integration')
    rosbridge_share = get_package_share_directory('rosbridge_server')
    workspace_root = os.path.abspath(os.path.join(sweeper_share, '..', '..', '..', '..'))
    web_root = os.path.join(workspace_root, 'web_control')
    web_map_dir = Path(web_root, 'static_map')
    nav2_map_yaml = web_map_dir / 'prior_map_2d.yaml'
    default_nav2_map = str(nav2_map_yaml)

    world = LaunchConfiguration('world')
    gazebo_gui = LaunchConfiguration('gazebo_gui')
    rosbridge_port = LaunchConfiguration('rosbridge_port')
    web_port = LaunchConfiguration('web_port')
    nav2_map_path = LaunchConfiguration('nav2_map_path')

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='my_map',
            description='Gazebo world to load for the full web control stack.',
        ),
        DeclareLaunchArgument(
            'gazebo_gui',
            default_value='true',
            choices=['true', 'false'],
            description='Show the Gazebo GUI. Set false for headless or WSL usage.',
        ),
        DeclareLaunchArgument(
            'rosbridge_port',
            default_value='9090',
            description='WebSocket port for rosbridge.',
        ),
        DeclareLaunchArgument(
            'web_port',
            default_value='8000',
            description='Port used by the static web server.',
        ),
        DeclareLaunchArgument(
            'nav2_map_path',
            default_value=default_nav2_map,
            description='Static OccupancyGrid YAML regenerated from prior_map.pcd.',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gen0_main_share, 'launch', 'spawn.launch.py')
            ),
            launch_arguments={
                'world': world,
                'rviz': 'false',
                'gazebo_gui': gazebo_gui,
                'ground_truth_localization': 'false',
                'static_odom_base': 'false',
            }.items(),
        ),
        TimerAction(
            period=6.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(sweeper_share, 'launch', 'interfaces.launch.py')
                    )
                )
            ],
        ),
        TimerAction(
            period=10.0,
            actions=[
                IncludeLaunchDescription(
                    XMLLaunchDescriptionSource(
                        os.path.join(rosbridge_share, 'launch', 'rosbridge_websocket_launch.xml')
                    ),
                    launch_arguments={
                        'port': rosbridge_port,
                        'send_action_goals_in_new_thread': 'true',
                    }.items(),
                )
            ],
        ),
        TimerAction(
            period=12.0,
            actions=[
                ExecuteProcess(
                    cmd=['python3', '-m', 'http.server', web_port, '--directory', web_root],
                    output='screen',
                )
            ],
        ),
        TimerAction(
            period=8.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(gen0_main_share, 'launch', 'gen0_navigation.launch.py')
                    ),
                    launch_arguments={
                        'map': nav2_map_path,
                        'map_source': 'yaml',
                        'publish_identity_map_to_odom': 'true',
                        'rviz': 'false',
                        'autostart': 'true',
                    }.items(),
                )
            ],
        ),
    ])
