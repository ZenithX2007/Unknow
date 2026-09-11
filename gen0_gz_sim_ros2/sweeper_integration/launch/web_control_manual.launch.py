import os

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
from launch_ros.actions import Node
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource


def generate_launch_description():
    gen0_main_share = get_package_share_directory('gen0_main')
    sweeper_share = get_package_share_directory('sweeper_integration')
    rosbridge_share = get_package_share_directory('rosbridge_server')
    workspace_root = os.path.abspath(
        os.path.join(sweeper_share, '..', '..', '..', '..'))
    web_root = os.path.join(workspace_root, 'web_control')

    world = LaunchConfiguration('world')
    gazebo_gui = LaunchConfiguration('gazebo_gui')
    render_engine = LaunchConfiguration('render_engine')
    render_env = LaunchConfiguration('render_env')
    rosbridge_port = LaunchConfiguration('rosbridge_port')
    web_port = LaunchConfiguration('web_port')
    ground_truth_odometry = LaunchConfiguration('ground_truth_odometry')

    return LaunchDescription([
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
        DeclareLaunchArgument(
            'ground_truth_odometry', default_value='false',
            choices=['true', 'false']),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gen0_main_share, 'launch', 'spawn.launch.py')),
            launch_arguments={
                'world': world,
                'rviz': 'false',
                'camera_view': 'false',
                'robot_state_publisher': 'false',
                'gazebo_gui': gazebo_gui,
                'render_engine': render_engine,
                'render_env': render_env,
                'ground_truth_localization': 'false',
                'static_odom_base': 'false',
            }.items(),
        ),
        TimerAction(
            period=6.0,
            actions=[IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(sweeper_share, 'launch', 'interfaces.launch.py')),
                launch_arguments={
                    'ground_truth_odometry': ground_truth_odometry,
                }.items(),
            )],
        ),
        TimerAction(
            period=8.0,
            actions=[IncludeLaunchDescription(
                XMLLaunchDescriptionSource(os.path.join(
                    rosbridge_share, 'launch', 'rosbridge_websocket_launch.xml')),
                launch_arguments={
                    'port': rosbridge_port,
                    'send_action_goals_in_new_thread': 'true',
                }.items(),
            )],
        ),
        TimerAction(
            period=9.0,
            actions=[Node(
                package='gen0_main',
                executable='camera_compressor',
                name='driver_camera_compressor',
                output='screen',
                parameters=[{
                    'input_topic': '/gen0_model/driver_camera',
                    'output_topic': '/gen0_model/driver_camera/compressed',
                    'max_width': 640,
                    'max_height': 360,
                    'max_fps': 8.0,
                    'jpeg_quality': 58,
                }],
            ), Node(
                package='gen0_main',
                executable='camera_compressor',
                name='scene_overview_compressor',
                output='screen',
                parameters=[{
                    'input_topic': '/scene_overview',
                    'output_topic': '/scene_overview/compressed',
                    'max_width': 960,
                    'max_height': 600,
                    'max_fps': 2.0,
                    'jpeg_quality': 65,
                }],
            )],
        ),
        TimerAction(
            period=10.0,
            actions=[ExecuteProcess(
                cmd=['python3', '-m', 'http.server', web_port,
                     '--directory', web_root],
                output='screen',
            )],
        ),
    ])
