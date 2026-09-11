"""Attach the mobile web UI, cameras, YOLO and LLM agent to a running stack."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource


def generate_launch_description():
    sweeper_share = get_package_share_directory('sweeper_integration')
    rosbridge_share = get_package_share_directory('rosbridge_server')
    llm_share = get_package_share_directory('gen0_llm_agent')
    workspace_root = os.path.abspath(
        os.path.join(sweeper_share, '..', '..', '..', '..'))
    web_root = os.path.join(workspace_root, 'web_control')

    rosbridge_port = LaunchConfiguration('rosbridge_port')
    rosbridge_address = LaunchConfiguration('rosbridge_address')
    web_port = LaunchConfiguration('web_port')
    provider = LaunchConfiguration('llm_provider')
    enable_yolo = LaunchConfiguration('enable_yolo')
    yolo_model = LaunchConfiguration('yolo_model')

    return LaunchDescription([
        SetEnvironmentVariable('PATH', '/usr/bin:/bin:/usr/sbin:/sbin'),
        DeclareLaunchArgument('rosbridge_port', default_value='9090'),
        DeclareLaunchArgument(
            'rosbridge_address', default_value='0.0.0.0',
            description='Listen address; 0.0.0.0 permits same-LAN phone access'),
        DeclareLaunchArgument('web_port', default_value='8000'),
        DeclareLaunchArgument('llm_provider', default_value='mock'),
        DeclareLaunchArgument(
            'enable_yolo', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument(
            'yolo_model', default_value=os.path.join(workspace_root, 'best_road.pt')),
        IncludeLaunchDescription(
            XMLLaunchDescriptionSource(os.path.join(
                rosbridge_share, 'launch', 'rosbridge_websocket_launch.xml')),
            launch_arguments={
                'port': rosbridge_port,
                'address': rosbridge_address,
                'send_action_goals_in_new_thread': 'true',
            }.items(),
        ),
        Node(
            package='gen0_main', executable='camera_compressor',
            name='driver_camera_compressor', output='screen',
            parameters=[{
                'input_topic': '/gen0_model/driver_camera',
                'output_topic': '/gen0_model/driver_camera/compressed',
                'max_width': 640, 'max_height': 360,
                'max_fps': 8.0, 'jpeg_quality': 58,
            }],
        ),
        Node(
            package='gen0_main', executable='camera_compressor',
            name='scene_overview_compressor', output='screen',
            parameters=[{
                'input_topic': '/scene_overview',
                'output_topic': '/scene_overview/compressed',
                'max_width': 960, 'max_height': 600,
                'max_fps': 2.0, 'jpeg_quality': 65,
            }],
        ),
        Node(
            package='yolo_detector', executable='yolo_node',
            name='yolo_detector', output='screen',
            condition=IfCondition(enable_yolo),
            parameters=[{'model_path': yolo_model}],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                llm_share, 'launch', 'llm_agent.launch.py')),
            launch_arguments={
                'provider': provider,
                'use_sim_time': 'true',
            }.items(),
        ),
        ExecuteProcess(
            cmd=['python3', '-m', 'http.server', web_port,
                 '--directory', web_root],
            output='screen',
        ),
    ])
