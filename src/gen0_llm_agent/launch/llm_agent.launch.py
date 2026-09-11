from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    share = get_package_share_directory('gen0_llm_agent')
    return LaunchDescription([
        DeclareLaunchArgument('provider', default_value='mock'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        Node(package='gen0_llm_agent', executable='llm_node', name='gen0_llm_agent',
             output='screen', parameters=[os.path.join(share, 'config', 'llm_agent.yaml'), {
                 'provider': LaunchConfiguration('provider'),
                 'use_sim_time': LaunchConfiguration('use_sim_time'),
             }]),
    ])
