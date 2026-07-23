from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, LogInfo, IncludeLaunchDescription, ExecuteProcess, OpaqueFunction 
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, TextSubstitution, PythonExpression
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os


def actors_launch(context, *args, **kwargs):
    actors_scenario = LaunchConfiguration('actors_scenario').perform(context)
    world = LaunchConfiguration('world').perform(context)

    if not actors_scenario:
        return []

    pkg_share_dir = get_package_share_directory('gen0_main')
    scenario_file_path = os.path.join(pkg_share_dir, 'worlds', 'scenarios', world, f"{actors_scenario}.sdf")

    actions = []

    if not os.path.exists(scenario_file_path):
        actions.append(LogInfo(msg=f"\033[93m[WARNING] Scenario {actors_scenario} does not exist for world {world}\033[0m"))
        return actions

    actors_launch_file = os.path.join(pkg_share_dir, 'launch', 'actors.launch.py')
    # If the file exists, include the actors.launch.py and add it to the actions list
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(actors_launch_file),
        launch_arguments={
            'world': world,
            'actors_scenario': actors_scenario
        }.items(),
    ))
        
    return actions


def generate_launch_description():
    # Launch Arugments
    world_arg=DeclareLaunchArgument(
            'world',
            default_value='my_map',
            description='Name of the world file (without extension) to be used in Gazebo simulation'
    )
    sim_arg=DeclareLaunchArgument(
            'use_sim_time', 
            default_value='true', 
            choices=['true', 'false']
    )
    actors_arg=DeclareLaunchArgument(
            'actors_scenario', 
            default_value= "", 
            description='The scenario for pedestrians (without extension to be used in Gazebo world file)'
    )
    rviz_arg=DeclareLaunchArgument(
            'rviz', 
            default_value= "false", 
            choices=['true', 'false']
    )
    gazebo_gui_arg=DeclareLaunchArgument(
            'gazebo_gui',
            default_value= "true",
            choices=['true', 'false'],
            description='Start the Gazebo GUI. Set false for WSL/headless mapping runs.'
    )
    start_gazebo_arg=DeclareLaunchArgument(
            'start_gazebo',
            default_value= "true",
            choices=['true', 'false'],
            description='Start Gazebo from this launch file. Set false when Gazebo is started manually.'
    )
    ground_turth_arg=DeclareLaunchArgument(
            'ground_truth_localization', 
            default_value= "false", 
            choices=['true', 'false']
    )

    # Paths
    pkg_share_dir = get_package_share_directory('gen0_main')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    bridge_file= PathJoinSubstitution([pkg_share_dir, 'config', 'bridge.yaml'])
    world_file= PathJoinSubstitution([pkg_share_dir, 'worlds/', LaunchConfiguration('world'), PythonExpression(["'", LaunchConfiguration('world'), "'", ' + ".sdf"'])])
    vehicle_file=os.path.join(pkg_share_dir, 'urdf', 'gen0_model.sdf')
    os.environ['IGN_GAZEBO_RESOURCE_PATH']= pkg_share_dir + "/meshes" # Load the meshes to the gazebo server

    # Files
    with open(vehicle_file, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        world_arg,
        sim_arg,
        actors_arg,
        rviz_arg,
        gazebo_gui_arg,
        start_gazebo_arg,
        ground_turth_arg,
        OpaqueFunction(function=actors_launch),
        ExecuteProcess(
            cmd=[
                'ign',
                'gazebo',
                '-r',
                '--force-version',
                '6',
                '--render-engine',
                'ogre',
                world_file,
            ],
            output='screen',
            condition=IfCondition(PythonExpression([
                "'", LaunchConfiguration('start_gazebo'), "' == 'true' and '",
                LaunchConfiguration('gazebo_gui'), "' == 'true'",
            ])),
        ),
        ExecuteProcess(
            cmd=[
                'ign',
                'gazebo',
                '-r',
                '-s',
                '--headless-rendering',
                '--force-version',
                '6',
                '--render-engine-server',
                'ogre',
                world_file,
            ],
            output='screen',
            condition=IfCondition(PythonExpression([
                "'", LaunchConfiguration('start_gazebo'), "' == 'true' and '",
                LaunchConfiguration('gazebo_gui'), "' == 'false'",
            ])),
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            parameters=[{
                'config_file': bridge_file,
                'qos_overrides./tf_static.publisher.durability': 'transient_local',
            }],
            output='screen'
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='both',
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
                {'robot_description': robot_desc},
            ]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(pkg_share_dir, 'config', 'gen0_main.rviz')],
            condition=IfCondition(LaunchConfiguration('rviz')),
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
        ),
        Node(
            package='gen0_main',
            executable='ground_truth_publisher',
            condition=IfCondition(LaunchConfiguration('ground_truth_localization')),
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
        ),
    ])
