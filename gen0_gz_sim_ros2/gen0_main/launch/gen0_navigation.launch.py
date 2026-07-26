import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def default_scurm_map():
    map_path = os.path.join(
        os.path.expanduser('~'),
        'SCURM_SentryNavigation',
        'sentry_bringup',
        'maps',
        'test_map.yaml',
    )
    return map_path if os.path.exists(map_path) else ''


def generate_launch_description():
    pkg_share_dir = get_package_share_directory('gen0_main')

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    map_yaml = LaunchConfiguration('map')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')
    start_vehicle_interface = LaunchConfiguration('start_vehicle_interface')

    lifecycle_nodes = [
        'map_server',
        'controller_server',
        'smoother_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
        'velocity_smoother',
    ]

    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites={
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'yaml_filename': map_yaml,
            },
            convert_types=True,
        ),
        allow_substs=True,
    )

    declare_namespace = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace.',
    )
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        choices=['true', 'false'],
        description='Use the Gazebo simulation clock.',
    )
    declare_map = DeclareLaunchArgument(
        'map',
        default_value=default_scurm_map(),
        description='Full path to a Nav2 occupancy-map YAML file.',
    )
    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_share_dir, 'config', 'nav2_gen0_params.yaml'),
        description='Full path to the gen0 Nav2 parameter file.',
    )
    declare_autostart = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        choices=['true', 'false'],
        description='Automatically configure and activate Nav2 lifecycle nodes.',
    )
    declare_use_respawn = DeclareLaunchArgument(
        'use_respawn',
        default_value='true',
        choices=['true', 'false'],
        description='Respawn Nav2 nodes if they crash.',
    )
    declare_log_level = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='ROS log level.',
    )
    declare_start_vehicle_interface = DeclareLaunchArgument(
        'start_vehicle_interface',
        default_value='true',
        choices=['true', 'false'],
        description='Start the gen0 Twist-to-vehicle-joint command adapter.',
    )

    load_nodes = GroupAction(
        actions=[
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
            ),
            Node(
                package='nav2_controller',
                executable='controller_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
            ),
            Node(
                package='nav2_smoother',
                executable='smoother_server',
                name='smoother_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
            ),
            Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
            ),
            Node(
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
            ),
            Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
            ),
            Node(
                package='nav2_waypoint_follower',
                executable='waypoint_follower',
                name='waypoint_follower',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
            ),
            Node(
                package='nav2_velocity_smoother',
                executable='velocity_smoother',
                name='velocity_smoother',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings
                + [('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', '/control/cmd_vel')],
            ),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                arguments=['--ros-args', '--log-level', log_level],
                parameters=[
                    {'use_sim_time': use_sim_time},
                    {'autostart': autostart},
                    {'node_names': lifecycle_nodes},
                ],
            ),
        ],
    )

    vehicle_interface_node = Node(
        package='gen0_interface',
        executable='cmdvel_to_vehicle',
        name='cmdvel_to_vehicle',
        output='screen',
        parameters=[{'cmd_vel_topic': '/control/cmd_vel'}],
        condition=IfCondition(start_vehicle_interface),
    )

    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        declare_namespace,
        declare_use_sim_time,
        declare_map,
        declare_params_file,
        declare_autostart,
        declare_use_respawn,
        declare_log_level,
        declare_start_vehicle_interface,
        load_nodes,
        vehicle_interface_node,
    ])
