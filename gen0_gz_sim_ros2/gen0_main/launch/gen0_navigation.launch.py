import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile, ParameterValue
from nav2_common.launch import RewrittenYaml

DEFAULT_GEN0_NAV_MAP_YAML = '/tmp/gen0_my_map_nav.yaml'
DEFAULT_GEN0_NAV_MAP_IMAGE = '/tmp/gen0_my_map_nav.pgm'
DEFAULT_GEN0_NAV_COSTMAP_OVERLAY = '/tmp/gen0_nav2_costmap_overlay.yaml'


def _scope_yaml(namespace, body):
    if not namespace:
        return body
    indented = '\n'.join(f'  {line}' if line else line for line in body.splitlines())
    return f'{namespace}:\n{indented}\n'


def ensure_default_gen0_nav_map(context, *args, **kwargs):
    map_yaml = LaunchConfiguration('map').perform(context)
    if map_yaml != DEFAULT_GEN0_NAV_MAP_YAML:
        return []

    width = 1600
    height = 1600
    resolution = 0.20
    origin_x = -140.0
    origin_y = -190.0

    os.makedirs(os.path.dirname(DEFAULT_GEN0_NAV_MAP_YAML), exist_ok=True)

    with open(DEFAULT_GEN0_NAV_MAP_IMAGE, 'wb') as image_file:
        image_file.write(f'P5\n{width} {height}\n255\n'.encode('ascii'))
        row = b'\xfe' * width
        for _ in range(height):
            image_file.write(row)

    with open(DEFAULT_GEN0_NAV_MAP_YAML, 'w', encoding='utf-8') as yaml_file:
        yaml_file.write(
            f'image: {DEFAULT_GEN0_NAV_MAP_IMAGE}\n'
            'mode: trinary\n'
            f'resolution: {resolution}\n'
            f'origin: [{origin_x}, {origin_y}, 0.0]\n'
            'negate: 0\n'
            'occupied_thresh: 0.65\n'
            'free_thresh: 0.25\n'
        )

    return []


def ensure_nav2_costmap_overlay(context, *args, **kwargs):
    costmap_source = LaunchConfiguration('costmap_source').perform(context)
    map_source = LaunchConfiguration('map_source').perform(context)
    namespace = LaunchConfiguration('namespace').perform(context)

    if costmap_source == 'scurm_terrain':
        global_track_unknown = 'false' if map_source == 'projected_map' else 'true'
        overlay = f"""local_costmap:
  local_costmap:
    ros__parameters:
      transform_tolerance: 0.5
      update_frequency: 20.0
      publish_frequency: 20.0
      global_frame: map
      track_unknown_space: false
      plugins: ["local_obstacle_layer", "inflation_layer"]
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 5.0
        inflation_radius: 1.10
      local_obstacle_layer:
        plugin: "costmap_intensity::ObstacleLayerIntensity"
        enabled: true
        footprint_clearing_enabled: true
        max_obstacle_intensity: 2.0
        min_obstacle_intensity: 0.3
        observation_sources: pointcloud
        pointcloud:
          topic: /gen0_mapping/terrain_map
          observation_persistence: 0.2
          expected_update_rate: 0.0
          max_obstacle_height: 2.0
          min_obstacle_height: -2.0
          obstacle_max_range: 7.0
          obstacle_min_range: 0.15
          raytrace_max_range: 8.0
          raytrace_min_range: 0.1
          clearing: false
          marking: true
          data_type: "PointCloud2"

global_costmap:
  global_costmap:
    ros__parameters:
      track_unknown_space: {global_track_unknown}
      plugins: ["static_layer", "global_inflation_layer"]
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: true
      global_inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 10.0
        inflation_radius: 1.10
"""
    else:
        overlay = """local_costmap:
  local_costmap:
    ros__parameters: {}
global_costmap:
  global_costmap:
    ros__parameters: {}
"""

    with open(DEFAULT_GEN0_NAV_COSTMAP_OVERLAY, 'w', encoding='utf-8') as yaml_file:
        yaml_file.write(_scope_yaml(namespace, overlay))

    return []


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
    vehicle_angular_z_sign = LaunchConfiguration('vehicle_angular_z_sign')
    vehicle_max_forward_speed = LaunchConfiguration('vehicle_max_forward_speed')
    vehicle_max_reverse_speed = LaunchConfiguration('vehicle_max_reverse_speed')
    vehicle_max_angular_z = LaunchConfiguration('vehicle_max_angular_z')
    vehicle_front_stop_enabled = LaunchConfiguration('vehicle_front_stop_enabled')
    vehicle_front_stop_distance = LaunchConfiguration('vehicle_front_stop_distance')
    vehicle_front_slow_distance = LaunchConfiguration('vehicle_front_slow_distance')
    publish_identity_map_to_odom = LaunchConfiguration('publish_identity_map_to_odom')
    odom_topic = LaunchConfiguration('odom_topic')
    map_source = LaunchConfiguration('map_source')
    map_server_topic = LaunchConfiguration('map_server_topic')
    projected_map_topic = LaunchConfiguration('projected_map_topic')
    projected_map_unknown_as_free = LaunchConfiguration('projected_map_unknown_as_free')
    default_nav_to_pose_bt_xml = LaunchConfiguration('default_nav_to_pose_bt_xml')
    default_nav_through_poses_bt_xml = LaunchConfiguration('default_nav_through_poses_bt_xml')

    scurm_nav_to_pose_bt = os.path.join(
        pkg_share_dir,
        'behavior_tree',
        'ackermann_forward_recovery.xml',
    )
    scurm_nav_through_poses_bt = os.path.join(
        pkg_share_dir,
        'behavior_tree',
        'navigate_through_poses_ackermann_forward_recovery.xml',
    )

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
                'odom_topic': odom_topic,
                'default_nav_to_pose_bt_xml': default_nav_to_pose_bt_xml,
                'default_nav_through_poses_bt_xml': default_nav_through_poses_bt_xml,
            },
            convert_types=True,
        ),
        allow_substs=True,
    )
    costmap_overlay_params = DEFAULT_GEN0_NAV_COSTMAP_OVERLAY

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
        default_value=DEFAULT_GEN0_NAV_MAP_YAML,
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
    declare_vehicle_angular_z_sign = DeclareLaunchArgument(
        'vehicle_angular_z_sign',
        default_value='1.0',
        description='Multiplier applied to Twist.angular.z before converting to Gen0 steering joints.',
    )
    declare_vehicle_max_forward_speed = DeclareLaunchArgument(
        'vehicle_max_forward_speed',
        default_value='0.65',
        description='Maximum forward velocity accepted by the Gen0 vehicle adapter.',
    )
    declare_vehicle_max_reverse_speed = DeclareLaunchArgument(
        'vehicle_max_reverse_speed',
        default_value='0.25',
        description='Maximum reverse velocity accepted by the Gen0 vehicle adapter.',
    )
    declare_vehicle_max_angular_z = DeclareLaunchArgument(
        'vehicle_max_angular_z',
        default_value='0.12',
        description='Maximum Twist.angular.z magnitude accepted by the Gen0 vehicle adapter.',
    )
    declare_vehicle_front_stop_enabled = DeclareLaunchArgument(
        'vehicle_front_stop_enabled',
        default_value='false',
        choices=['true', 'false'],
        description='Enable optional front laser hard-stop protection in the Gen0 vehicle adapter.',
    )
    declare_vehicle_front_stop_distance = DeclareLaunchArgument(
        'vehicle_front_stop_distance',
        default_value='0.65',
        description='Stop forward vehicle commands when front laser clearance is below this distance.',
    )
    declare_vehicle_front_slow_distance = DeclareLaunchArgument(
        'vehicle_front_slow_distance',
        default_value='1.5',
        description='Start scaling down forward vehicle commands below this front laser clearance.',
    )
    declare_publish_identity_map_to_odom = DeclareLaunchArgument(
        'publish_identity_map_to_odom',
        default_value='false',
        choices=['true', 'false'],
        description='Publish identity map -> odom for odom-only Nav2 validation without ICP/prior map.',
    )
    declare_costmap_source = DeclareLaunchArgument(
        'costmap_source',
        default_value='laser_scan',
        choices=['laser_scan', 'scurm_terrain'],
        description='Nav2 obstacle source: Gen0 2D LaserScan or SCURM terrain_map.',
    )
    declare_map_source = DeclareLaunchArgument(
        'map_source',
        default_value='yaml',
        choices=['yaml', 'projected_map'],
        description='Static map source for Nav2: YAML map_server or relayed online /projected_map.',
    )
    declare_map_server_topic = DeclareLaunchArgument(
        'map_server_topic',
        default_value='map',
        description='Topic name used by nav2_map_server. Use an unused topic when map_source=projected_map.',
    )
    declare_projected_map_topic = DeclareLaunchArgument(
        'projected_map_topic',
        default_value='/projected_map',
        description='Online occupancy grid to relay to /map when map_source=projected_map.',
    )
    declare_projected_map_unknown_as_free = DeclareLaunchArgument(
        'projected_map_unknown_as_free',
        default_value='false',
        choices=['true', 'false'],
        description='Convert unknown cells in projected_map to free cells for mapless SCURM validation.',
    )
    declare_odom_topic = DeclareLaunchArgument(
        'odom_topic',
        default_value='/odom',
        description='Odometry topic used by Nav2 controller, BT navigator, and velocity smoother.',
    )
    declare_default_nav_to_pose_bt_xml = DeclareLaunchArgument(
        'default_nav_to_pose_bt_xml',
        default_value=scurm_nav_to_pose_bt,
        description='Default NavigateToPose behavior tree XML.',
    )
    declare_default_nav_through_poses_bt_xml = DeclareLaunchArgument(
        'default_nav_through_poses_bt_xml',
        default_value=scurm_nav_through_poses_bt,
        description='Default NavigateThroughPoses behavior tree XML.',
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
                parameters=[configured_params, costmap_overlay_params, {'topic_name': map_server_topic}],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
            ),
            Node(
                package='nav2_controller',
                executable='controller_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params, costmap_overlay_params],
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
                parameters=[configured_params, costmap_overlay_params],
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
                parameters=[configured_params, costmap_overlay_params],
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
                parameters=[configured_params, costmap_overlay_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
            ),
            Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params, costmap_overlay_params],
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
                parameters=[configured_params, costmap_overlay_params],
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
                parameters=[configured_params, costmap_overlay_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings
                + [('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', '/control/cmd_vel_raw')],
            ),
            Node(
                package='gen0_main',
                executable='nav2_pose_guard',
                name='nav2_pose_guard',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                arguments=['--ros-args', '--log-level', log_level],
                parameters=[
                    {
                        'use_sim_time': use_sim_time,
                        'input_cmd_vel_topic': '/control/cmd_vel_raw',
                        'output_cmd_vel_topic': '/control/cmd_vel',
                        'map_topic': '/map',
                        'odom_topic': odom_topic,
                        'map_frame': 'map',
                        'base_frame': 'base_link',
                        'bounds_margin': 5.0,
                        'max_abs_z': 20.0,
                    }
                ],
            ),
            Node(
                package='gen0_main',
                executable='nav2_projected_map_relay',
                name='nav2_projected_map_relay',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                condition=IfCondition(PythonExpression(["'", map_source, "' == 'projected_map'"])),
                arguments=['--ros-args', '--log-level', log_level],
                parameters=[
                    {
                        'use_sim_time': use_sim_time,
                        'input_topic': projected_map_topic,
                        'output_topic': '/map',
                        'output_frame': 'map',
                        'unknown_as_free': ParameterValue(
                            projected_map_unknown_as_free,
                            value_type=bool,
                        ),
                    }
                ],
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
        parameters=[
            {
                'cmd_vel_topic': '/control/cmd_vel',
                'angular_z_sign': ParameterValue(vehicle_angular_z_sign, value_type=float),
                'max_forward_speed': ParameterValue(vehicle_max_forward_speed, value_type=float),
                'max_reverse_speed': ParameterValue(vehicle_max_reverse_speed, value_type=float),
                'max_angular_z': ParameterValue(vehicle_max_angular_z, value_type=float),
                'front_stop_enabled': ParameterValue(vehicle_front_stop_enabled, value_type=bool),
                'front_stop_distance': ParameterValue(vehicle_front_stop_distance, value_type=float),
                'front_slow_distance': ParameterValue(vehicle_front_slow_distance, value_type=float),
            }
        ],
        condition=IfCondition(start_vehicle_interface),
    )

    identity_map_to_odom_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='identity_map_to_odom',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        condition=IfCondition(publish_identity_map_to_odom),
    )

    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        declare_namespace,
        declare_use_sim_time,
        declare_map,
        OpaqueFunction(function=ensure_default_gen0_nav_map),
        declare_params_file,
        declare_autostart,
        declare_use_respawn,
        declare_log_level,
        declare_start_vehicle_interface,
        declare_vehicle_angular_z_sign,
        declare_vehicle_max_forward_speed,
        declare_vehicle_max_reverse_speed,
        declare_vehicle_max_angular_z,
        declare_vehicle_front_stop_enabled,
        declare_vehicle_front_stop_distance,
        declare_vehicle_front_slow_distance,
        declare_publish_identity_map_to_odom,
        declare_costmap_source,
        declare_map_source,
        declare_map_server_topic,
        declare_projected_map_topic,
        declare_projected_map_unknown_as_free,
        OpaqueFunction(function=ensure_nav2_costmap_overlay),
        declare_odom_topic,
        declare_default_nav_to_pose_bt_xml,
        declare_default_nav_through_poses_bt_xml,
        load_nodes,
        identity_map_to_odom_node,
        vehicle_interface_node,
    ])
