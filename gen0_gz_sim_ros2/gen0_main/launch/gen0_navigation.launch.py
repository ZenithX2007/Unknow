import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction, SetEnvironmentVariable, UnsetEnvironmentVariable
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
    projected_map_unknown_as_free = (
        LaunchConfiguration('projected_map_unknown_as_free').perform(context).lower()
        == 'true'
    )
    namespace = LaunchConfiguration('namespace').perform(context)

    if costmap_source == 'scurm_terrain':
        global_track_unknown = (
            'false'
            if map_source == 'projected_map' and projected_map_unknown_as_free
            else 'true'
        )
        # terrain_map is already registered in the global map/odom frame.  Its
        # cloud origin is therefore not a live sensor origin; raytrace clearing
        # would incorrectly use (0, 0) and fall outside the rolling window.
        pointcloud_clearing = 'false'
        overlay = f"""local_costmap:
  local_costmap:
    ros__parameters:
      transform_tolerance: 1.0
      update_frequency: 20.0
      publish_frequency: 20.0
      global_frame: map
      rolling_window: true
      width: 30
      height: 30
      resolution: 0.05
      footprint: "[[2.0, 1.0], [2.0, -1.0], [-2.0, -1.0], [-2.0, 1.0]]"
      footprint_padding: 0.05
      track_unknown_space: false
      plugins: ["static_layer", "local_obstacle_layer", "actor_obstacle_layer", "local_inflation_layer"]
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_topic: "/map"
        map_subscribe_transient_local: true
        subscribe_to_updates: true
      local_inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 5.0
        inflation_radius: 1.2
      local_obstacle_layer:
        plugin: "costmap_intensity::ObstacleLayerIntensity"
        enabled: true
        combination_method: 1
        clear_on_update: false
        footprint_clearing_enabled: true
        max_obstacle_intensity: 2.0
        min_obstacle_intensity: 0.3
        robot_origin_fallback_enabled: false
        robot_origin_fallback_distance: 1.0
        observation_sources: pointcloud
        pointcloud:
          topic: /gen0_mapping/terrain_map
          observation_persistence: 0.0
          expected_update_rate: 0.0
          max_obstacle_height: 2.0
          min_obstacle_height: -2.0
          obstacle_max_range: 7.0
          obstacle_min_range: 0.15
          raytrace_max_range: 8.0
          raytrace_min_range: 0.1
          clearing: {pointcloud_clearing}
          marking: true
          data_type: "PointCloud2"

      actor_obstacle_layer:
        plugin: "costmap_intensity::ObstacleLayerIntensity"
        enabled: true
        combination_method: 1
        clear_on_update: true
        footprint_clearing_enabled: false
        max_obstacle_intensity: 2.0
        min_obstacle_intensity: 0.3
        robot_origin_fallback_enabled: true
        robot_origin_fallback_distance: 1.0
        observation_sources: actors
        actors:
          topic: /gen0_mapping/actor_obstacles
          observation_persistence: 0.0
          expected_update_rate: 0.0
          max_obstacle_height: 2.0
          min_obstacle_height: -2.0
          obstacle_max_range: 14.0
          obstacle_min_range: 0.15
          raytrace_max_range: 15.0
          raytrace_min_range: 0.1
          clearing: true
          marking: true
          data_type: "PointCloud2"

global_costmap:
  global_costmap:
    ros__parameters:
      footprint: "[[2.0, 1.0], [2.0, -1.0], [-2.0, -1.0], [-2.0, 1.0]]"
      footprint_padding: 0.05
      track_unknown_space: {global_track_unknown}
      plugins: ["static_layer", "global_inflation_layer"]
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: true
      global_inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 10.0
        inflation_radius: 1.2
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
    default_nav2_rviz = os.path.join(
        pkg_share_dir,
        'config',
        'gen0_nav2_default_view.rviz',
    )

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    map_yaml = LaunchConfiguration('map')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')
    nav2_controller_frequency = LaunchConfiguration('nav2_controller_frequency')
    nav2_model_dt = LaunchConfiguration('nav2_model_dt')
    nav2_smoothing_frequency = LaunchConfiguration('nav2_smoothing_frequency')
    publish_identity_map_to_odom = LaunchConfiguration('publish_identity_map_to_odom')
    odom_topic = LaunchConfiguration('odom_topic')
    reference_odom_topic = LaunchConfiguration('reference_odom_topic')
    max_reference_odom_error = LaunchConfiguration('max_reference_odom_error')
    max_reference_yaw_error = LaunchConfiguration('max_reference_yaw_error')
    reference_odom_timeout = LaunchConfiguration('reference_odom_timeout')
    map_source = LaunchConfiguration('map_source')
    map_server_topic = LaunchConfiguration('map_server_topic')
    projected_map_topic = LaunchConfiguration('projected_map_topic')
    projected_map_unknown_as_free = LaunchConfiguration('projected_map_unknown_as_free')
    projected_map_fixed_geometry = LaunchConfiguration('projected_map_fixed_geometry')
    projected_map_fixed_origin_x = LaunchConfiguration('projected_map_fixed_origin_x')
    projected_map_fixed_origin_y = LaunchConfiguration('projected_map_fixed_origin_y')
    projected_map_fixed_width = LaunchConfiguration('projected_map_fixed_width')
    projected_map_fixed_height = LaunchConfiguration('projected_map_fixed_height')
    projected_map_fixed_resolution = LaunchConfiguration('projected_map_fixed_resolution')
    rviz = LaunchConfiguration('rviz')
    rviz_config = LaunchConfiguration('rviz_config')
    rviz_render_env = LaunchConfiguration('rviz_render_env')
    rviz_software = IfCondition(
        PythonExpression(["'", rviz_render_env, "' == 'software'"])
    )
    rviz_passthrough = IfCondition(
        PythonExpression(["'", rviz_render_env, "' != 'software'"])
    )
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
                'controller_frequency': nav2_controller_frequency,
                'model_dt': nav2_model_dt,
                'smoothing_frequency': nav2_smoothing_frequency,
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
    declare_rviz = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        choices=['true', 'false'],
        description='Open the Gen0 Nav2 RViz view.',
    )
    declare_rviz_config = DeclareLaunchArgument(
        'rviz_config',
        default_value=default_nav2_rviz,
        description='RViz config adapted from SCURM nav2_default_view.rviz.',
    )
    declare_rviz_render_env = DeclareLaunchArgument(
        'rviz_render_env',
        default_value='passthrough',
        choices=['auto', 'software', 'passthrough'],
        description='RViz OpenGL environment. software uses llvmpipe; auto/passthrough keep the host GL path.',
    )
    declare_nav2_controller_frequency = DeclareLaunchArgument(
        'nav2_controller_frequency',
        default_value='20.0',
        description='Controller loop frequency for the Ackermann Nav2 stack.',
    )
    declare_nav2_model_dt = DeclareLaunchArgument(
        'nav2_model_dt',
        default_value='0.05',
        description='MPPI model timestep. It must be at least the controller period.',
    )
    declare_nav2_smoothing_frequency = DeclareLaunchArgument(
        'nav2_smoothing_frequency',
        default_value='20.0',
        description='Velocity smoother frequency. Usually match nav2_controller_frequency.',
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
    declare_projected_map_fixed_geometry = DeclareLaunchArgument(
        'projected_map_fixed_geometry',
        default_value='true',
        choices=['true', 'false'],
        description='Keep the relayed projected map geometry fixed so Nav2 StaticLayer does not resize during navigation.',
    )
    declare_projected_map_fixed_origin_x = DeclareLaunchArgument(
        'projected_map_fixed_origin_x',
        default_value='-140.0',
        description='Fixed projected map origin X in the map frame.',
    )
    declare_projected_map_fixed_origin_y = DeclareLaunchArgument(
        'projected_map_fixed_origin_y',
        default_value='-135.0',
        description='Fixed projected map origin Y in the map frame.',
    )
    declare_projected_map_fixed_width = DeclareLaunchArgument(
        'projected_map_fixed_width',
        default_value='3300',
        description='Fixed projected map width in cells.',
    )
    declare_projected_map_fixed_height = DeclareLaunchArgument(
        'projected_map_fixed_height',
        default_value='3300',
        description='Fixed projected map height in cells.',
    )
    declare_projected_map_fixed_resolution = DeclareLaunchArgument(
        'projected_map_fixed_resolution',
        default_value='0.10',
        description='Fixed projected map resolution in meters per cell.',
    )
    declare_odom_topic = DeclareLaunchArgument(
        'odom_topic',
        default_value='/odom',
        description='Odometry topic used by Nav2 controller, BT navigator, and velocity smoother.',
    )
    declare_reference_odom_topic = DeclareLaunchArgument(
        'reference_odom_topic',
        default_value='',
        description='Reference odometry for Nav2 odom health checks; empty disables the check.',
    )
    declare_max_reference_odom_error = DeclareLaunchArgument(
        'max_reference_odom_error',
        default_value='0.0',
        description='Block Nav2 cmd_vel when odom differs from the reference by more than this many meters; 0 disables.',
    )
    declare_max_reference_yaw_error = DeclareLaunchArgument(
        'max_reference_yaw_error',
        default_value='0.0',
        description='Block Nav2 cmd_vel when odom yaw differs from the reference by more than this many radians; 0 disables.',
    )
    declare_reference_odom_timeout = DeclareLaunchArgument(
        'reference_odom_timeout',
        default_value='2.0',
        description='Maximum age in seconds for Nav2 reference odometry.',
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
                        'output_cmd_vel_topic': '/cmd_vel',
                        'map_topic': '/map',
                        'odom_topic': odom_topic,
                        'reference_odom_topic': reference_odom_topic,
                        'max_reference_odom_error': ParameterValue(
                            max_reference_odom_error,
                            value_type=float,
                        ),
                        'max_reference_yaw_error': ParameterValue(
                            max_reference_yaw_error,
                            value_type=float,
                        ),
                        'reference_odom_timeout': ParameterValue(
                            reference_odom_timeout,
                            value_type=float,
                        ),
                        'map_frame': 'map',
                        'base_frame': 'base_link',
                        'bounds_margin': 5.0,
                        'max_abs_z': 20.0,
                        'min_turning_radius': 6.62,
                        'curvature_warn_period': 2.0,
                        'max_pose_jump': 3.0,
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
                        'publish_period': 1.0,
                        'unknown_as_free': ParameterValue(
                            projected_map_unknown_as_free,
                            value_type=bool,
                        ),
                        'fixed_geometry': ParameterValue(
                            projected_map_fixed_geometry,
                            value_type=bool,
                        ),
                        'fixed_origin_x': ParameterValue(
                            projected_map_fixed_origin_x,
                            value_type=float,
                        ),
                        'fixed_origin_y': ParameterValue(
                            projected_map_fixed_origin_y,
                            value_type=float,
                        ),
                        'fixed_width': ParameterValue(
                            projected_map_fixed_width,
                            value_type=int,
                        ),
                        'fixed_height': ParameterValue(
                            projected_map_fixed_height,
                            value_type=int,
                        ),
                        'fixed_resolution': ParameterValue(
                            projected_map_fixed_resolution,
                            value_type=float,
                        ),
                        'merge_fixed_history': True,
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
        declare_rviz,
        declare_rviz_config,
        declare_rviz_render_env,
        declare_nav2_controller_frequency,
        declare_nav2_model_dt,
        declare_nav2_smoothing_frequency,
        declare_publish_identity_map_to_odom,
        declare_costmap_source,
        declare_map_source,
        declare_map_server_topic,
        declare_projected_map_topic,
        declare_projected_map_unknown_as_free,
        declare_projected_map_fixed_geometry,
        declare_projected_map_fixed_origin_x,
        declare_projected_map_fixed_origin_y,
        declare_projected_map_fixed_width,
        declare_projected_map_fixed_height,
        declare_projected_map_fixed_resolution,
        OpaqueFunction(function=ensure_nav2_costmap_overlay),
        declare_odom_topic,
        declare_reference_odom_topic,
        declare_max_reference_odom_error,
        declare_max_reference_yaw_error,
        declare_reference_odom_timeout,
        declare_default_nav_to_pose_bt_xml,
        declare_default_nav_through_poses_bt_xml,
        load_nodes,
        identity_map_to_odom_node,
        UnsetEnvironmentVariable(
            'LIBGL_ALWAYS_SOFTWARE',
            condition=rviz_passthrough,
        ),
        UnsetEnvironmentVariable(
            'MESA_LOADER_DRIVER_OVERRIDE',
            condition=rviz_passthrough,
        ),
        UnsetEnvironmentVariable(
            'QT_XCB_GL_INTEGRATION',
            condition=rviz_passthrough,
        ),
        SetEnvironmentVariable(
            'LIBGL_ALWAYS_SOFTWARE',
            '1',
            condition=rviz_software,
        ),
        SetEnvironmentVariable(
            'MESA_LOADER_DRIVER_OVERRIDE',
            'llvmpipe',
            condition=rviz_software,
        ),
        SetEnvironmentVariable(
            'QT_XCB_GL_INTEGRATION',
            'none',
            condition=rviz_software,
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config, '--ros-args', '--log-level', 'warn'],
            condition=IfCondition(rviz),
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen',
        ),
    ])
