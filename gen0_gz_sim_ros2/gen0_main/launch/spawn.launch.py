import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, LogInfo
from launch.actions import OpaqueFunction
from launch.actions import SetEnvironmentVariable
from launch.actions import TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node


GUI_ENV_TO_CLEAR = (
    'LIBGL_ALWAYS_SOFTWARE',
    'MESA_LOADER_DRIVER_OVERRIDE',
    'GALLIUM_DRIVER',
    'MESA_GL_VERSION_OVERRIDE',
    'QT_XCB_GL_INTEGRATION',
)


def prepend_env_path(env, name, value):
    existing = env.get(name, os.environ.get(name, ''))
    paths = [path for path in existing.split(os.pathsep) if path and path != value]
    env[name] = os.pathsep.join([value] + paths)


def launch_bool(context, name, default='false'):
    value = LaunchConfiguration(name).perform(context).strip().lower()
    if not value:
        value = default
    return value == 'true'


def gazebo_environment(
    pkg_share_dir,
    partition,
    render_env,
    d3d12_adapter='',
    actor_soft_stop='false',
    actor_soft_stop_margin='0.25',
    actor_soft_stop_release_margin='0.85',
    actor_soft_stop_vehicle_name='gen0_model',
):
    env = dict(os.environ)
    for resource_path in (
        os.path.join(pkg_share_dir, 'meshes'),
        os.path.join(pkg_share_dir, 'models'),
    ):
        prepend_env_path(env, 'IGN_GAZEBO_RESOURCE_PATH', resource_path)
        prepend_env_path(env, 'GZ_SIM_RESOURCE_PATH', resource_path)
    env['IGN_PARTITION'] = partition
    env['GZ_PARTITION'] = partition

    workspace_root = os.path.abspath(os.path.join(pkg_share_dir, '..', '..', '..', '..'))
    actor_pose_plugin_paths = (
        os.path.join(workspace_root, 'gen0_gz_sim_ros2', 'gz_plugins', 'build'),
        os.path.join(workspace_root, 'build', 'ActorPose'),
    )
    for actor_pose_plugin_path in actor_pose_plugin_paths:
        if any(
            os.path.exists(os.path.join(actor_pose_plugin_path, filename))
            for filename in ('libActorPose.so', 'libTrafficPose.so')
        ):
            prepend_env_path(env, 'IGN_GAZEBO_SYSTEM_PLUGIN_PATH', actor_pose_plugin_path)
            prepend_env_path(env, 'GZ_SIM_SYSTEM_PLUGIN_PATH', actor_pose_plugin_path)
            prepend_env_path(env, 'LD_LIBRARY_PATH', actor_pose_plugin_path)

    if render_env == 'unset':
        for name in GUI_ENV_TO_CLEAR:
            env.pop(name, None)
    elif render_env == 'software':
        env['LIBGL_ALWAYS_SOFTWARE'] = '1'
        env['MESA_LOADER_DRIVER_OVERRIDE'] = 'llvmpipe'
        env['GALLIUM_DRIVER'] = 'llvmpipe'
        env.pop('MESA_GL_VERSION_OVERRIDE', None)
        env.pop('QT_XCB_GL_INTEGRATION', None)
        env.pop('MESA_D3D12_DEFAULT_ADAPTER_NAME', None)

    if d3d12_adapter:
        env['MESA_D3D12_DEFAULT_ADAPTER_NAME'] = d3d12_adapter
    env['GEN0_ACTOR_SOFT_STOP'] = actor_soft_stop
    env['GEN0_ACTOR_SOFT_STOP_MARGIN'] = actor_soft_stop_margin
    env['GEN0_ACTOR_SOFT_STOP_RELEASE_MARGIN'] = actor_soft_stop_release_margin
    env['GEN0_ACTOR_SOFT_STOP_VEHICLE_NAME'] = actor_soft_stop_vehicle_name
    return env


def unpause_world_action(period):
    return TimerAction(
        period=period,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ign',
                    'service',
                    '-s',
                    '/world/default/control',
                    '--reqtype',
                    'ignition.msgs.WorldControl',
                    '--reptype',
                    'ignition.msgs.Boolean',
                    '--timeout',
                    '2000',
                    '--req',
                    'pause: false',
                ],
                output='screen',
            )
        ],
    )


def actors_launch(context, *args, **kwargs):
    actors_scenario = LaunchConfiguration('actors_scenario').perform(context)
    world = LaunchConfiguration('world').perform(context)

    if not actors_scenario:
        return []

    pkg_share_dir = get_package_share_directory('gen0_main')
    scenario_file_path = os.path.join(
        pkg_share_dir,
        'worlds',
        'scenarios',
        world,
        f'{actors_scenario}.sdf',
    )

    actors_launch_file = os.path.join(pkg_share_dir, 'launch', 'actors.launch.py')
    if not os.path.exists(scenario_file_path):
        return [
            LogInfo(
                msg=(
                    f'\033[93m[WARNING] Scenario {actors_scenario} does not '
                    f'exist for world {world}; skipping actor loading\033[0m'
                )
            )
        ]

    actions = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(actors_launch_file),
            launch_arguments={
                'world': world,
                'actors_scenario': actors_scenario,
            }.items(),
        )
    ]

    return actions


def gazebo_launch(context, *args, **kwargs):
    if not launch_bool(context, 'start_gazebo', 'true'):
        return [LogInfo(msg='Skipping Gazebo startup because start_gazebo:=false')]

    world = LaunchConfiguration('world').perform(context)
    gui_value = LaunchConfiguration('gui').perform(context).strip().lower()
    if not gui_value:
        gui_value = LaunchConfiguration('gazebo_gui').perform(context).strip().lower()
    gui = gui_value == 'true'
    render_engine = LaunchConfiguration('render_engine').perform(context)
    d3d12_adapter = LaunchConfiguration('d3d12_adapter').perform(context).strip()
    gui_visual_mode = LaunchConfiguration('gui_visual_mode').perform(context)
    render_env = LaunchConfiguration('render_env').perform(context)
    actor_soft_stop = LaunchConfiguration('actor_soft_stop').perform(context)
    actor_soft_stop_margin = LaunchConfiguration('actor_soft_stop_margin').perform(context)
    actor_soft_stop_release_margin = LaunchConfiguration(
        'actor_soft_stop_release_margin'
    ).perform(context)
    actor_soft_stop_vehicle_name = LaunchConfiguration(
        'actor_soft_stop_vehicle_name'
    ).perform(context)
    if render_env == 'auto':
        render_env = 'software' if gui else 'unset'
    partition = LaunchConfiguration('partition').perform(context)

    pkg_share_dir = get_package_share_directory('gen0_main')
    world_dir = os.path.join(pkg_share_dir, 'worlds', world)
    world_file_path = os.path.join(world_dir, f'{world}.sdf')
    if gui and gui_visual_mode == 'light':
        light_world_file_path = os.path.join(world_dir, f'{world}_gui_light.sdf')
        if os.path.exists(light_world_file_path):
            world_file_path = light_world_file_path
    gazebo_env = gazebo_environment(
        pkg_share_dir,
        partition,
        render_env,
        d3d12_adapter,
        actor_soft_stop,
        actor_soft_stop_margin,
        actor_soft_stop_release_margin,
        actor_soft_stop_vehicle_name,
    )

    if gui:
        gazebo_process = ExecuteProcess(
            cmd=[
                'ign',
                'gazebo',
                '-r',
                '--force-version',
                '6',
                '--render-engine',
                render_engine,
                world_file_path,
            ],
            env=gazebo_env,
            output='screen',
        )
    else:
        gazebo_process = ExecuteProcess(
            cmd=[
                'ign',
                'gazebo',
                '-r',
                '-s',
                '--headless-rendering',
                '--force-version',
                '6',
                '--render-engine-server',
                render_engine,
                world_file_path,
            ],
            env=gazebo_env,
            output='screen',
        )

    actions = []
    if gui and gui_visual_mode == 'light':
        actions.append(
            LogInfo(
                msg=(
                    'Using lightweight Gazebo GUI world visual. Pass '
                    'gui_visual_mode:=full to render the full textured city mesh.'
                )
            )
        )
    actions.append(LogInfo(msg=f'Gazebo render_env={render_env}'))

    actions += [gazebo_process, unpause_world_action(6.0)]

    if gui:
        actions.append(unpause_world_action(12.0))
        actions.append(unpause_world_action(18.0))

    return actions


def delayed_gazebo_launch(context, *args, **kwargs):
    actors_scenario = LaunchConfiguration('actors_scenario').perform(context)

    if not actors_scenario:
        return gazebo_launch(context, *args, **kwargs)

    return [
        TimerAction(
            period=2.0,
            actions=[
                OpaqueFunction(function=gazebo_launch),
            ],
        )
    ]


def generate_launch_description():
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='my_map',
        description='Name of the world file (without extension) to use in Gazebo.',
    )
    sim_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        choices=['true', 'false'],
    )
    actors_arg = DeclareLaunchArgument(
        'actors_scenario',
        default_value='',
        description='The pedestrian scenario filename without extension.',
    )
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        choices=['true', 'false'],
    )
    camera_view_arg = DeclareLaunchArgument(
        'camera_view',
        default_value='true',
        choices=['true', 'false'],
        description='Open a live window for the sweeper front camera.',
    )
    camera_topic_arg = DeclareLaunchArgument(
        'camera_topic',
        default_value='/gen0_model/front_camera',
        description='ROS image topic shown in the live camera window.',
    )
    camera_view_width_arg = DeclareLaunchArgument(
        'camera_view_width',
        default_value='960',
        description='Maximum camera window image width in pixels.',
    )
    camera_view_height_arg = DeclareLaunchArgument(
        'camera_view_height',
        default_value='720',
        description='Maximum camera window image height in pixels.',
    )
    gazebo_gui_arg = DeclareLaunchArgument(
        'gazebo_gui',
        default_value='true',
        choices=['true', 'false'],
        description='Start the Gazebo GUI. Set false for WSL/headless mapping runs.',
    )
    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='',
        choices=['', 'true', 'false'],
        description='Alias for gazebo_gui. Leave empty to use gazebo_gui.',
    )
    start_gazebo_arg = DeclareLaunchArgument(
        'start_gazebo',
        default_value='true',
        choices=['true', 'false'],
        description='Start Gazebo from this launch file. Set false when Gazebo is started manually.',
    )
    render_engine_arg = DeclareLaunchArgument(
        'render_engine',
        default_value='ogre',
        choices=['ogre', 'ogre2'],
        description='Gazebo rendering backend used by GPU lidar and camera sensors.',
    )
    d3d12_adapter_arg = DeclareLaunchArgument(
        'd3d12_adapter',
        default_value='',
        description='Optional WSL D3D12 adapter name. Leave empty to match the manual unset-based Gazebo startup.',
    )
    render_env_arg = DeclareLaunchArgument(
        'render_env',
            default_value='unset',
            choices=['auto', 'unset', 'software', 'passthrough'],
            description='Gazebo rendering environment. Use unset for the verified WSL/Gazebo path.',
        )
    gui_visual_mode_arg = DeclareLaunchArgument(
        'gui_visual_mode',
            default_value='full',
            choices=['light', 'full'],
            description='Use the full textured world visual by default; light is only an explicit fallback.',
        )
    partition_arg = DeclareLaunchArgument(
        'partition',
        default_value=f'gen0_{os.getpid()}',
        description='Gazebo transport partition for this launch instance.',
    )
    actor_soft_stop_arg = DeclareLaunchArgument(
        'actor_soft_stop',
        default_value='false',
        choices=['true', 'false'],
        description='Enable visual actor soft-stop around the vehicle to prevent actor/car clipping.',
    )
    actor_soft_stop_margin_arg = DeclareLaunchArgument(
        'actor_soft_stop_margin',
        default_value='0.25',
        description='Actor soft-stop clearance in meters outside actor radius.',
    )
    actor_soft_stop_release_margin_arg = DeclareLaunchArgument(
        'actor_soft_stop_release_margin',
        default_value='0.85',
        description='Clearance in meters required before a soft-stopped actor resumes walking.',
    )
    actor_soft_stop_vehicle_name_arg = DeclareLaunchArgument(
        'actor_soft_stop_vehicle_name',
        default_value='gen0_model',
        description='Gazebo vehicle model name used by actor soft-stop.',
    )
    ground_truth_arg = DeclareLaunchArgument(
        'ground_truth_localization',
        default_value='false',
        choices=['true', 'false'],
    )
    static_odom_base_arg = DeclareLaunchArgument(
        'static_odom_base',
        default_value='false',
        choices=['true', 'false'],
        description='Publish a fixed odom->base_link transform. Disable when a localization stack owns odom->base_link.',
    )

    pkg_share_dir = get_package_share_directory('gen0_main')
    default_bridge_file = PathJoinSubstitution([pkg_share_dir, 'config', 'bridge.yaml'])
    bridge_file_arg = DeclareLaunchArgument(
        'bridge_file',
        default_value=default_bridge_file,
        description='ros_gz_bridge YAML config. Use bridge_no_gz_odom_tf.yaml when FAST-LIO owns odom->base_link.',
    )
    vehicle_file = os.path.join(pkg_share_dir, 'urdf', 'gen0_model.sdf')

    with open(vehicle_file, 'r', encoding='utf-8') as infp:
        robot_desc = infp.read()
    # RViz receives this SDF from /robot_description without the SDF file's
    # directory context, so relative mesh URIs are not resolvable there.
    robot_desc = robot_desc.replace(
        '../meshes/',
        'package://gen0_main/meshes/',
    )

    return LaunchDescription([
        world_arg,
        sim_arg,
        actors_arg,
        rviz_arg,
        camera_view_arg,
        camera_topic_arg,
        camera_view_width_arg,
        camera_view_height_arg,
        gazebo_gui_arg,
        gui_arg,
        start_gazebo_arg,
        render_engine_arg,
        d3d12_adapter_arg,
        render_env_arg,
        gui_visual_mode_arg,
        partition_arg,
        actor_soft_stop_arg,
        actor_soft_stop_margin_arg,
        actor_soft_stop_release_margin_arg,
        actor_soft_stop_vehicle_name_arg,
        ground_truth_arg,
        static_odom_base_arg,
        bridge_file_arg,
        SetEnvironmentVariable('IGN_PARTITION', LaunchConfiguration('partition')),
        SetEnvironmentVariable('GZ_PARTITION', LaunchConfiguration('partition')),
        OpaqueFunction(function=actors_launch),
        OpaqueFunction(function=delayed_gazebo_launch),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            parameters=[{
                'config_file': LaunchConfiguration('bridge_file'),
                'qos_overrides./tf_static.publisher.durability': 'transient_local',
            }],
            output='screen',
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='both',
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
                {'robot_description': robot_desc},
            ],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(pkg_share_dir, 'config', 'gen0_main.rviz')],
            condition=IfCondition(LaunchConfiguration('rviz')),
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        ),
        Node(
            package='gen0_main',
            executable='camera_viewer',
            name='gen0_camera_viewer',
            output='screen',
            condition=IfCondition(LaunchConfiguration('camera_view')),
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'image_topic': LaunchConfiguration('camera_topic'),
                'max_width': LaunchConfiguration('camera_view_width'),
                'max_height': LaunchConfiguration('camera_view_height'),
            }],
        ),
        Node(
            package='gen0_main',
            executable='ground_truth_publisher',
            condition=IfCondition(LaunchConfiguration('ground_truth_localization')),
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='odom_baselink_tf',
            namespace='',
            arguments=[
                '0.0',
                '0.0',
                '0.0',
                '0.0',
                '0.0',
                '0.0',
                'odom',
                'base_link',
            ],
            condition=IfCondition(PythonExpression([
                "'", LaunchConfiguration('static_odom_base'), "' == 'true' and '",
                LaunchConfiguration('ground_truth_localization'), "' == 'false'",
            ])),
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        ),
    ])
