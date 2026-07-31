import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def default_prior_map():
    map_path = os.path.join(
        os.path.expanduser('~'),
        'SCURM_SentryNavigation',
        'sentry_bringup',
        'maps',
        'GlobalMap.pcd',
    )
    return map_path if os.path.exists(map_path) else ''


def generate_launch_description():
    map_path = LaunchConfiguration('map_path')

    sac_ia_gicp_node = Node(
        package='icp_relocalization',
        executable='sac_ia_gicp',
        name='sac_ia_gicp_node',
        parameters=[
            {'target_pcd_file': map_path},
            {'num_threads': 8},
            {'k_serach_source': 100},
            {'k_serach_target': 100},
            {'voxel_grid_leaf_size_source': 0.3},
            {'voxel_grid_leaf_size_target': 0.3},
            {'sac_ia_min_sample_distance': 0.1},
            {'sac_ia_correspondence_randomness': 50},
            {'sac_ia_num_samples':5},
            {'icp_max_correspondence_distance': 1.0},
            {'icp_max_iteration': 10000},
            {'icp_transformation_epsilon': 0.01},
            {'icp_euclidean_fitness_epsilon': 0.01},
            {'fitness_score_thre':0.1},
            {'mode':0},
            {'max_optimize_times':3}
        ],
        remappings=[
            ('source_cloud', '/gen0_mapping/cloud_registered_body'),
        ],
    )

    ld = LaunchDescription([
        DeclareLaunchArgument(
            'map_path',
            default_value=default_prior_map(),
            description='PCD prior map used by SAC-IA/GICP relocalization.',
        ),
    ])

    ld.add_action(sac_ia_gicp_node)

    return ld
