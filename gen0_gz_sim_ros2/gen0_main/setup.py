from setuptools import find_packages, setup
import os
from glob import glob


package_name = 'gen0_main'

# Iterate through all the files and subdirectories
# to build the data files array

data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ]


def package_files(data_files, directory_list):

    paths_dict = {}

    for directory in directory_list:

        for (path, directories, filenames) in os.walk(directory):
            directories[:] = [
                dirname for dirname in directories
                if dirname != '__pycache__'
            ]

            for filename in filenames:
                if filename.endswith('.pyc'):
                    continue

                file_path = os.path.join(path, filename)
                install_path = os.path.join('share', package_name, path)

                if install_path in paths_dict.keys():
                    paths_dict[install_path].append(file_path)

                else:
                    paths_dict[install_path] = [file_path]

    for key in paths_dict.keys():
        data_files.append((key, paths_dict[key]))

    return data_files

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files= package_files(data_files, ['launch/', 'worlds/', 'config/', 'behavior_tree/', 'urdf/', 'meshes/', 'models/']),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zjxue2007',
    maintainer_email='zjxue2007@example.com',
    description='Gazebo simulation assets and launch files for the Gen0 autonomous mapping demo.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'odom_frame_corrector = gen0_main.odom_frame_corrector:main',
        'ground_truth_publisher = gen0_main.pose_publisher:main',
        'actors_loader = gen0_main.actors_loader:main',
        'actors_spawner = gen0_main.actors_spawner:main',
        'gen0_3d_mapper = gen0_main.gen0_3d_mapper:main',
        'gazebo_livox_adapter = gen0_main.gazebo_livox_adapter:main',
        'simulated_world_lidar = gen0_main.simulated_world_lidar:main',
        'stable_odom = gen0_main.stable_odom:main',
        'odom_registered_scan = gen0_main.odom_registered_scan:main',
        'pointcloud_preview = gen0_main.pointcloud_preview:main',
        'pointcloud_accumulator_preview = gen0_main.pointcloud_accumulator_preview:main',
        'projected_terrain_map = gen0_main.projected_terrain_map:main',
        'actor_obstacle_costmap = gen0_main.actor_obstacle_costmap:main',
        'actor_collision_monitor = gen0_main.actor_collision_monitor:main',
        'nav2_pose_guard = gen0_main.nav2_pose_guard:main',
        'nav2_projected_map_relay = gen0_main.nav2_projected_map_relay:main',
        'trash_cleanup_node = gen0_main.trash_cleanup_node:main',
        ],
    },
)
