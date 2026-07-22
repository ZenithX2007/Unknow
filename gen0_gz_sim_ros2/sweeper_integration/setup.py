from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'sweeper_integration'


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'behavior_trees'), glob('behavior_trees/*.xml')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zjxue2007',
    maintainer_email='zjxue2007@example.com',
    description='Humble integration layer between Gen0 and navigation stacks.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cmd_vel_adapter = sweeper_integration.cmd_vel_adapter:main',
            'ground_truth_odometry = sweeper_integration.ground_truth_odometry:main',
            'nav2_lifecycle_bringup = sweeper_integration.nav2_lifecycle_bringup:main',
        ],
    },
)
