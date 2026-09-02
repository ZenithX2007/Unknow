from setuptools import find_packages, setup

package_name = 'gen0_interface'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zjxue2007',
    maintainer_email='zjxue2007@example.com',
    description='Gen0 mapping-drive and keyboard teleoperation tools for Gazebo Ackermann control.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mapping_drive = gen0_interface.mapping_drive:main',
            'keyboard_teleop = gen0_interface.keyboard_teleop:main',
        ],
    },
)
