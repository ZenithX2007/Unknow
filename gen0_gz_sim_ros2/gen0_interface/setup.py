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
    maintainer='av-lab',
    maintainer_email='syed.ahussain@ku.ac.ae',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cmdvel_to_vehicle = gen0_interface.cmdvel_to_vehicle:main',
            'mapping_drive = gen0_interface.mapping_drive:main',
            'keyboard_teleop = gen0_interface.keyboard_teleop:main',
        ],
    },
)
