from glob import glob

from setuptools import find_packages, setup


package_name = 'qcnet_prediction'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Unknow Maintainers',
    maintainer_email='user@example.com',
    description='ROS 2 trajectory prediction bridge for QCNet and EPSILON.',
    license='BSD-3-Clause',
    entry_points={
        'console_scripts': [
            'qcnet_prediction_node = qcnet_prediction.qcnet_prediction_node:main',
        ],
    },
)
