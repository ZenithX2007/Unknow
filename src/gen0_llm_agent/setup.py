from glob import glob
from setuptools import find_packages, setup

package_name = 'gen0_llm_agent'
setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/prompts', glob('prompts/*.txt')),
    ],
    install_requires=['setuptools', 'jsonschema>=4.0'],
    zip_safe=True,
    entry_points={'console_scripts': ['llm_node = gen0_llm_agent.llm_node:main']},
)
