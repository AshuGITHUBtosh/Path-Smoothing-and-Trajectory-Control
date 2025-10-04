from setuptools import setup
import os
from glob import glob

package_name = 'robot_nav_assignment'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='Ashutosh',
    maintainer_email='ashutosh10503@gmail.com',
    description='Path smoothing + trajectory controller assignment',
    license='MIT',
    entry_points={
        'console_scripts': [
            'path_server = robot_nav_assignment.path_server:main',
            'controller = robot_nav_assignment.controller_node:main',
            'simulator = robot_nav_assignment.simulator_node:main',
        ],
    },
)

