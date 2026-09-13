import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'ueh_solution'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
         glob('launch/*.launch.py')),
        ('share/' + package_name + '/config',
         glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Contestant',
    maintainer_email='contestant@ueh.edu.vn',
    description='UEH CRC 2026 autonomous driving solution',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'lane_node      = ueh_solution.lane_node:main',
            'lidar_node     = ueh_solution.lidar_node:main',
            'sign_node      = ueh_solution.sign_node:main',
            'light_node     = ueh_solution.light_node:main',
            'ped_node       = ueh_solution.pedestrian_detect_node:main',
            'behavior_node  = ueh_solution.behavior_node:main',
            'data_logger    = ueh_solution.data_logger:main',
        ],
    },
)
