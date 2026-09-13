import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'ueh_solution'


def norm(paths):
    """Normalize path separators to forward-slash for colcon/pip compatibility.

    glob() returns OS-native separators. On Windows these are backslashes,
    which setuptools/pip may reject when building a wheel or when colcon
    runs setup.py from a temporary directory. Forward-slashes work on all
    platforms (Linux, macOS, Windows). This is the same pattern used by
    the reference crc_sim/setup.py in this workspace.
    """
    return [p.replace(os.sep, '/') for p in paths]


setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Required by ament_python so 'ros2 pkg list' finds the package.
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        # package.xml must live in share/<pkg>/ for ros2 pkg info to work.
        ('share/' + package_name,
         ['package.xml']),
        # Launch files – installed into share/<pkg>/launch/.
        # norm() ensures forward-slashes on every OS.
        (os.path.join('share', package_name, 'launch'),
         norm(glob(os.path.join('launch', '*.launch.py')))),
        # Config files – installed into share/<pkg>/config/.
        (os.path.join('share', package_name, 'config'),
         norm(glob(os.path.join('config', '*.yaml')))),
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
