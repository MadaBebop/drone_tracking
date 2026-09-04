from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'drone_tracking'

setup(
    name=package_name,
    version='0.0.2',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mada',
    maintainer_email='riccardo.mahdavi@gmail.com',
    description='Drone tracking project',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'detector_node = drone_tracking.detector_node:main',
            'tracker_node = drone_tracking.tracker_node:main',
            'jammer_node = drone_tracking.jammer_node:main',
            'controller_node = drone_tracking.controller_node:main',
            'mission_node = drone_tracking.mission_node:main',
            'target_mover_node = drone_tracking.target_mover_node:main',
            'metrics_node = drone_tracking.metrics_node:main',
        ],
    },
)
