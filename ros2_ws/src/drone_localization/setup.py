from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'drone_localization'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'params'),
            glob('params/*.yaml')),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jaeyup',
    maintainer_email='jaeyup01@naver.com',
    description='LiDAR-based drone 3D localization (outside-in, EKF)',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'lidar_drone_tracker_cov = drone_localization.lidar_drone_tracker_cov:main',
            'flight_recorder = drone_localization.flight_recorder:main',
            'plot_flight = drone_localization.plot_flight:main',
            'analyze_waypoints = drone_localization.analyze_waypoints:main',
            'plot_flight_3d = drone_localization.plot_flight_3d:main',
            'evaluate_accuracy = drone_localization.evaluate_accuracy:main',
            'plot_accuracy = drone_localization.plot_accuracy:main',
            'archive_run = drone_localization.archive_run:main',
            'replay_bag = drone_localization.replay_bag:main',
        ],
    },
)
