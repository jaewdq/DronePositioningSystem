from setuptools import find_packages, setup

package_name = 'drone_mission'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jaeyup',
    maintainer_email='jaeyup01@naver.com',
    description='PX4 offboard waypoint mission: takeoff to (0,0,2), forward 4m, land',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'waypoint_land = drone_mission.waypoint_land:main',
            'generate_groundtruth_trajectory = drone_mission.generate_groundtruth_trajectory:main',
            'ground_truth_path_publisher = drone_mission.ground_truth_path_publisher:main',
            'competition_mission = drone_mission.competition_mission:main',
        ],
    },
)
