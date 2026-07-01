"""
Launch file for the two-LiDAR drone localization system.

The two ground LiDARs are now baked into the world (competition_map.sdf),
so this launch does NOT spawn them. It only:
  1. Bridges /ouster1/points and /ouster2/points  (Gazebo → ROS2 PointCloud2)
  2. Starts the lidar_drone_tracker node (fuses both clouds, EKF tracking)

Prerequisites (separate terminals, started first):
  Terminal 1: cd ~/drone_project/PX4-Autopilot
              PX4_GZ_WORLD=competition_map make px4_sitl gz_x500
              → loads the world with ground_lidar_1 (east) + ground_lidar_2 (north)

  Terminal 2: MicroXRCEAgent udp4 -p 8888

  Terminal (optional, to actually fly):
              ros2 run drone_mission competition_mission

Usage:
  ros2 launch drone_localization lidar_localization.launch.py

Run with a single LiDAR:
  ros2 launch drone_localization lidar_localization.launch.py enable_lidar2:=false

Visualise (optional):
  ros2 run rviz2 rviz2 -f map
    Add: /drone/estimated_pose, /lidar/filtered_points, /lidar/cluster_markers

NOTE: If you change a LiDAR pose, edit it in BOTH
  - Tools/simulation/gz/worlds/generate_competition_map.py (LIDARS) + regenerate
  - params/lidar_localization.yaml (lidarN_*)
so the physical sensor and the node's transform stay in sync.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('drone_localization')
    params_file = os.path.join(pkg_share, 'params', 'lidar_localization.yaml')

    enable_lidar2 = ParameterValue(
        LaunchConfiguration('enable_lidar2'), value_type=bool)

    # ------------------------------------------------------------------
    # Bridge the two 3D point-cloud topics: Gazebo PointCloudPacked → ROS2
    #
    # If your gz-sensors build outputs LaserScan instead of PointCloudPacked,
    # change each type to: @sensor_msgs/msg/LaserScan[gz.msgs.LaserScan
    # ------------------------------------------------------------------
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ouster_bridge',
        arguments=[
            '/ouster1/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '/ouster2/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
        ],
        output='screen',
    )

    tracker = Node(
        package='drone_localization',
        executable='lidar_drone_tracker',
        name='lidar_drone_tracker',
        output='screen',
        parameters=[
            params_file,
            {'enable_lidar2': enable_lidar2},
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'enable_lidar2', default_value='true',
            description='Use the second (north) LiDAR. false = single-LiDAR mode.'),
        bridge,
        tracker,
    ])
