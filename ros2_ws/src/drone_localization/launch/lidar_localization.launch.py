"""
Launch file for the two-LiDAR drone localization system.

The two ground LiDARs are now baked into the world (competition_map.sdf or
competition_map_os1.sdf), so this launch does NOT spawn them. It only:
  1. Bridges /lidar1/points and /lidar2/points  (Gazebo → ROS2 PointCloud2)
  2. Starts the lidar_drone_tracker node (fuses both clouds, KF tracking)
  3. Publishes a static identity transform (map -> world)

Nothing in this pipeline publishes TF (everything is already expressed
directly in the world/map frame), but RViz2 needs at least one /tf or
/tf_static message before it will resolve Fixed Frame "map" and render
anything — hence the static_transform_publisher below. It's a no-op
otherwise (identity transform, no effect on any coordinate computed
upstream in the tracker).

Prerequisites (separate terminals, started first):
  Terminal 1: cd ~/drone_project/PX4-Autopilot
              PX4_GZ_WORLD=competition_map make px4_sitl gz_x500
              → loads the world with ground_lidar_1 (east) + ground_lidar_2 (north)
              (OS1-128 instead: PX4_GZ_WORLD=competition_map_os1)

  Terminal 2: MicroXRCEAgent udp4 -p 8888

  Terminal (optional, to actually fly):
              ros2 run drone_mission competition_mission

Usage (OS0-SR, default):
  ros2 launch drone_localization lidar_localization.launch.py

Usage (OS1-128):
  ros2 launch drone_localization lidar_localization.launch.py \\
      params_file:=lidar_localization_os1.yaml

Visualise (optional):
  ros2 run rviz2 rviz2 -f map
    Add: /drone/estimated_pose, /lidar/filtered_points, /lidar/cluster_markers
    (TF is provided by this launch file now, so displays should go green
     without needing a separate static_transform_publisher.)

NOTE: If you change a LiDAR pose, edit it in BOTH
  - Tools/simulation/gz/worlds/generate_competition_map*.py (LIDARS) + regenerate
  - params/lidar_localization*.yaml (lidarN_*)
so the physical sensor and the node's transform stay in sync. The OS0-SR and
OS1-128 configs are separate world+yaml pairs — always match the world you
launched with the matching params_file.
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('drone_localization')

    params_file_arg = DeclareLaunchArgument(
        'params_file', default_value='lidar_localization.yaml',
        description='Params filename under params/. '
                     'lidar_localization.yaml = OS0-SR (default), '
                     'lidar_localization_os1.yaml = OS1-128.')
    params_path = PathJoinSubstitution(
        [pkg_share, 'params', LaunchConfiguration('params_file')])

    # ------------------------------------------------------------------
    # Bridge the two 3D point-cloud topics: Gazebo PointCloudPacked → ROS2
    #
    # If your gz-sensors build outputs LaserScan instead of PointCloudPacked,
    # change each type to: @sensor_msgs/msg/LaserScan[gz.msgs.LaserScan
    # ------------------------------------------------------------------
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='lidar_bridge',
        arguments=[
            '/lidar1/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '/lidar2/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
        ],
        output='screen',
    )

    tracker = Node(
        package='drone_localization',
        executable='lidar_drone_tracker',
        name='lidar_drone_tracker',
        output='screen',
        parameters=[
            params_path,
            # 비었으면 트래커가 덤프를 끈다. 켜려면:
            #   cluster_dump_path:=~/drone_project/flight_logs/clusters.npz
            {'cluster_dump_path': LaunchConfiguration('cluster_dump_path')},
        ],
    )

    # Identity transform so RViz2 has TF data to resolve Fixed Frame "map"
    # (see module docstring). Args are positional in tf2_ros on Humble:
    # x y z yaw pitch roll parent_frame child_frame.
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_world_static_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'world'],
    )

    return LaunchDescription([
        params_file_arg,
        DeclareLaunchArgument(
            'cluster_dump_path', default_value='',
            description='선택된 드론 클러스터의 원본 점들을 npz로 저장할 경로. '
                        '비우면 저장하지 않음 (plot_cluster_shape 로 확인).'),
        bridge,
        tracker,
        static_tf,
    ])
