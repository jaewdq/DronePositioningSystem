"""
Launch file for the two-LiDAR drone localization system (simulation).

The two ground LiDARs are baked into the Gazebo world, so this launch does
NOT spawn them. It only:
  1. Bridges /lidar1/points and /lidar2/points  (Gazebo → ROS2 PointCloud2)
  2. Starts the tracker node — by default the paper tracker
     (lidar_drone_tracker_cov, PF/MF/SU in one node) with the paper
     parameters (lidar_localization_cov.yaml, 24 x 15 field)
  3. Publishes a static identity transform (map -> world)

Nothing in this pipeline publishes TF (everything is already expressed
directly in the world/map frame), but RViz2 needs at least one /tf or
/tf_static message before it will resolve Fixed Frame "map" and render
anything — hence the static_transform_publisher below. It's a no-op
otherwise (identity transform, no effect on any coordinate computed
upstream in the tracker).

Prerequisites (separate terminals, started first):
  Terminal 1: cd ~/drone_project/PX4-Autopilot
              PX4_GZ_NO_FOLLOW=1 PX4_GZ_WORLD=competition_map make px4_sitl gz_x500
  Terminal 2: MicroXRCEAgent udp4 -p 8888
  Terminal (optional, to actually fly):
              ros2 run drone_mission competition_mission

Usage:
  ros2 launch drone_localization lidar_localization.launch.py                 # 24x15, SU
  ros2 launch drone_localization lidar_localization.launch.py fusion_mode:=position
  ros2 launch drone_localization lidar_localization.launch.py \\
      params_file:=lidar_localization_indoor.yaml                             # 실내 케이지

The same arguments (params_file, fusion_mode) as the real-system launch in
jaewdq/LidarDronePositioning, so its competition GUI can drive this one too.

NOTE: If you change a LiDAR pose, edit it in BOTH
  - the world generator (gazebo/worlds/generate_*.py) + regenerate + redeploy
  - the params yaml (lidarN_*)
so the physical sensor and the node's transform stay in sync. They are not
linked automatically; changing only one silently breaks localization.
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('drone_localization')

    params_file_arg = DeclareLaunchArgument(
        'params_file', default_value='lidar_localization_cov.yaml',
        description='Params filename under params/. '
                    'lidar_localization_cov.yaml = 논문 조건 (24x15), '
                    'lidar_localization_indoor.yaml = 실내 케이지.')
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

    # 노드 이름은 params yaml이 키로 쓰므로 바꾸면 파라미터가 통째로 안 먹는다
    # (예전에 params_file이 어긋나 검출률 0%가 났던 것과 같은 종류의 조용한 실패).
    # fusion_mode 는 yaml 뒤에 넣어 launch 인자가 yaml 값보다 우선한다.
    tracker = Node(
        package='drone_localization',
        executable=LaunchConfiguration('tracker'),
        name='lidar_drone_tracker',
        output='screen',
        parameters=[
            params_path,
            {'fusion_mode': LaunchConfiguration('fusion_mode')},
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
            'fusion_mode', default_value='sequential',
            description="논문 융합 구조: 'sequential' (SU, 기본) | 'position' (MF) | "
                        "'cloud' (PF). lidar_drone_tracker_cov 전용."),
        DeclareLaunchArgument(
            'tracker', default_value='lidar_drone_tracker_cov',
            description='lidar_drone_tracker_cov (논문 트래커)'),
        DeclareLaunchArgument(
            'cluster_dump_path', default_value='',
            description='선택된 드론 클러스터의 원본 점들을 npz로 저장할 경로.'),
        bridge,
        tracker,
        static_tf,
    ])
