# DronePositioningSystem

지상 고정 LiDAR 2대만으로 비행 중인 드론의 3D 위치를 실시간 추정하는 **outside-in 측위 시스템**.
제24회 한국로봇항공기경연대회 중급부문(실내 조난자 탐색 임무) 시뮬레이션 검증용.

PX4 SITL + Gazebo Harmonic + ROS2 Humble 환경에서 동작한다.

## 개요

- **Ouster OS0-SR 64ch 등가 gpu_lidar 2대**를 임무구역(30×20m) 대각 코너 밖에 배치
  (금지구역 33×23m 밖, 필드 전 구역 커버 + 삼각측량)
- 각 LiDAR 포인트클라우드를 월드좌표로 변환·**융합** → 전처리(voxel/RANSAC 지면제거/SOR)
  → DBSCAN 클러스터링 → 드론 클러스터 선택 → **6상태 EKF**(등속도 모델) 추적
- 출력: `/drone/estimated_pose` (PoseWithCovarianceStamped, 월드 ENU)
- 실제 비행 궤적(PX4 ground-truth)과 추정값을 기록·비교하는 분석 도구 포함

## 디렉토리 구조

```
ros2_ws/src/
  drone_localization/          # 측위 시스템 (핵심)
    drone_localization/
      lidar_drone_tracker.py   # 2-LiDAR 융합 + EKF 추적 노드
      flight_recorder.py       # 실제/추정 궤적 CSV 기록
      plot_flight.py           # 계획 vs 실제 vs 추정 시각화 + 오차분석
    params/lidar_localization.yaml   # 모든 파라미터
    launch/lidar_localization.launch.py
  drone_mission/               # 비행 미션 + ground-truth 궤적
    drone_mission/
      competition_mission.py         # PX4 offboard 궤적 비행 노드
      generate_groundtruth_trajectory.py
      ground_truth_path_publisher.py
      waypoint_land.py

gazebo/
  worlds/
    generate_competition_map.py      # 맵(격자+마커+LiDAR 2대) 생성기
    competition_map.sdf              # 생성된 world
    competition_map_coordinates.txt  # 격자/마커/LiDAR 좌표 (정답지)
  models/
    generate_aruco_markers.py        # ArUco 마커 모델 생성기
    ground_lidar_sensor/             # gpu_lidar 센서 모델
    aruco_marker_id*/                # 마커 모델 4개
```

## 의존성

- ROS2 Humble, PX4-Autopilot (SITL), Gazebo Harmonic (gz-sim 8)
- `ros-humble-ros-gzharmonic` (LiDAR PointCloud2 브릿지)
- Micro-XRCE-DDS-Agent (PX4 ↔ ROS2)
- `px4_msgs`, `px4_ros_com` (별도 클론 필요)
- Python: `open3d`, `scipy`, `numpy`, `matplotlib` (`pip3 install open3d scipy matplotlib`)
- 마커 생성 시: `opencv-contrib-python-headless` (numpy<2 격리환경 권장)

## 배포 (Gazebo 파일을 PX4 트리에 연결)

`gazebo/` 안의 파일은 PX4-Autopilot 트리에 복사/심볼릭 링크한다:

```bash
PX4=~/drone_project/PX4-Autopilot/Tools/simulation/gz
cp gazebo/worlds/*  $PX4/worlds/
cp -r gazebo/models/*  $PX4/models/
```

ROS2 워크스페이스 빌드:

```bash
cd ros2_ws && colcon build --packages-select drone_localization drone_mission
source install/setup.bash
```

## 실행

```bash
# 터미널 1 — PX4 SITL + Gazebo (LiDAR 2대 포함 맵)
cd ~/drone_project/PX4-Autopilot
PX4_GZ_WORLD=competition_map make px4_sitl gz_x500

# 터미널 2 — Micro-XRCE-DDS Agent
MicroXRCEAgent udp4 -p 8888

# 터미널 3 — 측위 시스템 (브릿지 + 추적 노드)
ros2 launch drone_localization lidar_localization.launch.py

# 터미널 4 — 비행 기록
ros2 run drone_localization flight_recorder

# 터미널 5 — 궤적 비행
ros2 run drone_mission competition_mission

# 비행 후 — 결과 시각화 (계획 vs 실제 vs 추정 + 오차)
ros2 run drone_localization plot_flight
```

## 채점 기준 대응 (측위 관련)

| 항목 | 기준(상) | 현재 성능 |
|---|---|---|
| 경로점 위치 추정 수평오차 X,Y | < 0.5m | median ~0.1m |
| 구조경로 추종 수평/수직 오차 | < 0.5m | 수평 ~0.1m (Z 편향 보정 진행 중) |

LiDAR 스펙(Ouster OS0-SR 64ch), 드론 중심점 정의, 실제-추정 오차 분석은 개발 진행 중.
