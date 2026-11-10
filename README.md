# DronePositioningSystem — 시뮬레이션 (Gazebo + PX4)

지상 LiDAR 2대로 UAV 위치를 추적하는 시스템의 시뮬레이션 저장소다.
실내 드론 세트장 현장 운용은 [LidarDronePositioning](https://github.com/jaewdq/LidarDronePositioning) 이 맡는다.
트래커는 두 저장소가 같은 파일이다 (여기 `lidar_drone_tracker_cov.py` = 저쪽 `lidar_drone_tracker.py`). 고칠 때는 양쪽을 함께 맞춘다.

## 전제조건

- Ubuntu 22.04, ROS 2 Humble, Gazebo Harmonic, `ros-humble-ros-gzharmonic`
- PX4-Autopilot (SITL 빌드), Micro-XRCE-DDS-Agent
- `px4_msgs` · `px4_ros_com` 워크스페이스 (별도)
- QGroundControl — 켜져 있어야 PX4 가 시동을 허용한다 (없으면 `gcs_connection_lost`)
- Python: numpy, scipy, scikit-learn, open3d, matplotlib, pyyaml

※ 아래 경로는 예시다

- 저장소: `~/drone_project/DronePositioningSystem`
- PX4: `~/drone_project/PX4-Autopilot`
- px4_msgs 워크스페이스: `~/drone_project/ros2_ws`
- QGroundControl: `~/Applications/QGroundControl.AppImage`

---

## 구성

### 맵 (`gazebo/worlds`)

- `competition_map` — 24 × 15 m 필드, OS1-128 등가 2대가 긴 변 가운데 (12, −2, 2) / (12, 17, 2) 에서 마주 본다. 논문 실험 조건.
    - 생성: `generate_competition_map.py` (`LIDAR_PROFILE=OS0` 이면 OS0 64ch 월드 `competition_map_os0`)
- `indoor_cage` — 실내 그물 케이지 5 × 4 m, 필드 3 × 2 m, 원점 (0, 0) = 필드 남서 모서리 = 이륙 지점. OS0-SR 64ch 2대.
    - 생성: `generate_indoor_cage.py`
    - `LIDAR_LAYOUT=real LIDAR_H=1.26` → 실기 센서 위치 (1.354, −1.34) / (1.376, 3.054) → `indoor_cage_real_h126`
    - `LIDAR_LAYOUT=short` → 짧은 변 배치, `LIDAR_H=0.8` → 높이만 바꾼 비교용 월드

### 기체 (`gazebo/models`)

- 비행 물리는 PX4 x500 그대로, LiDAR 가 보는 외형만 Crazyflie 2 메시(bitcraze/crazyflie-simulation, MIT)로 바꾼다.
- `x500_cf2` (1배, 프로펠러 포함 약 9.6 × 10 × 2.7 cm), `x500_cf2x2` (2배, 약 19.2 × 20 × 5.4 cm — 가드 포함 Tello 18 × 18 cm 와 비슷)
- 생성: `CF2_SCALE=2 python3 gazebo/models/generate_x500_cf2.py` (높이만 따로: `CF2_SCALE_Z`)
- PX4 airframe: `gazebo/px4_airframes` (4022, 4023)

### ROS 2 패키지 (`ros2_ws/src`)

- `drone_localization` — `lidar_drone_tracker_cov` (PF / MF / SU, 등속 KF), `replay_bag` (bag 재생 비교), `flight_recorder`, 그림·평가 도구
    - 설정: `lidar_localization_cov.yaml` (24 × 15 논문 조건), `lidar_localization_indoor.yaml` (실내)
- `drone_mission` — `competition_mission` (24 × 15), `indoor_survey_mission` (실내 지그재그), `indoor_shapes_mission` (실내 경로 5종)

### 실험 (`experiments/indoor`)

| 스크립트 | 하는 일 |
|---|---|
| `live_flight.sh` | 실내 라이브 비행 한 번 (보면서 날리고, 끝나면 경로 그림·PF/MF/SU 비교) |
| `shapes_replicate.sh` + `shapes_eval.py` | 경로 5종 × N회 녹화 → PF/MF/SU 재생 → 평가 |
| `paper_replicate.sh` + `paper_eval.py` | 논문 절차 재현 (N회 녹화, 외재 오차 조건, 평가, 그림) |
| `compare_fusion.sh` + `plot_fusion.py` | bag 하나를 PF/MF/SU 로 재생 비교 |
| `run_height_test.sh` | LiDAR 높이·배치·기체를 바꿔 비행 한 번 |
| `count_target_points.py` + `analyze_points.py` | 프레임마다 각 라이다가 기체에 찍은 점·링 수 |

---

## 0. 설치 (처음 한 번)

```bash
cd ~/drone_project/DronePositioningSystem
PX4=~/drone_project/PX4-Autopilot

# 월드·모델·airframe 을 PX4 트리에 배포
cp gazebo/worlds/*.sdf gazebo/worlds/*_coordinates.txt $PX4/Tools/simulation/gz/worlds/
cp -r gazebo/models/*/ $PX4/Tools/simulation/gz/models/
cp gazebo/px4_airframes/* $PX4/ROMFS/px4fmu_common/init.d-posix/airframes/
```

- `$PX4/ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt` 의 `4021_gz_x500_flow` 아래에 `4022_gz_x500_cf2`, `4023_gz_x500_cf2x2` 두 줄을 추가한다.

```bash
(cd $PX4 && make px4_sitl)
cd ros2_ws && colcon build && source install/setup.bash
```

- 같은 LAN 에서 ROS 2 가 섞이지 않도록 `~/.bashrc` 에 `ROS_DOMAIN_ID`, `GZ_PARTITION`, `GZ_IP=127.0.0.1` 을 둔다.

---

## 1. 실내 라이브 비행

```bash
~/Applications/QGroundControl.AppImage &
experiments/indoor/live_flight.sh run01
```

- PX4·Gazebo·트래커·RViz·bag 녹화를 띄우고 `indoor_survey_mission` 으로 비행한다.
- 끝나면 `path_result.png`(경로·오차), `fusion/fusion_compare.png`(PF/MF/SU 재생)가 열린다.
- 결과: `~/drone_project/flight_logs/indoor/live/run01/`
- 환경변수: `MODEL` (기본 `x500_cf2x2`), `WORLD` (기본 `indoor_cage`), `NO_COMPARE=1`

## 2. 실내 경로 5종 실험

```bash
~/Applications/QGroundControl.AppImage &
cd ~/drone_project/DronePositioningSystem/experiments/indoor
ROOT=~/drone_project/flight_logs/indoor/shapes N=3 ./shapes_replicate.sh record replay eval
```

- 경로: `helix`, `circle`, `figure8`, `square`, `backforth` (`SHAPES` 로 고른다)
- 월드는 실기 배치(`LAYOUT=real`, 높이 1.26 m), 기체 `MODEL` (기본 `x500_cf2x2`), 화면 없이 비행한다.
- 재생: PF/MF/SU, ROI 4 × 3 m. 평가: `<ROOT>/cmp_state/shapes_summary.csv`
- bag 은 비행 1회에 2~3 GB 다. 디스크를 먼저 확인한다.

## 3. 24 × 15 m 맵 (터미널별)

```bash
cd ~/drone_project/PX4-Autopilot && PX4_GZ_NO_FOLLOW=1 PX4_GZ_WORLD=competition_map make px4_sitl gz_x500
MicroXRCEAgent udp4 -p 8888
ros2 launch drone_localization lidar_localization.launch.py fusion_mode:=position
ros2 run drone_localization flight_recorder
ros2 run drone_mission competition_mission
```

## 4. 재생 비교

```bash
python3 -m drone_localization.replay_bag --bag <bag> --out <결과 폴더> \
  --clock arrival --ground-truth-topic /world/<월드>/dynamic_pose/info --quiet \
  --ros-args --params-file <yaml> -p fusion_mode:=cloud
# MF = position, SU = sequential
```

- `--clock arrival`: bag 수신 시각으로 처리 (기본, 라이브와 같다)
- `--clock stamp`: 센서 측정 시각으로 처리 (Gazebo 는 두 센서를 차례로 처리해 수신 시각이 수십 ms 어긋난다)
- 출력: `estimate_details.csv`, `state_details.csv`, `measurements.csv`, `actual_ground_truth_path.csv`, `processing_cost.csv`, `manifest.json`

---

## 알아둘 것

- 비행 제어는 PX4 자체 위치로 한다. LiDAR 추정은 비행에 쓰지 않는다 (측위 평가용).
- Gazebo `gpu_lidar` 는 스캔 전체를 한 순간에 그린다. 실제 라이다처럼 돌며 찍지 않는다.
- 결과 수치(성능 비교표)는 저장소에 두지 않는다.
