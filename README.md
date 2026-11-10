# 실내 LiDAR UAV 추적 — 시뮬레이션 매뉴얼 (Gazebo + PX4)

지상 LiDAR 2대로 UAV 위치를 추적하는 시스템의 시뮬레이션 저장소다.
실내 드론 세트장 현장 운용은 [LidarDronePositioning](https://github.com/jaewdq/LidarDronePositioning) 이 맡는다.
트래커는 두 저장소가 같은 파일이다 (여기 `lidar_drone_tracker_cov.py` = 저쪽 `lidar_drone_tracker.py`). 고칠 때는 양쪽을 함께 맞춘다.

## 전제조건

- Ubuntu 22.04
- ROS 2 Humble (ros-humble-desktop) — 설치: https://docs.ros.org/en/humble/Installation.html
- Gazebo Harmonic, `ros-humble-ros-gzharmonic`
- PX4-Autopilot (SITL 빌드), Micro-XRCE-DDS-Agent
- `px4_msgs` · `px4_ros_com` 워크스페이스 (별도)
- QGroundControl — 켜져 있어야 PX4 가 시동을 허용한다 (없으면 `gcs_connection_lost`)
- Python: numpy, scipy, scikit-learn, open3d, matplotlib, pyyaml

※ 아래 경로는 예시다

- 저장소: `~/drone_project/DronePositioningSystem`
- PX4: `~/drone_project/PX4-Autopilot`
- px4_msgs 워크스페이스: `~/drone_project/ros2_ws`
- QGroundControl: `~/Applications/QGroundControl.AppImage`
- 경로를 바꿨으면 아래 명령과 `experiments/indoor/*.sh` 안의 경로도 같이 바꾼다.

---

## 구성

| 위치 | 내용 |
|---|---|
| `gazebo/worlds` | 월드 생성기와 생성된 월드 (`*.sdf`, `*_coordinates.txt`) |
| `gazebo/models` | 기체 모델 생성기, ArUco 마커, 경기장 바닥 |
| `gazebo/px4_airframes` | 기체 모델용 PX4 airframe (4022, 4023) |
| `ros2_ws/src/drone_localization` | 트래커, 재생 비교, 비행 기록·그림 도구, 설정 yaml |
| `ros2_ws/src/drone_mission` | 비행 미션 (PX4 offboard) |
| `experiments/indoor` | 실내 실험 스크립트 |

### 맵

- `competition_map` — 24 × 15 m 필드, OS1-128 등가 2대가 긴 변 가운데 (12, −2, 2) / (12, 17, 2) 에서 마주 본다. 논문 실험 조건.
    - 생성: `generate_competition_map.py` (`LIDAR_PROFILE=OS0` 이면 OS0 64ch 월드 `competition_map_os0`)
- `indoor_cage` — 실내 그물 케이지 5 × 4 m, 필드 3 × 2 m, 원점 (0, 0) = 필드 남서 모서리 = 이륙 지점. OS0-SR 64ch 2대가 긴 변 가운데 (1.5, −0.7, 1.2) / (1.5, 2.7, 1.2).
    - 생성: `generate_indoor_cage.py`
    - `LIDAR_LAYOUT=real LIDAR_H=1.26` → 실기 센서 위치 (1.354, −1.34) / (1.376, 3.054) → `indoor_cage_real_h126`
    - `LIDAR_LAYOUT=short` → 짧은 변 배치, `LIDAR_H=0.8` → 높이만 바꾼 비교용 월드

### 기체

- 비행 물리는 PX4 x500 그대로, LiDAR 가 보는 외형만 Crazyflie 2 메시(bitcraze/crazyflie-simulation, MIT)로 바꾼다.
- `x500_cf2` — 1배, 프로펠러 포함 약 9.6 × 10 × 2.7 cm
- `x500_cf2x2` — 2배, 약 19.2 × 20 × 5.4 cm (가드 포함 Tello 18 × 18 cm 와 비슷). 실내 실험 기본.
- 생성: `CF2_SCALE=2 python3 gazebo/models/generate_x500_cf2.py` (높이만 따로: `CF2_SCALE_Z`)

### 패키지

- `drone_localization`
    - `lidar_drone_tracker_cov` — PF / MF / SU 융합, 등속 KF. 위치 `/drone/estimated_pose`, 위치·속도 `/drone/estimated_odometry`
    - `replay_bag` — 녹화한 bag 을 PF / MF / SU 로 재생 비교
    - `flight_recorder`, `plot_flight`, `plot_flight_3d`, `evaluate_accuracy`, `plot_accuracy` — 24 × 15 비행 기록·그림·평가
    - `launch/lidar_localization.launch.py` — Gazebo LiDAR 브리지 + 트래커 + map→world TF
    - 설정: `lidar_localization_cov.yaml` (24 × 15 논문 조건), `lidar_localization_indoor.yaml` (실내)
- `drone_mission`
    - `competition_mission` — 24 × 15 비행
    - `indoor_survey_mission` — 실내 지그재그 (세로줄 5개, 줄 끝 3초 호버)
    - `indoor_shapes_mission <helix|circle|figure8|square|backforth>` — 실내 경로 5종

### 실험 스크립트

| 스크립트 | 하는 일 |
|---|---|
| `live_flight.sh` | 실내 라이브 비행 한 번 (1장의 터미널을 한 번에) |
| `shapes_replicate.sh` + `shapes_eval.py` | 경로 5종 × N회 녹화 → PF/MF/SU 재생 → 평가 (2장) |
| `run_height_test.sh` | 화면 없이 비행 한 번 (LiDAR 높이·배치·기체·미션 변경). `shapes_replicate.sh` 가 비행마다 부른다 |
| `paper_replicate.sh` + `paper_eval.py` | 논문 절차 재현 (N회 녹화, 외재 오차 조건, 평가, 그림) |
| `compare_fusion.sh` + `plot_fusion.py` | bag 하나를 PF/MF/SU 로 재생해 그림 |
| `count_target_points.py` + `analyze_points.py` | 프레임마다 각 라이다가 기체에 찍은 점·링 수 |
| `path_viz.py` + `indoor.rviz` | 참값·추정 궤적 표시, 끝나면 CSV·그림 저장 |

---

## 0. 설치 (처음 한 번)

### 0-1. PX4 에 월드·모델·airframe 배포

```bash
cd ~/drone_project/DronePositioningSystem
PX4=~/drone_project/PX4-Autopilot
cp gazebo/worlds/*.sdf gazebo/worlds/*_coordinates.txt $PX4/Tools/simulation/gz/worlds/
cp -r gazebo/models/*/ $PX4/Tools/simulation/gz/models/
cp gazebo/px4_airframes/* $PX4/ROMFS/px4fmu_common/init.d-posix/airframes/
```

- `$PX4/ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt` 의 `4021_gz_x500_flow` 아래에 `4022_gz_x500_cf2`, `4023_gz_x500_cf2x2` 두 줄을 추가한다.

```bash
(cd $PX4 && make px4_sitl)
```

### 0-2. ROS 2 워크스페이스

```bash
cd ~/drone_project/DronePositioningSystem/ros2_ws && colcon build
```

- `~/.bashrc` 에 넣는다: `source /opt/ros/humble/setup.bash`, px4_msgs 워크스페이스, 이 워크스페이스 (`install/setup.bash`)
- 같은 LAN 에서 ROS 2 가 섞이지 않도록 `ROS_DOMAIN_ID`, `GZ_PARTITION`, `GZ_IP=127.0.0.1` 도 둔다.

### 0-3. 월드 다시 만들기 (설정을 바꿨을 때만)

```bash
cd ~/drone_project/DronePositioningSystem/gazebo/worlds
python3 generate_indoor_cage.py                                   # indoor_cage
LIDAR_LAYOUT=real LIDAR_H=1.26 python3 generate_indoor_cage.py    # indoor_cage_real_h126
python3 generate_competition_map.py                               # competition_map
```

- 만든 뒤 0-1 처럼 PX4 worlds 폴더에 복사한다.

---

## 개념

### 시뮬 LiDAR

- Gazebo `gpu_lidar` 는 스캔 전체를 한 순간에 그린다. 실제 라이다처럼 돌며 찍지 않는다.
- 두 센서의 스캔 시각(스탬프)은 같지만, Gazebo 가 센서를 순서대로 처리해 LiDAR2 가 수십 ms 늦게 도착한다.
- 그래서 수신 시각으로 처리하면 두 측정이 다른 시각이 되고(SU 가 유리), 측정 시각으로 처리하면 실기처럼 같은 시각이 된다.

### 융합 방식

- PF (점군 융합): 두 점군을 합친 뒤 군집을 한 번 찾는다.
- MF (관측 융합): 센서마다 위치를 찾고, 불확실성 가중으로 합쳐 KF 를 한 번 갱신한다.
- SU (순차 갱신): 센서마다 찾은 위치로 KF 를 차례로 두 번 갱신한다. 시뮬 라이브 기본값.

### 참값

- 실내: Gazebo 기체 pose (`/world/<월드>/dynamic_pose/info`, 브리지로 받는다)
- 24 × 15: PX4 위치 (`/fmu/out/vehicle_local_position_v1`)
- 비행 제어는 PX4 자체 위치로 한다. LiDAR 추정은 비행에 쓰지 않는다 (측위 평가용).

---

## 터미널 구성 (실내 라이브 비행)

| 터미널 | 내용 | 비고 |
|---|---|---|
| T1 | QGroundControl | 끝까지 켜 둔다 |
| T2 | PX4 + Gazebo | 끝까지 켜 둔다 |
| T3 | 통신 에이전트 (MicroXRCEAgent) | 끝까지 켜 둔다 |
| T4 | 트래커 (LiDAR 브리지 포함) | |
| T5 | 참값 브리지 + 경로 표시 | 끝나면 CSV·그림 저장 |
| T6 | RViz | |
| T7 | 비행 미션 | 착륙하면 스스로 끝남 |
| (T8) | bag 녹화 | 선택. PF/MF/SU 재생 비교할 때만 |

- 끌 때는 역순(T7 → T2)으로 Ctrl+C.

## 1. 실내 라이브 비행 — 터미널별

T1 — QGroundControl

```bash
~/Applications/QGroundControl.AppImage
```

T2 — PX4 + Gazebo (월드 indoor_cage, 기체 2배)

```bash
cd ~/drone_project/PX4-Autopilot
PX4_GZ_NO_FOLLOW=1 PX4_GZ_WORLD=indoor_cage make px4_sitl gz_x500_cf2x2
# "Ready for takeoff" 이면 정상
```

T3 — 통신 에이전트

```bash
MicroXRCEAgent udp4 -p 8888
```

T4 — 트래커 (기본 SU, `fusion_mode:=position` 이면 MF)

```bash
ros2 launch drone_localization lidar_localization.launch.py params_file:=lidar_localization_indoor.yaml
```

T5 — 참값 브리지 + 경로 표시

```bash
ros2 run ros_gz_bridge parameter_bridge "/world/indoor_cage/dynamic_pose/info@geometry_msgs/msg/PoseArray[gz.msgs.Pose_V" &
python3 ~/drone_project/DronePositioningSystem/experiments/indoor/path_viz.py --world indoor_cage --save ~/drone_project/flight_logs/indoor/live/run01
```

T6 — RViz

```bash
rviz2 -d ~/drone_project/DronePositioningSystem/experiments/indoor/indoor.rviz
```

(T8) — bag 녹화 (선택)

```bash
ros2 bag record -o ~/drone_project/flight_logs/indoor/live/run01/bag \
  /lidar1/points /lidar2/points /world/indoor_cage/dynamic_pose/info /fmu/out/vehicle_local_position_v1
```

T7 — 비행

```bash
ros2 run drone_mission indoor_survey_mission
# 또는: ros2 run drone_mission indoor_shapes_mission helix
```

- 착륙하면 T5 를 Ctrl+C → `true_path.csv`, `estimated_path.csv`, `path_result.png` 가 저장된다.

## 1-1. 실내 라이브 비행 — 한 번에 (`live_flight.sh`)

```bash
~/Applications/QGroundControl.AppImage &
~/drone_project/DronePositioningSystem/experiments/indoor/live_flight.sh run01
```

- 위 T2~T7 을 차례로 띄우고, 비행이 끝나면 경로 그림을 연다. Enter 를 누르면 모두 닫는다.
- 결과: `~/drone_project/flight_logs/indoor/live/run01/` (`true_path.csv`, `estimated_path.csv`, `path_result.png`, 로그)
- 환경변수
    - `MODEL` — 기체 (기본 `x500_cf2x2`)
    - `WORLD` — 월드 (기본 `indoor_cage`)
    - `MISSION` — 미션 (기본 `indoor_survey_mission`, 예: `MISSION="indoor_shapes_mission helix"`)
    - `RECORD_BAG=1` — bag 녹화 + 끝나고 PF/MF/SU 재생 비교 그림 (1분에 2~3 GB)

---

## 2. 실내 경로 5종 실험 (`shapes_replicate.sh`)

```bash
~/Applications/QGroundControl.AppImage &
cd ~/drone_project/DronePositioningSystem/experiments/indoor
ROOT=~/drone_project/flight_logs/indoor/shapes N=3 ./shapes_replicate.sh record replay eval
```

단계마다 하는 일

1. `record` — 경로마다 N회, `run_height_test.sh` 로 화면 없이 비행한다.
    - 월드: 실기 배치 `indoor_cage_real_h126` 을 만들고 PX4 + Gazebo 를 띄운다.
    - 통신 에이전트, LiDAR·참값 브리지, 트래커(센서 위치는 월드 파일에서 읽음), 점 수 측정기를 띄운다.
    - bag 을 녹화하고 `indoor_shapes_mission <경로>` 로 비행한다. 결과: `<ROOT>/flights/F_<경로>_<회>/`
2. `replay` — 비행마다 bag 을 PF / MF / SU 로 재생한다 (ROI 4 × 3 m). 결과: `<ROOT>/cmp_state/F_*/{PF,MF,SU}0/`
3. `eval` — `shapes_eval.py` 로 평가한다. 결과: `<ROOT>/cmp_state/shapes_per_flight.csv`, `shapes_summary.csv`

- 환경변수: `SHAPES` (경로 고르기), `MODEL` (기본 `x500_cf2x2`), `CLOCK` (기본 `arrival` = 수신 시각, `stamp` = 측정 시각)
- 재생 비교가 목적이라 bag 은 항상 녹화한다. 비행 1회 2~3 GB, 디스크를 먼저 확인한다.

---

## 3. 24 × 15 m 맵 (터미널별)

```bash
cd ~/drone_project/PX4-Autopilot && PX4_GZ_NO_FOLLOW=1 PX4_GZ_WORLD=competition_map make px4_sitl gz_x500   # T2
MicroXRCEAgent udp4 -p 8888                                                                                # T3
ros2 launch drone_localization lidar_localization.launch.py                                                # T4 (lidar_localization_cov.yaml)
ros2 run drone_localization flight_recorder                                                                # T5 착륙하면 runNNN/ 으로 보관, 그림
ros2 run drone_mission competition_mission                                                                 # T7
```

---

## 4. 끝난 뒤 (오프라인)

### PF / MF / SU 비교 재생 — bag 을 녹화했을 때만

```bash
python3 -m drone_localization.replay_bag --bag <bag> --out <결과 폴더> \
  --clock arrival --ground-truth-topic /world/<월드>/dynamic_pose/info --quiet \
  --ros-args --params-file <yaml> -p fusion_mode:=cloud
# MF = position, SU = sequential
```

- `--clock arrival` = 수신 시각 (기본, 라이브와 같다), `--clock stamp` = 센서 측정 시각
- 출력: `estimate_details.csv`, `state_details.csv`, `measurements.csv`, `actual_ground_truth_path.csv`, `processing_cost.csv`, `manifest.json`
- `live_flight.sh` 를 `RECORD_BAG=1` 로 돌린 폴더(bag/, world.sdf 가 있음)는 그림까지 한 번에: `experiments/indoor/compare_fusion.sh <run 폴더>` → `python3 experiments/indoor/plot_fusion.py <run 폴더>/fusion`

### 결과 수치

- 성능 비교표는 저장소에 두지 않는다.
