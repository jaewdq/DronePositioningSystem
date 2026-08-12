# DronePositioningSystem

지상 고정 LiDAR 2대만으로 비행 중인 드론의 3D 위치를 실시간 추정하는 **outside-in 측위 시스템**.
제24회 한국로봇항공기경연대회 중급부문(실내 조난자 탐색 임무) 심사 보조용 시뮬레이션 검증 프로젝트.

PX4 SITL + Gazebo Harmonic + ROS2 Humble 환경에서 동작한다.

> **왜 outside-in인가**: 참가팀마다 기체와 탑재 센서가 달라 온보드 측위 성능에 편차가 크다.
> 심사에는 기체 형상·탑재장비와 무관하게 동일 기준으로 재는 "외부 심판"이 필요하다.

---

## 1. 개요

- **Ouster OS1-128 등가 gpu_lidar 2대**를 임무구역(30×20m) 밖(금지구역 33×23m 밖)
  남북(ns)으로 배치. 수직FOV ±21.2°, 높이 2.0m — 호버 고도(2m)와 같게 두어 어느
  거리에서나 목표를 0° 각도로 본다.
- 각 LiDAR 포인트클라우드를 월드좌표로 변환·**융합** → ROI z-band 필터 → DBSCAN 클러스터링
  → 드론 클러스터 선택 → **6상태 KF**(등속도 모델, 표준 선형 칼만필터) 추적
- 출력: `/drone/estimated_pose` (PoseWithCovarianceStamped, 월드 ENU)
- 비행 후 **경로점(마커) 위 호버 구간을 자동 검출**해 마커별 위치를 재추정 → 채점 근거 산출
- 실제 비행 궤적(PX4 ground-truth)과 비교·시각화하는 분석 도구 포함

### 현재 성능 (정상 비행 4회 · 경로점 28개)

| 항목 | 값 |
|---|---|
| 경로점 수평오차 | 평균 **0.089 m**, 중앙 0.082 m, 최대 0.163 m |
| 경로점 수평 RMSE | **0.095 m** |
| 런간 재현성 | **0.015 m** (런 평균의 표준편차) |
| 채점기준 <0.5m 통과 | **28/28 (100%)** |
| 축별 평균오차 | X +0.055 m, Y −0.012 m |
| 수직(Z) | bias +0.200 m, **std 0.028 m** — bias는 오차가 아니라 기준점 정의차, [§6.1](#61--z-기준면datum이-정의되지-않음--채점-방식-미결) 참조 |
| 검출률 | **100%** (비행 구간 전체) |

대회 채점기준(경로점 위치추정 수평오차 <0.5m = 상)에 여유 있게 들어간다.
**강건성 시험(비/안개, 가림)과 배치별 비교는 아직 하지 않았다.**

---

## 2. 디렉토리 구조

```
ros2_ws/src/
  drone_localization/                  # 측위 시스템 (핵심)
    drone_localization/
      lidar_drone_tracker.py           # 2-LiDAR 융합 + KF 추적 노드
      lidar_drone_tracker_fusion.py    # [실험] 센서별 클러스터링 + KF 2회 갱신 (기본 비활성)
      flight_recorder.py               # 궤적 CSV 기록 + 착륙 감지 시 자동 보관·시각화
      archive_run.py                   # 수동 보관 (runNNN/)
      analyze_waypoints.py             # [오프라인] 호버 구간 검출 → 경로점 위치 재추정
      evaluate_accuracy.py             # 정확도 수치 (콘솔) — 모든 플롯의 계산 원본
      plot_accuracy.py                 # ★ 정확도 요약 (3D 궤적 + 경로점 수평/수직) — 착륙 시 자동 실행
      plot_flight.py                   # 계획 vs 실제 vs 추정 (평면+고도+3D 요약)
      plot_flight_3d.py                # 전체 경로 3D 전용 뷰
      compare_fusion.py                # [오프라인] 융합 방식 비교 (밀도가중/동일/역분산)
    params/lidar_localization.yaml     # 센서 pose·필터·KF 전체 튜닝값
    launch/lidar_localization.launch.py
  drone_mission/                       # 비행 시나리오 (측위계의 시험용 표적)
    drone_mission/
      competition_mission.py           # PX4 offboard 궤적 재생
      generate_groundtruth_trajectory.py
      ground_truth_path_publisher.py   # RViz2에 계획 경로 표시 (선택)

gazebo/
  worlds/
    generate_competition_map.py        # 맵 생성기 (격자+마커+LiDAR 2대)
    competition_map.sdf                # 생성된 world
    competition_map_coordinates.txt    # 격자/마커/LiDAR 좌표 (정답지)
  models/
    generate_aruco_markers.py
    generate_ground_texture.py         # 잔디+흙 바닥 텍스처 생성기 (고정 시드)
    arena_ground/                      # 생성된 바닥 모델 (grass.png + SDF)
    aruco_marker_id*/                  # 마커 모델 4개

docs/
  DESIGN.md                            # 설계서 (센서배치 근거·알고리즘·평가기준)
  KALMAN_FILTER_DESIGN.md              # 칼만필터 적용 구조와 개선안
```

> **좌표 동기화 주의**: LiDAR/마커 좌표는 `generate_competition_map*.py`(월드 생성)와
> `params/*.yaml`(노드 변환)에 **각각** 적혀 있고 자동 동기화되지 않는다. 한쪽만 고치면
> 측위가 통째로 어긋난다.

---

## 3. 파이프라인

```
[Gazebo] ground_lidar_1(남) → /lidar1/points
         ground_lidar_2(북) → /lidar2/points
              │
              ▼  ros_gz_bridge
      sensor_msgs/PointCloud2 ×2
              │
              ▼  lidar_drone_tracker
   월드좌표 변환 → 신선도 검사(0.25s) → 동시성 검사(50ms) → 융합
   → ROI 필터 (XY: 필드+2m, Z: 1.0~3.0m 호버 밴드)
   → DBSCAN 클러스터링 → KF predict → 클러스터 선택(최근접 + 적응 게이트)
   → 클러스터 median = 드론 위치 → KF update (6상태 등속도)
              │
              ▼
     /drone/estimated_pose  ──▶ flight_recorder ──▶ estimated_path.csv
                                                          │
     [PX4] /fmu/out/vehicle_local_position_v1 ─▶ actual_path.csv
                                                          │
                                    ┌─────────────────────┴────────────┐
                                    ▼                                  ▼
                          analyze_waypoints              plot_accuracy (착륙 시 자동)
                    (호버 구간 → 경로점 위치 재추정)      evaluate_accuracy (콘솔 수치)
```

### 두 LiDAR를 합치는 방식

두 LiDAR는 동기화 없이 각자 퍼블리시한다. 수신 즉시 센서 좌표를 월드 좌표로 변환해
센서별 버퍼에 **최신 것만** 저장하고(큐가 아닌 단일 슬롯), 별도의 10Hz 타이머가 버퍼를
검사한다.

- **신선도** — 받은 지 0.25초가 지났으면 센서 두절로 보고 제외
- **동시성** — 남은 것들의 *촬영* 시각 차이가 50ms 이내일 때만 합침. 벗어나면 최신 것만 사용

시간이 벌어진 스캔을 합치지 않는 이유는 형상이 왜곡되기 때문이다. 1.5m/s로 날 때 0.1초
차이는 15cm 이동에 해당하고, 그대로 병합하면 드론이 진행 방향으로 늘어난 것처럼 관측되어
중심이 틀어진다. 두 센서 모두 10Hz라 100ms 주기에서 가장 가까운 짝의 시간차는 구조적으로
최대 50ms이므로, 실측에서는 두 센서 모두 전체 프레임의 100%에 기여한다.

### 설계상 중요한 선택들

- **지면제거 단계가 없다.** ROI의 z-band(1.0~3.0m)가 호버 고도만 통과시키므로 바닥이
  애초에 파이프라인에 들어오지 않는다. RANSAC/높이컷 지면제거를 대체한 것으로,
  이착륙·지상 구간 추적을 포기하는 대신 얻은 단순함이다.
- **클러스터 중심은 median.** 이상치 포인트 하나가 평균은 눈에 띄게 끌지만 median은
  거의 못 움직인다.
- **호버 판정은 시간이 아니라 "느림 + 근접"** — 규정에 호버 시간 조항이 없어 팀마다
  체류 패턴이 다르다. 고정 시간을 전제하면 안 된다.
- **경로점 게이트 반경 4.5m** — 채점기준 하 등급이 <4m이므로, 좁게 잡으면 항법이
  부정확한 팀은 표본이 아예 안 잡혀 채점 자체가 불가능해진다.
- **경로점 산출은 오프라인이 정본.** 노드 안의 실시간 검출도 살아 있지만 조각남 문제가
  있다 ([§6.2](#62-실시간-경로점-검출이-하나의-호버를-여러-조각으로-나눔)).
- **연관 게이트는 KF 공분산에 비례.** 고정값이면 예측이 한번 밀린 뒤 진짜 드론이
  게이트 밖으로 밀려나고, 관측이 끊기니 예측이 더 밀리는 악순환이 생긴다. 다만 잡동사니를
  물지 않도록 상한을 둔다.
- **predict → 연관 → update 순서.** 연관을 먼저 하면 직전 프레임의 사후추정값을 기준으로
  비교하게 되어, 1.5m/s에서 0.15m를 그냥 버리는 셈이 된다.
- **ROI 통과 최소 점수는 `cluster_min_points`와 동일.** 예전에는 10으로 하드코딩돼
  있었는데, ROI를 z-band로 좁힌 뒤로는 그 안에 드론밖에 없어서(중앙값 14점, 원거리 3~7점)
  정상 검출을 통째로 버리고 있었다 ([§6.7](#67-해결됨-roi-최소-점수-문턱값이-검출을-막고-있었음)).

---

## 4. 의존성

- ROS2 Humble, PX4-Autopilot (SITL), Gazebo Harmonic (gz-sim 8)
- `ros-humble-ros-gzharmonic` (LiDAR PointCloud2 브릿지)
- Micro-XRCE-DDS-Agent (PX4 ↔ ROS2)
- `px4_msgs`, `px4_ros_com` (별도 클론 필요)
- Python: `open3d`, `scipy`, `numpy`, `matplotlib`, `pyyaml`
- 마커 재생성 시: `opencv-contrib-python-headless` (numpy<2 격리환경 권장)

---

## 5. 실행

### 빌드 / 배포

```bash
# Gazebo 파일을 PX4 트리에 복사
PX4=~/drone_project/PX4-Autopilot/Tools/simulation/gz
cp gazebo/worlds/*  $PX4/worlds/
cp -r gazebo/models/*  $PX4/models/

# ROS2 워크스페이스 빌드
cd ros2_ws
colcon build --packages-select drone_localization drone_mission
source install/setup.bash
```

### 사전 설정 — ROS2 도메인 격리 (권장)

같은 LAN에 다른 PC가 기본 도메인(0)으로 ROS2를 돌리면 노드와 토픽이 섞인다. 실제로 남의
미션 노드가 우리 PX4에 offboard 명령을 보내 드론이 엉뚱하게 나는 일이 있었다. `~/.bashrc`에:

```bash
export ROS_DOMAIN_ID=2       # 0~101 중 다른 사람과 겹치지 않는 값
export GZ_PARTITION=<이름>   # Gazebo transport는 DDS와 별개라 따로 막아야 함
export GZ_IP=127.0.0.1
```

QGC는 MAVLink(UDP)라 위 설정이 안 통한다. 남의 PX4에 붙는 게 의심되면 QGC의
Application Settings → Comm Links에서 자동연결(UDP)을 끄고 `127.0.0.1:14550` 수동 링크를
만들면 된다.

### 실행

```bash
# 터미널 1 — PX4 SITL + Gazebo
cd ~/drone_project/PX4-Autopilot
PX4_GZ_NO_FOLLOW=1 PX4_GZ_WORLD=competition_map make px4_sitl gz_x500

# 터미널 2 — Micro-XRCE-DDS Agent
MicroXRCEAgent udp4 -p 8888

# 터미널 3 — QGroundControl (상태 확인용)
~/Applications/QGroundControl.AppImage

# 터미널 4 — 측위 시스템 (브릿지 + 추적 노드)
ros2 launch drone_localization lidar_localization.launch.py

# 터미널 5 — 비행 기록 (비행 전에 켜 둘 것)
ros2 run drone_localization flight_recorder

# 터미널 6 — 궤적 비행
ros2 run drone_mission competition_mission
```

> `PX4_GZ_NO_FOLLOW=1`을 빼면 드론 스폰 시 카메라가 드론에 고정돼 시점 조작이 안 된다.

`flight_recorder`는 **착륙을 자동 감지**해서 그 비행을 `runNNN/`으로 보관하고 정확도
그림을 띄운 뒤, 새 CSV를 열어 다음 비행을 바로 기록한다. 노드를 껐다 켤 필요가 없고,
반복 측정 시 이전 데이터가 덮어써지지 않는다. Ctrl+C로 끝내도 같은 처리를 한다.

### 분석 도구

전부 오프라인 스크립트라 `python3`로 바로 실행된다 (`--output_dir`로 특정 런 지정 가능):

```bash
DPS=~/drone_project/DronePositioningSystem/ros2_ws/src/drone_localization/drone_localization

python3 $DPS/plot_accuracy.py      # 정확도 요약 (착륙 시 자동 실행되는 것과 동일)
python3 $DPS/evaluate_accuracy.py  # 같은 수치를 콘솔로
python3 $DPS/analyze_waypoints.py  # 경로점 재추정 → CSV
python3 $DPS/plot_flight_3d.py     # 3D 전용 뷰 (LiDAR 위치·ROI 밴드 포함)
python3 $DPS/archive_run.py        # 수동 보관

# 예: 특정 비행만
python3 $DPS/plot_accuracy.py --output_dir ~/drone_project/flight_logs/run040
```

### RViz2로 보려면

```bash
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map world   # 필수
ros2 run drone_mission ground_truth_path_publisher                  # 선택(계획 경로)
rviz2
```

Fixed Frame을 `map`으로 두고 Add → **By topic** 에서 추가:
`/lidar/filtered_points`, `/lidar/cluster_markers`, `/drone/estimated_pose`,
`/drone/waypoint_estimates`, `/ground_truth_path`

> TF를 발행하는 노드가 프로젝트에 없어(전부 `frame_id='map'` 절대좌표 사용) RViz2가
> "No tf data"로 표시를 막는다. 위 static_transform_publisher가 그 우회책이다.

### 산출물

`~/drone_project/flight_logs/` 에 생성된다 (gitignore 대상):

| 파일 | 내용 |
|---|---|
| `runNNN/` | **비행별 보관 디렉토리** (착륙 감지 시 자동 생성) |
| `actual_path.csv` | PX4 실제 궤적 (NED→ENU 변환) — 진행 중인 비행 |
| `estimated_path.csv` | LiDAR 추정 궤적 — 진행 중인 비행 |
| `accuracy_summary.png` | 정확도 요약 (착륙 시 자동 생성) |
| `waypoint_estimates_offline.csv` | **경로점별 재추정 위치 (정본)** |
| `waypoint_estimates.csv` | 실시간 검출 결과 (참고용, §6.2) |
| `accuracy_runs.png`, `flight_plot_3d.png` | 다중 런 비교 / 3D 뷰 |

---

## 6. 알려진 문제 (Known Issues)

### 6.1 ★ Z 기준면(datum)이 정의되지 않음 — 채점 방식 미결

**가장 중요한 미해결 항목.**

LiDAR는 드론 클러스터의 **월드 절대고도**를 잰다. 반면 PX4(및 참가팀 텔레메트리)의 고도는
**이륙지점 기준 상대고도**다. 두 값은 "착지 상태에서 기체 기준점이 지면 위 몇 m에 있는가"
만큼 차이 난다.

시뮬레이션에서 확인한 사례 — x500은 `model.sdf`에 `<pose>0 0 .24 ...>`로 정의되어 착지 시
base_link가 지면 위 0.24m에 있다:

| | 값 |
|---|---|
| PX4 보고 고도 (호버) | 1.999 m |
| → base_link 절대고도 (+0.24) | 2.239 m |
| LiDAR 추정 절대고도 | 2.236 m |
| **기준면 정렬 후 실제 오차** | **−0.007 m** |

즉 **측위는 처음부터 정확했고**, 한동안 "Z에 +0.2m 편향이 있다"고 본 것은 기준면을
맞추지 않은 비교 때문이었다.

정렬하지 않은 상태의 수치(§1 표의 bias +0.200m)는 이 오프셋이 그대로 들어간 값이다.
같이 적힌 **std 0.028m가 실제 고도 정밀도**다 — bias는 상수라 빼면 사라지지만 산포는
못 없앤다. 경로점 median 평균화 덕에 표본 단위 산포(약 0.07m)보다 크게 줄어든 값이다.

**문제는 이 오프셋을 대회에서 알 수 없다는 것이다.** 0.24m는 x500의 값이고 참가팀 기체는
전부 다르다. 해결 방향 두 가지:

- **(a) 절대고도로 채점** — "지면 위 몇 m". 오프셋 자체가 불필요. 다만 기체별 착지 시
  센트로이드 높이 차이(대략 ±0.1m)가 그대로 오차에 들어간다.
- **(b) 패드 대비 상승고도로 채점** — 이륙 전 패드 위 센트로이드를 한 번 재고 이후
  `현재 − 기준선`. 기체 형상이 상쇄되고 참가팀 텔레메트리와 숫자가 맞는다. **base_link
  위치를 알 필요가 없다**(가정이 아니라 측정). 대가로 지상의 드론이 보여야 해서
  ROI z-band를 낮춰야 하고, 그러면 지면 클러터 문제가 되돌아온다.

→ **아직 정하지 않았다.** 정하기 전까지 보고되는 Z는 "월드 절대고도"다.

**부수 영향**: `flight_recorder`는 PX4의 상대고도를 그대로 월드 z로 기록한다. 따라서
`plot_flight`의 Z bias와 `plot_flight_3d --color_by_error`의 오차 색상은 현재 이 오프셋만큼
과장되어 나온다. 기준면 정책이 정해지면 함께 고쳐야 한다.

### 6.2 실시간 경로점 검출이 하나의 호버를 여러 조각으로 나눔

정지 중에도 KF 속도 추정이 프레임 단위로 미세하게 흔들려 문턱값을 넘나든다. 유예 로직
(`waypoint_grace_frames`)을 넣었지만 완전히 해결되지 않았다 — 실측에서 7회 방문이
13조각으로 기록된 적이 있다.

**오프라인 분석기(`analyze_waypoints`)는 같은 로그에서 7/7을 정확히 뽑는다.** 전체 궤적을
한 번에 보고 판단하므로 속도 추정을 넓은 창(±0.3초)으로 매끄럽게 낼 수 있고, 끊긴 구간도
시간 간격만 보고 병합할 수 있기 때문이다.

→ **경로점 산출은 오프라인 결과를 정본으로 쓸 것.** 실시간 쪽은 RViz 실시간 확인용으로만
남겨두었다.

### 6.3 LiDAR 하우징 자기 오검출 가능성 (미검증)

각 LiDAR는 반대편 LiDAR의 하우징/지지대를 본다. 크기·형태상 드론 클러스터와 비슷해 오검출될
수 있고, 특히 센서 마운트 높이(2.0m)가 ROI z-band(1.0~3.0m) 안에 들어와 있어 구조적으로
걸러지지 않는다.

top-level 워크스페이스에는 센서 위치 반경 0.8m를 무시하는 대응(`sensor_exclusion_radius`)이
있으나 이 저장소에는 아직 포팅하지 않았다. 실측에서 문제가 확인되지도 않았다.

### 6.4 파이프라인 단계 2개가 비활성 상태

디버깅 중 변수를 줄이려고 꺼둔 뒤 되돌리지 않았다 (코드에 주석으로 남아 있음):

- **Voxel downsampling** — 클러스터 포인트가 적을 때 median 정확도 저하 우려로 비활성.
  OS1-128은 OS0-SR보다 훨씬 조밀해 재검토 필요.
- **SOR (통계적 이상치 제거)** — 비/안개 산란잡음 대응용인데 현재 그런 잡음원이 없어 비활성.
  강건성 시험을 하려면 되살려야 한다.

### 6.5 클러스터링 파라미터가 OS1용으로 재튜닝되지 않음

`cluster_tolerance`, `cluster_min_points` 등은 OS0-SR(수직 분해능 ~1.42°) 기준으로 맞춘
값이다. OS1-128은 ~0.33°로 4배 이상 조밀해 클러스터 크기·밀도가 다르다. 실비행 로그를 보고
조정해야 한다.

### 6.6 배치 비교 미완료

반복 검증(4회)은 했지만, 설계서(`docs/DESIGN.md`)가 비교한 4가지 배치(ns/cross/edge/diag)
중 ns와 diag만 실행해봤다. 현재 저장소는 **OS1-128 / ns 단일 구성**이며, OS0-SR / diag
구성은 `~/drone_project/os0sr_archive/`로 옮겨 이 PC에만 남겼다(저장소에서는 제거).
다른 배치를 시험하려면 `generate_competition_map.py`의 `LIDARS`와 yaml의 `lidarN_*`를
함께 고쳐 재생성해야 한다. 비/안개 등 강건성 시험도 하지 않았다.

### 6.7 (해결됨) ROI 최소 점수 문턱값이 검출을 막고 있었음

기록용. ROI 통과 점이 10개 미만이면 프레임을 통째로 버리는 코드가 있었다. ROI가 z 0~6m로
넓어 바닥이 수천 점을 공급하던 시절에는 "10점 미만 = 센서가 아무것도 못 봄"이 맞았지만,
ROI를 호버 고도 밴드(1~3m)로 좁힌 뒤로는 그 안에 드론밖에 없다. 드론이 내놓는 점은 중앙값
14개, 원거리에서는 3~7개라 **정상 검출이 통째로 버려지고 있었다** — 검출률 37%, 미검출
사유는 거의 전부 "ROI 통과 <10점"이었다.

문턱값을 `cluster_min_points`에 맞추자 같은 경로에서 검출 프레임이 428 → 1107개로 늘었다.
이 하나가 당시 관찰되던 여러 증상(호버 중 0.8~0.9m 이상치, 한쪽 센서만 기여하는 것처럼
보이던 현상, 코스팅 중 오버슛)의 공통 원인이었다.

**교훈**: 파이프라인 앞단의 전제를 바꾸면(ROI 범위), 그 전제에 맞춰 고른 상수(문턱값 10)도
같이 재검토해야 한다.

### 6.8 X축 평균오차의 원인 미규명

정상 비행 4회에서 X 평균오차가 +0.055m로 Y(−0.012m)보다 일관되게 크다. 두 가설을 세웠으나
데이터가 모두 반박했다:

- **처리 지연** — 정지 중에도 오차가 그대로 남는다(이동 +0.074 vs 정지 +0.077)
- **센서 쪽으로 끌림** — 드론이 센서(x=15) 양쪽 어디에 있든 부호가 안 바뀐다

즉 위치·속도와 무관한 고정 오프셋으로 보인다. LiDAR 좌표는 yaml/월드 생성기/배포된 SDF가
모두 일치함을 확인했으므로 좌표 불일치도 아니다. 크기가 채점 기준의 11%라 급하지 않지만
원인은 아직 모른다.

---

## 7. 범위

이 저장소는 **지상 LiDAR 측위 시스템**을 만든다. `drone_mission`의 비행 스크립트는 실제
대회에서 참가팀이 조종할 드론을 대신해 시뮬레이션 필드에 "움직이는 표적"을 만들어주는
시험용 스캐폴딩이다 — `/drone/estimated_pose`를 피드백으로 쓰는 폐루프 항법이 아니라
미리 계산한 궤적을 그대로 재생하는 open-loop 스크립트다(의도된 설계).
