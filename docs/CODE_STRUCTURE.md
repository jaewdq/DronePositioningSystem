# DronePositioningSystem — 코드 구조 정리

> 지상 고정 LiDAR 2대로 비행 중인 드론의 3D 위치를 실시간 추정하는 outside-in 측위 시스템.
> 제24회 한국로봇항공기경연대회 중급부문(실내 조난자 탐색 임무) 시뮬레이션 검증용.

---

## 1. 디렉토리 트리

```
DronePositioningSystem/
├── README.md                          실행 방법, 의존성, 채점기준 대응표
├── docs/
│   └── DESIGN.md                      설계서 (센서배치·알고리즘·평가기준·설계 근거)
├── gazebo/                             시뮬레이션 "무대" — PX4-Autopilot 트리에 복사해서 사용
│   ├── worlds/
│   │   ├── generate_competition_map.py   맵(격자+마커+LiDAR 2대) 생성기 — 실행하면 아래 2개 산출
│   │   ├── competition_map.sdf           생성된 Gazebo world (실행용)
│   │   ├── competition_map_coordinates.txt  격자/마커 좌표 "정답지"
│   │   └── competition_map_README.md     맵 생성/배포 방법
│   └── models/
│       ├── generate_aruco_markers.py     ArUco 마커 PNG+SDF 생성기 (단독 실행 가능)
│       ├── aruco_marker_id{1..4}_at_x{X}_y{Y}/   마커 모델 4개 (PNG+model.config+model.sdf)
│       └── ground_lidar_sensor/          Ouster OS0-SR 64ch 등가 gpu_lidar 센서 모델
└── ros2_ws/src/
    ├── drone_localization/               ★ 측위 시스템 (프로젝트의 핵심)
    │   ├── drone_localization/
    │   │   ├── lidar_drone_tracker.py    2-LiDAR 융합 + EKF 추적 노드 (732줄, 핵심 알고리즘)
    │   │   ├── flight_recorder.py        실제(PX4)/추정(LiDAR) 궤적을 CSV로 동시 기록
    │   │   └── plot_flight.py            계획 vs 실제 vs 추정 시각화 + 오차(정밀도) 분석
    │   ├── params/lidar_localization.yaml  전체 파라미터 (센서 pose, 필터 임계값, EKF 노이즈 등)
    │   └── launch/lidar_localization.launch.py  브릿지 + 추적 노드 실행
    └── drone_mission/                    비행 시나리오 스캐폴딩 (측위 시스템의 "시험용 무빙 타겟")
        └── drone_mission/
            ├── competition_mission.py    대회 궤적 재생 (open-loop, PX4 offboard)
            ├── waypoint_land.py          단순 왕복 테스트용 offboard 스크립트
            ├── generate_groundtruth_trajectory.py  대회 시나리오 궤적 생성 (경로 primitive 기반)
            └── ground_truth_path_publisher.py      위 궤적을 RViz2 Path/MarkerArray로 발행
```

---

## 2. 파일별 요약

### 2.1 `drone_localization` (핵심 측위 시스템)

| 파일 | 역할 | 비고 |
|---|---|---|
| `lidar_drone_tracker.py` | 2대 LiDAR PointCloud2 구독 → world좌표 변환·융합 → ROI/Voxel/RANSAC지면제거/SOR 전처리 → DBSCAN 클러스터링 → 최근접+게이트 연관 → 6상태 칼만필터(등속도) 추적 → `/drone/estimated_pose` 발행 | `EKF6D` 클래스(예측/보정) + `LidarDroneTracker` 노드 클래스로 구성. `open3d`, `scipy` 필요 |
| `flight_recorder.py` | `/fmu/out/vehicle_local_position_v1`(PX4 실측, NED→ENU 변환) 과 `/drone/estimated_pose`(LiDAR 추정)를 같은 wall-clock 기준으로 CSV 2개에 실시간 기록 | `flight_logs/actual_path.csv`, `estimated_path.csv` |
| `plot_flight.py` | 두 CSV + 계획 궤적(`generate_groundtruth_trajectory`)을 불러와 top-down/고도/3D 궤적 플롯 + 시간정렬 후 오차(median/RMS/max, Z bias) 계산 | 시간축이 다른 두 로그를 ±30s 스캔해 최적 offset을 찾아 정렬 |
| `params/lidar_localization.yaml` | LiDAR 2대의 world pose, ROI, voxel leaf, RANSAC/SOR/DBSCAN 임계값, home-seed, EKF 노이즈, 코스팅 프레임 수 등 전체 튜닝값 | launch 파일이 항상 이 값을 노드에 주입 (노드 코드 내 기본값과 다름 — 아래 "참고" 참조) |
| `launch/lidar_localization.launch.py` | `ros_gz_bridge`로 `/lidar1,2/points`를 ROS2 PointCloud2로 브릿지 + `lidar_drone_tracker` 노드 실행 | 항상 LiDAR 2대 구독 (단일 LiDAR 모드는 제거됨) |

### 2.2 `drone_mission` (비행 시나리오 — 측위 시스템의 테스트 대상)

> **중요**: 이 패키지는 참가자가 조종할 실제 드론을 대신해 시뮬레이션 필드에 "움직이는 표적"을 만들어주는 용도다. `/drone/estimated_pose`(LiDAR 추정치)를 피드백으로 쓰는 폐루프 항법이 아니라, 미리 계산한 궤적을 그대로 재생하는 open-loop 스크립트다.

| 파일 | 역할 | 비고 |
|---|---|---|
| `generate_groundtruth_trajectory.py` | 대회 시나리오(이륙→라인트레이싱으로 id1~4 순회(호버)→직선 경로로 역순 복귀(호버)→착륙)를 `Primitive`(move/hover/turn) 리스트로 만들고 균일 dt(0.1s)로 샘플링. `build_primitives()`/`sample_trajectory()`/`generate_trajectory()`가 다른 모듈의 import 진입점 | 다른 3개 파일이 모두 이 모듈을 import해서 궤적을 공유 |
| `competition_mission.py` | 위 궤적을 world(ENU)→PX4 NED로 변환해 PX4 offboard position setpoint로 재생. armed/offboard 진입까지 재시도 로직 포함, 마지막은 PX4 자체 NAV_LAND로 착륙 | LiDAR 추정치 미사용 (의도된 설계) |
| `waypoint_land.py` | (0,0,3) 이륙 → 호버 → 전방 5m → 호버 → 착륙, 훨씬 단순한 offboard 상태머신 | competition_mission의 원형/디버깅용 |
| `ground_truth_path_publisher.py` | 계획 궤적을 `nav_msgs/Path` + 마커/호버포인트로 RViz2에 발행 (래치 QoS) | 시각화 전용, 제어와 무관 |

### 2.3 `gazebo` (시뮬레이션 환경 생성)

| 파일 | 역할 | 비고 |
|---|---|---|
| `generate_competition_map.py` | 30×20m 필드(7×5 격자), ArUco 마커 4개, LiDAR 2대를 배치한 `competition_map.sdf` 생성 | `LIDARS` 리스트가 실제 배치 좌표의 단일 진실 소스 — `lidar_localization.yaml`과 반드시 동기화 필요 (자동 동기화 아님, 수동 관리) |
| `generate_aruco_markers.py` | `cv2.aruco.DICT_4X4_50`로 마커 PNG + Gazebo model.sdf 생성 | `generate_competition_map.py`가 import해서 재사용 |
| `ground_lidar_sensor/model.sdf` | Ouster OS0-SR 64ch 스펙(±45.4° 수직FOV, 360°/1024, 0.3~35m, σ=0.8cm, 10Hz)의 `gpu_lidar` 센서 + 지지대 시각 모델 | |

---

## 3. 데이터 흐름 (실행 시점)

```
[Gazebo: competition_map.sdf]
  ground_lidar_1(SW) → /lidar1/points (gz)
  ground_lidar_2(NE) → /lidar2/points (gz)
         │
         ▼  ros_gz_bridge (launch 파일이 실행)
  sensor_msgs/PointCloud2  ×2
         │
         ▼  lidar_drone_tracker.py
  전처리 → DBSCAN → 연관 → EKF  ──▶ /drone/estimated_pose
                                          │
  [drone_mission: competition_mission.py] │  (open-loop, 서로 독립적으로 동시 실행)
  사전계산 궤적 재생 → PX4 offboard        │
         │                                │
         ▼ (PX4 실제 상태)                 ▼ (LiDAR 추정)
  /fmu/out/vehicle_local_position_v1 ─┬─▶ flight_recorder.py → actual_path.csv / estimated_path.csv
                                       │
                                       ▼
                                  plot_flight.py
                             (궤적 오버레이 + 오차 통계)
```

두 서브시스템(측위계 / 비행 스캐폴딩)은 **서로 데이터를 주고받지 않고**, `flight_recorder`가 사후에 두 로그를 나란히 저장해 `plot_flight`가 사후 비교·평가하는 구조다.

---

## 4. 설계서 대비 참고 사항 (docs/DESIGN.md와 대조)

- **LiDAR 배치가 설계 결론(ns)과 다르게 구현됨**: 설계서 §1은 ns(남북, 최소점 9.0) 배치를 최적으로 결론냈지만, 실제 `params/lidar_localization.yaml`과 `generate_competition_map.py`의 `LIDARS`는 diag(대각선, 최소점 5.1) 배치로 되어 있음. 발표 자료에서 "왜 diag를 선택/유지했는지" 또는 "ns로 바꿀 계획"인지 명확히 해두면 좋음.
- **`layout` 실행 인자는 문서에만 존재**: DESIGN.md의 실행 표는 `layout:=ns` 같은 배치 전환 인자를 언급하지만, 코드베이스에는 해당 launch 인자나 `competition_map_ns` 월드가 없음 (grep 결과 없음). 다중 배치 비교 실험을 하려면 이 스위칭 기능을 실제로 구현해야 함.
- **노드 기본 파라미터(`_declare_parameters`)가 yaml보다 튜닝이 덜 됨**: 예) `cluster_tolerance` 기본 0.25 vs yaml 0.40, `max_assoc_dist` 기본 3.0 vs yaml 1.5. launch로 실행하면 yaml이 항상 이기므로 실제 운용엔 문제없지만, 노드 단독 실행 시 주의.
- **추적 상실 후 재획득 로직 없음**: `coast_max_frames`(1.5s)를 넘겨도 EKF는 계속 등속도로 외삽되고 `initialized`가 리셋되지 않음 → 장시간 미검출 후 재획득 시 게이트(`max_assoc_dist`)에 막혀 영영 못 붙잡을 가능성. DESIGN.md §4.5 "초기화·실패모드"에 미해결 항목으로 이미 인지되어 있음.

---

*(코드 스냅샷 기준: 2026-07-29 세션에서 정리. 이후 코드가 바뀌면 이 문서도 갱신 필요.)*
