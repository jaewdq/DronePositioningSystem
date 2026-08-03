# tracker 두 버전 비교 — git본 vs top-level본

두 파일:
- **git본**: `DronePositioningSystem/ros2_ws/src/drone_localization/drone_localization/lidar_drone_tracker.py` (git 관리, 지금 계속 작업 중인 파일, 684줄)
- **top-level본**: `~/drone_project/ros2_ws/src/drone_localization/drone_localization/lidar_drone_tracker.py` (live 빌드 워크스페이스, 827줄, 2026-07-07 이후 정체)

세 그룹으로 나눠서 봄: **① git본이 이미 이긴 것(포팅 불필요) / ② 목표가 갈라진 것(의도된 차이) / ③ top-level에만 있고 판단 필요한 것**

---

## ① 이름/안정성 정리 — git본이 이미 우위, 가져올 것 없음

| 항목 | git본 | top-level본 |
|---|---|---|
| 필터 클래스명 | `KF6D` (F,H 상수행렬이라 EKF 아님, 이름 바로잡음) | `EKF6D` (틀린 이름 그대로) |
| 공분산 업데이트 | Joseph form (`IKH@P@IKH.T + K@R@K.T`) | `(I-KH)@P` (반올림 오차에 덜 안정적) |
| 칼만이득 계산 | `np.linalg.solve` | `np.linalg.inv` (역행렬 직접 계산, 느리고 덜 안정적) |
| 라이다 구독 | 항상 2대, 전용 콜백 함수(`_cb_lidar1/2`) | `enable_lidar2`로 단일모드 가능, 클로저(`_make_pc_callback`) |
| 두 라이다 시간동기화 | ROS 헤더 stamp 비교, 50ms 이내만 융합 | 없음 (수신시각 staleness만 체크, 두 소스 시간차 무시) |
| 트래킹 유실 처리 | `coast_max`→경고 유지, `reset_after`(5초)→KF 리셋 후 재획득 대기 | `coast_max` 넘으면 즉시 트랙 드롭(콜드스타트로 전환, 리셋 개념 없음) |
| 미사용 import/플래그 | `PointField`, `HAS_SCIPY/O3D` 등 정리됨 | 안 정리됨, 그대로 남아있음 |
| `_validate_pose` (금지구역 체크) | 삭제 (시뮬 환경이라 불필요 판단) | 존재 |

**결론**: 이 표의 항목들은 top-level에서 가져올 게 없음. 오히려 나중에 여유 있으면 top-level 쪽을 git본 기준으로 다시 맞추는 게 맞는 방향.

---

## ② 목표가 갈라진 부분 — "누가 낫다"가 아니라 설계 목표 차이

| 항목 | git본 | top-level본 |
|---|---|---|
| 추적 목표 | **호버링 구간만** | 이착륙 포함 **전체 루트** |
| 지면 처리 | `_remove_ground` 자체가 없음. ROI z-band(1.0~3.0m)가 대신함 | `ground_method` 파라미터로 height-cut(기본 0.10m) 또는 RANSAC 선택 가능 |
| ROI z 범위 | 1.0 ~ 3.0m (좁음, 호버 전용) | 0.0 ~ 6.0m (넓음, 지면부터 다 봄) |
| 클러스터 높이 게이트 | 없음 (ROI가 이미 좁혀놔서 중복이라 삭제함) | `min_drone_height`(0.3)~`max_drone_height`(5.5) 별도 존재 |
| `home_z` 시드값 | 0.15 (**패드 기준값이 아직 남아있음 — 안 맞음, 미해결**) | 0.3 (패드 기준, 자기 목표와는 일치) |

**결론**: git본은 "호버만 본다"로 확정된 설계고, top-level은 아직 "전체 루트"를 전제로 함. 이 차이는 의도된 것이라 top-level 걸 가져올 필요 없음. 다만 git본의 `home_z=0.15`는 새 목표(호버만)에 안 맞는 채로 남아있는 미해결 이슈 (전에 짚었던 것).

---

## ③ top-level에만 있는 기능 — 포팅 여부가 아직 안 정해진 것

| 기능 | 설명 | 상태 |
|---|---|---|
| **z-캘리브레이션** (`enable_z_calibration`, `_z_baseline`, `_update_z_calibration()`) | 패드 위 저고도(z<0.6m)에서 센트로이드 샘플을 모아 "이 기체의 지면 기준 오프셋"을 측정, 이후 고도값에서 빼줌 | ❌ **git본에 못 씀** — ROI가 1m 밑을 아예 안 보니 샘플 모을 기회가 없음. 호버-only 결정과 구조적으로 충돌. |
| **`sensor_exclusion_radius`(0.8m)** | 각 LiDAR 설치 위치 반경 0.8m 안의 포인트를 통째로 버려서, 반대쪽 센서 하우징이 가짜 클러스터로 잡히는 것 방지 | ⏸️ **보류** (오늘 "나중에 고려하자") — z-band가 1~3m로 좁아지면서 센서 마운트 높이(1.5~2m)와 겹쳐 위험이 커진 상태. 나중에 다시 꺼낼 것. |

---

## 정리: 지금 뭘 해야 하나

1. ①은 끝난 얘기 — 손 댈 것 없음
2. ②는 확정된 방향 — `home_z` 미스매치만 아직 미해결로 남음
3. ③ 두 개는 **보류 중** — z-캘리브레이션은 이 설계로는 아예 불가능(포기), exclusion radius는 나중에 다시 논의
