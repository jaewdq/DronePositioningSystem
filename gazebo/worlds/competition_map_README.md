# competition_map — 제24회 한국로봇항공기경연대회 중급부문 맵

## 디렉토리 구조

```
Tools/simulation/gz/
├── worlds/
│   ├── competition_map.sdf              # 실행용 world 파일
│   ├── competition_map_coordinates.txt  # 격자/마커 좌표 전체 목록 (정답지)
│   ├── generate_competition_map.py      # 위 두 파일을 생성하는 스크립트
│   └── competition_map_README.md        # 이 문서
└── models/
    ├── generate_aruco_markers.py        # ArUco PNG + 모델 생성 스크립트 (단독 실행 가능)
    ├── aruco_marker_id1_at_x4_y4/
    │   ├── model.config
    │   ├── model.sdf
    │   └── marker.png                   # 실제 ArUco 텍스처 (흰 여백 포함)
    ├── aruco_marker_id2_at_x4_y16/
    ├── aruco_marker_id3_at_x26_y4/
    └── aruco_marker_id4_at_x13_y12/
```

## 텍스처(머티리얼) 연결 방식

각 마커 모델의 `model.sdf`는 같은 디렉토리의 `marker.png`를 PBR 알베도 맵으로 참조한다:

```xml
<material>
  <pbr>
    <metal>
      <albedo_map>model://aruco_marker_id1_at_x4_y4/marker.png</albedo_map>
    </metal>
  </pbr>
</material>
```

`model://<이름>/...` 경로는 `GZ_SIM_RESOURCE_PATH` 환경변수에 등록된 디렉토리들을 기준으로 풀린다.
PX4 SITL은 `make px4_sitl` 실행 시 `Tools/simulation/gz/models`를 자동으로 resource path에
추가하므로, 마커 모델 디렉토리를 그 안에 두기만 하면 별도 설정 없이 인식된다.

## 맵을 다시 생성하고 싶을 때 (마커 위치 재배치 등)

```bash
cd ~/drone_project/PX4-Autopilot/Tools/simulation/gz/worlds

# OpenCV(aruco)가 시스템 numpy와 충돌하면 격리 환경 사용:
python3 -m pip install --target /tmp/arucopkgs "numpy<2" "opencv-contrib-python-headless"

PYTHONPATH=/tmp/arucopkgs python3 generate_competition_map.py
```

- `generate_competition_map.py` 안의 `RANDOM_SEED = 42`를 `None`으로 바꾸면 마커 위치가 매번 랜덤,
  다른 정수로 바꾸면 그 값에 대응하는 고정 배치로 재현 가능하다.
- 재생성하면 `competition_map.sdf`, `competition_map_coordinates.txt`, 그리고
  `models/aruco_marker_id*_at_x*_y*/` 디렉토리가 새 좌표 기준으로 덮어써진다(기존 마커 모델
  디렉토리 이름이 달라지므로, 이전 좌표의 디렉토리는 수동 삭제 필요).

## PX4 SITL과 함께 실행

```bash
cd ~/drone_project/PX4-Autopilot
PX4_GZ_WORLD=competition_map make px4_sitl gz_x500
```

드론(x500)은 world 원점 (0, 0, 0) — 맵 내 초록색 원형 `home_pad` 위치 — 에 스폰된다.
이는 대회 규정의 이착륙 지점(①, ⑦)에 해당한다.

## 맵 사양 요약

| 항목 | 값 |
|---|---|
| 임무구역 | 30m(X) x 20m(Y) |
| 격자 칸 수 | X 7칸 x 4.2857m, Y 5칸 x 4.0000m (정확히 4m로 등분 불가하여 근사, 본문 주석 참고) |
| 구획선 폭 | 10cm |
| 마커 크기 | 0.5m x 0.5m |
| 마커 개수/사전 | 4개, `cv2.aruco.DICT_4X4_50`, ID 1~4 (방문 순서와 일치) |
| 마커 배치 제약 | 임무구역 경계에서 최소 1격자 셀 이상 안쪽 (24개 후보 중 랜덤 4개) |
| 벽/조명 | 없음 — 기본 야외 sun light 1개만 사용 (격자/마커만 있는 단순 구성) |
