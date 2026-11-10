#!/usr/bin/env python3
"""
제24회 한국로봇항공기경연대회 중급부문 "실내 조난자 탐색 임무" 맵 생성기 — OS1-128 버전.

generate_competition_map.py(OS0-SR, 원본, 그대로 둠)의 맵/마커 배치는 동일하게
유지하고, 지상 고정 LiDAR만 Ouster OS1(Standard Range) 128채널 + 남북(ns) 배치로
교체한 버전. OS1은 수직시야각이 OS0-SR의 절반(±21.2° vs ±45.4°)이라 배치 높이를
다시 계산함 (호버링(z≈2m) 구간만 보도록 설계 — 이착륙 구간은 이 배치로는 커버
안 함). XY는 docs/DESIGN.md §1에서 이론상 최고 검출밀도로 나온 ns 배치를 채택.

생성물:
  - worlds/competition_map.sdf              실행용 Gazebo world
  - worlds/competition_map_coordinates.txt  격자 교차점 전체 좌표 + 마커 4개 좌표
  - models/aruco_marker_id{N}_at_x{X}_y{Y}/     마커 모델 4개 (OS0-SR 버전과 공유, 동일 좌표라 재사용됨)

실행:
  python3 -m pip install --target /tmp/arucopkgs "numpy<2" "opencv-contrib-python-headless"
  PYTHONPATH=/tmp/arucopkgs python3 generate_competition_map.py
"""

import os
import sys

WORLDS_DIR = os.path.dirname(os.path.abspath(__file__))
MODELS_DIR = os.path.join(os.path.dirname(WORLDS_DIR), "models")
sys.path.insert(0, MODELS_DIR)
from generate_aruco_markers import generate_marker  # noqa: E402
from generate_ground_texture import generate_ground_model  # noqa: E402

# ---------------------------------------------------------------------------
# 맵 사양 (대회 규정집 기준) — generate_competition_map.py와 완전히 동일
# ---------------------------------------------------------------------------
# 지침서 v03 §3: 경기장 24m × 15m, safety area 32m × 23m.
# 이전 판(30×20)에서 줄어들었다. 경기장이 작아지면 최대 관측거리가 26.3m →
# 20.4m로 줄고, 점 개수는 거리²에 반비례하므로 표적당 점이 약 1.7배 늘어난다.
# 센서 한 대가 얻는 점이 중앙값 6개(24%는 2~3개)뿐이던 문제가 그만큼 완화된다.
AREA_X = 24.0
AREA_Y = 15.0
# safety area — 이탈하면 미션 중단(최종 0점)이므로 채점·ROI 판단의 바깥 경계다.
SAFETY_X = 32.0
SAFETY_Y = 23.0
LINE_WIDTH = 0.10       # 구획선 폭 10cm (규정)

# 바닥 적층 순서 (z-fighting 방지). 아래에서 위로:
#   ground_plane(0) < 잔디(GRASS_TOP_Z) < 구획선(LINE_Z) < 마커(MARKER_Z)
# 각 층은 자기 두께만큼 공간을 차지하므로 간격을 두께보다 크게 잡는다.
GRASS_THICKNESS = 0.002
GRASS_TOP_Z = 0.001                     # 잔디 상단면 높이
GRASS_MARGIN = 6.0                      # 경기장 밖으로 잔디를 더 깔 여유 (m)
LINE_THICKNESS = 0.002
LINE_Z = GRASS_TOP_Z + LINE_THICKNESS   # 구획선 중심 높이 → 상단 0.005
MARKER_Z = LINE_Z + 0.002               # 마커는 구획선 위

# 격자 한 칸 3m × 3m → 24/3 = 8칸, 15/3 = 5칸. 이전 판은 30/7 = 4.286m라
# 격자 좌표가 소수였는데, 3m로 떨어지면서 격자선이 전부 정수가 된다.
N_CELLS_X = 8
N_CELLS_Y = 5
CELL_X = AREA_X / N_CELLS_X
CELL_Y = AREA_Y / N_CELLS_Y

x_positions = [round(i * CELL_X, 4) for i in range(N_CELLS_X + 1)]
y_positions = [round(j * CELL_Y, 4) for j in range(N_CELLS_Y + 1)]
all_intersections = [(x, y) for x in x_positions for y in y_positions]

interior_x = x_positions[1:-1]
interior_y = y_positions[1:-1]
interior_intersections = [(x, y) for x in interior_x for y in interior_y]

# 24×15 격자 교점. 이전 판(30×20)의 배치 비율을 그대로 옮기고 3m 격자에
# 맞춰 정수로 떨어뜨렸다 — 남서/북서/남동에 셋, 중앙에 하나.
#   (4.2857,4)→(3,3)  (4.2857,16)→(3,12)  (25.7143,4)→(21,3)  (12.8571,12)→(9,9)
marker_coords = [
    (3.0, 3.0),     # id1
    (3.0, 12.0),    # id2
    (21.0, 3.0),    # id3
    (9.0, 9.0),     # id4
]
markers = [(i, x, y) for i, (x, y) in enumerate(marker_coords, start=1)]

# ---------------------------------------------------------------------------
# 지상 고정 LiDAR 2대 — OS1-128 (Standard Range), 남북(ns) 배치
# ---------------------------------------------------------------------------
# XY: docs/DESIGN.md §1의 ns 배치 좌표 그대로 — (15,-1.5) 남 + (15,21.5) 북,
# 필드 X중앙(15)에 맞춰 마주보게 배치. 이 배치가 검출밀도 이론상 최고(최소점
# 9.0, diag는 5.1)였던 근거는 "최대거리가 짧은 축(Y,20m) 기준 26.2m로 diag의
# 38.1m보다 훨씬 짧다"는 것 — 거리가 짧을수록 점 밀도가 높아짐(점수∝1/거리²).
#
# 설계 전제: 이 배치는 "호버링 구간(z≈2m)만" 본다. 전체 루트(이착륙 포함) 추적은
# 포기 — OS1의 수직시야각(±21.2°)이 OS0-SR(±45.4°)의 절반이라 지면부터 호버까지
# 한 배치로 커버하려면 기존 설계 근거가 그대로 재사용 안 됨(docs/DESIGN.md 참고).
#
# 높이 h=2.0m 선택 근거: 목표고도(2.0m)와 센서높이를 맞춰 어느 배치·거리에서든
# 각도 = atan2(2.0-h, d) = 0°가 되게 하는 원리는 diag와 동일. 다만 ns는 가장
# 가까운 지점이 필드 경계 바로 앞(남쪽 센서 (15,-1.5)에서 (15,0)까지 수평거리
# 1.5m — diag의 코너 2.12m보다 더 가까움)이라, 그 지점에서의 고도 변동 허용치가
# ±21.2°×tan → 1.5m×tan(21.2°) ≈ **±0.58m** (diag는 ±0.82m였음). 대회 기준
# 수직오차 허용치(0.5m)보다는 여전히 넉넉하지만, diag보다 여유가 줄어든 트레이드
# 오프임 — ns가 커버리지/밀도는 이론상 낫지만 근접점 고도마진은 더 빡빡함.
# pitch=0(수평)은 여전히 최적: 목표고도와 센서높이가 같아서 틀 필요 없음.
#
# topic은 OS0-SR 버전과 동일하게 /lidar1, /lidar2 (한 번에 하나의 world만 로드되므로
# 토픽명이 겹쳐도 문제없음 — 어느 쪽을 PX4_GZ_WORLD로 띄우느냐로 결정됨).
# 센서 변종별 사거리 (Ouster 데이터시트, 10% 반사율 기준이 실효 한계).
# 수직 FOV·채널·각분해능·정밀도는 두 변종이 동일하므로 배치 계산은 그대로다.
#   OS1     80% 170m / 10% 90m, 최소거리 0m
#   OS1-SR  80%  85m / 10% 45m, 최소거리 0.3m
LIDAR_VARIANTS = {
    "OS1":    {"min": 0.1, "max": 170.0},
    "OS1-SR": {"min": 0.3, "max":  85.0},
    # OS0 64ch (Ouster 데이터시트): 80% 35m / 10% 15m, 최소거리 0.3m, 정확도 ±0.8cm.
    # gpu_lidar엔 반사율 모델이 없어 max에서 딱 끊기므로 80% 기준 35m로 녹화하고,
    # 어두운 기체(10%, 15m) 조건은 재생 때 거리 필터로 만든다(녹화는 한 번).
    "OS0":    {"min": 0.3, "max":  35.0},
}

# 두 센서를 다른 주기로 돌려 위상을 드리프트시키는 실험은 되돌렸다 —
# "주기가 바뀌면 의미가 없다"(같은 10Hz에서 위상만 봐야 한다). 고정
# 위상차 실험은 generate_delayed_lidar.py + spawn_delayed_lidar.py로
# 대신한다: lidar2를 이 월드에서 완전히 빼고 별도 모델로 뽑은 뒤,
# 시뮬레이션이 돈 뒤 정해진 시각에 스폰해 위상을 고정한다.

# 남쪽 센서를 SR로 둔 이유: 마커 4개 중 3개가 필드 남쪽 절반(y=4~12)에 있어
# 북쪽 센서가 구조적으로 더 먼 거리를 담당한다. 실측(run040) 기준 최대
# 관측거리가 남 20.6m / 북 26.3m로, 사거리 여유가 적은 SR은 가까운 쪽에
# 두는 편이 안전하다(10% 반사 45m 대비 여유 2.2배 vs 1.7배).
# 24×15로 줄면서 X중앙이 15 → 12. Y는 실외 설치와 같게 남 -2.0 / 북 17.0(필드 경계에서 각 2 m).
# 두 센서 모두 코너까지 12.1~20.4m (이전 판 최대 26.3m) — 최장거리가 5.9m
# 짧아져 표적당 점이 약 1.7배 늘어난다.
LIDARS = [
    # (name, topic, x, y, z, roll, pitch, yaw, variant)
    ("ground_lidar_1", "/lidar1", 12.0, -2.0, 2.0, 0.0, 0.0,  1.5708, "OS1-SR"),  # 남쪽, 필드중앙X, 북향
    ("ground_lidar_2", "/lidar2", 12.0, 17.0, 2.0, 0.0, 0.0, -1.5708, "OS1"),     # 북쪽, 필드중앙X, 남향
]

# GUI에 LiDAR 레이(파란 부채꼴)를 그릴지 여부.
# OS1-128은 1024 x 128 = 131,072 레이 x 2대 = 262,144개를 매 프레임 그려야 해서
# (OS0-SR 64ch의 2배) 켜두면 드론 스폰 직후 뷰포트 조작이 사실상 불가능해진다.
# 레이를 꺼도 포인트클라우드 발행에는 전혀 영향이 없다 — 측위 동작은 동일하고,
# 점을 보고 싶으면 RViz2의 /lidar1,2/points 를 쓰면 된다.
# 센서 방향을 눈으로 확인해야 할 때만 잠깐 True로 바꿔 재생성할 것.
# 2026-08-18: True로 두고 비행했더니 스캔이 통째로 건너뛰었다. 스캔 간격
# 중앙값은 정확히 100ms(=10Hz)인데 평균 도착률은 6.0/5.1Hz — 주기가 느린 게
# 아니라 프레임이 빠지는 것이라, Gazebo가 레이 렌더링을 못 따라간 것으로 본다.
# 측위는 포인트클라우드만 쓰므로 꺼도 동작에 영향이 없다.
# 센서 방향을 눈으로 확인할 때만 잠깐 True로 바꿔 재생성할 것.
LIDAR_VISUALIZE = False

# 스캔 해상도. OS1-128의 실제 모드는 1024x10Hz / 2048x10Hz / 1024x20Hz 등이고,
# 실기체는 1024를 쓰므로 시뮬도 1024로 맞춘다.
# 1024x128 x 2대 = 262,144레이/프레임. 2026-08에는 드론이 뜨면 5.5~7Hz로 프레임이
# 빠져서(HEADLESS=1도 동일) 원인을 가르려고 한동안 512로 낮춰 썼다.
# 2026-09-28 1024 녹화(bags_1024 L1~L5)는 두 LiDAR 모두 10.00Hz로 손실이 없어
# 1024를 기본으로 되돌렸다. 프레임이 다시 빠지면 녹화 후 간격부터 확인할 것.
LIDAR_HSAMPLES = 1024
LIDAR_VSAMPLES = 128
LIDAR_VHALF = 0.37001      # 수직 FOV 절반 [rad] (±21.2°)
LIDAR_NOISE = 0.005        # gaussian stddev [m]
LIDAR_CHANNELS = 128
WORLD_NAME = "competition_map"

# 센서 프로필 선택. 기본(OS1)은 위 값 그대로 competition_map.sdf를 만든다.
#   LIDAR_PROFILE=OS0 python3 generate_competition_map.py
# 로 돌리면 두 센서를 OS0 64ch로 바꾼 competition_map_os0.sdf를 따로 만든다
# (OS1 world는 안 건드림). 배치(위치·자세)는 동일 — 센서 사양만 바뀐다.
# OS0는 1024x64x2 = 131,072레이/프레임으로 OS1(1024x128x2)의 절반 부하다.
# 빔 배치는 균일(90.8°/64ch). PX4는 PX4_GZ_WORLD 값을 파일명과 world 이름
# 둘 다로 쓰므로 world 이름도 파일명과 맞춘다(make_delayed_lidar2.py 참고) —
# 참값 토픽이 /world/competition_map_os0/dynamic_pose/info 로 바뀐다.
LIDAR_PROFILE = os.environ.get("LIDAR_PROFILE", "OS1").strip().upper()
if LIDAR_PROFILE == "OS0":
    LIDARS = [(*row[:8], "OS0") for row in LIDARS]
    LIDAR_HSAMPLES = 1024
    LIDAR_VSAMPLES = 64
    LIDAR_VHALF = 0.79238      # ±45.4° (수직 FOV 90.8°)
    LIDAR_NOISE = 0.008
    LIDAR_CHANNELS = 64
    WORLD_NAME = "competition_map_os0"
elif LIDAR_PROFILE != "OS1":
    raise SystemExit(f"LIDAR_PROFILE={LIDAR_PROFILE!r} — OS1, OS0만 지원")
SENSOR_LABEL = f"{'OS0' if LIDAR_PROFILE == 'OS0' else 'OS1'}-{LIDAR_CHANNELS}"

# ---------------------------------------------------------------------------
# 1) ArUco 마커 모델 생성 (PNG + model.config/model.sdf) — OS0-SR 버전과 좌표 동일,
#    같은 디렉토리를 재생성(덮어쓰기)하므로 두 world가 같은 마커 모델을 공유함.
# ---------------------------------------------------------------------------
marker_model_names = []
for marker_id, x, y in markers:
    name = generate_marker(MODELS_DIR, marker_id, x, y)
    marker_model_names.append(name)
    print(f"마커 생성: {name}  (x={x}, y={y})")

# 잔디 바닥 모델 (텍스처 PNG 포함) 생성
ground_model_name, _tex_px, _ground_span = generate_ground_model(
    MODELS_DIR, area_x=AREA_X, area_y=AREA_Y, margin=GRASS_MARGIN)
print(f"잔디 바닥 생성: {ground_model_name}  "
      f"({_ground_span[0]}x{_ground_span[1]}m, 텍스처 {_tex_px[0]}x{_tex_px[1]}px)")


# ---------------------------------------------------------------------------
# 2) SDF 조각 생성 함수들
# ---------------------------------------------------------------------------
def grid_line_models() -> str:
    """구획선: 폭 10cm 흰색. 잔디 위에서 확실히 보이도록 흰색을 쓰고,
    emissive를 약간 넣어 그림자 진 쪽에서도 회색으로 죽지 않게 한다."""
    line_material = '''
          <material>
            <ambient>0.92 0.92 0.92 1</ambient>
            <diffuse>1.0 1.0 1.0 1</diffuse>
            <specular>0.1 0.1 0.1 1</specular>
            <emissive>0.25 0.25 0.25 1</emissive>
          </material>'''
    parts = []
    idx = 0
    for x in x_positions:
        idx += 1
        parts.append(f'''
    <model name="gridline_v_{idx}">
      <static>true</static>
      <pose>{x} {AREA_Y / 2} {LINE_Z} 0 0 0</pose>
      <link name="link">
        <visual name="visual">
          <geometry>
            <box><size>{LINE_WIDTH} {AREA_Y} {LINE_THICKNESS}</size></box>
          </geometry>{line_material}
        </visual>
      </link>
    </model>''')
    for y in y_positions:
        idx += 1
        parts.append(f'''
    <model name="gridline_h_{idx}">
      <static>true</static>
      <pose>{AREA_X / 2} {y} {LINE_Z} 0 0 0</pose>
      <link name="link">
        <visual name="visual">
          <geometry>
            <box><size>{AREA_X} {LINE_WIDTH} {LINE_THICKNESS}</size></box>
          </geometry>{line_material}
        </visual>
      </link>
    </model>''')
    return "".join(parts)


def marker_includes() -> str:
    parts = []
    for name, (marker_id, x, y) in zip(marker_model_names, markers):
        parts.append(f'''
    <!-- 경로점 id={marker_id}: ({x}, {y}) -->
    <include>
      <uri>model://{name}</uri>
      <name>{name}</name>
      <pose>{x} {y} {MARKER_Z} 0 0 0</pose>
    </include>''')
    return "".join(parts)


def _lidar_model_block(name, topic, x, y, z, roll, pitch, yaw, variant) -> str:
    """센서 한 대 분량의 <model> 블록.

    지지대 기둥(바닥→센서) + 청색 하우징(OS0-SR의 주황과 구분) + 흰 캡
    + gpu_lidar 센서. 센서 스펙은 Ouster OS1 128ch 등가 (수평 360°/1024,
    수직 ±21.2°/128, 10Hz, gaussian noise 0.5cm). 사거리만 변종별로 다르며
    LIDAR_VARIANTS에서 가져온다.

    world에 정적으로 넣을 때와, 나중에 spawn_delayed_lidar.py로 실행 중인
    시뮬레이션에 끼워넣을 때(고정 위상차 실험) 둘 다 이 함수를 쓴다 — 센서
    스펙이 갈라지면 위상차 말고 다른 게 같이 달라져 실험이 오염된다.
    """
    rng = LIDAR_VARIANTS[variant]
    pole_len = z
    pole_cz = -z / 2.0
    return f'''
    <!-- 지상 고정 LiDAR({variant} {LIDAR_CHANNELS}ch): {name} @ world ({x}, {y}, {z}), yaw={yaw} rad, topic {topic}/points -->
    <model name="{name}">
      <static>true</static>
      <pose>{x} {y} {z} {roll} {pitch} {yaw}</pose>
      <link name="base">
        <visual name="pole">
          <pose>0 0 {pole_cz} 0 0 0</pose>
          <geometry>
            <cylinder><radius>0.04</radius><length>{pole_len}</length></cylinder>
          </geometry>
          <material>
            <ambient>0.15 0.15 0.15 1</ambient>
            <diffuse>0.25 0.25 0.25 1</diffuse>
          </material>
        </visual>
        <visual name="housing">
          <geometry>
            <cylinder><radius>0.09</radius><length>0.16</length></cylinder>
          </geometry>
          <material>
            <ambient>0.05 0.45 0.9 1</ambient>
            <diffuse>0.05 0.5 1.0 1</diffuse>
            <specular>0.3 0.3 0.3 1</specular>
            <emissive>0.0 0.15 0.3 1</emissive>
          </material>
        </visual>
        <visual name="cap">
          <pose>0 0 0.09 0 0 0</pose>
          <geometry>
            <cylinder><radius>0.07</radius><length>0.02</length></cylinder>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.85 1</ambient>
            <diffuse>0.9 0.9 0.95 1</diffuse>
          </material>
        </visual>
        <sensor name="{name}_lidar" type="gpu_lidar">
          <always_on>1</always_on>
          <update_rate>10</update_rate>
          <topic>{topic}</topic>
          <gz_frame_id>{name}_frame</gz_frame_id>
          <visualize>{'true' if LIDAR_VISUALIZE else 'false'}</visualize>
          <lidar>
            <scan>
              <horizontal>
                <samples>{LIDAR_HSAMPLES}</samples>
                <resolution>1</resolution>
                <min_angle>-3.14159265358979</min_angle>
                <max_angle>3.14159265358979</max_angle>
              </horizontal>
              <vertical>
                <samples>{LIDAR_VSAMPLES}</samples>
                <resolution>1</resolution>
                <min_angle>{-LIDAR_VHALF}</min_angle>
                <max_angle>{LIDAR_VHALF}</max_angle>
              </vertical>
            </scan>
            <range>
              <min>{rng['min']}</min>
              <max>{rng['max']}</max>
              <resolution>0.001</resolution>
            </range>
            <noise>
              <type>gaussian</type>
              <mean>0.0</mean>
              <stddev>{LIDAR_NOISE}</stddev>
            </noise>
          </lidar>
        </sensor>
      </link>
    </model>'''


# 고정 위상차 실험에서 world에 넣지 않고 나중에 스폰할 센서 이름.
# 비워두면(기본값) 기존과 동일하게 둘 다 정적으로 world에 들어간다.
DELAYED_LIDAR_NAMES: set[str] = set()


def lidar_models() -> str:
    """DELAYED_LIDAR_NAMES에 없는 센서만 world에 인라인으로 삽입."""
    parts = [
        _lidar_model_block(name, topic, x, y, z, roll, pitch, yaw, variant)
        for name, topic, x, y, z, roll, pitch, yaw, variant in LIDARS
        if name not in DELAYED_LIDAR_NAMES
    ]
    return "".join(parts)


def write_delayed_lidar_sdf(name: str, out_path: str) -> None:
    """DELAYED_LIDAR_NAMES에 든 센서 하나를 독립 스폰용 SDF 파일로 쓴다.

    spawn_delayed_lidar.py가 실행 중인 시뮬레이션에 이 파일을 끼워넣는다.
    world 파일과 정확히 같은 _lidar_model_block()을 쓰므로, 나중에 붙는
    센서라는 것 말고는 스펙이 하나도 안 갈린다.
    """
    entry = next((row for row in LIDARS if row[0] == name), None)
    if entry is None:
        raise ValueError(f'LIDARS에 {name!r}가 없습니다')
    block = _lidar_model_block(*entry)
    sdf = f'<sdf version="1.9">{block}\n</sdf>\n'
    with open(out_path, 'w') as f:
        f.write(sdf)
    print(f'지연 스폰용 SDF 생성: {out_path}  (모델 {name})')


def coordinate_comment_block() -> str:
    lines = ["    <!--", "      격자 교차점 전체 좌표 (X x Y, m), 원점(0,0)=이착륙 지점:"]
    for x in x_positions:
        row = ", ".join(f"({x},{y})" for y in y_positions)
        lines.append(f"        x={x}: {row}")
    lines.append("")
    lines.append("      마커(경로점) 배치 좌표:")
    for marker_id, x, y in markers:
        lines.append(f"        id={marker_id}: ({x}, {y})")
    lines.append("    -->")
    return "\n".join(lines)


# ---------------------------------------------------------------------------
# 3) world SDF 조립
# ---------------------------------------------------------------------------
world = f'''<?xml version="1.0" encoding="UTF-8"?>
<sdf version="1.9">
  <world name="{WORLD_NAME}">
{coordinate_comment_block()}

    <physics type="ode">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>250</real_time_update_rate>
    </physics>
    <gravity>0 0 -9.8</gravity>
    <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
    <atmosphere type="adiabatic"/>

    <scene>
      <grid>false</grid>
      <ambient>0.5 0.5 0.5 1</ambient>
      <!-- <sky>가 렌더링되면 background는 가려지지만, 스카이돔을 지원하지
           않는 렌더러/헤드리스에서는 이 하늘색이 그대로 배경이 된다. -->
      <background>0.53 0.72 0.92 1</background>
      <shadows>true</shadows>
      <sky>
        <clouds>
          <speed>2.5</speed>
          <direction>0.7</direction>
          <humidity>0.55</humidity>
          <mean_size>0.35</mean_size>
        </clouds>
      </sky>
    </scene>

    <!-- 바닥(먼 배경 + 충돌면): 잔디 텍스처 박스 바깥으로 이어지는 벌판.
         텍스처 없이 잔디색 단색이라 지평선까지 자연스럽게 이어진다. -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>200 200</size>
            </plane>
          </geometry>
          <surface>
            <friction><ode/></friction>
            <bounce/>
            <contact/>
          </surface>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>200 200</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.24 0.34 0.16 1</ambient>
            <diffuse>0.40 0.55 0.26 1</diffuse>
            <specular>0.02 0.02 0.02 1</specular>
          </material>
        </visual>
        <pose>0 0 0 0 -0 0</pose>
        <inertial>
          <pose>0 0 0 0 -0 0</pose>
          <mass>1</mass>
          <inertia>
            <ixx>1</ixx><ixy>0</ixy><ixz>0</ixz>
            <iyy>1</iyy><iyz>0</iyz><izz>1</izz>
          </inertia>
        </inertial>
        <enable_wind>false</enable_wind>
      </link>
      <pose>0 0 0 0 -0 0</pose>
      <self_collide>false</self_collide>
    </model>

    <!-- 잔디 바닥 (경기장 + 여유 {GRASS_MARGIN}m, 타일링 텍스처).
         상단면이 정확히 z={GRASS_TOP_Z}가 되도록 두께 절반만큼 내려서 배치. -->
    <include>
      <uri>model://{ground_model_name}</uri>
      <name>{ground_model_name}</name>
      <pose>{AREA_X / 2} {AREA_Y / 2} {GRASS_TOP_Z - GRASS_THICKNESS / 2} 0 0 0</pose>
    </include>

    <!-- 이착륙 지점(원점, world 좌표 0,0) 표시 패드.
         바닥이 잔디(녹색)라 기존 녹색 패드는 묻혀서 안 보이므로 흰색 계열로. -->
    <model name="home_pad">
      <static>true</static>
      <pose>0 0 {MARKER_Z} 0 0 0</pose>
      <link name="link">
        <visual name="visual">
          <geometry>
            <cylinder><radius>0.5</radius><length>0.001</length></cylinder>
          </geometry>
          <material>
            <ambient>0.90 0.90 0.88 1</ambient>
            <diffuse>0.98 0.98 0.96 1</diffuse>
            <emissive>0.22 0.22 0.20 1</emissive>
          </material>
        </visual>
      </link>
    </model>

    <!-- 격자 구획선: 폭 {LINE_WIDTH}m, X {N_CELLS_X}칸 x {CELL_X:.4f}m, Y {N_CELLS_Y}칸 x {CELL_Y:.4f}m -->
{grid_line_models()}

    <!-- 경로점 ArUco 마커 4개 (DICT_4X4_50, 0.5m x 0.5m), 경계에서 1셀 이상 안쪽 -->
{marker_includes()}

    <!-- 지상 고정 LiDAR 2대 ({SENSOR_LABEL}, 호버 구간 전용, 33x23 금지구역 밖) -->
{lidar_models()}

    <light name="sunUTC" type="directional">
      <pose>0 0 500 0 -0 0</pose>
      <cast_shadows>true</cast_shadows>
      <intensity>1</intensity>
      <direction>0.001 0.625 -0.78</direction>
      <diffuse>0.904 0.904 0.904 1</diffuse>
      <specular>0.271 0.271 0.271 1</specular>
      <attenuation>
        <range>2000</range>
        <linear>0</linear>
        <constant>1</constant>
        <quadratic>0</quadratic>
      </attenuation>
      <spot>
        <inner_angle>0</inner_angle>
        <outer_angle>0</outer_angle>
        <falloff>0</falloff>
      </spot>
    </light>

    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <world_frame_orientation>ENU</world_frame_orientation>
      <latitude_deg>47.397971057728974</latitude_deg>
      <longitude_deg>8.546163739800146</longitude_deg>
      <elevation>0</elevation>
    </spherical_coordinates>
  </world>
</sdf>
'''

out_sdf = os.path.join(WORLDS_DIR, f"{WORLD_NAME}.sdf")
with open(out_sdf, "w") as f:
    f.write(world)
print(f"\nworld 파일 생성: {out_sdf}")

# ---------------------------------------------------------------------------
# 4) 좌표 텍스트 파일 (라인트레이싱/정답 비교용)
# ---------------------------------------------------------------------------
out_txt = os.path.join(WORLDS_DIR, f"{WORLD_NAME}_coordinates.txt")
with open(out_txt, "w") as f:
    f.write("제24회 한국로봇항공기경연대회 중급부문 - competition_map 좌표 목록\n")
    f.write(f"임무구역: {AREA_X}m x {AREA_Y}m, 원점(0,0)=이착륙 지점\n")
    f.write(f"격자: X {N_CELLS_X}칸 x {CELL_X:.4f}m, Y {N_CELLS_Y}칸 x {CELL_Y:.4f}m, 구획선 폭 {LINE_WIDTH}m\n\n")

    f.write("=== 격자 교차점 전체 목록 (%d개) ===\n" % len(all_intersections))
    for x in x_positions:
        for y in y_positions:
            f.write(f"  ({x}, {y})\n")

    f.write("\n=== 마커 배치 가능 영역(경계 1셀 이상 안쪽, %d개) ===\n" % len(interior_intersections))
    for x in interior_x:
        for y in interior_y:
            f.write(f"  ({x}, {y})\n")

    f.write("\n=== 실제 배치된 ArUco 마커 4개 (정답, OS0-SR 버전과 동일) ===\n")
    for name, (marker_id, x, y) in zip(marker_model_names, markers):
        f.write(f"  id={marker_id}  (x={x}, y={y})  model={name}\n")

    f.write(f"\n=== 지상 고정 LiDAR 2대 ({SENSOR_LABEL}, 호버 구간 전용) ===\n")
    for name, topic, x, y, z, roll, pitch, yaw, variant in LIDARS:
        rng = LIDAR_VARIANTS[variant]
        f.write(f"  {name}  world=({x}, {y}, {z})  yaw={yaw} rad  "
                f"topic={topic}/points  {variant} "
                f"(range {rng['min']}~{rng['max']}m)\n")

print(f"좌표 파일 생성: {out_txt}")
print("\n완료.")
