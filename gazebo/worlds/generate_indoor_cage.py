#!/usr/bin/env python3
"""
실내 그물 케이지 맵 생성기 — OS0-SR 64ch 2대, 삼각대 설치.

대회 맵(generate_competition_map.py)과 별개의 소형 실내 시험장이다.
  - 케이지(그물 벽) 5 x 4 m, 그 안 필드 3 x 2 m (사방 1 m 여유)
  - 원점(0,0) = 필드 남서 모서리(그물이 뚫린 칸 쪽) = 이륙 지점, 필드 북동 모서리가
    (3, 2). PX4가 world 원점에 기체를 스폰한다.
  - 필드 안 표식(격자·마커·이착륙 패드) 없음
  - 바닥: 30 cm 흰색/회색 타일 반복, 옆면: 그물(가는 줄 격자)
  - 추적 범위: 이착륙(z=0)부터 최대 3 m까지 전 구간

생성물:
  - worlds/indoor_cage.sdf              실행용 Gazebo world
  - worlds/indoor_cage_coordinates.txt  필드/케이지/LiDAR 좌표 (정답지)

실행:
  python3 generate_indoor_cage.py
  cp indoor_cage.sdf indoor_cage_coordinates.txt \
     ~/drone_project/PX4-Autopilot/Tools/simulation/gz/worlds/
  PX4_GZ_NO_FOLLOW=1 PX4_GZ_WORLD=indoor_cage make px4_sitl gz_x500_cf2
"""

import math
import os

WORLDS_DIR = os.path.dirname(os.path.abspath(__file__))

# LiDAR 설치 높이·배치. 기본은 긴 변 가운데 + 1.2 m (2026-10-27 확정, 아래 LIDAR_LAYOUTS).
# 비교 시험은 LIDAR_H=0.8, LIDAR_LAYOUT=short 처럼 넘겨 따로 world를 만든다
# (indoor_cage_short_h080). PX4는 PX4_GZ_WORLD를 파일명과 world 이름 둘 다로 쓴다.
DEFAULT_LAYOUT = "long"
LIDAR_H = float(os.environ.get("LIDAR_H", "1.2"))
LIDAR_LAYOUT = os.environ.get("LIDAR_LAYOUT", DEFAULT_LAYOUT)
WORLD_NAME = "indoor_cage"
if LIDAR_LAYOUT != DEFAULT_LAYOUT:
    WORLD_NAME += f"_{LIDAR_LAYOUT}"
if "LIDAR_H" in os.environ:
    WORLD_NAME += f"_h{round(LIDAR_H * 100):03d}"

# ---------------------------------------------------------------------------
# 공간 사양 — 원점이 필드 남서 모서리, 필드는 (0,0)~(FIELD_X, FIELD_Y).
# 케이지는 필드와 중심을 같이 하고 사방으로 1 m 크다.
# ---------------------------------------------------------------------------
FIELD_X = 3.0           # 필드(비행 구역) 긴 변
FIELD_Y = 2.0
CAGE_X = 5.0            # 그물 케이지 안쪽 치수
CAGE_Y = 6.0 if LIDAR_LAYOUT == "real" else 4.0   # real: 실기 센서(y=-1.34, 3.05)가 그물 안에 들도록 넓힌다
MID_X, MID_Y = FIELD_X / 2.0, FIELD_Y / 2.0          # 필드·케이지 중심 (1.5, 1.0)
CAGE_X0, CAGE_X1 = MID_X - CAGE_X / 2.0, MID_X + CAGE_X / 2.0   # -1 ~ 4
CAGE_Y0, CAGE_Y1 = MID_Y - CAGE_Y / 2.0, MID_Y + CAGE_Y / 2.0   # -1 ~ 3
CAGE_H = 3.5            # 그물 높이 (최대 비행고도 3 m + 여유)
MAX_ALT = 3.0           # 추적해야 하는 최대 고도

# 바닥 타일: 한 칸 30 cm, 흰색/회색 체커. 원점(이륙 지점)이 타일 모서리에 오게
# 격자를 원점에 맞추고, 케이지를 덮을 만큼 깐다. 색은 gpu_lidar에 영향이 없다.
TILE = 0.30
TILE_Z = 0.0005         # 회색 타일 윗면 (흰 바닥면 바로 위)

# 그물: 가는 줄을 ±45° 대각선 두 방향으로 엮은 마름모 눈 (실물 2 x 1 m 황마 그물
# 사진 기준, 200 cm 폭에 마름모 약 30개). gpu_lidar는 투과율이 없어 그물을 판으로
# 넣으면 벽이 되지만, 줄로 엮으면 빔 일부만 줄에 맞아 실제 그물처럼 성긴 점이 찍힌다.
# 눈 크기·줄 굵기는 실측 그물에 맞춰 바꿀 것.
NET_DIAMOND = 0.07      # 마름모 가로 폭 = 같은 방향 줄 사이 수평 간격 [m]
NET_STRING = 0.004      # 줄 굵기 [m]
# 그물을 걸지 않는 칸 (출입구 등): (면, 칸 번호). 긴 변은 기둥 4개 사이 3칸,
# 칸 번호는 x가 작은 쪽부터 0. 면은 "s"(y=-1) / "n"(y=+3).
NET_OPEN_BAYS = {("s", 0)}

# ---------------------------------------------------------------------------
# LiDAR — Ouster OS0-SR 64ch 2대, 긴 변 가운데에서 마주보기 (기본, 2026-10-27 확정)
# ---------------------------------------------------------------------------
# long (기본): 24x15 시스템과 같은 모양 — 긴 변 가운데에서 짧은 축을 가로질러 마주본다.
#   (1.5, -0.7) / (1.5, 2.7), 그물(y=-1, 3)에서 0.3 m 안쪽, 긴 변 기둥 사이 칸의 한가운데.
#   삼각대 다리 하나가 필드를 향하므로(yaw = 필드 방향, 다리 0번 = 모델 +x)
#   그물 쪽 두 다리는 0.4·cos60° = 0.2 m만 나온다 → 그물까지 0.1 m 여유.
#   필드 최대 거리 3.09 m, 이착륙 지점까지 1.66 / 3.09 m.
#   Tello급 실측(비행 1.0 m): 0.8/1.0/1.2 m 모두 완전 놓침 0%, 두 센서 동시 64~80%.
#
# short (보관): 짧은 변 가운데에서 긴 축을 가로질러 마주본다.
#   (-0.8, 1.0) / (3.8, 1.0). 필드 최대 거리 3.93 m. 시뮬 삼각대(발끝 반경 0.4 m)로는
#   짧은 변 가운데 기둥(0.2 m 뒤)과 다리가 겹친다. Tello급 실측: 완전 놓침 3~7%.
#
# 높이 h=1.2 m 근거 (수직 FOV ±45.4°, tan = 1.015, 주 비행 고도 1.0 m, 최대 2 m):
#   수평거리 d에서 보이는 고도 범위 = h ± 1.015·d
#   - 이착륙 지점(LiDAR1에서 d=1.66): 바닥부터 보인다. 반대편(d=3.09)도 바닥까지.
#   - 주 비행 고도 1.0 m는 필드 전체를 두 센서가 다 본다. 센서 높이를 비행 고도와
#     같게(1.0 m) 두면 기체를 정확히 옆에서 봐서 검출이 가장 나빴다.
#
# OS0는 360° 스캔이라 yaw는 커버리지와 무관하다 — yaml의 lidarN_yaw와만 맞추면 된다.
# 기본 배치 좌표는 params/lidar_localization_indoor.yaml의 lidar1_*/lidar2_*와
# **반드시 같아야 한다** (자동 동기화 없음. run_height_test.sh는 SDF에서 읽는다).
LIDAR_LAYOUTS = {
    "short": [
        # (name, topic, x, y, z, roll, pitch, yaw)
        ("ground_lidar_1", "/lidar1", -0.8, 1.0, LIDAR_H, 0.0, 0.0, 0.0),     # 서쪽, 동향
        ("ground_lidar_2", "/lidar2",  3.8, 1.0, LIDAR_H, 0.0, 0.0, 3.1416),  # 동쪽, 서향
    ],
    "long": [
        ("ground_lidar_1", "/lidar1", 1.5, -0.7, LIDAR_H, 0.0, 0.0, 1.5708),  # 남쪽, 북향
        ("ground_lidar_2", "/lidar2", 1.5,  2.7, LIDAR_H, 0.0, 0.0, -1.5708), # 북쪽, 남향
    ],
    # real: 2026-11-03 실기 캘리브레이션 위치 (lidar_localization_os0_indoor.yaml). 높이도 실측값.
    "real": [
        ("ground_lidar_1", "/lidar1", 1.354, -1.340, 1.264, 0.0, 0.0, 1.5708),
        ("ground_lidar_2", "/lidar2", 1.376,  3.054, 1.251, 0.0, 0.0, -1.5708),
    ],
}
LIDARS = LIDAR_LAYOUTS[LIDAR_LAYOUT]

# OS0 SR 64ch 사양 (데이터시트 REV7 SR, FW 3.1 — 실물도 64채널).
#   수직 FOV 90.8° (±45.4°), 1024 @ 10 Hz, 거리 정밀도 1σ 0.8 cm(근거리).
#   최소 거리: 설정 가능 0.3 m, **출고 기본값 0.5 m** — 실물 드라이버 기본값에 맞춘다.
#     이 배치에서 센서~필드 최단 거리는 0.7 m(긴 변)/0.8 m(짧은 변)라 영향이 없지만,
#     캘리브레이션 오차 때문에 ROI를 센서 쪽으로 넓히면 센서 0.5 m 안쪽은 원래 안 보인다.
#   최대 거리: 반사율 10%에서 15 m. gpu_lidar는 반사율이 없어 35 m로 둔다(케이지 안은 4 m 이내).
#   시뮬에 없는 것: 빔 퍼짐 0.35°(얇은 표적이 실물에서 더 잘 잡힘), 거리 편향 ±2.5 cm.
LIDAR_HSAMPLES = 1024
LIDAR_VSAMPLES = 64
LIDAR_VHALF = 0.79238      # ±45.4° (수직 FOV 90.8°)
LIDAR_NOISE = 0.008        # gaussian stddev [m]
LIDAR_RANGE = (0.5, 35.0)
LIDAR_VISUALIZE = False    # True면 레이 렌더링 부하로 프레임이 빠진다 (대회 맵 주석 참고)

# 삼각대 다리: 바닥에서 연직축과 이루는 각, 발 위치 반경
TRIPOD_SPLAY = math.radians(20.0)


# ---------------------------------------------------------------------------
# SDF 조각
# ---------------------------------------------------------------------------
def _tripod_legs(h: float) -> str:
    """센서 아래 삼각대 다리 3개. 센서 기준 좌표로 그린다(모델 pose = 센서)."""
    top = h - 0.10                      # 다리가 모이는 높이 (센서 하우징 아래)
    length = top / math.cos(TRIPOD_SPLAY)
    r_foot = top * math.tan(TRIPOD_SPLAY)
    parts = []
    for i in range(3):
        az = 2.0 * math.pi * i / 3.0
        cx = 0.5 * r_foot * math.cos(az)
        cy = 0.5 * r_foot * math.sin(az)
        cz = -0.10 - top / 2.0
        # 연직 원기둥을 기울여 아래 끝이 발(cos az, sin az) 쪽으로 가게 한다
        # (축 윗방향 = (-cos az, -sin az)·sin SPLAY + z·cos SPLAY)
        roll = TRIPOD_SPLAY * math.sin(az)
        pitch = -TRIPOD_SPLAY * math.cos(az)
        parts.append(f'''
        <visual name="leg_{i}">
          <pose>{cx:.4f} {cy:.4f} {cz:.4f} {roll:.5f} {pitch:.5f} 0</pose>
          <geometry>
            <cylinder><radius>0.012</radius><length>{length:.4f}</length></cylinder>
          </geometry>
          <material>
            <ambient>0.12 0.12 0.12 1</ambient>
            <diffuse>0.2 0.2 0.2 1</diffuse>
          </material>
        </visual>''')
    return "".join(parts)


def _lidar_model_block(name, topic, x, y, z, roll, pitch, yaw) -> str:
    """삼각대 + 주황 하우징(OS0) + gpu_lidar 한 대."""
    return f'''
    <!-- 지상 고정 LiDAR(OS0-SR {LIDAR_VSAMPLES}ch, 삼각대): {name} @ ({x}, {y}, {z}), yaw={yaw}, topic {topic}/points -->
    <model name="{name}">
      <static>true</static>
      <pose>{x} {y} {z} {roll} {pitch} {yaw}</pose>
      <link name="base">{_tripod_legs(z)}
        <visual name="housing">
          <geometry>
            <cylinder><radius>0.043</radius><length>0.08</length></cylinder>
          </geometry>
          <material>
            <ambient>0.9 0.4 0.05 1</ambient>
            <diffuse>1.0 0.45 0.05 1</diffuse>
            <specular>0.3 0.3 0.3 1</specular>
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
              <min>{LIDAR_RANGE[0]}</min>
              <max>{LIDAR_RANGE[1]}</max>
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


def lidar_models() -> str:
    return "".join(_lidar_model_block(*row) for row in LIDARS)


def cage_posts() -> str:
    """그물 케이지 기둥: 긴 변마다 4개(등간격), 짧은 변마다 3개(모서리 + 가운데).
    짧은 변 가운데 기둥은 LiDAR 삼각대 바로 뒤(0.2 m)에 선다.
    기둥·그물 점은 ROI(필드 밖)에서 잘린다."""
    long_xs = [CAGE_X0 + i * CAGE_X / 3.0 for i in range(4)]    # 긴 변 4개
    posts = [(x, y) for y in (CAGE_Y0, CAGE_Y1) for x in long_xs]
    posts += [(CAGE_X0, MID_Y), (CAGE_X1, MID_Y)]               # 짧은 변 가운데
    parts = []
    for i, (px, py) in enumerate(posts):
        parts.append(f'''
    <model name="cage_post_{i}">
      <static>true</static>
      <pose>{px} {py} {CAGE_H / 2.0} 0 0 0</pose>
      <link name="link">
        <visual name="visual">
          <geometry>
            <cylinder><radius>0.025</radius><length>{CAGE_H}</length></cylinder>
          </geometry>
          <material>
            <ambient>0.55 0.55 0.55 1</ambient>
            <diffuse>0.7 0.7 0.7 1</diffuse>
          </material>
        </visual>
      </link>
    </model>''')
    return "".join(parts)


def _diagonal_segments(width, height, sign):
    """면(로컬 x: -width/2~width/2, z: 0~height)을 가로지르는 45° 줄들.
    sign=+1은 x - z = c (오른쪽 위로), -1은 x + z = c (오른쪽 아래로).
    각 줄을 면 경계로 잘라 (중심 x, 중심 z, 길이)를 돌려준다."""
    x0, x1 = -width / 2.0, width / 2.0
    segs = []
    # c 범위: 면 모서리 네 개에서의 (x - sign*z) 값의 최소~최대
    corners = [x - sign * z for x in (x0, x1) for z in (0.0, height)]
    k = math.ceil(min(corners) / NET_DIAMOND)
    while k * NET_DIAMOND <= max(corners):
        c = k * NET_DIAMOND
        k += 1
        # 줄 위의 점: x = c + sign*z. z 구간을 x 경계로 자른다.
        za, zb = sorted(((x0 - c) * sign, (x1 - c) * sign))
        z_lo, z_hi = max(0.0, za), min(height, zb)
        if z_hi - z_lo < 1e-3:
            continue
        zc = (z_lo + z_hi) / 2.0
        segs.append((c + sign * zc, zc, (z_hi - z_lo) * math.sqrt(2.0)))
    return segs


def _net_side(tag, cx, cy, yaw, width) -> str:
    """그물 한 면: 폭 width, 높이 CAGE_H, 마름모 눈. 면 기준 좌표(가로=로컬 x)로
    그린 뒤 모델 pose의 yaw로 돌린다. 줄은 그림자를 끄고 충돌 없이 visual만 둔다."""
    s = NET_STRING
    visuals = []
    for sign, pitch in ((+1, -math.pi / 4.0), (-1, math.pi / 4.0)):
        for i, (lx, lz, length) in enumerate(_diagonal_segments(width, CAGE_H, sign)):
            visuals.append(
                f'<visual name="d{"p" if sign > 0 else "m"}{i}"><cast_shadows>false</cast_shadows>'
                f'<pose>{lx:.4f} 0 {lz:.4f} 0 {pitch:.5f} 0</pose>'
                f'<geometry><box><size>{length:.4f} {s} {s}</size></box></geometry>'
                f'<material><ambient>0.45 0.35 0.2 1</ambient><diffuse>0.6 0.48 0.3 1</diffuse></material>'
                f'</visual>')
    body = "\n        ".join(visuals)
    return f'''
    <model name="net_{tag}">
      <static>true</static>
      <pose>{cx} {cy} 0 0 0 {yaw}</pose>
      <link name="link">
        {body}
      </link>
    </model>'''


def cage_net() -> str:
    """옆면 그물 (지붕 없음). 긴 변은 기둥 사이 칸마다 따로 걸어
    NET_OPEN_BAYS에 든 칸은 비운다."""
    bay = CAGE_X / 3.0
    parts = []
    for side, y in (("s", CAGE_Y0), ("n", CAGE_Y1)):
        for i in range(3):
            if (side, i) in NET_OPEN_BAYS:
                continue
            parts.append(_net_side(f"{side}{i}", CAGE_X0 + (i + 0.5) * bay, y, 0.0, bay))
    parts.append(_net_side("w", CAGE_X0, MID_Y, 1.5708, CAGE_Y))
    parts.append(_net_side("e", CAGE_X1, MID_Y, 1.5708, CAGE_Y))
    return "".join(parts)


def floor_tiles() -> str:
    """흰 바닥면 위에 회색 타일만 깔아 체커를 만든다. 원점이 타일 모서리.
    케이지를 덮도록 바깥쪽 타일 한 줄씩 더 깐다."""
    i0, i1 = int(math.floor(CAGE_X0 / TILE)) - 1, int(math.ceil(CAGE_X1 / TILE)) + 1
    j0, j1 = int(math.floor(CAGE_Y0 / TILE)) - 1, int(math.ceil(CAGE_Y1 / TILE)) + 1
    visuals = []
    for i in range(i0, i1):
        for j in range(j0, j1):
            if (i + j) % 2 == 0:
                continue
            cx = (i + 0.5) * TILE
            cy = (j + 0.5) * TILE
            visuals.append(
                f'<visual name="t_{i}_{j}"><pose>{cx:.3f} {cy:.3f} {TILE_Z / 2.0} 0 0 0</pose>'
                f'<geometry><box><size>{TILE} {TILE} {TILE_Z}</size></box></geometry>'
                f'<material><ambient>0.45 0.45 0.45 1</ambient><diffuse>0.55 0.55 0.55 1</diffuse></material>'
                f'</visual>')
    body = "\n        ".join(visuals)
    return f'''
    <model name="floor_tiles">
      <static>true</static>
      <link name="link">
        {body}
      </link>
    </model>'''


def coordinate_comment_block() -> str:
    lines = ["    <!--",
             "      원점(0,0) = 필드 남서 모서리 = 이륙 지점",
             f"      필드  x 0~{FIELD_X}, y 0~{FIELD_Y}",
             f"      케이지 x {CAGE_X0}~{CAGE_X1}, y {CAGE_Y0}~{CAGE_Y1}  ({CAGE_X} x {CAGE_Y} m, 높이 {CAGE_H} m)"]
    for name, topic, x, y, z, *_ in LIDARS:
        lines.append(f"      {name} ({topic}): ({x}, {y}, {z})")
    lines.append("    -->")
    return "\n".join(lines)


# ---------------------------------------------------------------------------
# world SDF 조립
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
      <ambient>0.6 0.6 0.6 1</ambient>
      <background>0.85 0.85 0.85 1</background>
      <shadows>true</shadows>
    </scene>

    <!-- 실내 바닥 (흰색, 충돌면). 회색 타일은 floor_tiles가 위에 깐다 -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane><normal>0 0 1</normal><size>40 40</size></plane>
          </geometry>
          <surface>
            <friction><ode/></friction>
            <bounce/>
            <contact/>
          </surface>
        </collision>
        <visual name="visual">
          <geometry>
            <plane><normal>0 0 1</normal><size>40 40</size></plane>
          </geometry>
          <material>
            <ambient>0.85 0.85 0.85 1</ambient>
            <diffuse>0.95 0.95 0.95 1</diffuse>
            <specular>0.05 0.05 0.05 1</specular>
          </material>
        </visual>
      </link>
    </model>

    <!-- 바닥 타일 ({TILE} m 흰색/회색 체커, 원점이 타일 모서리) -->
{floor_tiles()}


    <!-- 그물 케이지 기둥 ({CAGE_X} x {CAGE_Y} m, 높이 {CAGE_H} m) -->
{cage_posts()}

    <!-- 옆면 그물 (마름모 눈 폭 {NET_DIAMOND} m, 줄 {NET_STRING} m) -->
{cage_net()}

    <!-- 지상 고정 LiDAR 2대 (OS0-SR {LIDAR_VSAMPLES}ch, 삼각대 {LIDAR_H} m) -->
{lidar_models()}

    <light name="ceiling_light" type="directional">
      <pose>0 0 10 0 0 0</pose>
      <cast_shadows>true</cast_shadows>
      <intensity>1</intensity>
      <direction>0.1 0.2 -1</direction>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
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
print(f"world 파일 생성: {out_sdf}")

out_txt = os.path.join(WORLDS_DIR, f"{WORLD_NAME}_coordinates.txt")
with open(out_txt, "w") as f:
    f.write("실내 그물 케이지 맵 좌표 목록\n")
    f.write("원점(0,0) = 필드 남서 모서리(그물이 뚫린 칸 쪽) = 이륙 지점\n\n")
    f.write(f"필드:   x 0~{FIELD_X}, y 0~{FIELD_Y} ({FIELD_X} x {FIELD_Y} m)\n")
    f.write(f"케이지: x {CAGE_X0}~{CAGE_X1}, y {CAGE_Y0}~{CAGE_Y1} "
            f"({CAGE_X} x {CAGE_Y} m, 높이 {CAGE_H} m)\n")
    f.write(f"최대 추적 고도: {MAX_ALT} m\n")
    f.write(f"바닥 타일: {TILE} m 체커 (원점이 타일 모서리)\n")
    f.write(f"그물: 마름모 눈 폭 {NET_DIAMOND} m, 줄 {NET_STRING} m, 옆면 4장\n")
    f.write("기둥: 긴 변마다 4개(등간격), 짧은 변마다 3개(모서리 + 가운데)\n")
    for side, i in sorted(NET_OPEN_BAYS):
        y = CAGE_Y0 if side == "s" else CAGE_Y1
        x0 = CAGE_X0 + i * CAGE_X / 3.0
        f.write(f"그물 없는 칸: y={y}, x {x0:.2f}~{x0 + CAGE_X / 3.0:.2f}\n")
    f.write("\n")
    f.write("=== 지상 LiDAR (OS0-SR 64ch, 삼각대) ===\n")
    for name, topic, x, y, z, roll, pitch, yaw in LIDARS:
        f.write(f"  {name} ({topic}/points): x={x}, y={y}, z={z}, "
                f"roll={roll}, pitch={pitch}, yaw={yaw}\n")
print(f"좌표 파일 생성: {out_txt}")
