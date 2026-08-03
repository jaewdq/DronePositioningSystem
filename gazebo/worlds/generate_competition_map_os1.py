#!/usr/bin/env python3
"""
제24회 한국로봇항공기경연대회 중급부문 "실내 조난자 탐색 임무" 맵 생성기 — OS1-128 버전.

generate_competition_map.py(OS0-SR, 원본, 그대로 둠)의 맵/마커 배치는 동일하게
유지하고, 지상 고정 LiDAR만 Ouster OS1(Standard Range) 128채널 + 남북(ns) 배치로
교체한 버전. OS1은 수직시야각이 OS0-SR의 절반(±21.2° vs ±45.4°)이라 배치 높이를
다시 계산함 (호버링(z≈2m) 구간만 보도록 설계 — 이착륙 구간은 이 배치로는 커버
안 함). XY는 docs/DESIGN.md §1에서 이론상 최고 검출밀도로 나온 ns 배치를 채택.

생성물:
  - worlds/competition_map_os1.sdf              실행용 Gazebo world
  - worlds/competition_map_os1_coordinates.txt  격자 교차점 전체 좌표 + 마커 4개 좌표
  - models/aruco_marker_id{N}_at_x{X}_y{Y}/     마커 모델 4개 (OS0-SR 버전과 공유, 동일 좌표라 재사용됨)

실행:
  python3 -m pip install --target /tmp/arucopkgs "numpy<2" "opencv-contrib-python-headless"
  PYTHONPATH=/tmp/arucopkgs python3 generate_competition_map_os1.py
"""

import os
import sys

WORLDS_DIR = os.path.dirname(os.path.abspath(__file__))
MODELS_DIR = os.path.join(os.path.dirname(WORLDS_DIR), "models")
sys.path.insert(0, MODELS_DIR)
from generate_aruco_markers import generate_marker  # noqa: E402

# ---------------------------------------------------------------------------
# 맵 사양 (대회 규정집 기준) — generate_competition_map.py와 완전히 동일
# ---------------------------------------------------------------------------
AREA_X = 30.0
AREA_Y = 20.0
LINE_WIDTH = 0.10
LINE_THICKNESS = 0.001
MARKER_Z = 0.002

N_CELLS_X = 7
N_CELLS_Y = 5
CELL_X = AREA_X / N_CELLS_X
CELL_Y = AREA_Y / N_CELLS_Y

x_positions = [round(i * CELL_X, 4) for i in range(N_CELLS_X + 1)]
y_positions = [round(j * CELL_Y, 4) for j in range(N_CELLS_Y + 1)]
all_intersections = [(x, y) for x in x_positions for y in y_positions]

interior_x = x_positions[1:-1]
interior_y = y_positions[1:-1]
interior_intersections = [(x, y) for x in interior_x for y in interior_y]

marker_coords = [
    (4.2857, 4.0),    # id1
    (4.2857, 16.0),   # id2
    (25.7143, 4.0),   # id3
    (12.8571, 12.0),  # id4
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
LIDARS = [
    # (name, topic, x, y, z, roll, pitch, yaw)
    ("ground_lidar_1", "/lidar1", 15.0, -1.5, 2.0, 0.0, 0.0,  1.5708),  # 남쪽, 필드중앙X, 북향
    ("ground_lidar_2", "/lidar2", 15.0, 21.5, 2.0, 0.0, 0.0, -1.5708),  # 북쪽, 필드중앙X, 남향
]

# ---------------------------------------------------------------------------
# 1) ArUco 마커 모델 생성 (PNG + model.config/model.sdf) — OS0-SR 버전과 좌표 동일,
#    같은 디렉토리를 재생성(덮어쓰기)하므로 두 world가 같은 마커 모델을 공유함.
# ---------------------------------------------------------------------------
marker_model_names = []
for marker_id, x, y in markers:
    name = generate_marker(MODELS_DIR, marker_id, x, y)
    marker_model_names.append(name)
    print(f"마커 생성: {name}  (x={x}, y={y})")


# ---------------------------------------------------------------------------
# 2) SDF 조각 생성 함수들
# ---------------------------------------------------------------------------
def grid_line_models() -> str:
    parts = []
    idx = 0
    for x in x_positions:
        idx += 1
        parts.append(f'''
    <model name="gridline_v_{idx}">
      <static>true</static>
      <pose>{x} {AREA_Y / 2} {LINE_THICKNESS} 0 0 0</pose>
      <link name="link">
        <visual name="visual">
          <geometry>
            <box><size>{LINE_WIDTH} {AREA_Y} {LINE_THICKNESS}</size></box>
          </geometry>
          <material>
            <ambient>0.02 0.02 0.02 1</ambient>
            <diffuse>0.02 0.02 0.02 1</diffuse>
            <specular>0.05 0.05 0.05 1</specular>
          </material>
        </visual>
      </link>
    </model>''')
    for y in y_positions:
        idx += 1
        parts.append(f'''
    <model name="gridline_h_{idx}">
      <static>true</static>
      <pose>{AREA_X / 2} {y} {LINE_THICKNESS} 0 0 0</pose>
      <link name="link">
        <visual name="visual">
          <geometry>
            <box><size>{AREA_X} {LINE_WIDTH} {LINE_THICKNESS}</size></box>
          </geometry>
          <material>
            <ambient>0.02 0.02 0.02 1</ambient>
            <diffuse>0.02 0.02 0.02 1</diffuse>
            <specular>0.05 0.05 0.05 1</specular>
          </material>
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


def lidar_models() -> str:
    """지상 고정 gpu_lidar 2대를 world에 인라인으로 삽입.

    각 모델: 지지대 기둥(바닥→센서) + 청색 하우징(OS0-SR의 주황과 구분) + 흰 캡
    + gpu_lidar 센서. 센서 스펙은 Ouster OS1(Standard Range) 128ch 등가
    (수평 360°/1024, 수직 ±21.2°/128, range 0.1~170m, 10Hz, gaussian noise 0.5cm).
    """
    parts = []
    for name, topic, x, y, z, roll, pitch, yaw in LIDARS:
        pole_len = z
        pole_cz = -z / 2.0
        parts.append(f'''
    <!-- 지상 고정 LiDAR(OS1-128): {name} @ world ({x}, {y}, {z}), yaw={yaw} rad, topic {topic}/points -->
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
          <visualize>true</visualize>
          <lidar>
            <scan>
              <horizontal>
                <samples>1024</samples>
                <resolution>1</resolution>
                <min_angle>-3.14159265358979</min_angle>
                <max_angle>3.14159265358979</max_angle>
              </horizontal>
              <vertical>
                <samples>128</samples>
                <resolution>1</resolution>
                <min_angle>-0.37001</min_angle>
                <max_angle>0.37001</max_angle>
              </vertical>
            </scan>
            <range>
              <min>0.1</min>
              <max>170.0</max>
              <resolution>0.001</resolution>
            </range>
            <noise>
              <type>gaussian</type>
              <mean>0.0</mean>
              <stddev>0.005</stddev>
            </noise>
          </lidar>
        </sensor>
      </link>
    </model>''')
    return "".join(parts)


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
  <world name="competition_map_os1">
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
      <ambient>0.4 0.4 0.4 1</ambient>
      <background>0.7 0.7 0.7 1</background>
      <shadows>true</shadows>
    </scene>

    <!-- 바닥: 임무구역과 대비되는 밝은 회색 바닥 -->
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
            <ambient>0.85 0.85 0.83 1</ambient>
            <diffuse>0.85 0.85 0.83 1</diffuse>
            <specular>0.2 0.2 0.2 1</specular>
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

    <!-- 이착륙 지점(원점, world 좌표 0,0) 표시 패드 -->
    <model name="home_pad">
      <static>true</static>
      <pose>0 0 0.0015 0 0 0</pose>
      <link name="link">
        <visual name="visual">
          <geometry>
            <cylinder><radius>0.5</radius><length>0.001</length></cylinder>
          </geometry>
          <material>
            <ambient>0.1 0.7 0.2 1</ambient>
            <diffuse>0.1 0.7 0.2 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- 격자 구획선: 폭 {LINE_WIDTH}m, X {N_CELLS_X}칸 x {CELL_X:.4f}m, Y {N_CELLS_Y}칸 x {CELL_Y:.4f}m -->
{grid_line_models()}

    <!-- 경로점 ArUco 마커 4개 (DICT_4X4_50, 0.5m x 0.5m), 경계에서 1셀 이상 안쪽 -->
{marker_includes()}

    <!-- 지상 고정 LiDAR 2대 (OS1-128, 호버 구간 전용, 33x23 금지구역 밖) -->
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

out_sdf = os.path.join(WORLDS_DIR, "competition_map_os1.sdf")
with open(out_sdf, "w") as f:
    f.write(world)
print(f"\nworld 파일 생성: {out_sdf}")

# ---------------------------------------------------------------------------
# 4) 좌표 텍스트 파일 (라인트레이싱/정답 비교용)
# ---------------------------------------------------------------------------
out_txt = os.path.join(WORLDS_DIR, "competition_map_os1_coordinates.txt")
with open(out_txt, "w") as f:
    f.write("제24회 한국로봇항공기경연대회 중급부문 - competition_map_os1 좌표 목록\n")
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

    f.write("\n=== 지상 고정 LiDAR 2대 (OS1-128, 호버 구간 전용) ===\n")
    for name, topic, x, y, z, roll, pitch, yaw in LIDARS:
        f.write(f"  {name}  world=({x}, {y}, {z})  yaw={yaw} rad  "
                f"topic={topic}/points\n")

print(f"좌표 파일 생성: {out_txt}")
print("\n완료.")
