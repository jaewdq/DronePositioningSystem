#!/usr/bin/env python3
"""
제24회 한국로봇항공기경연대회 중급부문 "실내 조난자 탐색 임무" 맵 생성기.

생성물:
  - worlds/competition_map.sdf              실행용 Gazebo world
  - worlds/competition_map_coordinates.txt  격자 교차점 전체 좌표 + 마커 4개 좌표
  - models/aruco_marker_id{N}_at_x{X}_y{Y}/ 마커 모델 4개 (generate_aruco_markers.py 호출)

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

# ---------------------------------------------------------------------------
# 맵 사양 (대회 규정집 기준)
# ---------------------------------------------------------------------------
AREA_X = 30.0          # 임무구역 가로 (m)
AREA_Y = 20.0          # 임무구역 세로 (m)
LINE_WIDTH = 0.10       # 구획선 폭 10cm
LINE_THICKNESS = 0.001  # 구획선 두께(z 방향, 바닥에서 살짝 띄움)
MARKER_Z = 0.002        # 마커 높이 (바닥 0 < 구획선 0.001 < 마커 0.002, z-fighting 방지)

# 30m / 4m = 7.5, 20m / 4m = 5 → 정확히 4m로는 X축이 안 나눠떨어진다.
# 후보: 7칸(4.286m, |오차| 0.286m) vs 8칸(3.75m, |오차| 0.25m).
# 8칸이 4m에 더 가깝지만 7칸과 거의 동률이고, 7칸 쪽이 교차점 수가 적당해
# "TBR 4개 경로점"을 배치하기에 더 자연스러워 7칸(가로) x 5칸(세로)을 채택.
N_CELLS_X = 7
N_CELLS_Y = 5
CELL_X = AREA_X / N_CELLS_X   # 4.2857m
CELL_Y = AREA_Y / N_CELLS_Y   # 4.0m (정확히 4m)

x_positions = [round(i * CELL_X, 4) for i in range(N_CELLS_X + 1)]   # 8개
y_positions = [round(j * CELL_Y, 4) for j in range(N_CELLS_Y + 1)]   # 6개
all_intersections = [(x, y) for x in x_positions for y in y_positions]

# "임무구역 경계에서 최소 1격자 셀 이상 안쪽" 제약:
# 바깥쪽 테두리 인덱스(0번째, 마지막번째)를 제외한 내부 교차점만 후보로 삼는다.
interior_x = x_positions[1:-1]   # 6개
interior_y = y_positions[1:-1]   # 4개
interior_intersections = [(x, y) for x in interior_x for y in interior_y]  # 24개

# 처음엔 random.sample(interior_intersections, 4)로 랜덤 배치했으나, 2->1->0->3 식
# 0-인덱스 방문순서가 헷갈려서 마커 번호를 방문순서 그대로 1->2->3->4로 바꾸고,
# 그 과정에서 id4(옛 id3)가 id2->id3(옛 id1->id0) 구간 코너와 겹쳐 재배치했다.
# generate_groundtruth_trajectory.py의 MARKERS와 항상 동일하게 유지할 것.
marker_coords = [
    (4.2857, 4.0),    # id1
    (4.2857, 16.0),   # id2
    (25.7143, 4.0),   # id3
    (12.8571, 12.0),  # id4 (재배치됨)
]
markers = [(i, x, y) for i, (x, y) in enumerate(marker_coords, start=1)]

# ---------------------------------------------------------------------------
# 지상 고정 LiDAR 2대 (측위 시스템용, outside-in)
# ---------------------------------------------------------------------------
# 임무구역은 world 좌표 X:[0,30] Y:[0,20], 중심 (15,10).
# 안전여유 33x23 금지구역 = |x-15|<16.5 AND |y-10|<11.5
#   → 금지: x in (-1.5, 31.5) AND y in (-1.5, 21.5). 이 밖에 설치해야 함.
# 두 대를 "직교 방향"(동쪽+북쪽)으로 배치해 삼각측량 정확도를 높이고
# 가림(occlusion)에 강건하게 한다. gpu_lidar 센서의 3D 클라우드는
# <topic>/points 로 나가므로 topic을 /ouster1, /ouster2 로 준다.
# generate_groundtruth... 가 아니라 drone_localization 의 YAML(lidar1_*/lidar2_*)
# 과 이 좌표를 항상 동일하게 유지할 것.
# 배치 근거(커버리지 계산): 필드 대각선 36m > 사거리 35m 라 한 대로는 반대
# 코너를 못 본다. 두 대를 대각 코너(금지구역 경계)에 두면 union 100% 커버,
# 그 중 95.5%는 두 대가 동시에 봐서 삼각측량 품질이 최고.
# 높이 1.5m: LiDAR~landpad 수평거리 2.12m 이므로 바닥(z=0)을 ±45°로 보려면
# h<=2.12 여야 한다. 1.5m면 패드에 앉은 드론(z=0)도 하향각 35°로 여유있게 보이고
# (여유 ~20°) 커버리지 100% 유지 → "바닥 시작점부터 위치추정" 조건 만족.
# 드론(2m)을 살짝 아래서 위로 봐서 항상 하늘 배경(분리 깔끔). 고도변화 민감도는
# 거리에만 의존해 2.0m와 동일. 코앞 위쪽 천장은 3.5m(미션 2m엔 충분).
# yaw는 360° 스캔이라 커버리지에 무영향(SDF↔YAML만 일치하면 됨) — 시각적으로
# 필드 중심(15,10)을 향하게만 둔다.
LIDARS = [
    # (name, topic, x, y, z, roll, pitch, yaw)
    ("ground_lidar_1", "/ouster1", -1.5, -1.5, 1.5, 0.0, 0.0, 0.6094),   # landpad 코너(남서), 필드향
    ("ground_lidar_2", "/ouster2", 31.5, 21.5, 1.5, 0.0, 0.0, -2.5322),  # 대각(북동), 필드향
]

# ---------------------------------------------------------------------------
# 1) ArUco 마커 모델 생성 (PNG + model.config/model.sdf)
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
    for x in x_positions:  # 세로선 (X 고정, Y 전체를 가로지름)
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
    for y in y_positions:  # 가로선 (Y 고정, X 전체를 가로지름)
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

    각 모델: 지지대 기둥(바닥→센서) + 주황 하우징 + 흰 캡 + gpu_lidar 센서.
    센서 스펙은 Ouster OS0-SR 64ch 등가 (수평 360°/1024, 수직 ±45.4°/64,
    range 0.3~35m, 10Hz, gaussian noise 0.8cm).
    """
    parts = []
    for name, topic, x, y, z, roll, pitch, yaw in LIDARS:
        pole_len = z                    # 바닥(0)부터 센서 높이(z)까지
        pole_cz = -z / 2.0              # 모델 원점 기준 기둥 중심
        parts.append(f'''
    <!-- 지상 고정 LiDAR: {name} @ world ({x}, {y}, {z}), yaw={yaw} rad, topic {topic}/points -->
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
            <ambient>0.9 0.45 0.05 1</ambient>
            <diffuse>1.0 0.5 0.05 1</diffuse>
            <specular>0.3 0.3 0.3 1</specular>
            <emissive>0.3 0.15 0.0 1</emissive>
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
                <samples>64</samples>
                <resolution>1</resolution>
                <min_angle>-0.79234</min_angle>
                <max_angle>0.79234</max_angle>
              </vertical>
            </scan>
            <range>
              <min>0.3</min>
              <max>35.0</max>
              <resolution>0.001</resolution>
            </range>
            <noise>
              <type>gaussian</type>
              <mean>0.0</mean>
              <stddev>0.008</stddev>
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
  <world name="competition_map">
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

    <!-- 지상 고정 LiDAR 2대 (측위용, 33x23 금지구역 밖, 동쪽+북쪽 직교 배치) -->
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

out_sdf = os.path.join(WORLDS_DIR, "competition_map.sdf")
with open(out_sdf, "w") as f:
    f.write(world)
print(f"\nworld 파일 생성: {out_sdf}")

# ---------------------------------------------------------------------------
# 4) 좌표 텍스트 파일 (라인트레이싱/정답 비교용)
# ---------------------------------------------------------------------------
out_txt = os.path.join(WORLDS_DIR, "competition_map_coordinates.txt")
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

    f.write("\n=== 실제 배치된 ArUco 마커 4개 (정답) ===\n")
    for name, (marker_id, x, y) in zip(marker_model_names, markers):
        f.write(f"  id={marker_id}  (x={x}, y={y})  model={name}\n")

    f.write("\n=== 지상 고정 LiDAR 2대 (측위 시스템, 금지구역 33x23 밖) ===\n")
    for name, topic, x, y, z, roll, pitch, yaw in LIDARS:
        f.write(f"  {name}  world=({x}, {y}, {z})  yaw={yaw} rad  "
                f"topic={topic}/points\n")

print(f"좌표 파일 생성: {out_txt}")
print("\n완료.")
