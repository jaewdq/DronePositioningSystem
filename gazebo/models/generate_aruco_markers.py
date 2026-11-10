#!/usr/bin/env python3
"""
ArUco 마커 PNG + Gazebo 모델(model.config/model.sdf)을 생성하는 스크립트.

사전: cv2.aruco.DICT_4X4_50 (4x4 비트, 사전 내 ID 0~49 사용 가능)

== 실행 환경 안내 ==
이 시스템은 기본 python3의 numpy가 2.x인데 opencv-python은 numpy 1.x로 빌드되어
바로 `import cv2`가 실패할 수 있다 (ABI 불일치). 격리된 환경에서 실행할 것:

    python3 -m pip install --target /tmp/arucopkgs "numpy<2" "opencv-contrib-python-headless"
    PYTHONPATH=/tmp/arucopkgs python3 generate_aruco_markers.py

== 사용법 ==
이 파일을 직접 실행하면 MARKERS 리스트에 정의된 (id, x, y)로 마커를 생성한다.
다른 스크립트에서 generate_marker(models_dir, marker_id, x, y) 함수를 import해서
재사용할 수도 있다 (generate_competition_map.py 가 이렇게 사용함).

== 생성 결과물 ==
각 마커마다 아래 디렉토리가 생성된다 (Gazebo 모델 표준 구조):

    models/aruco_marker_id{ID}_at_x{X}_y{Y}/
        model.config      # 모델 메타데이터
        model.sdf         # 0.5m x 0.5m 평면 + PBR 텍스처 머티리얼
        marker.png        # 실제 ArUco 마커 이미지 (흰 여백 포함)

model.sdf 안에서 텍스처는 다음과 같이 연결된다:

    <material>
      <pbr>
        <metal>
          <albedo_map>model://aruco_marker_id{ID}_at_x{X}_y{Y}/marker.png</albedo_map>
        </metal>
      </pbr>
    </material>

`model://` 접두사는 GZ_SIM_RESOURCE_PATH 환경변수에 등록된 디렉토리를 기준으로
풀린다. PX4 SITL은 Tools/simulation/gz/models 를 자동으로 resource path에
추가하므로, 이 스크립트가 만든 디렉토리를 그대로 그 안에 두면 별도 설정 없이
인식된다.
"""

import glob
import os
import shutil

# cv2는 마커 PNG를 **처음 만들 때만** 필요하다. 패턴은 marker_id에만 의존하고
# 좌표와 무관하므로, 이미 만들어둔 같은 ID의 PNG가 있으면 복사해서 쓴다.
# 맵 좌표가 바뀌면 디렉토리 이름은 달라지지만 이미지는 그대로다.
# (여기서 cv2를 무조건 import하면, cv2가 numpy와 ABI가 안 맞는 환경에서
#  맵 생성 자체가 막힌다 — 실제로 numpy 2.2.6 + 구 opencv에서 그랬다.)
try:
    import cv2
    _CV2_ERR = None
except Exception as e:                              # noqa: BLE001
    cv2 = None
    _CV2_ERR = e

ARUCO_DICTIONARY = 0 if cv2 is None else cv2.aruco.DICT_4X4_50
MARKER_SIZE_M = 0.5        # 실제 마커 한 변 길이 (50cm)
MARKER_PIXELS = 300        # 마커 패턴 자체의 텍스처 해상도
BORDER_PIXELS = 50         # 흰색 여백(quiet zone) 두께, 인식률을 위해 필요

# (marker_id, x, y) — generate_competition_map.py 가 실제 좌표로 덮어써서 호출함.
# 이 파일을 단독 실행할 때 쓰이는 기본 예시값.
MARKERS = [
    (1, 4.0, 4.0),
    (2, 9.0, 8.0),
    (3, 13.0, 12.0),
    (4, 21.0, 16.0),
]


def model_name(marker_id: int, x: float, y: float) -> str:
    return f"aruco_marker_id{marker_id}_at_x{round(x)}_y{round(y)}"


def _write_marker_png(models_dir: str, model_dir: str, marker_id: int) -> None:
    """마커 PNG를 만든다. 같은 ID의 기존 PNG가 있으면 복사, 없으면 cv2로 생성.

    좌표만 바뀐 맵 갱신에서는 복사 경로를 타므로 cv2가 필요 없다."""
    dest = os.path.join(model_dir, "marker.png")
    for src in sorted(glob.glob(os.path.join(
            models_dir, f"aruco_marker_id{marker_id}_at_*", "marker.png"))):
        if os.path.abspath(src) != os.path.abspath(dest):
            shutil.copyfile(src, dest)
            return

    if cv2 is None:
        raise RuntimeError(
            f"id{marker_id}의 기존 marker.png가 없고 cv2도 못 씁니다 ({_CV2_ERR}).\n"
            "  새 마커를 만들려면 opencv를 numpy와 맞는 버전으로 설치하세요:\n"
            "    pip3 install -U 'opencv-python>=4.10'")

    import numpy as np
    aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICTIONARY)
    marker_img = cv2.aruco.generateImageMarker(aruco_dict, marker_id, MARKER_PIXELS)
    total_px = MARKER_PIXELS + 2 * BORDER_PIXELS
    canvas = np.full((total_px, total_px), 255, dtype=np.uint8)
    canvas[BORDER_PIXELS:BORDER_PIXELS + MARKER_PIXELS,
           BORDER_PIXELS:BORDER_PIXELS + MARKER_PIXELS] = marker_img
    cv2.imwrite(dest, cv2.cvtColor(canvas, cv2.COLOR_GRAY2BGRA))


def generate_marker(models_dir: str, marker_id: int, x: float, y: float) -> str:
    """marker_id용 PNG + model.config + model.sdf를 만들고 모델 디렉토리명을 반환한다."""
    name = model_name(marker_id, x, y)
    model_dir = os.path.join(models_dir, name)
    os.makedirs(model_dir, exist_ok=True)

    _write_marker_png(models_dir, model_dir, marker_id)

    with open(os.path.join(model_dir, "model.config"), "w") as f:
        f.write(f"""<?xml version="1.0"?>
<model>
  <name>{name}</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <description>ArUco DICT_4X4_50 id={marker_id}, 0.5m x 0.5m, 경연장 좌표 ({x}, {y})</description>
</model>
""")

    with open(os.path.join(model_dir, "model.sdf"), "w") as f:
        f.write(f"""<?xml version="1.0" encoding="UTF-8"?>
<sdf version="1.9">
  <model name="{name}">
    <static>true</static>
    <pose>0 0 0.001 0 0 0</pose>
    <link name="base">
      <visual name="base_visual">
        <geometry>
          <plane>
            <normal>0 0 1</normal>
            <size>{MARKER_SIZE_M} {MARKER_SIZE_M}</size>
          </plane>
        </geometry>
        <material>
          <diffuse>1 1 1 1</diffuse>
          <specular>0.2 0.2 0.2 1</specular>
          <pbr>
            <metal>
              <albedo_map>model://{name}/marker.png</albedo_map>
            </metal>
          </pbr>
        </material>
      </visual>
    </link>
  </model>
</sdf>
""")

    return name


if __name__ == "__main__":
    models_dir = os.path.dirname(os.path.abspath(__file__))
    for marker_id, x, y in MARKERS:
        name = generate_marker(models_dir, marker_id, x, y)
        print(f"생성됨: {name}")
