#!/usr/bin/env python3
"""
잔디 바닥 텍스처 + Gazebo 모델(arena_ground) 생성 스크립트.

== 왜 텍스처를 생성하나 ==
단색 녹색 평면은 "풀밭"으로 안 보이고, Gazebo 기본 리소스의 잔디 텍스처는
fuel 캐시(~/.gz/fuel/.../baylands/3/...)에 있어 경로에 버전이 박히고 다른
PC에는 없을 수 있다. 그래서 마커(generate_aruco_markers.py)와 같은 방식으로
저장소 안에서 직접 만들어 쓴다 — 외부 다운로드 의존 없음.

== 타일링 ==
텍스처는 상하좌우로 이어 붙여도 이음매가 안 보이도록 만든다:
  • 저주파 색 얼룩은 FFT 저역통과로 생성 → 주기함수라 태생적으로 타일링됨
  • 풀잎 스트로크는 가장자리 근처에 그릴 때 반대편에도 같이 그림(wrap)

== 생성 결과물 ==
    models/arena_ground/
        model.config
        model.sdf      경기장 크기 평면 + PBR 알베도(grass.png)
        grass.png      타일링되는 잔디 텍스처

== 사용법 ==
    python3 generate_ground_texture.py
    # 또는 generate_competition_map_os1.py 가 import해서 호출
"""

import os

import numpy as np
from PIL import Image, ImageDraw, ImageFilter

# === 스케일 ===
# SDF의 단순 박스에는 UV 반복 설정이 없어 텍스처 1장이 바닥 전체에 한 번
# 늘어난다(타일링 아님). 따라서 모든 크기를 px가 아니라 "실제 몇 m"로 정하고
# 해상도를 곱해 px로 환산한다 — 바닥 크기가 바뀌어도 질감 크기가 유지된다.
PX_PER_M = 48            # 해상도 (1m당 px). 48 → 약 2cm/px
SEED = 7                 # 재현성

BLADE_LEN_M = (0.12, 0.30)   # 풀잎(포기) 길이 범위 (m)
BLADE_DENSITY = 42           # 1m²당 풀잎 스트로크 개수

# 잔디 색 팔레트 (RGB).
# 채도 높은 원색 녹색은 인공 잔디처럼 보인다. 실제 공원 잔디는 누런 기가
# 섞인 연한 황록색에 가까우므로 그쪽으로 맞춘다.
# 전체적으로 밝고 연두빛(녹색보다 노랑 쪽으로 치우친 황록)으로 잡는다.
GRASS_DARK = (96, 118, 62)     # 그늘진 잔디
GRASS_MID = (142, 168, 86)     # 기본 잔디
GRASS_LIGHT = (182, 202, 118)  # 볕 받은 잔디
GRASS_DRY = (196, 194, 124)    # 마른 풀

# 흙 팔레트 — 잔디가 벗겨져 드러난 맨땅.
# 붉은 기가 강하면 녹색 옆에서 분홍 반점처럼 튀므로 채도를 낮춘 갈색으로 쓴다.
DIRT_DARK = (104, 88, 68)
DIRT_LIGHT = (146, 129, 104)

# 전체 면적 중 흙이 드러나는 비율 (0~1). 값이 클수록 휑해진다.
DIRT_FRACTION = 0.32
# 흙 패치 하나의 대략적인 지름 (m). 사람이 밟고 다녀 닳은 자리 정도 크기.
DIRT_PATCH_M = 4.5
# 잔디 자체의 옅은 얼룩 크기 (m)
GRASS_PATCH_M = 2.5


def _noise(shape, span_m, rng, feature_m: float) -> np.ndarray:
    """FFT 저역통과로 만든 0~1 노이즈. 지정한 크기(feature_m)보다 잔 성분은 버린다.

    차단 주파수를 픽셀이 아니라 "1m당 주기 수"로 계산하므로, 가로세로 비율이
    달라도 얼룩이 한쪽으로 늘어나지 않는다(월드 기준 등방).

    이산 푸리에 성분은 모두 주기함수라, 저주파만 남기고 역변환하면 경계에서
    자동으로 연속이 된다 — 지금은 타일링하지 않지만 이어붙일 여지는 남겨둔다."""
    h, w = shape
    span_y, span_x = span_m

    spec = np.fft.fft2(rng.normal(size=shape))

    # 각 성분의 공간주파수를 cycles/m로 환산
    fy = np.fft.fftfreq(h) * h / span_y
    fx = np.fft.fftfreq(w) * w / span_x
    radius = np.hypot(*np.meshgrid(fy, fx, indexing='ij'))

    spec[radius > 1.0 / feature_m] = 0
    out = np.real(np.fft.ifft2(spec))

    out -= out.min()
    if out.max() > 0:
        out /= out.max()
    return out


def _blend(c0, c1, t):
    return tuple(int(a + (b - a) * t) for a, b in zip(c0, c1))


def generate_grass_png(path: str, span_x_m: float, span_y_m: float,
                       seed: int = SEED) -> tuple:
    """바닥 크기(m)에 맞춰 잔디+흙 텍스처를 만든다. (가로px, 세로px) 반환.

    텍스처는 바닥 박스 전체에 한 번 늘어나므로, 가로세로 비율을 박스와
    똑같이 맞춰야 풀잎이 한쪽으로 늘어나지 않는다."""
    rng = np.random.default_rng(seed)

    w = int(round(span_x_m * PX_PER_M))
    h = int(round(span_y_m * PX_PER_M))
    shape = (h, w)
    span = (span_y_m, span_x_m)

    # --- 1) 바탕: 잔디의 옅은 얼룩 (넓은 색조 변화 + 잔 결) ---
    # 바탕은 배경일 뿐이고 질감은 아래 풀잎이 만든다. 진폭이 크면 멀리서
    # 둥근 얼룩("곰팡이")으로 뭉쳐 보이므로 눌러서 평탄하게 유지한다.
    patch = _noise(shape, span, rng, GRASS_PATCH_M)
    fine = _noise(shape, span, rng, GRASS_PATCH_M / 4.0)
    mix = np.clip(patch * 0.45 + fine * 0.55, 0.0, 1.0)

    base = np.zeros((h, w, 3), dtype=np.float64)
    for i, (d, m, l) in enumerate(zip(GRASS_DARK, GRASS_MID, GRASS_LIGHT)):
        lo = d + (m - d) * np.clip(mix * 2.0, 0, 1)
        hi = m + (l - m) * np.clip(mix * 2.0 - 1.0, 0, 1)
        base[:, :, i] = np.where(mix < 0.5, lo, hi)
    base = base * 0.62 + np.array(GRASS_MID, dtype=np.float64) * 0.38

    # --- 2) 흙이 드러난 영역 ---
    # 분위수로 자르면 노이즈 분포와 무관하게 면적 비율(DIRT_FRACTION)을
    # 정확히 맞출 수 있다.
    dirt_noise = _noise(shape, span, rng, DIRT_PATCH_M)
    # 저주파만 쓰면 패치가 매끈한 타원이 되어 인위적이다. 고주파를 조금 섞어
    # 경계를 너덜너덜하게 만든다.
    dirt_noise = dirt_noise * 0.82 + _noise(shape, span, rng, 0.5) * 0.18
    thr = np.quantile(dirt_noise, 1.0 - DIRT_FRACTION)
    # 경계를 딱 자르면 도려낸 것처럼 보이므로 완만하게 넘어가게 한다.
    dirt_amt = np.clip((dirt_noise - thr) / 0.16, 0.0, 1.0)

    speckle = _noise(shape, span, rng, 0.3)
    dirt_rgb = np.zeros((h, w, 3), dtype=np.float64)
    for i, (dk, lt) in enumerate(zip(DIRT_DARK, DIRT_LIGHT)):
        dirt_rgb[:, :, i] = dk + (lt - dk) * speckle

    a = dirt_amt[:, :, None]
    base = (base * (1.0 - a) + dirt_rgb * a).astype(np.uint8)

    img = Image.fromarray(base, mode='RGB')
    draw = ImageDraw.Draw(img)

    # --- 3) 풀잎 스트로크 ---
    n_blades = int(span_x_m * span_y_m * BLADE_DENSITY)
    len_lo = max(2, int(BLADE_LEN_M[0] * PX_PER_M))
    len_hi = max(len_lo + 1, int(BLADE_LEN_M[1] * PX_PER_M))

    xs = rng.integers(0, w, n_blades)
    ys = rng.integers(0, h, n_blades)
    lens = rng.integers(len_lo, len_hi, n_blades)
    angles = rng.normal(-np.pi / 2, 0.45, n_blades)   # 대체로 위를 향하되 기운다
    shades = rng.random(n_blades)
    # 흙이 드러난 곳일수록 풀잎을 적게 그려 "듬성듬성"하게 만든다.
    keep_roll = rng.random(n_blades)
    tone = rng.random(n_blades)

    for x, y, ln, ang, sh, keep, tn in zip(
            xs, ys, lens, angles, shades, keep_roll, tone):
        if keep < dirt_amt[y, x] * 0.92:
            continue
        if sh < 0.06:
            color = _blend(GRASS_MID, GRASS_DRY, tn)
        elif sh < 0.55:
            color = _blend(GRASS_DARK, GRASS_MID, sh / 0.55)
        else:
            color = _blend(GRASS_MID, GRASS_LIGHT, (sh - 0.55) / 0.45)

        x0, y0 = float(x), float(y)
        draw.line([(x0, y0),
                   (x0 + float(np.cos(ang) * ln), y0 + float(np.sin(ang) * ln))],
                  fill=color, width=1)

    # 아주 약한 블러로 픽셀 튐만 눌러준다 (질감은 남김)
    img = img.filter(ImageFilter.GaussianBlur(0.3))
    img.save(path)
    return w, h


MODEL_CONFIG = """<?xml version="1.0"?>
<model>
  <name>arena_ground</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <description>
    잔디 텍스처를 입힌 경기장 바닥. generate_ground_texture.py 가 생성.
  </description>
</model>
"""


def _model_sdf(area_x: float, area_y: float, margin: float) -> str:
    """경기장 + 여유분을 덮는 얇은 박스. 평면(plane) 대신 박스를 쓰는 이유는
    UV가 면 단위로 정의되어 텍스처가 정상적으로 입혀지기 때문이다.

    바닥 충돌면은 world의 ground_plane이 담당하므로 여기는 시각 전용
    (collision 없음). z를 아주 살짝 올려 ground_plane과의 z-fighting을 피한다."""
    size_x = area_x + margin * 2
    size_y = area_y + margin * 2
    return f"""<?xml version="1.0" encoding="UTF-8"?>
<sdf version="1.9">
  <model name="arena_ground">
    <static>true</static>
    <link name="link">
      <visual name="visual">
        <geometry>
          <box><size>{size_x} {size_y} 0.002</size></box>
        </geometry>
        <material>
          <ambient>0.32 0.42 0.24 1</ambient>
          <diffuse>0.62 0.78 0.46 1</diffuse>
          <specular>0.02 0.02 0.02 1</specular>
          <pbr>
            <metal>
              <albedo_map>model://arena_ground/grass.png</albedo_map>
              <metalness>0.0</metalness>
              <roughness>0.95</roughness>
            </metal>
          </pbr>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""


def generate_ground_model(models_dir: str, area_x: float = 30.0,
                          area_y: float = 20.0, margin: float = 6.0) -> str:
    """arena_ground 모델 디렉토리를 생성하고 모델 이름을 반환."""
    name = 'arena_ground'
    out_dir = os.path.join(models_dir, name)
    os.makedirs(out_dir, exist_ok=True)

    span_x = area_x + margin * 2
    span_y = area_y + margin * 2
    w, h = generate_grass_png(os.path.join(out_dir, 'grass.png'), span_x, span_y)
    with open(os.path.join(out_dir, 'model.config'), 'w') as f:
        f.write(MODEL_CONFIG)
    with open(os.path.join(out_dir, 'model.sdf'), 'w') as f:
        f.write(_model_sdf(area_x, area_y, margin))
    return name, (w, h), (span_x, span_y)


if __name__ == '__main__':
    models_dir = os.path.dirname(os.path.abspath(__file__))
    n, (w, h), (sx, sy) = generate_ground_model(models_dir)
    print(f'잔디 바닥 모델 생성: {os.path.join(models_dir, n)}')
    print(f'  바닥 {sx}x{sy}m, 텍스처 {w}x{h}px ({PX_PER_M}px/m)')
