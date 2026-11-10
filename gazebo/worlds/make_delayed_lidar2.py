#!/usr/bin/env python3
"""고정 위상차 실험용 world/모델 분리.

competition_map.sdf(정상 생성 결과, 두 라이다 다 정적으로 들어있음)에서
ground_lidar_2의 <model> 블록을 그대로 뽑아 독립 스폰용 파일로 만들고,
그 블록이 빠진 world 사본을 만든다. 센서 스펙을 다시 만들지 않고 원본
world에서 통째로 오려내므로, "나중에 스폰된다"는 것 말고는 스펙이 하나도
갈리지 않는다 — 새로 XML을 짜면서 생기는 오타/불일치 위험이 없다.

generate_competition_map.py를 먼저 돌려 competition_map.sdf가 최신 상태여야
한다(두 라이다 update_rate가 같은, 원래 버전이어야 함 — 드리프트 실험용
버전이면 안 된다).

출력:
  ground_lidar_2_delayed.sdf   spawn_delayed_lidar.py가 끼워넣을 독립 모델
  competition_map_delayed.sdf  ground_lidar_2가 빠진 world (PX4_GZ_WORLD=
                                competition_map_delayed 로 띄움)
"""
import os

WORLDS_DIR = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(WORLDS_DIR, 'competition_map.sdf')
MODEL_NAME = 'ground_lidar_2'

sdf = open(SRC).read()

model_tag = f'<model name="{MODEL_NAME}">'
if model_tag not in sdf:
    raise SystemExit(
        f'{SRC}에 {model_tag}가 없습니다. generate_competition_map.py를 '
        '먼저 돌려 world를 최신 상태로 만드세요.')

model_i = sdf.index(model_tag)
# _lidar_model_block()은 설명 코멘트 한 줄(<!-- 지상 고정 LiDAR... -->)을
# <model> 바로 앞에 같이 낸다. 코멘트만 남기면 이름이 grep에 걸리니, 그
# 줄의 시작까지 같이 잘라낸다 — 코멘트 시작은 <model 앞의 마지막 개행.
comment_start = sdf.rindex('\n', 0, sdf.rindex('<!--', 0, model_i)) + 1
i = comment_start
# 라이다 모델엔 하위 <model>이 없으므로 첫 </model>이 곧 닫는 태그다.
end_tag = '</model>'
j = sdf.index(end_tag, model_i) + len(end_tag)
block = sdf[model_i:j]  # 스폰용 파일엔 코멘트 없이 <model>부터만 담는다

if '<model name="' in block[len(f'<model name="{MODEL_NAME}">'):-len(end_tag)]:
    raise SystemExit(
        f'{MODEL_NAME} 블록 안에 중첩 <model>이 생겼습니다 — '
        'generate_competition_map.py 구조가 바뀐 것 같습니다. 이 스크립트의 '
        '가정(하위 model 없음)이 더 이상 안 맞으니 손으로 확인하세요.')

# --- 1) 독립 스폰용 SDF (코멘트 없이 <model>부터) ---
delayed_sdf = os.path.join(WORLDS_DIR, 'ground_lidar_2_delayed.sdf')
with open(delayed_sdf, 'w') as f:
    f.write(f'<sdf version="1.9">{block}\n</sdf>\n')
print(f'스폰용 모델 생성: {delayed_sdf}  ({len(block)} bytes)')

# --- 2) ground_lidar_2가 빠진 world 사본 (코멘트 줄부터 </model>까지 제거) ---
world_without = sdf[:i] + sdf[j:]

# PX4는 PX4_GZ_WORLD 값을 두 군데에 쓴다 — ① 로드할 파일명(worlds/<값>.sdf)
# ② _world_name으로 삼아 /world/<값>/clock, .../pose/info, .../imu 등 자기
# 클럭·센서 구독 경로를 직접 만든다(GZBridge.cpp). 내부 <world name>을
# 파일명과 다르게 두면 ②가 실제 gz 토픽(/world/competition_map/...)과
# 어긋나 PX4가 클럭을 못 받아 SITL이 통째로 멈춘다. 그래서 내부 이름도
# 파일명과 맞춰 바꾼다 — 참값 브리지(dynamic_pose/info)도 이 이름을 따라가야
# 한다는 뜻이니, 이 world로 실행할 땐 브리지 명령의 토픽도 같이 바꿀 것.
NEW_WORLD_NAME = 'competition_map_delayed'
old_world_tag = '<world name="competition_map">'
if old_world_tag not in world_without:
    raise SystemExit(f'{old_world_tag}를 못 찾았습니다 — world 태그가 바뀐 것 같습니다.')
world_without = world_without.replace(
    old_world_tag, f'<world name="{NEW_WORLD_NAME}">', 1)

delayed_world = os.path.join(WORLDS_DIR, 'competition_map_delayed.sdf')
with open(delayed_world, 'w') as f:
    f.write(world_without)
print(f'{MODEL_NAME} 뺀 world 생성: {delayed_world}')

# 확인 — 이름이 완전히 사라졌는지, 나머지(라이다1·world 새 이름)는 살아있는지.
assert MODEL_NAME not in world_without, '빼낸 world에 여전히 이름이 남아있습니다'
assert 'ground_lidar_1' in world_without, 'ground_lidar_1까지 같이 지워졌습니다'
assert f'<world name="{NEW_WORLD_NAME}">' in world_without, 'world 이름 교체가 안 됐습니다'
print('완료. 원본 world는 competition_map.sdf 그대로 안 건드렸습니다.')
print(f'PX4_GZ_WORLD={NEW_WORLD_NAME} 로 띄우고, 참값 브리지도 '
      f'/world/{NEW_WORLD_NAME}/dynamic_pose/info 로 구독할 것.')
