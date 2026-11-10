#!/usr/bin/env python3
"""고정 위상차 실험 — lidar2를 시뮬레이션이 돈 뒤 정해진 시각에 끼워넣는다.

경위: 두 gpu_lidar를 처음부터 world에 넣어두면 둘 다 t=0부터 주기를 세서
완벽히 위상고정된다(update_rate만으로는 위상차를 못 만든다 — SDF에
"몇 ms 밀려서 찍어라"라는 필드가 없다). 대신 lidar2를 world에서 빼두고
(make_delayed_lidar2.py가 이미 만듦) 돌고 있는 시뮬레이션에 원하는 순간
끼워넣으면, 그 순간부터 자기 주기를 세기 시작하니 10Hz는 그대로 유지한 채
lidar1과 고정된 위상차가 생긴다.

정밀도의 한계: 물리 스텝이 4ms(max_step_size)라 "정확히 30ms"는 못 만든다
— 4ms의 배수(28ms, 32ms...) 근처로만 맞을 수 있다. 그래도 사람이 손으로
맞추는 것과는 비교가 안 되게 정밀하고, 지금까지 잰 지연(0.03~0.23초)보다
훨씬 작은 오차다.

시각 기준은 시뮬레이션 시계(/world/<world>/clock)다. 실제 스폰이 반영되는
시점은 create 서비스 요청이 gz의 다음 처리 틱에 걸리는 시각이라, 요청을
보낸 시각과 완전히 같지는 않다 — 그 오차도 물리 스텝 한 칸(4ms) 안팎이다.

주의 — "월드 시작 후 정확히 N초"가 아니라 "10Hz 주기(100ms) 안에서 N ms
위치"를 기다린다. 전자는 스크립트를 T1 직후 수백 ms 안에 못 띄우면(사람이
터미널을 바꿔 명령을 치는 동안 이미 지나가 버린다) 영영 못 맞춘다. 후자는
100ms마다 그 위상이 다시 돌아오므로 스크립트를 언제 띄우든 늦어도 100ms
안에 기회가 온다 — 그리고 lidar1이 0,100,200...ms에 찍는 한, lidar2를
"...mod 100ms = N ms"인 아무 순간에 끼워넣어도 이후 위상차는 항상 N ms로
고정된다(수학적으로 절대시각 버전과 동일한 결과).

사용:
  python3 spawn_delayed_lidar.py                    # 기본 20ms, competition_map_delayed
  python3 spawn_delayed_lidar.py --offset-ms 20
  python3 spawn_delayed_lidar.py --world competition_map_delayed \
      --model ground_lidar_2_delayed.sdf
"""
import argparse
import os
import sys
import time

import gz.transport13 as transport
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.clock_pb2 import Clock
from gz.msgs10.entity_factory_pb2 import EntityFactory

WORLDS_DIR = os.path.dirname(os.path.abspath(__file__))
STEP_S = 0.004  # world의 max_step_size — 이 배수 근처로만 위상을 맞출 수 있다.


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--world', default='competition_map_delayed',
                    help='make_delayed_lidar2.py가 만든 world의 <world name>')
    ap.add_argument('--model', default='ground_lidar_2_delayed.sdf',
                    help='끼워넣을 독립 스폰용 SDF 경로 (상대경로면 이 스크립트 폴더 기준)')
    ap.add_argument('--entity-name', default='ground_lidar_2',
                    help='world 안에서 이 모델이 가질 이름')
    ap.add_argument('--offset-ms', type=float, default=20.0,
                    help='lidar1(0,100,200...ms)의 주기 안에서 몇 ms 위치에 '
                         f'끼워넣을지(고정 위상차). {STEP_S*1000:.0f}ms의 배수에 '
                         '가장 가깝게 반올림된다.')
    ap.add_argument('--period-ms', type=float, default=100.0,
                    help='센서 주기(=1000/update_rate). 기본 10Hz=100ms.')
    ap.add_argument('--warmup-s', type=float, default=1.0,
                    help='곧바로 요청을 쏘지 않고 이만큼 기다려 gz-transport '
                         '서비스 디스커버리가 끝날 시간을 준다. 그 사이 들어오는 '
                         '클럭 메시지는 위상 판정에만 쓰고 스폰은 안 한다.')
    ap.add_argument('--timeout-s', type=float, default=60.0,
                    help='이 시간 안에 목표 위상이 안 오면 포기하고 종료')
    args = ap.parse_args()

    period_s = args.period_ms / 1000.0
    phase_s = round((args.offset_ms / 1000.0) / STEP_S) * STEP_S
    if not (0.0 <= phase_s < period_s):
        sys.exit(f'--offset-ms는 0~{args.period_ms}(주기) 사이여야 합니다')

    model_path = args.model
    if not os.path.isabs(model_path):
        model_path = os.path.join(WORLDS_DIR, model_path)
    if not os.path.isfile(model_path):
        sys.exit(f'모델 파일이 없습니다: {model_path}\n'
                 f'먼저 make_delayed_lidar2.py를 돌리세요.')

    print(f'목표 위상: 주기 {period_s*1000:.0f}ms 안에서 {phase_s*1000:.1f}ms 지점 '
          f'(요청 {args.offset_ms:.1f}ms를 {STEP_S*1000:.0f}ms 배수로 반올림)')
    print(f'world={args.world}  model={model_path}  entity={args.entity_name}')

    node = transport.Node()
    service = f'/world/{args.world}/create'
    start_wall = time.monotonic()
    state = {'spawned': False, 'sim_time': None, 'result': None, 'attempts': 0}

    def try_spawn(sim_t):
        state['attempts'] += 1
        req = EntityFactory()
        req.sdf_filename = model_path
        req.name = args.entity_name
        # 디스커버리가 덜 끝나 첫 요청이 실패할 수 있으니 여기서 몇 번 재시도한다.
        # 재시도해도 실제 스폰은 여전히 '같은 위상 지점'에서 일어난다 — sim_t가
        # 다음 위상 지점까지 굴러가는 게 아니라, 여기 재시도들은 실시간으로
        # 곧바로 이어지므로(<1초) 위상 오차에 실질적 영향이 없다.
        for attempt in range(5):
            result, rep = node.request(service, req, EntityFactory, Boolean, 3000)
            if result:
                state['spawned'] = True
                state['result'] = (True, bool(rep.data), sim_t, attempt + 1)
                return
            time.sleep(0.3)
        state['spawned'] = True
        state['result'] = (False, False, sim_t, 5)

    def on_clock(msg: Clock):
        if state['spawned'] or state['attempts'] > 0:
            return
        sim_t = msg.sim.sec + msg.sim.nsec * 1e-9
        state['sim_time'] = sim_t
        # 워밍업 동안은 위상 판정만 하고 실제 스폰은 안 한다 — gz-transport
        # 디스커버리가 끝나기 전에 쏘면 첫 시도가 확실히 실패한다.
        if time.monotonic() - start_wall < args.warmup_s:
            return
        phase = sim_t % period_s
        if abs(phase - phase_s) > STEP_S / 2.0:
            return
        state['attempts'] = 1  # 재진입 방지 — 콜백은 다른 스레드에서 온다
        try_spawn(sim_t)

    topic = f'/world/{args.world}/clock'
    node.subscribe(Clock, topic, on_clock)
    print(f'{topic} 구독 시작. 워밍업 {args.warmup_s:.1f}s 후 위상이 맞는 순간을 기다립니다...')

    deadline = time.monotonic() + args.timeout_s
    while not state['spawned'] and time.monotonic() < deadline:
        time.sleep(0.01)

    if not state['spawned']:
        sys.exit(f'{args.timeout_s}s 동안 목표 sim time에 도달하지 못했습니다. '
                 f'월드 이름이 맞는지({args.world}), 시뮬레이션이 실행 중인지 확인하세요.')

    # request()는 콜백 스레드 안에서 이미 끝났지만, 결과가 채워질 때까지 잠깐 대기.
    for _ in range(200):
        if state['result'] is not None:
            break
        time.sleep(0.01)

    result, ok, sim_t, n_attempts = state['result']
    if not result:
        sys.exit(f'{service} 서비스 요청이 5번 다 실패했습니다(응답 없음/타임아웃). '
                 f'world 이름이 맞는지, UserCommands 플러그인이 로드됐는지 확인하세요.')
    if not ok:
        sys.exit(f'서비스는 응답했지만 스폰 실패(Boolean.data=false)했습니다. '
                 f'entity 이름 중복이나 SDF 오류를 의심하세요.')

    phase = sim_t % period_s
    print(f'스폰 성공: sim time {sim_t*1000:.1f}ms (위상 {phase*1000:.1f}ms, '
          f'목표 {phase_s*1000:.1f}ms) 에 {args.entity_name} 삽입됨. '
          f'{n_attempts}번째 시도에서 성공.')


if __name__ == '__main__':
    main()
