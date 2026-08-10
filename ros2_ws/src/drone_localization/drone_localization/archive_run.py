#!/usr/bin/env python3
"""
방금 비행한 로그를 runNNN/ 으로 보관.

flight_recorder는 매 실행마다 actual_path.csv / estimated_path.csv 를
덮어쓰므로, 여러 번 비행한 결과를 비교하려면 비행이 끝날 때마다 따로
떠 놓아야 한다. 이 스크립트가 그 한 단계를 담당한다.

사용법:
  python3 archive_run.py                  # runNNN 자동 번호
  ros2 run drone_localization archive_run # (빌드했다면 이것도 가능)
  ros2 run drone_localization archive_run --name ns_layout_1
  #   --move   복사 대신 이동 (원본 삭제)
"""

import argparse
import os
import re
import shutil

FILES = ('actual_path.csv', 'estimated_path.csv')


def _next_run_dir(base):
    used = []
    for name in os.listdir(base):
        m = re.fullmatch(r'run(\d+)', name)
        if m and os.path.isdir(os.path.join(base, name)):
            used.append(int(m.group(1)))
    return f'run{max(used) + 1 if used else 1:03d}'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--output_dir',
                    default=os.path.expanduser('~/drone_project/flight_logs'))
    ap.add_argument('--name', default=None,
                    help='보관 디렉토리 이름 (기본: runNNN 자동 증가)')
    ap.add_argument('--move', action='store_true')
    args, _ = ap.parse_known_args()

    base = args.output_dir
    if not os.path.isdir(base):
        print(f'로그 디렉토리가 없습니다: {base}')
        return

    missing = [f for f in FILES if not os.path.isfile(os.path.join(base, f))]
    if missing:
        print(f'보관할 파일이 없습니다: {", ".join(missing)}\n'
              'flight_recorder를 켜고 비행한 뒤 실행하세요.')
        return

    # 추정 로그가 헤더만 있으면 트래커가 안 돌았다는 뜻 — 비교에 못 쓴다.
    est = os.path.join(base, 'estimated_path.csv')
    with open(est) as f:
        n_est = sum(1 for _ in f) - 1
    if n_est < 10:
        print(f'경고: estimated_path.csv에 표본이 {max(n_est, 0)}개뿐입니다. '
              '트래커가 동작했는지 확인하세요.')

    name = args.name or _next_run_dir(base)
    dest = os.path.join(base, name)
    if os.path.exists(dest):
        print(f'이미 존재합니다: {dest}')
        return
    os.makedirs(dest)

    for f in FILES:
        src = os.path.join(base, f)
        if args.move:
            shutil.move(src, os.path.join(dest, f))
        else:
            shutil.copy2(src, os.path.join(dest, f))

    with open(os.path.join(base, 'actual_path.csv')) as f:
        n_act = sum(1 for _ in f) - 1 if not args.move else 0

    print(f'보관 완료: {dest}  (추정 {n_est}표본'
          + (f', 실측 {n_act}표본' if not args.move else '') + ')')
    print('여러 런을 비교하려면: ros2 run drone_localization plot_runs')


if __name__ == '__main__':
    main()
