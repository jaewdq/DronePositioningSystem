#!/usr/bin/env python3
"""
융합 방식 3종 오프라인 비교 — 재비행 없이 같은 로그로 판정.

클러스터 덤프에는 점마다 출처 센서가 기록돼 있으므로, 이미 날린 비행
하나로 아래 세 방식을 모두 계산해 편향과 산포를 직접 비교할 수 있다.

  현재   전체 점의 median. 점을 많이 내놓는(=가까운) 센서가 자동으로
         지배하므로, 그 센서가 보는 면 쪽으로 센트로이드가 끌린다.
  최적   센서별 median을 점 개수로 가중평균. 통계적으로 산포는 최소지만
         가중이 여전히 가까운 센서로 쏠려 편향은 남는다.
  동일   센서별 median을 1:1 평균. 두 면의 편향이 대칭이라 상쇄되지만,
         점이 적은 센서의 잡음을 그대로 절반 떠안는다.

편향은 반복해도 줄지 않고 산포는 줄어드는 성질이 다르므로, 둘을 갈라서
본다. 판단 기준은 "어느 쪽이 총 오차(RMSE)를 낮추는가".

사용법:
  python3 compare_fusion.py
  #   --dump FILE        클러스터 덤프 (기본 ~/drone_project/flight_logs/clusters.npz)
  #   --output_dir DIR   ground truth(actual_path.csv)가 있는 디렉토리
  #   --min_pts N        센서별 최소 점 개수, 미만이면 분리 불가로 처리 (기본 2)
"""

import argparse
import csv
import os

import numpy as np


def _load_actual(path):
    if not os.path.isfile(path):
        return None
    rows = []
    with open(path) as f:
        r = csv.reader(f)
        next(r, None)
        for row in r:
            if len(row) >= 4:
                rows.append([float(v) for v in row[:4]])
    if not rows:
        return None
    a = np.array(rows)
    return a[:, 0], a[:, 1], a[:, 2], a[:, 3]


def _fuse(pts, src, min_pts):
    """세 방식의 센트로이드를 한 프레임에서 계산.

    반환: (현재, 최적, 동일, 분리가능여부)
    분리 불가(한쪽 센서 점이 부족)하면 세 값 모두 현재 방식으로 채운다 —
    그 프레임은 애초에 선택지가 없으므로 비교 대상에서 빼기 위함."""
    cur = np.median(pts, axis=0)
    m0, m1 = src == 0, src == 1
    n0, n1 = int(m0.sum()), int(m1.sum())
    if n0 < min_pts or n1 < min_pts:
        return cur, cur, cur, False
    c0, c1 = np.median(pts[m0], axis=0), np.median(pts[m1], axis=0)
    opt = (c0 * n0 + c1 * n1) / (n0 + n1)
    eq = (c0 + c1) / 2.0
    return cur, opt, eq, True


def _report(name, est, gx, gy, gz, gyy):
    ex, ey, ez = est[:, 0] - gx, est[:, 1] - gy, est[:, 2] - gz
    exy = np.hypot(ex, ey)
    # 편향은 위치에 따라 부호가 뒤집히는 계통 성분이므로, 남/북 구간의
    # 평균 차이(진폭)로 크기를 잰다. 전체 평균만 보면 서로 상쇄돼 숨는다.
    south, north = gyy < 8, gyy > 12
    amp = (ey[north].mean() - ey[south].mean()
           if south.sum() > 10 and north.sum() > 10 else float('nan'))
    print(f'  {name:6s} 수평 평균 {exy.mean():.4f}  RMSE {np.sqrt((exy**2).mean()):.4f}  '
          f'| y편향 진폭 {amp:+.4f}  | 산포(y std) {ey.std():.4f}')
    return exy, amp


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--dump', default=os.path.expanduser(
        '~/drone_project/flight_logs/clusters.npz'))
    ap.add_argument('--output_dir', default=os.path.expanduser(
        '~/drone_project/flight_logs'))
    ap.add_argument('--min_pts', type=int, default=2)
    args, _ = ap.parse_known_args()

    if not os.path.isfile(args.dump):
        print(f'덤프가 없습니다: {args.dump}\n'
              'cluster_dump_path 파라미터를 주고 비행한 뒤 트래커를 Ctrl+C로 종료하세요.')
        return
    d = np.load(args.dump)
    if 'sources' not in d:
        print('이 덤프에는 출처 센서 정보가 없습니다(구버전).\n'
              '트래커를 다시 빌드한 뒤 새로 비행해 주세요.')
        return

    pts, off, tc, src = d['points'], d['offsets'], d['times'], d['sources']
    n = len(tc)

    cur = np.zeros((n, 3)); opt = np.zeros((n, 3)); eq = np.zeros((n, 3))
    ok = np.zeros(n, bool); cnt = np.zeros((n, 2), int)
    for i in range(n):
        s = slice(off[i], off[i + 1])
        p, sr = pts[s], src[s]
        cnt[i] = [(sr == 0).sum(), (sr == 1).sum()]
        cur[i], opt[i], eq[i], ok[i] = _fuse(p, sr, args.min_pts)

    print('=' * 66)
    print(' 융합 방식 비교')
    print('=' * 66)
    print(f' 프레임 {n}개 · 점 {len(pts)}개')
    print(f' 센서별 점 개수(프레임당 중앙값): '
          f'lidar1 {np.median(cnt[:, 0]):.0f}개  lidar2 {np.median(cnt[:, 1]):.0f}개')
    print(f' 양쪽 {args.min_pts}점 이상 확보: {ok.sum()}/{n}프레임 ({100 * ok.mean():.0f}%)')
    if ok.sum() < 20:
        print('\n분리 가능한 프레임이 너무 적어 비교가 무의미합니다.')
        print('→ 한쪽 센서가 드론을 거의 못 보고 있다는 뜻이며, 이 경우')
        print('  측정 층위 융합(센서별 DBSCAN)은 애초에 성립하지 않습니다.')
        return

    actual = _load_actual(os.path.join(args.output_dir, 'actual_path.csv'))
    if actual is None:
        print(f'\nactual_path.csv 를 찾지 못해 오차 비교는 생략합니다.')
        return
    ta, xa, ya, za = actual

    # 덤프 시각은 ROS 클록, actual은 recorder의 monotonic 상대시각이라
    # 원점이 완전히 다르다. 첫 표본끼리 맞춰도 "기록 시작 ~ 첫 검출" 만큼
    # 어긋나 있고 그 값은 수십 초까지 벌어질 수 있으므로, 비행 전체 길이를
    # 덮는 범위를 성기게 훑은 뒤 그 주변만 촘촘히 다시 본다.
    t0 = tc - tc[0] + ta[0]

    def _best_shift(lo, hi, step):
        b = None
        for sh in np.arange(lo, hi + step / 2, step):
            tt = t0 + sh
            m = (tt >= ta[0]) & (tt <= ta[-1]) & ok
            if m.sum() < 20:
                continue
            e = np.hypot(cur[m, 0] - np.interp(tt[m], ta, xa),
                         cur[m, 1] - np.interp(tt[m], ta, ya))
            s = float(np.median(e))
            if b is None or s < b[0]:
                b = (s, float(sh))
        return b

    span = float(ta[-1] - ta[0])
    coarse = _best_shift(-span, span, 0.5)
    if coarse is None:
        print('\n시간정렬 실패: 겹치는 구간이 없습니다.')
        return
    best = _best_shift(coarse[1] - 1.0, coarse[1] + 1.0, 0.02) or coarse
    # 정렬이 제대로 됐다면 중앙 오차가 수십 cm여야 한다. 그보다 크면
    # 엉뚱한 지점에 맞춘 것이므로 그 수치로 결론을 내면 안 된다.
    if best[0] > 1.0:
        print(f'\n시간정렬 실패: 최적 지점에서도 중앙오차 {best[0]:.2f}m.')
        print('두 로그가 같은 비행이 아닐 수 있습니다(덤프와 actual_path.csv 확인).')
        return
    t = t0 + best[1]
    m = (t >= ta[0]) & (t <= ta[-1]) & ok
    gx = np.interp(t[m], ta, xa); gy = np.interp(t[m], ta, ya)
    gz = np.interp(t[m], ta, za)

    print(f'\n 시간정렬 보정 {best[1]:+.2f}s · 비교 표본 {m.sum()}개')
    print('-' * 66)
    _report('현재', cur[m], gx, gy, gz, gy)
    _report('최적', opt[m], gx, gy, gz, gy)
    _report('동일', eq[m], gx, gy, gz, gy)
    print('-' * 66)

    r = {k: np.sqrt((np.hypot(v[m, 0] - gx, v[m, 1] - gy) ** 2).mean())
         for k, v in (('현재', cur), ('최적', opt), ('동일', eq))}
    win = min(r, key=r.get)
    print(f' RMSE 최소: {win} ({r[win]:.4f}m)')
    gain = r['현재'] - r[win]
    if win == '현재' or gain < 0.005:
        print(' → 현재 방식 유지. 구조를 바꿀 만한 이득이 없습니다.')
    else:
        print(f' → {win} 방식이 현재보다 {gain:.4f}m 낫습니다. 구조 변경 검토 가치 있음.')


if __name__ == '__main__':
    main()
