#!/usr/bin/env python3
"""
compare_fusion.sh 결과(<run>/fusion/{PF,MF,SU})를 그림과 표로 비교한다.

  python3 plot_fusion.py <run>/fusion        →  <run>/fusion/fusion_compare.png

평가 규칙은 논문 eval_aligned.py 와 같다: 0.01 s 격자의 평가 시각마다 각 구조의
최신 사후 상태(0.25 s 이내)를 등속 모델로 평가 시각까지 예측해 참값과 비교.
실내는 이착륙도 보므로 비행 구간 기준만 참값 z ≥ MIN_Z 로 바꿨다.
"""

import os
import sys
import xml.etree.ElementTree as ET

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Rectangle

MIN_Z = 0.15            # 이보다 낮으면 이착륙 접지 구간 (ROI 하한 0.10 m 아래로 기체가 걸침)
VALID_S = 0.25          # 상태가 이보다 오래되면 그 시각은 추정 없음
HOVER_SPEED = 0.25      # 참값 수평속도가 이 이하면 호버/정지 구간
METHODS = (('PF', '점군 융합', 'tab:blue'), ('MF', '관측 융합', 'tab:orange'),
           ('SU', '순차 갱신', 'tab:green'))

for f in ('NanumGothic', 'Noto Sans CJK JP', 'Noto Sans CJK KR'):
    if any(f == x.name for x in matplotlib.font_manager.fontManager.ttflist):
        plt.rcParams['font.family'] = f
        break
plt.rcParams['axes.unicode_minus'] = False


def load(p):
    return np.atleast_2d(np.loadtxt(p, delimiter=',', skiprows=1))


def interp(d, t):
    return np.column_stack([np.interp(t, d[:, 0], d[:, j]) for j in (1, 2, 3)])


def rmse(e):
    return float(np.sqrt(np.mean(np.sum(e ** 2, 1)))) if len(e) else float('nan')


def lidar_xy(sdf):
    out = []
    if os.path.exists(sdf):
        for m in ET.parse(sdf).getroot().iter('model'):
            if m.get('name', '').startswith('ground_lidar_'):
                out.append(tuple(map(float, m.find('pose').text.split()[:3])))
    return out


def main():
    fdir = os.path.abspath(sys.argv[1] if len(sys.argv) > 1 else '.')
    missing = [m for m, _, _ in METHODS
               if not os.path.exists(os.path.join(fdir, m, 'state_details.csv'))]
    if missing:
        sys.exit(f'{fdir} 에 {", ".join(missing)} 결과가 없습니다 — compare_fusion.sh 를 먼저 돌리세요.')
    truth = load(os.path.join(fdir, 'PF', 'actual_ground_truth_path.csv'))
    times = np.arange(np.ceil(truth[0, 0] / .01) * .01, truth[-1, 0], .01)
    gt = interp(truth, times)
    fly = gt[:, 2] >= MIN_Z
    speed = np.linalg.norm((interp(truth, times + .3) - interp(truth, times - .3))[:, :2], axis=1) / .6
    hover = speed <= HOVER_SPEED

    est, valid, states = {}, {}, {}
    for m, _, _ in METHODS:
        s = load(os.path.join(fdir, m, 'state_details.csv'))
        states[m] = s
        idx = np.searchsorted(s[:, 0], times, side='right') - 1
        i = np.maximum(idx, 0)
        valid[m] = (idx >= 0) & (times - s[i, 0] <= VALID_S)
        est[m] = s[i, 1:4] + s[i, 4:7] * (times - s[i, 0])[:, None]
    common = fly & np.logical_and.reduce([valid[m] for m, _, _ in METHODS])

    print(f'평가 구간: 비행 {fly.sum() / 100:.1f} s, 세 구조 공통 유효 {common.sum() / 100:.1f} s')
    print(f"{'구조':6s} {'추정 유지%':>9s} {'XY RMSE':>8s} {'3D RMSE':>8s} {'호버 XY':>8s} {'이동 XY':>8s} "
          f"{'Z 편향':>8s} {'XY 최대':>8s}")
    rows = {}
    for m, name, _ in METHODS:
        e = est[m] - gt
        exy = np.linalg.norm(e[:, :2], axis=1)
        r = dict(cov=valid[m][fly].mean() * 100, xy=rmse(e[common, :2]), d3=rmse(e[common]),
                 hov=rmse(e[common & hover, :2]), mov=rmse(e[common & ~hover, :2]),
                 zb=float(e[common, 2].mean()), xmax=float(exy[common].max()))
        rows[m] = r
        print(f"{m} {name:5s} {r['cov']:8.1f}% {r['xy']:8.3f} {r['d3']:8.3f} {r['hov']:8.3f} {r['mov']:8.3f} "
              f"{r['zb']:+8.3f} {r['xmax']:8.3f}")

    # ---------------------------------------------------------------- 그림
    fig = plt.figure(figsize=(16, 9))
    gs = fig.add_gridspec(3, 3, width_ratios=[1.25, 1, 1], hspace=0.45, wspace=0.3)
    ax = fig.add_subplot(gs[:, 0])
    ax.add_patch(Rectangle((0, 0), 3, 2, fc='0.94', ec='k', ls='--', lw=1))
    for k, (x, y, _) in enumerate(lidar_xy(os.path.join(os.path.dirname(fdir), 'world.sdf')), 1):
        ax.plot(x, y, 'o', color='0.2', ms=10)
        ax.annotate(f'LiDAR{k}', (x, y), textcoords='offset points', xytext=(8, 0), va='center', fontsize=9)
    ax.plot(gt[fly, 0], gt[fly, 1], color='k', lw=2.5, label='참값', zorder=1)
    for m, name, c in METHODS:
        s = states[m]
        ok = interp(truth, s[:, 0])[:, 2] >= MIN_Z
        ax.plot(s[ok, 1], s[ok, 2], '.', color=c, ms=3, alpha=0.8, label=f'{m} {name}')
    ax.plot(0, 0, 'k^', ms=10); ax.plot(3, 2, 'kv', ms=10)
    ax.set(aspect='equal', xlim=(-0.6, 3.6), ylim=(-1.0, 3.0), xlabel='x [m]', ylabel='y [m]',
           title='(a) 위에서 본 동선 (비행 구간)')
    ax.legend(loc='upper left', fontsize=8, markerscale=3)

    t0 = times[0]
    for row, (j, lab) in enumerate(((0, 'x'), (1, 'y'), (2, 'z'))):
        a = fig.add_subplot(gs[row, 1])
        a.plot(times - t0, gt[:, j], 'k', lw=2, label='참값')
        for m, _, c in METHODS:
            v = np.where(valid[m], est[m][:, j], np.nan)
            a.plot(times - t0, v, color=c, lw=1, label=m)
        a.set(ylabel=f'{lab} [m]'); a.grid(alpha=0.3)
        if row == 0:
            a.set_title('(b) 시간별 위치'); a.legend(fontsize=7, ncol=4)
        if row == 2:
            a.set_xlabel('시간 [s]')

    a = fig.add_subplot(gs[0:2, 2])
    for m, _, c in METHODS:
        e = np.linalg.norm((est[m] - gt)[:, :2], axis=1)
        a.plot(times - t0, np.where(valid[m] & fly, e, np.nan), color=c, lw=1, label=m)
    a.set(title='(c) 수평 오차', xlabel='시간 [s]', ylabel='수평 오차 [m]'); a.grid(alpha=0.3); a.legend(fontsize=8)

    a = fig.add_subplot(gs[2, 2])
    keys = (('xy', '전체'), ('hov', '호버'), ('mov', '이동'))
    w = 0.25
    for k, (m, _, c) in enumerate(METHODS):
        vals = [rows[m][key] for key, _ in keys]
        b = a.bar(np.arange(3) + (k - 1) * w, vals, w, color=c, label=m)
        a.bar_label(b, fmt='%.3f', fontsize=7)
    a.set_xticks(range(3), [lab for _, lab in keys])
    a.set(title='(d) 수평 RMSE [m]'); a.legend(fontsize=7)

    out = os.path.join(fdir, 'fusion_compare.png')
    fig.savefig(out, dpi=120, bbox_inches='tight')
    print('그림:', out)


if __name__ == '__main__':
    main()
