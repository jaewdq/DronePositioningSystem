#!/usr/bin/env python3
"""
QR 마커별 "실제 위치 vs 측정 위치" 공간 시각화.

오차를 크기(0.1m)로만 보고하면 "왜 그만큼 틀렸는지"를 설명할 수 없다.
이 도구는 각 마커에서 호버할 때 측정값이 실제 위치를 기준으로 **어느
방향으로 얼마나** 벗어났는지를 그려서, 무작위 잡음인지 한쪽으로 쏠린
계통 편향인지 눈으로 구분할 수 있게 한다.

  왼쪽  필드 전체 — 마커·LiDAR 배치와 각 마커의 편향 방향(확대 표시)
  오른쪽 마커별 확대 — 실제 위치를 원점에 두고 방문마다 측정값을 점으로

사용법:
  python3 plot_spatial.py
  #   --runs N       비교할 최근 런 개수 (기본 5)
  #   --save_only    창 없이 PNG만 저장
"""

import argparse
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
for _p in (os.path.dirname(_HERE),
           os.path.join(os.path.dirname(os.path.dirname(_HERE)),
                        'drone_mission')):
    if os.path.isdir(_p) and _p not in sys.path:
        sys.path.insert(0, _p)

import numpy as np

from drone_localization.evaluate_accuracy import (
    _align, _find_lag, _load_csv, _load_markers, _segment_visits,
    _smoothed_speed,
)
from drone_localization.plot_accuracy import (
    INK_MUTED, INK_PRIMARY, INK_SECONDARY, SERIES_1, SURFACE, _setup_font,
    _style,
)
from drone_localization.plot_runs import _find_runs

ACCENT = SERIES_1
# 이 거리를 넘는 방문은 그림에 런 이름을 붙여 추적 가능하게 한다.
OUTLIER_M = 0.3
AREA_X, AREA_Y = 30.0, 20.0
GRIDLINE = '#e1e0d9'
BASELINE = '#c3c2b7'


def _collect(base, n_runs, speed_threshold=0.25, gate_radius=4.5,
             min_duration=0.4, merge_gap=1.0):
    """마커별로 (실제 median, 추정 median) 쌍을 런 전체에서 모은다."""
    markers = _load_markers()
    if markers is None:
        return None, None, None

    out = {m: [] for m in markers}
    used = []
    for name, path in _find_runs(base, n_runs):
        actual = _load_csv(os.path.join(path, 'actual_path.csv'))
        estimate = _load_csv(os.path.join(path, 'estimated_path.csv'))
        if actual is None or estimate is None or len(estimate[0]) < 10:
            continue
        al = _align(actual, estimate, 0.0)
        if al is None:
            continue
        te, xe, ye, ze, xa, ya, za = al
        hover = _smoothed_speed(te, xa, ya) <= speed_threshold

        mids = list(markers.keys())
        mxy = np.array([markers[m] for m in mids])
        d = np.linalg.norm(np.stack([xa, ya], axis=1)[:, None, :]
                           - mxy[None, :, :], axis=2)
        ni = np.argmin(d, axis=1)
        near = d[np.arange(len(te)), ni] <= gate_radius
        nearest = np.array([mids[i] for i in ni])

        got = False
        for v in _segment_visits(te, near, hover, nearest,
                                 min_duration, merge_gap):
            k = v['idxs']
            pe = np.median(np.stack([xe[k], ye[k]], axis=1), axis=0)
            pa = np.median(np.stack([xa[k], ya[k]], axis=1), axis=0)
            out[v['marker_id']].append((pa, pe, name))
            got = True
        if got:
            used.append(name)
    return markers, out, used


def _lidars():
    """params yaml에서 LiDAR 좌표를 읽는다 (배치 맥락 표시용)."""
    try:
        import yaml
        from ament_index_python.packages import get_package_share_directory
        p = os.path.join(get_package_share_directory('drone_localization'),
                         'params', 'lidar_localization.yaml')
    except Exception:
        p = os.path.join(os.path.dirname(_HERE), 'params',
                         'lidar_localization.yaml')
    try:
        import yaml
        d = yaml.safe_load(open(p))['lidar_drone_tracker']['ros__parameters']
        return [(d['lidar1_x'], d['lidar1_y']), (d['lidar2_x'], d['lidar2_y'])]
    except Exception:
        return []


def _panel_field(ax, markers, data, mag=None):
    """필드 배치도 — 마커마다 실제/측정 좌표와 둘 사이 거리를 표기.

    편향을 확대 화살표로 그리는 방법도 있지만, 배율이 들어가면 "실제로
    이만큼 틀렸다"는 인상을 준다. 실척으로 두면 두 점이 사실상 겹치는데,
    30m 필드에서 0.1m 오차가 안 보이는 것 자체가 정직한 그림이다.
    방향과 크기는 옆의 숫자와 오른쪽 확대도가 담당한다."""
    for i in range(8):
        x = i * AREA_X / 7
        ax.plot([x, x], [0, AREA_Y], color=GRIDLINE, lw=0.6, zorder=1)
    for j in range(6):
        y = j * AREA_Y / 5
        ax.plot([0, AREA_X], [y, y], color=GRIDLINE, lw=0.6, zorder=1)
    ax.plot([0, AREA_X, AREA_X, 0, 0], [0, 0, AREA_Y, AREA_Y, 0],
            color=BASELINE, lw=1.2, zorder=2)

    for lx, ly in _lidars():
        ax.plot(lx, ly, marker='^', ms=11, color=INK_SECONDARY,
                mec=SURFACE, mew=1.2, zorder=5)
        ax.annotate('LiDAR', (lx, ly), textcoords='offset points',
                    xytext=(0, -14 if ly < 0 else 8), ha='center',
                    fontsize=8, color=INK_SECONDARY)

    for mid, (mx, my) in markers.items():
        pairs = data.get(mid, [])
        # 실제 위치(사각) 위에 측정 평균(✕)을 실척으로 겹쳐 찍는다.
        ax.plot(mx, my, marker='s', ms=9, color=INK_SECONDARY,
                mec=SURFACE, mew=1.0, zorder=5)
        if not pairs:
            ax.annotate(f'id{mid}', (mx, my), textcoords='offset points',
                        xytext=(10, 6), fontsize=9, color=INK_PRIMARY)
            continue

        est = np.mean([pe for pa, pe, _ in pairs], axis=0)
        act = np.mean([pa for pa, pe, _ in pairs], axis=0)
        dist = float(np.hypot(*(est - act)))
        ax.plot(est[0], est[1], marker='X', ms=8, color=ACCENT,
                mec=SURFACE, mew=1.0, zorder=6)

        # 라벨은 필드 중심 반대쪽으로 빼서 서로 겹치지 않게 한다.
        dx = 12 if mx >= AREA_X / 2 else -12
        dy = 10 if my >= AREA_Y / 2 else -10
        ax.annotate(
            f'id{mid}   Δ {dist:.3f} m\n'
            f'실제 ({mx:.2f}, {my:.2f})\n'
            f'측정 ({est[0]:.2f}, {est[1]:.2f})',
            (mx, my), textcoords='offset points', xytext=(dx, dy),
            ha='right' if dx < 0 else 'left',
            va='top' if dy < 0 else 'bottom',
            fontsize=8, color=INK_SECONDARY, linespacing=1.5,
            bbox=dict(boxstyle='round,pad=0.35', fc=SURFACE, ec=GRIDLINE,
                      lw=0.8))

    ax.set_aspect('equal')
    ax.set_xlim(-6, AREA_X + 6)
    ax.set_ylim(-5, AREA_Y + 5)
    ax.set_xlabel('world X [m]', fontsize=9, color=INK_SECONDARY)
    ax.set_ylabel('world Y [m]', fontsize=9, color=INK_SECONDARY)
    ax.set_title('필드 배치 — ■ 실제 위치, ✕ 측정 평균 (실척)',
                 fontsize=11.5, color=INK_PRIMARY, loc='left', pad=10)
    _style(ax)
    ax.grid(False)


def _panel_target(ax, mid, true_xy, pairs, lim):
    """마커 하나를 확대 — 실제 위치를 원점(십자)에 두고 측정값을 점으로."""
    for r, lbl in ((0.1, '0.1m'), (0.25, None), (0.5, '0.5m')):
        if r <= lim:
            ax.add_artist(plt_circle(r))
            if lbl:
                ax.annotate(lbl, (0, r), textcoords='offset points',
                            xytext=(3, 2), fontsize=7, color=INK_MUTED)
    ax.axhline(0, color=BASELINE, lw=0.8, zorder=2)
    ax.axvline(0, color=BASELINE, lw=0.8, zorder=2)

    if pairs:
        d = np.array([pe - pa for pa, pe, _ in pairs])
        ax.plot(d[:, 0], d[:, 1], 'o', ms=5, color=ACCENT, alpha=0.5,
                mec='none', zorder=3)
        # 크게 벗어난 점은 어느 비행에서 나왔는지 표시해야 추적이 된다.
        norms = np.hypot(d[:, 0], d[:, 1])
        for (pa, pe, rn), nv, dd in zip(pairs, norms, d):
            if nv > OUTLIER_M:
                # 라벨은 원점 반대쪽(위/아래)으로 뺀다. 중앙에는 점 무리와
                # 평균 표시가 몰려 있어 안쪽으로 붙이면 겹친다.
                up = dd[1] >= 0
                ax.annotate(f'{rn}  {nv:.2f}m', (dd[0], dd[1]),
                            textcoords='offset points',
                            xytext=(0, 11 if up else -11),
                            ha='center', va='bottom' if up else 'top',
                            fontsize=7.5, color=INK_SECONDARY)

        m = d.mean(axis=0)
        ax.plot(m[0], m[1], marker='X', ms=11, color=ACCENT, mec=SURFACE,
                mew=1.2, zorder=5)
        ax.annotate(f'평균 ({m[0]:+.3f}, {m[1]:+.3f})', (m[0], m[1]),
                    textcoords='offset points', xytext=(8, -10),
                    fontsize=7.5, color=ACCENT)

    ax.set_aspect('equal')
    ax.set_xlim(-lim, lim)
    ax.set_ylim(-lim, lim)
    ax.set_title(f'id{mid}  실제 ({true_xy[0]:.2f}, {true_xy[1]:.2f})',
                 fontsize=9.5, color=INK_PRIMARY, loc='left', pad=6)
    ax.tick_params(colors=INK_MUTED, labelsize=7, length=0)
    for s in ('top', 'right'):
        ax.spines[s].set_visible(False)
    for s in ('left', 'bottom'):
        ax.spines[s].set_color(BASELINE)
    ax.set_facecolor(SURFACE)


def plt_circle(r):
    import matplotlib.patches as mp
    return mp.Circle((0, 0), r, fill=False, ec=GRIDLINE, lw=0.9, zorder=1)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--output_dir',
                    default=os.path.expanduser('~/drone_project/flight_logs'))
    ap.add_argument('--runs', type=int, default=5)
    ap.add_argument('--save_only', action='store_true')
    args, _ = ap.parse_known_args()

    markers, data, used = _collect(args.output_dir, args.runs)
    if not markers or not any(data.values()):
        print('마커별 호버 구간을 찾지 못했습니다. archive_run으로 보관된 '
              '런이 있는지 확인하세요.')
        return
    print(f'{len(used)}개 런 사용: {", ".join(used)}')
    for mid in sorted(data):
        pairs = data[mid]
        if not pairs:
            continue
        d = np.array([pe - pa for pa, pe, _ in pairs])
        est = np.array([pe for pa, pe, _ in pairs]).mean(axis=0)
        print(f'  id{mid}: 실제 ({markers[mid][0]:7.3f}, {markers[mid][1]:6.3f})  '
              f'측정평균 ({est[0]:7.3f}, {est[1]:6.3f})  '
              f'편차 ({d[:,0].mean():+.3f}, {d[:,1].mean():+.3f})  n={len(pairs)}')

    _setup_font()
    import matplotlib
    if args.save_only:
        matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    lim = max(0.3, float(np.max([np.abs(pe - pa).max()
                                 for v in data.values()
                                 for pa, pe, _ in v])) * 1.25)

    fig = plt.figure(figsize=(15.0, 6.4), facecolor=SURFACE)
    gs = fig.add_gridspec(2, 4, width_ratios=[2.1, 0.12, 1, 1],
                          hspace=0.42, wspace=0.30,
                          left=0.05, right=0.98, top=0.80, bottom=0.10)

    _panel_field(fig.add_subplot(gs[:, 0]), markers, data)
    ids = sorted(markers)
    for i, mid in enumerate(ids[:4]):
        ax = fig.add_subplot(gs[i // 2, 2 + i % 2])
        _panel_target(ax, mid, markers[mid], data.get(mid, []), lim)

    fig.text(0.05, 0.93, 'QR 마커 실제 위치 vs 측정 위치', fontsize=15,
             color=INK_PRIMARY, weight='bold')
    fig.text(0.05, 0.875,
             f'{len(used)}개 런 · 호버 구간 median · ● 방문 1회, ✕ 평균 · '
             '왼쪽은 실척(오차가 작아 두 점이 겹침), 오른쪽은 실제 위치를 원점에 둔 확대도',
             fontsize=9.5, color=INK_MUTED)

    out = os.path.join(args.output_dir, 'accuracy_spatial.png')
    fig.savefig(out, dpi=150, facecolor=SURFACE)
    print(f'\n그림 저장: {out}')
    if not args.save_only:
        try:
            plt.show()
        except Exception as e:
            print(f'창을 못 열었습니다({e}). PNG를 열어보세요: {out}')


if __name__ == '__main__':
    main()
