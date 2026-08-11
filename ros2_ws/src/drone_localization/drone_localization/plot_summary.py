#!/usr/bin/env python3
"""
비행 요약 — 궤적과 축별 정확도 한 장.

plot_accuracy.py가 채점기준·분포까지 담은 상세본이라면, 이쪽은 비행 직후
바로 보는 간단본이다. 착륙이 감지되면 flight_recorder가 이 스크립트를
자동으로 띄운다.

담는 것:
  왼쪽   실제 비행 궤적 (평면) — 실측 vs 추정, 마커/홈 오버레이
  오른쪽 X / Y / Z 축별 오차 시계열

축을 나눠 그리는 이유: LiDAR가 남북(Y축)으로 배치돼 있어 Y는 센서를 잇는
방향, X는 그에 직교하는 방향이다. 두 축의 오차 성격이 다른데 수평오차
하나로 합치면 그 차이가 사라진다.

사용:
  ros2 run drone_localization plot_summary
  python3 plot_summary.py --output_dir ~/drone_project/flight_logs/run021
"""

import argparse
import os

import numpy as np

from drone_localization.evaluate_accuracy import (
    _align, _find_lag, _load_csv, _load_markers, _segment_visits,
    _smoothed_speed,
)

SURFACE = '#fcfcfb'
INK_PRIMARY = '#0b0b0b'
INK_SECONDARY = '#52514e'
INK_MUTED = '#898781'
GRIDLINE = '#e1e0d9'
BASELINE = '#c3c2b7'

ACTUAL = '#2a78d6'      # 실측 (파랑)
ESTIMATE = '#eb6834'    # 추정 (주황)

AREA_X, AREA_Y = 30.0, 20.0


def _setup_font():
    import matplotlib
    for name in ('Noto Sans CJK JP', 'Noto Sans CJK KR', 'NanumGothic'):
        try:
            matplotlib.rcParams['font.family'] = name
            matplotlib.rcParams['axes.unicode_minus'] = False
            return
        except Exception:
            continue


def _style(ax):
    ax.set_facecolor(SURFACE)
    for s in ('top', 'right'):
        ax.spines[s].set_visible(False)
    for s in ('left', 'bottom'):
        ax.spines[s].set_color(BASELINE)
        ax.spines[s].set_linewidth(0.8)
    ax.grid(True, color=GRIDLINE, lw=0.6, zorder=0)
    ax.set_axisbelow(True)
    ax.tick_params(colors=INK_MUTED, labelsize=8.5, length=0)


def _panel_track(ax, xa, ya, xe, ye, markers):
    """평면 궤적. 실측 위에 추정을 겹쳐 어긋남을 눈으로 보게 한다."""
    # 필드 외곽 + 격자
    for i in range(8):
        x = i * AREA_X / 7
        ax.plot([x, x], [0, AREA_Y], color=GRIDLINE, lw=0.6, zorder=1)
    for j in range(6):
        y = j * AREA_Y / 5
        ax.plot([0, AREA_X], [y, y], color=GRIDLINE, lw=0.6, zorder=1)
    ax.plot([0, AREA_X, AREA_X, 0, 0], [0, 0, AREA_Y, AREA_Y, 0],
            color=BASELINE, lw=1.2, zorder=2)

    ax.plot(xa, ya, '-', color=ACTUAL, lw=1.8, label='실측 (PX4)', zorder=4)
    ax.plot(xe, ye, '-', color=ESTIMATE, lw=1.2, alpha=0.85,
            label='LiDAR 추정', zorder=5)

    if markers:
        mx = [p[0] for p in markers.values()]
        my = [p[1] for p in markers.values()]
        ax.scatter(mx, my, marker='s', s=60, facecolors=INK_SECONDARY,
                   edgecolors=SURFACE, linewidths=1.2, zorder=6)
        for mid, (x, y) in markers.items():
            ax.annotate(f'id{mid}', (x, y), textcoords='offset points',
                        xytext=(0, 10), ha='center', fontsize=8.5,
                        color=INK_SECONDARY)
    ax.scatter([0], [0], marker='o', s=70, facecolors='none',
               edgecolors=INK_SECONDARY, linewidths=1.4, zorder=6)
    ax.annotate('HOME', (0, 0), textcoords='offset points', xytext=(0, -14),
                ha='center', fontsize=8.5, color=INK_MUTED)

    ax.set_aspect('equal')
    ax.set_xlim(-3, AREA_X + 3)
    ax.set_ylim(-4, AREA_Y + 4)
    ax.set_xlabel('X [m]', fontsize=9, color=INK_SECONDARY)
    ax.set_ylabel('Y [m]', fontsize=9, color=INK_SECONDARY)
    ax.set_title('실제 비행 궤적', fontsize=12, color=INK_PRIMARY,
                 loc='left', pad=10)
    leg = ax.legend(fontsize=8.5, frameon=False, loc='upper right')
    for t in leg.get_texts():
        t.set_color(INK_SECONDARY)
    _style(ax)


def _panel_axis(ax, t, err, name, hover, last=False):
    """축 하나의 오차 시계열. 호버 구간을 옅게 칠해 구분한다."""
    # 호버 구간 음영 — 정지 중 오차가 이동 중과 다른지 바로 보인다.
    if hover is not None and hover.any():
        d = np.diff(hover.astype(int))
        starts = list(np.where(d == 1)[0] + 1)
        ends = list(np.where(d == -1)[0] + 1)
        if hover[0]:
            starts = [0] + starts
        if hover[-1]:
            ends = ends + [len(hover) - 1]
        for s, e in zip(starts, ends):
            ax.axvspan(t[s], t[e], color=ACTUAL, alpha=0.06, zorder=1)

    ax.axhline(0, color=BASELINE, lw=0.9, zorder=2)
    ax.plot(t, err, '-', color=ESTIMATE, lw=1.0, zorder=3)

    mean, rmse = err.mean(), np.sqrt((err ** 2).mean())
    ax.axhline(mean, color=INK_MUTED, lw=0.9, ls='--', zorder=2)
    ax.text(0.995, 0.94, f'평균 {mean:+.3f}   RMSE {rmse:.3f}   최대 {np.abs(err).max():.3f} m',
            transform=ax.transAxes, ha='right', va='top', fontsize=8.5,
            color=INK_SECONDARY)

    lim = max(np.abs(err).max() * 1.15, 0.1)
    ax.set_ylim(-lim, lim)
    ax.set_ylabel(f'{name} 오차 [m]', fontsize=9, color=INK_SECONDARY)
    if last:
        ax.set_xlabel('비행 시간 [s]', fontsize=9, color=INK_SECONDARY)
    else:
        ax.tick_params(labelbottom=False)
    _style(ax)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--output_dir',
                    default=os.path.expanduser('~/drone_project/flight_logs'))
    ap.add_argument('--save_only', action='store_true')
    args, _ = ap.parse_known_args()

    actual = _load_csv(os.path.join(args.output_dir, 'actual_path.csv'))
    estimate = _load_csv(os.path.join(args.output_dir, 'estimated_path.csv'))
    if actual is None or estimate is None:
        print(f'로그를 읽지 못했습니다: {args.output_dir}')
        return

    lag = _find_lag(actual, estimate)
    al = _align(actual, estimate, 0.0)
    if al is None:
        print('시간정렬 실패')
        return
    te, xe, ye, ze, xa, ya, za = al

    ex, ey, ez = xe - xa, ye - ya, ze - za
    t0 = te[0]
    t = te - t0
    hover = _smoothed_speed(te, xa, ya) <= 0.25

    # 비행시간: 실제로 떠 있던 구간 (고도 0.5m 이상)
    ta, xa_f, ya_f, za_f = actual
    air = za_f > 0.5
    flight_sec = (ta[air][-1] - ta[air][0]) if air.any() else 0.0
    # 검출률: 비행 구간에서 10Hz로 나올 수 있는 표본 대비 실제 표본
    det = 100.0 * len(te) / max(flight_sec * 10.0, 1.0)

    markers = _load_markers()
    n_wp, wp_mean = 0, float('nan')
    if markers is not None:
        mids = list(markers.keys())
        mxy = np.array([markers[m] for m in mids])
        d = np.linalg.norm(np.stack([xa, ya], 1)[:, None, :] - mxy[None, :, :],
                           axis=2)
        ni = np.argmin(d, axis=1)
        near = d[np.arange(len(te)), ni] <= 4.5
        nearest = np.array([mids[i] for i in ni])
        errs = []
        for v in _segment_visits(te, near, hover, nearest, 0.4, 1.0):
            k = v['idxs']
            pe = np.median(np.stack([xe[k], ye[k]], 1), 0)
            pa = np.median(np.stack([xa[k], ya[k]], 1), 0)
            errs.append(float(np.hypot(*(pe - pa))))
        n_wp = len(errs)
        if errs:
            wp_mean = float(np.mean(errs))

    _setup_font()
    import matplotlib
    if args.save_only:
        matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    fig = plt.figure(figsize=(14, 7.5), facecolor=SURFACE)
    gs = fig.add_gridspec(3, 2, width_ratios=[1.15, 1], hspace=0.16,
                          wspace=0.18, left=0.055, right=0.98,
                          top=0.84, bottom=0.09)

    _panel_track(fig.add_subplot(gs[:, 0]), xa, ya, xe, ye, markers)
    for i, (err, name) in enumerate(((ex, 'X'), (ey, 'Y'), (ez, 'Z'))):
        _panel_axis(fig.add_subplot(gs[i, 1]), t, err, name, hover,
                    last=(i == 2))

    fig.text(0.055, 0.94, '비행 요약', fontsize=17, color=INK_PRIMARY,
             weight='bold')
    line1 = (f'비행시간 {flight_sec:.0f}초   ·   표본 {len(te)}개   ·   '
             f'검출률 {det:.0f}%   ·   경로점 {n_wp}개')
    line2 = (f'축별 RMSE   X {np.sqrt((ex**2).mean()):.3f}   '
             f'Y {np.sqrt((ey**2).mean()):.3f}   '
             f'Z {np.sqrt((ez**2).mean()):.3f} m'
             + (f'   ·   경로점 수평오차 평균 {wp_mean:.3f} m'
                if n_wp else '')
             + f'   ·   추정 지연 {abs(lag):.2f}s')
    fig.text(0.055, 0.905, line1, fontsize=10, color=INK_SECONDARY)
    fig.text(0.055, 0.873, line2, fontsize=9.5, color=INK_MUTED)
    fig.text(0.98, 0.905, os.path.basename(os.path.normpath(args.output_dir)),
             fontsize=10, color=INK_MUTED, ha='right')

    out = os.path.join(args.output_dir, 'flight_summary.png')
    fig.savefig(out, dpi=140, facecolor=SURFACE)
    print(f'그림 저장: {out}')
    if not args.save_only:
        try:
            plt.show()
        except Exception as e:
            print(f'창을 못 열었습니다({e}). PNG를 열어보세요: {out}')


if __name__ == '__main__':
    main()
