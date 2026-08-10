#!/usr/bin/env python3
"""
여러 번 비행한 결과를 모아 비교 — 재현성(repeatability) 시각화.

한 번의 비행으로 나온 수치는 그날의 운(비행 궤적, 노이즈)이 섞여 있어
"이 시스템의 성능"이라고 말하기 어렵다. 같은 조건으로 여러 번 날린 뒤
런마다 값이 얼마나 흔들리는지를 함께 보여야 신뢰할 수 있는 지표가 된다.

이 도구는 flight_logs/runNNN/ 들을 스캔해 최근 N개(기본 5개)를 읽고,
런별 수치와 그 평균·표준편차를 한 장으로 정리한다.

패널 구성:
  A 런별 경로점 수평오차 (평균/RMSE) + 전체 평균선 ± 표준편차 밴드
  B 요약 수치 — N개 런 평균 ± 표준편차
  C 런별 수직오차 bias / 정밀도 std
  D 마커별 수평오차를 런에 걸쳐 비교 — 특정 위치가 계속 불리한지 확인

준비:
  비행이 끝날 때마다  ros2 run drone_localization archive_run

사용법:
  python3 plot_runs.py                    # 어디서든 바로 실행 가능
  ros2 run drone_localization plot_runs           # (빌드했다면 이것도 가능)
  #   --runs N       비교할 최근 런 개수 (기본 5)
  #   --auto_align   처리지연 보정 후 평가
  #   --save_only    창 없이 PNG만 저장
"""

import argparse
import os
import re
import sys

# ROS 노드가 아니라 오프라인 분석 스크립트이므로 `python3 plot_runs.py` 로도
# 바로 돌아가야 한다. 직접 실행하면 형제 패키지를 못 찾으므로, 소스 트리
# 위치를 스스로 계산해 sys.path에 넣는다. (colcon 설치본으로 실행할 때는
# 이미 경로가 잡혀 있어 아래 삽입이 무해하게 무시된다.)
_HERE = os.path.dirname(os.path.abspath(__file__))
for _p in (os.path.dirname(_HERE),
           os.path.join(os.path.dirname(os.path.dirname(_HERE)),
                        'drone_mission')):
    if os.path.isdir(_p) and _p not in sys.path:
        sys.path.insert(0, _p)

import numpy as np
from matplotlib.transforms import blended_transform_factory

from drone_localization.evaluate_accuracy import (
    _align, _find_lag, _load_csv, _load_markers, _segment_visits,
    _smoothed_speed,
)
from drone_localization.plot_accuracy import (
    INK_MUTED, INK_PRIMARY, INK_SECONDARY, SERIES_1, SURFACE,
    THRESHOLD_HIGH, _setup_font, _style,
)

# 계열이 하나뿐인 그림이라 강조색도 하나만 쓴다.
ACCENT = SERIES_1

MIN_EST_SAMPLES = 10


def _find_runs(base, limit):
    """보관된 런 디렉토리를 오래된 것 → 최근 것 순으로 찾아 최근 limit개 반환.

    `_` 나 `.` 로 시작하는 이름은 건너뛴다 — 예전 설정으로 찍은 보관본
    (_old 등)이 섞이면 서로 다른 센서/파라미터의 수치를 같은 축에 놓고
    비교하게 되어 결과가 무의미해진다.

    정렬은 runNNN이면 번호순, 사용자가 --name 으로 붙인 이름이면 수정시각
    순으로 하고, 이름 있는 쪽을 항상 뒤(최근)에 놓는다.
    """
    numbered, named = [], []
    for name in sorted(os.listdir(base)):
        if name.startswith(('_', '.')):
            continue
        path = os.path.join(base, name)
        if not os.path.isdir(path):
            continue
        if not os.path.isfile(os.path.join(path, 'estimated_path.csv')):
            continue
        m = re.fullmatch(r'run(\d+)', name)
        if m:
            numbered.append((int(m.group(1)), name, path))
        else:
            named.append((os.path.getmtime(path), name, path))
    numbered.sort(key=lambda r: r[0])
    named.sort(key=lambda r: r[0])
    ordered = [(n, p) for _, n, p in numbered] + [(n, p) for _, n, p in named]
    return ordered[-limit:]


def _metrics_for_run(path, auto_align, speed_threshold, gate_radius,
                     min_duration, merge_gap, markers):
    """런 하나의 지표를 계산. 데이터가 부실하면 None."""
    actual = _load_csv(os.path.join(path, 'actual_path.csv'))
    estimate = _load_csv(os.path.join(path, 'estimated_path.csv'))
    if actual is None or estimate is None or len(estimate[0]) < MIN_EST_SAMPLES:
        return None

    lag = _find_lag(actual, estimate)
    al = _align(actual, estimate, lag if auto_align else 0.0)
    if al is None:
        return None
    te, xe, ye, ze, xa, ya, za = al

    exy = np.hypot(xe - xa, ye - ya)
    ez = ze - za
    hover = _smoothed_speed(te, xa, ya) <= speed_threshold

    per_marker = {}
    wp_xy, wp_z = [], []
    if markers is not None:
        mids = list(markers.keys())
        mxy = np.array([markers[m] for m in mids])
        d = np.linalg.norm(np.stack([xa, ya], axis=1)[:, None, :]
                           - mxy[None, :, :], axis=2)
        ni = np.argmin(d, axis=1)
        near = d[np.arange(len(te)), ni] <= gate_radius
        nearest = np.array([mids[i] for i in ni])
        for v in _segment_visits(te, near, hover, nearest,
                                 min_duration, merge_gap):
            k = v['idxs']
            pe = np.median(np.stack([xe[k], ye[k], ze[k]], axis=1), axis=0)
            pa = np.median(np.stack([xa[k], ya[k], za[k]], axis=1), axis=0)
            e_xy = float(np.hypot(pe[0] - pa[0], pe[1] - pa[1]))
            per_marker.setdefault(v['marker_id'], []).append(e_xy)
            wp_xy.append(e_xy)
            wp_z.append(float(pe[2] - pa[2]))

    if not wp_xy:
        return None
    wp_xy = np.array(wp_xy)
    wp_z = np.array(wp_z)
    return {
        'wp_errors': wp_xy.tolist(),
        'n_visits': len(wp_xy),
        'wp_mean': float(wp_xy.mean()),
        'wp_rmse': float(np.sqrt((wp_xy ** 2).mean())),
        'wp_max': float(wp_xy.max()),
        'z_bias': float(wp_z.mean()),
        'z_std': float(wp_z.std()),
        'hover_mean': float(exy[hover].mean()) if hover.sum() else float('nan'),
        'per_marker': {k: float(np.mean(v)) for k, v in per_marker.items()},
        # 마커별 원시 방문 오차 (런 전체를 합쳐 집계할 때 필요)
        'per_marker_raw': {k: list(v) for k, v in per_marker.items()},
        'lag': lag,
    }



def _value_table(ax, xs, rows, row_label_x=-0.075):
    """축 아래에 값 표를 붙인다.

    막대 안이나 수염 옆에 숫자를 넣으면 막대 높이·여백에 따라 위치가
    들쭉날쭉해지고 서로 겹친다. 표로 내려 세로 정렬을 맞추면 값끼리
    비교하기도 쉽다. x는 데이터 좌표, y는 축 비율을 섞어 쓴다."""
    tr = blended_transform_factory(ax.transData, ax.transAxes)
    for r, (label, values, fmt) in enumerate(rows):
        y = -0.145 - r * 0.085
        ax.text(row_label_x, y, label, transform=ax.transAxes,
                ha='right', va='center', fontsize=8.5, color=INK_MUTED)
        for x, v in zip(xs, values):
            ax.text(x, y, fmt.format(v), transform=tr,
                    ha='center', va='center', fontsize=8.5,
                    color=INK_SECONDARY)

# ---------------------------------------------------------------------------
# 패널 — 색은 ACCENT 한 계열만 쓴다. 계열이 하나뿐이면 색으로 구분할 대상이
# 없으므로, 여러 색을 쓰는 건 정보가 아니라 장식이 된다. 나머지는 전부
# ink 톤으로 낮춰 데이터가 앞에 오게 한다.
# ---------------------------------------------------------------------------
def _panel_waypoints(ax, names, runs):
    """막대=런 평균, 수염=그 런의 최댓값까지.

    평균만 그리면 "가끔 한 경로점이 크게 튄다"는 이 시스템의 실제 실패
    양상이 보이지 않고, 점만 뿌리면 런 사이 비교가 어렵다. 막대로 수준을
    비교하고 수염으로 최악을 함께 보여준다."""
    xs = np.arange(len(runs))
    means = np.array([np.mean(r['wp_errors']) for r in runs])
    maxes = np.array([np.max(r['wp_errors']) for r in runs])

    ax.bar(xs, means, width=0.5, color=ACCENT, zorder=3)
    ax.errorbar(xs, means, yerr=[np.zeros(len(runs)), maxes - means],
                fmt='none', ecolor=INK_SECONDARY, elinewidth=1.1, capsize=5,
                zorder=4)
    ax.axhline(THRESHOLD_HIGH, color=INK_MUTED, lw=1.0, ls='--', zorder=2)
    ax.text(len(runs) - 0.45, THRESHOLD_HIGH - 0.015,
            f'채점기준 "상" < {THRESHOLD_HIGH}m', ha='right', va='top',
            fontsize=8.5, color=INK_MUTED)

    ax.set_xticks(xs)
    ax.set_xticklabels(names, fontsize=9, color=INK_SECONDARY)
    ax.set_xlim(-0.55, len(runs) - 0.45)
    # 축은 반드시 최댓값을 포함시킨다. 기준선(0.5m)에 맞춰 자르면 정작
    # 가장 중요한 이상치가 화면 밖으로 사라진다.
    ax.set_ylim(0, max(maxes.max(), THRESHOLD_HIGH) * 1.18)
    ax.set_ylabel('수평오차 [m]', fontsize=9.5, color=INK_SECONDARY)
    ax.set_title('경로점 수평오차', fontsize=11.5, color=INK_PRIMARY,
                 loc='left', pad=12)
    _style(ax)
    _value_table(ax, xs, [('평균', means, '{:.3f}'),
                          ('최대', maxes, '{:.2f}')])


def _panel_markers(ax, runs):
    """QR 마커별 호버 정확도 — 모든 런의 방문을 마커 단위로 합쳐 집계.

    런별 패널은 "언제 나빴나"를 보여주지만, 특정 마커가 구조적으로
    불리한지(LiDAR에서 멀거나 시야가 겹치는 위치인지)는 마커 축으로
    묶어야 보인다."""
    ids = sorted({m for r in runs for m in r['per_marker_raw']})
    vals = [np.array([e for r in runs
                      for e in r['per_marker_raw'].get(m, [])]) for m in ids]
    xs = np.arange(len(ids))
    means = np.array([v.mean() for v in vals])
    maxes = np.array([v.max() for v in vals])

    ax.bar(xs, means, width=0.5, color=ACCENT, zorder=3)
    ax.errorbar(xs, means, yerr=[np.zeros(len(ids)), maxes - means],
                fmt='none', ecolor=INK_SECONDARY, elinewidth=1.1, capsize=5,
                zorder=4)
    ax.axhline(THRESHOLD_HIGH, color=INK_MUTED, lw=1.0, ls='--', zorder=2)
    ax.text(len(ids) - 0.45, THRESHOLD_HIGH - 0.015,
            f'채점기준 "상" < {THRESHOLD_HIGH}m', ha='right', va='top',
            fontsize=8.5, color=INK_MUTED)

    ax.set_xticks(xs)
    ax.set_xticklabels([f'id{m}' for m in ids], fontsize=9.5,
                       color=INK_SECONDARY)
    ax.set_xlim(-0.55, len(ids) - 0.45)
    ax.set_ylim(0, max(maxes.max(), THRESHOLD_HIGH) * 1.18)
    ax.set_ylabel('수평오차 [m]', fontsize=9.5, color=INK_SECONDARY)
    ax.set_title('QR 마커별 호버 정확도', fontsize=11.5, color=INK_PRIMARY,
                 loc='left', pad=12)
    _style(ax)
    _value_table(ax, xs, [('평균', means, '{:.3f}'),
                          ('최대', maxes, '{:.2f}'),
                          ('방문', [len(v) for v in vals], '{:d}회')])


def _panel_z(ax, names, runs):
    """왼쪽 패널과 같은 막대 형식으로 통일 — 막대=bias(계통), 수염=±std(정밀도)."""
    bias = np.array([r['z_bias'] for r in runs])
    std = np.array([r['z_std'] for r in runs])
    xs = np.arange(len(runs))

    ax.bar(xs, bias, width=0.5, color=ACCENT, zorder=3)
    ax.errorbar(xs, bias, yerr=std, fmt='none', ecolor=INK_SECONDARY,
                elinewidth=1.1, capsize=5, zorder=4)
    m = bias.mean()
    ax.axhline(m, color=INK_MUTED, lw=1.0, ls='--', zorder=2)
    ax.text(len(runs) - 0.45, m + 0.005, f'평균 +{m:.3f}m (센트로이드 정의차)',
            ha='right', va='bottom', fontsize=8.5, color=INK_MUTED)

    ax.set_xticks(xs)
    ax.set_xticklabels(names, fontsize=9, color=INK_SECONDARY)
    ax.set_xlim(-0.55, len(runs) - 0.45)
    ax.set_ylim(0, (bias + std).max() * 1.30)
    ax.set_ylabel('수직오차 [m]', fontsize=9.5, color=INK_SECONDARY)
    ax.set_title('수직오차', fontsize=11.5, color=INK_PRIMARY,
                 loc='left', pad=12)
    _style(ax)
    _value_table(ax, xs, [('bias', bias, '{:+.3f}'),
                          ('std', std, '{:.3f}')])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--output_dir',
                    default=os.path.expanduser('~/drone_project/flight_logs'))
    ap.add_argument('--runs', type=int, default=5)
    ap.add_argument('--auto_align', action='store_true')
    ap.add_argument('--speed_threshold', type=float, default=0.25)
    ap.add_argument('--gate_radius', type=float, default=4.5)
    ap.add_argument('--min_duration', type=float, default=0.4)
    ap.add_argument('--merge_gap', type=float, default=1.0)
    ap.add_argument('--save_only', action='store_true')
    args, _ = ap.parse_known_args()

    base = args.output_dir
    if not os.path.isdir(base):
        print(f'로그 디렉토리가 없습니다: {base}')
        return

    markers = _load_markers()
    found = _find_runs(base, args.runs)
    if not found:
        print(f'{base} 에 runNNN 디렉토리가 없습니다.\n'
              '비행이 끝날 때마다 다음을 실행해 보관하세요:\n'
              '  ros2 run drone_localization archive_run')
        return

    names, runs = [], []
    for name, path in found:
        m = _metrics_for_run(path, args.auto_align, args.speed_threshold,
                             args.gate_radius, args.min_duration,
                             args.merge_gap, markers)
        if m is None:
            print(f'  건너뜀: {name} (추정 표본 부족 또는 호버 구간 없음)')
            continue
        names.append(name)
        runs.append(m)

    if not runs:
        print('비교할 수 있는 런이 없습니다. 트래커가 동작한 상태로 '
              '비행하고 archive_run 으로 보관했는지 확인하세요.')
        return

    print(f'{len(runs)}개 런 비교: {", ".join(names)}')
    for n, r in zip(names, runs):
        print(f'  {n}: 경로점 {r["n_visits"]}개  수평 평균 {r["wp_mean"]:.3f} / '
              f'RMSE {r["wp_rmse"]:.3f}  Z bias {r["z_bias"]:+.3f} '
              f'std {r["z_std"]:.3f}')

    if len(runs) == 1:
        print('\n런이 1개뿐이라 재현성(런 간 편차)은 0으로 표시됩니다. '
              '여러 번 비행해 보관하면 의미 있는 비교가 됩니다.')

    _setup_font()
    import matplotlib
    if args.save_only:
        matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    fig = plt.figure(figsize=(16.5, 5.4), facecolor=SURFACE)
    gs = fig.add_gridspec(1, 3, wspace=0.24,
                          left=0.06, right=0.98, top=0.74, bottom=0.24)

    _panel_waypoints(fig.add_subplot(gs[0, 0]), names, runs)
    _panel_markers(fig.add_subplot(gs[0, 1]), runs)
    _panel_z(fig.add_subplot(gs[0, 2]), names, runs)

    wp = np.array([e for r in runs for e in r['wp_errors']])
    fig.text(0.05, 0.925, '측위 정확도 — 반복 비행 비교', fontsize=15,
             color=INK_PRIMARY, weight='bold')
    fig.text(0.05, 0.862,
             f'{len(runs)}개 런 · 경로점 {len(wp)}개 · 기준 PX4 실측 · '
             f'수평 평균 {wp.mean():.3f}m (RMSE {np.sqrt((wp**2).mean()):.3f}m)  ·  막대=평균, 수염=최댓값(수직은 ±std)',
             fontsize=9.5, color=INK_MUTED)

    out_png = os.path.join(base, 'accuracy_runs.png')
    fig.savefig(out_png, dpi=150, facecolor=SURFACE)
    print(f'\n그림 저장: {out_png}')

    if not args.save_only:
        try:
            plt.show()
        except Exception as e:
            print(f'창을 못 열었습니다({e}). PNG를 열어보세요: {out_png}')


if __name__ == '__main__':
    main()
