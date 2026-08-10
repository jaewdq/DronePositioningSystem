#!/usr/bin/env python3
"""
측위 정확도 지표 시각화 — 보고/발표용 한 장 요약.

evaluate_accuracy.py가 콘솔로 내는 수치를 그림 한 장으로 정리한다.
숫자 자체는 같은 계산을 쓰므로 두 도구의 결과는 항상 일치한다.

패널 구성:
  A 경로점별 수평오차 + 채점기준선 — "기준 대비 얼마나 여유 있나"
  B 핵심 수치 요약 (hero numbers)
  C 경로점별 수직오차 + bias/std 분해 — "계통오차와 정밀도 분리"
  D 구간별 수평오차 누적분포 — "표본의 몇 %가 어느 오차 안에 드나"

사용법:
  python3 plot_accuracy.py                # 어디서든 바로 실행 가능
  ros2 run drone_localization plot_accuracy       # (빌드했다면 이것도 가능)
  # 옵션:
  #   --output_dir DIR   로그 디렉토리 (기본 ~/drone_project/flight_logs)
  #   --auto_align       처리지연 보정 후 평가
  #   --save_only        창 없이 PNG만 저장
"""

import argparse
import os
import sys

# ROS 노드가 아니라 오프라인 분석 스크립트이므로 `python3 plot_accuracy.py` 로도
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

from drone_localization.evaluate_accuracy import (
    _align, _find_lag, _load_csv, _load_markers, _segment_visits,
    _smoothed_speed,
)

# ---------------------------------------------------------------------------
# 팔레트 — dataviz 레퍼런스에서 역할별로 가져온 값 (hex 변형 없음).
# 계열은 categorical 슬롯 1~3만 사용. 씬/크롬 요소는 ink 토큰으로만 그린다.
# ---------------------------------------------------------------------------
SURFACE = '#fcfcfb'
INK_PRIMARY = '#0b0b0b'
INK_SECONDARY = '#52514e'
INK_MUTED = '#898781'
GRIDLINE = '#e1e0d9'
BASELINE = '#c3c2b7'

SERIES_1 = '#2a78d6'   # blue   — 수평오차 / 호버
SERIES_2 = '#eb6834'   # orange — 이동
SERIES_3 = '#1baf7a'   # aqua   — 수직오차

# 채점기준 "상" 임계 (수평/수직 공통)
THRESHOLD_HIGH = 0.5


def _setup_font():
    import matplotlib
    for name in ('Noto Sans CJK JP', 'Noto Sans CJK KR', 'NanumGothic',
                 'Malgun Gothic'):
        try:
            matplotlib.rcParams['font.family'] = name
            matplotlib.rcParams['axes.unicode_minus'] = False
            return name
        except Exception:
            continue
    return None


def _style(ax):
    """격자/축을 뒤로 물리고 데이터가 앞에 오게 한다."""
    ax.set_facecolor(SURFACE)
    for side in ('top', 'right'):
        ax.spines[side].set_visible(False)
    for side in ('left', 'bottom'):
        ax.spines[side].set_color(BASELINE)
        ax.spines[side].set_linewidth(0.8)
    ax.grid(True, axis='y', color=GRIDLINE, lw=0.6, zorder=0)
    ax.set_axisbelow(True)
    ax.tick_params(colors=INK_MUTED, labelsize=8.5, length=0)


def _compute(out_dir, auto_align, speed_threshold, gate_radius,
             min_duration, merge_gap):
    """evaluate_accuracy와 동일한 계산 경로를 그대로 사용."""
    actual = _load_csv(os.path.join(out_dir, 'actual_path.csv'))
    estimate = _load_csv(os.path.join(out_dir, 'estimated_path.csv'))
    if actual is None or estimate is None:
        return None

    lag = _find_lag(actual, estimate)
    al = _align(actual, estimate, lag if auto_align else 0.0)
    if al is None:
        return None
    te, xe, ye, ze, xa, ya, za = al

    exy = np.hypot(xe - xa, ye - ya)
    ez = ze - za
    speed = _smoothed_speed(te, xa, ya)
    hover = speed <= speed_threshold

    visits = []
    markers = _load_markers()
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
            visits.append({
                'label': f'id{v["marker_id"]}\n#{v["visit"]}',
                'err_xy': float(np.hypot(pe[0] - pa[0], pe[1] - pa[1])),
                'err_z': float(pe[2] - pa[2]),
            })

    return {'te': te, 'exy': exy, 'ez': ez, 'hover': hover,
            'visits': visits, 'lag': lag, 'auto_align': auto_align}


def _panel_waypoint_xy(ax, visits):
    labels = [v['label'] for v in visits]
    vals = np.array([v['err_xy'] for v in visits])
    xs = np.arange(len(vals))

    ax.bar(xs, vals, width=0.55, color=SERIES_1, zorder=3)
    for x, v in zip(xs, vals):
        ax.text(x, v + 0.012, f'{v:.3f}', ha='center', va='bottom',
                fontsize=8, color=INK_SECONDARY, zorder=4)

    ax.axhline(THRESHOLD_HIGH, color=INK_MUTED, lw=1.0, ls='--', zorder=2)
    ax.text(len(vals) - 0.4, THRESHOLD_HIGH - 0.022,
            f'채점기준 "상"  < {THRESHOLD_HIGH}m', ha='right', va='top',
            fontsize=8.5, color=INK_SECONDARY)

    mean = float(vals.mean())
    ax.axhline(mean, color=SERIES_1, lw=0.9, ls=':', alpha=0.8, zorder=2)
    # 막대와 겹치지 않도록 기준선 라벨은 오른쪽 끝 위에 둔다.
    ax.text(len(vals) - 0.4, mean + 0.014, f'평균 {mean:.3f}m',
            ha='right', va='bottom', fontsize=8.5, color=SERIES_1)

    ax.set_xticks(xs)
    ax.set_xticklabels(labels, fontsize=8.5, color=INK_SECONDARY)
    ax.set_ylim(0, THRESHOLD_HIGH * 1.12)
    ax.set_ylabel('수평오차 [m]', fontsize=9, color=INK_SECONDARY)
    ax.set_title('A. 경로점별 수평오차', fontsize=11, color=INK_PRIMARY,
                 loc='left', pad=10)
    _style(ax)


def _panel_summary(ax, d):
    ax.axis('off')
    vx = np.array([v['err_xy'] for v in d['visits']])
    vz = np.array([v['err_z'] for v in d['visits']])
    hov = d['exy'][d['hover']]

    # 단위는 라벨에 넣는다. 큰 숫자 옆에 별도 text로 붙이면 폭을 미리 알 수
    # 없어 겹친다.
    tiles = [
        ('경로점 수평 RMSE [m]', f'{np.sqrt((vx ** 2).mean()):.3f}', SERIES_1),
        ('경로점 수평 평균 [m]', f'{vx.mean():.3f}', SERIES_1),
        ('수직 정밀도 std [m]', f'{vz.std():.3f}', SERIES_3),
        ('호버 수평 평균 [m]', f'{hov.mean():.3f}', SERIES_1),
    ]
    for i, (label, value, color) in enumerate(tiles):
        col, row = i % 2, i // 2
        x, y = 0.06 + col * 0.50, 0.70 - row * 0.42
        ax.text(x, y + 0.13, label, fontsize=9, color=INK_SECONDARY,
                transform=ax.transAxes)
        ax.text(x, y - 0.06, value, fontsize=26, color=color, weight='bold',
                transform=ax.transAxes)
        # 값 왼쪽의 얇은 색 막대로 계열을 표시 (텍스트는 ink 유지)
        ax.plot([x - 0.022, x - 0.022], [y - 0.075, y + 0.16],
                color=color, lw=2.5, transform=ax.transAxes,
                clip_on=False, solid_capstyle='round')

    n = len(d['visits'])
    note = (f'경로점 {n}개 · 수직 bias {vz.mean():+.3f}m'
            '(센트로이드 정의차)')
    ax.text(0.04, 0.02, note, fontsize=8.5, color=INK_MUTED,
            transform=ax.transAxes)
    ax.set_title('B. 핵심 수치', fontsize=11, color=INK_PRIMARY,
                 loc='left', pad=10)


def _panel_waypoint_z(ax, visits):
    labels = [v['label'] for v in visits]
    vals = np.array([v['err_z'] for v in visits])
    xs = np.arange(len(vals))
    bias, std = float(vals.mean()), float(vals.std())

    # bias±std 밴드를 먼저 깔아 "정밀도 범위"를 배경으로 보여준다.
    ax.axhspan(bias - std, bias + std, color=SERIES_3, alpha=0.12, zorder=1)
    ax.bar(xs, vals, width=0.55, color=SERIES_3, zorder=3)
    ax.axhline(bias, color=SERIES_3, lw=1.1, zorder=4)
    ax.axhline(0, color=BASELINE, lw=0.9, zorder=2)

    ax.text(len(vals) - 0.4, bias + std + 0.006,
            f'bias {bias:+.3f} ± std {std:.3f}', ha='right', va='bottom',
            fontsize=8.5, color=INK_SECONDARY)

    ax.set_xticks(xs)
    ax.set_xticklabels(labels, fontsize=8.5, color=INK_SECONDARY)
    ax.set_ylabel('수직오차 [m]', fontsize=9, color=INK_SECONDARY)
    ax.set_ylim(0, max(vals.max() * 1.35, bias + std + 0.06))
    ax.set_title('C. 경로점별 수직오차 — 계통(bias) vs 정밀도(std)',
                 fontsize=11, color=INK_PRIMARY, loc='left', pad=10)
    _style(ax)


def _panel_cdf(ax, d):
    def ecdf(v):
        s = np.sort(v)
        return s, np.arange(1, len(s) + 1) / len(s) * 100.0

    for vals, color, name in ((d['exy'][d['hover']], SERIES_1, '호버'),
                              (d['exy'][~d['hover']], SERIES_2, '이동')):
        if len(vals) < 2:
            continue
        x, y = ecdf(vals)
        ax.plot(x, y, color=color, lw=2.0, label=f'{name} (n={len(vals)})',
                zorder=3)
        med = float(np.median(vals))
        ax.plot([med], [50], 'o', ms=6, color=color, mec=SURFACE, mew=1.5,
                zorder=4)

    ax.axvline(THRESHOLD_HIGH, color=INK_MUTED, lw=1.0, ls='--', zorder=2)
    ax.text(THRESHOLD_HIGH - 0.02, 8, f'{THRESHOLD_HIGH}m', ha='right',
            fontsize=8.5, color=INK_SECONDARY)

    ax.set_xlabel('수평오차 [m]', fontsize=9, color=INK_SECONDARY)
    ax.set_ylabel('누적 비율 [%]', fontsize=9, color=INK_SECONDARY)
    ax.set_ylim(0, 102)
    ax.set_xlim(0, max(0.62, float(np.percentile(d['exy'], 99))))
    ax.set_title('D. 수평오차 누적분포 (전체 표본)', fontsize=11,
                 color=INK_PRIMARY, loc='left', pad=10)
    leg = ax.legend(fontsize=8.5, frameon=False, loc='lower right')
    for t in leg.get_texts():
        t.set_color(INK_SECONDARY)
    _style(ax)
    ax.grid(True, axis='x', color=GRIDLINE, lw=0.6, zorder=0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--output_dir',
                    default=os.path.expanduser('~/drone_project/flight_logs'))
    ap.add_argument('--auto_align', action='store_true')
    ap.add_argument('--speed_threshold', type=float, default=0.25)
    ap.add_argument('--gate_radius', type=float, default=4.5)
    ap.add_argument('--min_duration', type=float, default=0.4)
    ap.add_argument('--merge_gap', type=float, default=1.0)
    ap.add_argument('--save_only', action='store_true')
    args, _ = ap.parse_known_args()

    d = _compute(args.output_dir, args.auto_align, args.speed_threshold,
                 args.gate_radius, args.min_duration, args.merge_gap)
    if d is None:
        print(f'로그를 읽지 못했습니다: {args.output_dir}\n'
              'flight_recorder를 켜고 비행한 뒤 다시 실행하세요.')
        return
    if not d['visits']:
        print('경로점(호버 구간)을 찾지 못했습니다.')
        return

    _setup_font()
    import matplotlib
    if args.save_only:
        matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    fig = plt.figure(figsize=(13.5, 8.4), facecolor=SURFACE)
    gs = fig.add_gridspec(2, 2, hspace=0.42, wspace=0.20,
                          left=0.07, right=0.97, top=0.87, bottom=0.09)

    _panel_waypoint_xy(fig.add_subplot(gs[0, 0]), d['visits'])
    _panel_summary(fig.add_subplot(gs[0, 1]), d)
    _panel_waypoint_z(fig.add_subplot(gs[1, 0]), d['visits'])
    _panel_cdf(fig.add_subplot(gs[1, 1]), d)

    align_note = (f'처리지연 {abs(d["lag"]):.2f}s 보정' if d['auto_align']
                  else f'지연 보정 없음 (추정이 실측보다 {abs(d["lag"]):.2f}s 뒤처짐)')
    fig.text(0.07, 0.945, '지상 LiDAR 측위 정확도', fontsize=16,
             color=INK_PRIMARY, weight='bold')
    fig.text(0.07, 0.905,
             f'기준: PX4 실측 위치 · 표본 {len(d["te"])}개 · {align_note}',
             fontsize=9.5, color=INK_MUTED)

    out_png = os.path.join(args.output_dir, 'accuracy_summary.png')
    fig.savefig(out_png, dpi=150, facecolor=SURFACE)
    print(f'그림 저장: {out_png}')

    if not args.save_only:
        try:
            plt.show()
        except Exception as e:
            print(f'창을 못 열었습니다({e}). 저장된 PNG를 열어보세요: {out_png}')


if __name__ == '__main__':
    main()
