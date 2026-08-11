#!/usr/bin/env python3
"""
측위 정확도 지표 시각화 — 보고/발표용 한 장 요약.

evaluate_accuracy.py가 콘솔로 내는 수치를 그림 한 장으로 정리한다.
숫자 자체는 같은 계산을 쓰므로 두 도구의 결과는 항상 일치한다.

패널 구성:
  A 전체 측위 경로 비교 (3D) — 오차가 어느 구간에서 생겼나
  B 경로점별 수평오차 — 방문마다 얼마나 어긋났나
  C 경로점별 수직오차 + bias/std 분해 — 계통오차와 정밀도 분리

핵심 수치는 별도 패널 없이 헤더에 둔다(전에는 패널 하나를 통째로 썼는데
헤더와 내용이 겹쳐 자리만 차지했다).

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

# 필드 사양 (generate_competition_map.py 와 동일하게 유지)
AREA_X, AREA_Y = 30.0, 20.0
N_CELLS_X, N_CELLS_Y = 7, 5


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

    # 비행시간은 실측 로그 전체가 아니라 "떠 있던 구간"으로 잰다. 기록을
    # 일찍 켜거나 늦게 끄면 지상 대기 시간이 통째로 섞이기 때문.
    ta_all, _, _, za_all = actual
    air = za_all > 0.5
    flight_sec = float(ta_all[air][-1] - ta_all[air][0]) if air.any() else 0.0

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
            'ex': xe - xa, 'ey': ye - ya,
            'xa': xa, 'ya': ya, 'za': za, 'xe': xe, 'ye': ye, 'ze': ze,
            'markers': markers, 'flight_sec': flight_sec,
            'visits': visits, 'lag': lag, 'auto_align': auto_align}


def _panel_waypoint_xy(ax, visits):
    labels = [v['label'] for v in visits]
    vals = np.array([v['err_xy'] for v in visits])
    xs = np.arange(len(vals))

    ax.bar(xs, vals, width=0.55, color=SERIES_1, zorder=3)
    for x, v in zip(xs, vals):
        ax.text(x, v + 0.012, f'{v:.3f}', ha='center', va='bottom',
                fontsize=8, color=INK_SECONDARY, zorder=4)

    mean = float(vals.mean())
    ax.axhline(mean, color=SERIES_1, lw=0.9, ls=':', alpha=0.8, zorder=2)
    # 막대와 겹치지 않도록 기준선 라벨은 오른쪽 끝 위에 둔다.
    ax.text(len(vals) - 0.4, mean + 0.004, f'평균 {mean:.3f}m',
            ha='right', va='bottom', fontsize=8.5, color=SERIES_1)

    ax.set_xticks(xs)
    ax.set_xticklabels(labels, fontsize=8.5, color=INK_SECONDARY)
    # 축은 실제 값에 맞춘다. 채점기준(0.5m)에 맞춰 잡으면 막대가 아래
    # 1/3에만 몰려 값끼리 비교가 안 된다.
    ax.set_ylim(0, vals.max() * 1.28)
    ax.set_ylabel('수평오차 [m]', fontsize=9, color=INK_SECONDARY)
    ax.set_title('B. 경로점별 수평오차', fontsize=11, color=INK_PRIMARY,
                 loc='left', pad=10)
    _style(ax)


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


def _panel_track3d(ax, d, z_exag=3.0):
    """전체 궤적 3D 비교 — 실측 위에 추정을 겹친다.

    통계는 "얼마나 틀렸나"만 말해주지, 그 오차가 어디서 어떻게 생겼는지는
    못 보여준다. 궤적을 겹쳐 그리면 코너·직선·호버 중 어디가 문제인지
    한눈에 잡힌다. 평면도로는 고도 성분이 사라지므로 3D로 그린다.

    z_exag: 필드가 30x20m인데 고도는 2m뿐이라 실척으로는 Z가 납작해져
    고도 변화가 안 보인다. Z축만 늘려 프로파일이 읽히게 한다."""
    zf = 0.0
    for i in range(N_CELLS_X + 1):
        x = i * AREA_X / N_CELLS_X
        ax.plot([x, x], [0, AREA_Y], [zf, zf], color=GRIDLINE, lw=0.5)
    for j in range(N_CELLS_Y + 1):
        y = j * AREA_Y / N_CELLS_Y
        ax.plot([0, AREA_X], [y, y], [zf, zf], color=GRIDLINE, lw=0.5)
    ax.plot([0, AREA_X, AREA_X, 0, 0], [0, 0, AREA_Y, AREA_Y, 0], [zf] * 5,
            color=BASELINE, lw=1.1)

    ax.plot(d['xa'], d['ya'], d['za'], '-', color=INK_SECONDARY, lw=2.4,
            alpha=0.5, label='실측 (PX4)')
    ax.plot(d['xe'], d['ye'], d['ze'], '-', color=SERIES_2, lw=1.1,
            label='LiDAR 추정')

    markers = d.get('markers')
    if markers:
        mx = [p[0] for p in markers.values()]
        my = [p[1] for p in markers.values()]
        ax.scatter(mx, my, [zf] * len(mx), marker='s', s=42,
                   facecolors=INK_SECONDARY, edgecolors=SURFACE,
                   linewidths=1.0, depthshade=False)
        for mid, (x, y) in markers.items():
            ax.text(x, y, zf + 0.22, f'id{mid}', fontsize=8,
                    ha='center', color=INK_SECONDARY)
    ax.scatter([0], [0], [zf], marker='o', s=52, facecolors='none',
               edgecolors=INK_MUTED, linewidths=1.3, depthshade=False)

    for axis in (ax.xaxis, ax.yaxis, ax.zaxis):
        axis.pane.fill = False
        axis.pane.set_edgecolor(GRIDLINE)
        axis.line.set_color(BASELINE)
        axis._axinfo['grid'].update(color=GRIDLINE, linewidth=0.5)
        axis.set_tick_params(colors=INK_MUTED, labelsize=7.5)
    ax.set_facecolor(SURFACE)

    zmax = max(float(np.max(d['za'])), float(np.max(d['ze'])), 2.5)
    ax.set_xlim(-1, AREA_X + 1)
    ax.set_ylim(-2, AREA_Y + 2)
    ax.set_zlim(0, zmax * 1.1)
    try:
        ax.set_box_aspect((AREA_X + 2, AREA_Y + 4, zmax * 1.1 * z_exag))
    except Exception:
        pass  # 구버전 matplotlib
    ax.set_xlabel('X [m]', fontsize=8.5, color=INK_SECONDARY, labelpad=-2)
    ax.set_ylabel('Y [m]', fontsize=8.5, color=INK_SECONDARY, labelpad=-2)
    ax.set_zlabel('고도 [m]', fontsize=8.5, color=INK_SECONDARY, labelpad=-4)
    ax.view_init(elev=26, azim=-62)
    ax.set_title(f'A. 전체 측위 경로 비교 (3D, Z {z_exag:.0f}배 강조)',
                 fontsize=11, color=INK_PRIMARY, loc='left', pad=2)
    leg = ax.legend(fontsize=8.5, frameon=False, loc='upper right')
    for t in leg.get_texts():
        t.set_color(INK_SECONDARY)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--output_dir',
                    default=os.path.expanduser('~/drone_project/flight_logs'))
    ap.add_argument('--auto_align', action='store_true')
    ap.add_argument('--speed_threshold', type=float, default=0.25)
    ap.add_argument('--gate_radius', type=float, default=4.5)
    ap.add_argument('--min_duration', type=float, default=0.4)
    ap.add_argument('--merge_gap', type=float, default=1.0)
    ap.add_argument('--z_exag', type=float, default=3.0,
                    help='3D 궤적의 Z축 표시 배율 (1.0=실제 비율)')
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
    # 이 PC에는 apt/pip matplotlib이 섞여 있어 mpl_toolkits가 옛 버전으로
    # 잡히면 Axes3D import가 깨진다. plot_flight_3d가 쓰는 교정을 그대로 사용.
    from drone_localization.plot_flight_3d import _fix_mpl_toolkits
    _fix_mpl_toolkits()
    import matplotlib
    if args.save_only:
        matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D  # noqa: F401  (3d projection 등록)

    # 수치 전용 패널은 헤더와 내용이 겹쳐 자리만 차지했다. 숫자는 헤더로
    # 올리고, 남은 공간을 실제 그래프에 준다.
    #   위  경로점 수평오차 (채점기준선까지 보여야 해서 가로로 길게)
    #   아래 수직오차 | 누적분포
    # 수치 전용 패널은 헤더와 내용이 겹쳐 자리만 차지했다. 숫자는 헤더로
    # 올리고, 남은 공간을 실제 그래프에 준다. 헤더가 3층(제목/부제/수치)이라
    # top을 넉넉히 내려야 A 패널 제목과 부딪히지 않는다.
    # 3D 궤적은 왼쪽 세로 칸에, 막대 둘은 오른쪽에 쌓는다.
    fig = plt.figure(figsize=(14.5, 8.2), facecolor=SURFACE)
    gs = fig.add_gridspec(2, 2, width_ratios=[1.1, 1],
                          hspace=0.38, wspace=0.10,
                          left=0.02, right=0.975, top=0.775, bottom=0.075)

    _panel_track3d(fig.add_subplot(gs[:, 0], projection='3d'), d, args.z_exag)
    _panel_waypoint_xy(fig.add_subplot(gs[0, 1]), d['visits'])
    _panel_waypoint_z(fig.add_subplot(gs[1, 1]), d['visits'])

    vx = np.array([v['err_xy'] for v in d['visits']])
    vz = np.array([v['err_z'] for v in d['visits']])
    m, s = divmod(int(round(d['flight_sec'])), 60)
    flight_txt = f'{m}분 {s}초' if m else f'{s}초'

    fig.text(0.065, 0.965, '지상 LiDAR 측위 정확도', fontsize=16,
             color=INK_PRIMARY, weight='bold')
    # 수직 bias는 오차가 아니라 기준점 정의 차이라, 수치 타일이 아니라
    # 부제에 함께 적어 성격을 구분한다.
    fig.text(0.065, 0.933,
             f'기준 PX4 실측 · 비행시간 {flight_txt} · 표본 {len(d["te"])}개 · '
             f'경로점 {len(vx)}개 · 수직 bias {vz.mean():+.3f}m(센트로이드 정의차)',
             fontsize=9.5, color=INK_MUTED)

    # 축별 평균오차. 부호를 살려 적는다 — 절대값으로 뭉개면 "어느 쪽으로
    # 치우쳤나"가 사라지는데, 그게 편향을 읽는 유일한 단서다.
    tiles = [('X 평균오차', f'{d["ex"].mean():+.3f}', SERIES_1),
             ('Y 평균오차', f'{d["ey"].mean():+.3f}', SERIES_1),
             ('Z 평균오차', f'{d["ez"].mean():+.3f}', SERIES_3),
             ('경로점 수평 RMSE', f'{np.sqrt((vx ** 2).mean()):.3f}', SERIES_1)]
    for i, (label, value, color) in enumerate(tiles):
        x = 0.075 + i * 0.235
        fig.text(x, 0.882, label, fontsize=8.5, color=INK_MUTED)
        fig.text(x, 0.837, value, fontsize=20, color=color, weight='bold')
        fig.text(x + 0.070, 0.842, 'm', fontsize=9.5, color=INK_MUTED)
        fig.lines.append(plt.Line2D(
            [x - 0.010, x - 0.010], [0.832, 0.897], transform=fig.transFigure,
            color=color, lw=2.4, solid_capstyle='round'))

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
