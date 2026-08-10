#!/usr/bin/env python3
"""
DBSCAN이 잡은 드론 클러스터의 모양 확인.

트래커를 cluster_dump_path 파라미터와 함께 돌리면 선택된 클러스터의 원본
점들이 npz로 남는다. 이 도구는 그 점들을 센트로이드 기준 상대좌표로 모아
"실제로 드론이 어떤 점 뭉치로 보이는가"를 그린다.

Z bias(+0.24m)의 원인을 눈으로 확인하는 것이 주 용도다. 센트로이드가
기체 어디쯤에 찍히는지, 점들이 위/아래 어느 쪽에 몰리는지가 보인다.

패널:
  A 측면도(XZ) — 점들의 수직 분포. Z bias의 직접적 근거
  B 평면도(XY) — 수평 분포와 대칭성
  C 크기 추이 — 시간에 따른 점 개수 / 수직 범위
  D 요약 수치

사용법:
  # 1) 비행할 때 덤프를 켠다
  ros2 launch drone_localization lidar_localization.launch.py \
      params_file:=lidar_localization_os1.yaml \
      cluster_dump_path:=~/drone_project/flight_logs/clusters.npz
  # 2) 비행 후
  python3 plot_cluster_shape.py --dump ~/drone_project/flight_logs/clusters.npz
"""

import argparse
import os

import numpy as np

SURFACE = '#fcfcfb'
INK_PRIMARY = '#0b0b0b'
INK_SECONDARY = '#52514e'
INK_MUTED = '#898781'
GRIDLINE = '#e1e0d9'
BASELINE = '#c3c2b7'
ACCENT = '#2a78d6'
ACCENT_2 = '#eb6834'


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
    for side in ('top', 'right'):
        ax.spines[side].set_visible(False)
    for side in ('left', 'bottom'):
        ax.spines[side].set_color(BASELINE)
        ax.spines[side].set_linewidth(0.8)
    ax.grid(True, color=GRIDLINE, lw=0.6, zorder=0)
    ax.set_axisbelow(True)
    ax.tick_params(colors=INK_MUTED, labelsize=8.5, length=0)


def _load(path):
    d = np.load(os.path.expanduser(path))
    return d['times'], d['points'], d['offsets'], d['centroids']


def _relative(points, offsets, centroids):
    """각 프레임의 점을 그 프레임 센트로이드 기준 상대좌표로 변환.

    절대좌표 그대로 겹쳐 그리면 비행 궤적이 겹쳐 보일 뿐 모양은 안 보인다.
    센트로이드를 원점에 맞춰야 여러 프레임의 모양을 포개서 볼 수 있다."""
    rel = np.empty_like(points)
    for i in range(len(offsets) - 1):
        a, b = offsets[i], offsets[i + 1]
        rel[a:b] = points[a:b] - centroids[i]
    return rel


def _panel_side(ax, rel, z_bias=None):
    ax.scatter(rel[:, 0], rel[:, 2], s=3, color=ACCENT, alpha=0.18,
               edgecolors='none', zorder=3)
    ax.axhline(0, color=ACCENT_2, lw=1.4, zorder=4)
    ax.axvline(0, color=BASELINE, lw=0.8, zorder=2)
    ax.text(ax.get_xlim()[1], 0.012, '센트로이드(추정 위치)', ha='right',
            va='bottom', fontsize=8.5, color=ACCENT_2)

    # PX4가 보고하는 기준점(base_link)의 상대 위치. 센트로이드가 그보다
    # z_bias 만큼 위에 찍히므로, 기준점은 -z_bias 에 놓인다. 이 선이 점
    # 뭉치의 어디를 지나는지가 Z bias의 정체를 그대로 보여준다.
    if z_bias is not None:
        ax.axhline(-z_bias, color=INK_PRIMARY, lw=1.2, ls='--', zorder=4)
        ax.text(ax.get_xlim()[0], -z_bias - 0.012,
                f'PX4 기준점(base_link)  Δ{z_bias:+.3f}m', ha='left', va='top',
                fontsize=8.5, color=INK_PRIMARY)

    # 수직 분포의 사분위를 함께 표시 — 위/아래 어느 쪽으로 치우쳤는지
    q = np.percentile(rel[:, 2], [25, 50, 75])
    ax.axhspan(q[0], q[2], color=ACCENT, alpha=0.10, zorder=1)
    ax.set_xlabel('센트로이드 기준 X [m]', fontsize=9, color=INK_SECONDARY)
    ax.set_ylabel('센트로이드 기준 Z [m]', fontsize=9, color=INK_SECONDARY)
    ax.set_title('A. 측면도 — 점들의 수직 분포', fontsize=11,
                 color=INK_PRIMARY, loc='left', pad=10)
    ax.set_aspect('equal')
    _style(ax)


def _panel_top(ax, rel):
    ax.scatter(rel[:, 0], rel[:, 1], s=3, color=ACCENT, alpha=0.18,
               edgecolors='none', zorder=3)
    ax.plot(0, 0, marker='+', ms=14, mew=2, color=ACCENT_2, zorder=4)
    ax.set_xlabel('센트로이드 기준 X [m]', fontsize=9, color=INK_SECONDARY)
    ax.set_ylabel('센트로이드 기준 Y [m]', fontsize=9, color=INK_SECONDARY)
    ax.set_title('B. 평면도 — 수평 분포', fontsize=11,
                 color=INK_PRIMARY, loc='left', pad=10)
    ax.set_aspect('equal')
    _style(ax)


def _panel_size(ax, times, offsets, points):
    counts = np.diff(offsets)
    t = times - times[0]
    zspan = np.array([np.ptp(points[offsets[i]:offsets[i + 1], 2])
                      for i in range(len(counts))])
    ax.plot(t, counts, lw=1.2, color=ACCENT, label='점 개수', zorder=3)
    ax.set_xlabel('시간 [s]', fontsize=9, color=INK_SECONDARY)
    ax.set_ylabel('클러스터 점 개수', fontsize=9, color=ACCENT)
    _style(ax)

    ax2 = ax.twinx()
    ax2.plot(t, zspan, lw=1.0, color=ACCENT_2, alpha=0.75,
             label='수직 범위', zorder=3)
    ax2.set_ylabel('수직 범위 [m]', fontsize=9, color=ACCENT_2)
    ax2.tick_params(colors=INK_MUTED, labelsize=8.5, length=0)
    for side in ('top', 'left'):
        ax2.spines[side].set_visible(False)
    ax2.spines['right'].set_color(BASELINE)
    ax.set_title('C. 시간에 따른 클러스터 크기', fontsize=11,
                 color=INK_PRIMARY, loc='left', pad=10)


def _panel_stats(ax, rel, offsets, points, airframe=(0.45, 0.45, 0.25)):
    ax.axis('off')
    counts = np.diff(offsets)
    z = rel[:, 2]
    zq = np.percentile(z, [5, 95])
    ext = [np.percentile([np.ptp(points[offsets[i]:offsets[i + 1], k])
                          for i in range(len(counts))], 50) for k in range(3)]
    cap = [100 * ext[k] / airframe[k] for k in range(3)]

    # 점의 위/아래 개수 비율은 쓰지 않는다 — 센트로이드가 median이라
    # 정의상 항상 50:50이 나오는 항등식이라 아무 정보가 없다. 대신
    # 평균-중앙 차이와 꼬리 길이 비로 비대칭을 잰다.
    lines = [
        ('프레임 수', f'{len(counts)}'),
        ('점 개수 (중앙/최소/최대)',
         f'{int(np.median(counts))} / {counts.min()} / {counts.max()}'),
        ('', ''),
        ('클러스터 크기 (중앙)', ''),
        ('  X · Y · Z',
         f'{ext[0]:.2f} × {ext[1]:.2f} × {ext[2]:.2f} m'),
        ('  기체 제원 대비 포착률',
         f'{cap[0]:.0f}% · {cap[1]:.0f}% · {cap[2]:.0f}%'),
        ('', ''),
        ('수직 분포 비대칭', ''),
        ('  평균 − 중앙', f'{z.mean() - np.median(z):+.3f} m'),
        ('  아래꼬리(5%) / 위꼬리(95%)',
         f'{zq[0]:+.3f} / {zq[1]:+.3f} m'),
        ('  아래가 위보다',
         f'{abs(zq[0]) / max(abs(zq[1]), 1e-9):.1f}배 김'),
    ]
    for i, (k, v) in enumerate(lines):
        y = 0.94 - i * 0.093
        if not k:
            continue
        ax.text(0.02, y, k, fontsize=9.5, color=INK_SECONDARY,
                transform=ax.transAxes)
        ax.text(0.98, y, v, fontsize=9.5, color=INK_PRIMARY, ha='right',
                weight='bold' if v else 'normal', transform=ax.transAxes)
    ax.set_title('D. 요약', fontsize=11, color=INK_PRIMARY, loc='left', pad=10)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--dump',
                    default=os.path.expanduser(
                        '~/drone_project/flight_logs/clusters.npz'))
    ap.add_argument('--z_bias', type=float, default=None,
                    help='PX4 대비 Z bias [m]. 주면 측면도에 기준선을 그린다 '
                         '(evaluate_accuracy 출력값).')
    ap.add_argument('--save_only', action='store_true')
    args, _ = ap.parse_known_args()

    if not os.path.isfile(os.path.expanduser(args.dump)):
        print(f'덤프 파일이 없습니다: {args.dump}\n'
              '트래커를 cluster_dump_path 파라미터와 함께 실행한 뒤 비행하세요.')
        return

    times, points, offsets, centroids = _load(args.dump)
    rel = _relative(points, offsets, centroids)

    _setup_font()
    import matplotlib
    if args.save_only:
        matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    fig = plt.figure(figsize=(13.5, 8.6), facecolor=SURFACE)
    gs = fig.add_gridspec(2, 2, hspace=0.38, wspace=0.24,
                          left=0.07, right=0.95, top=0.87, bottom=0.08)
    _panel_side(fig.add_subplot(gs[0, 0]), rel, args.z_bias)
    _panel_top(fig.add_subplot(gs[0, 1]), rel)
    _panel_size(fig.add_subplot(gs[1, 0]), times, offsets, points)
    _panel_stats(fig.add_subplot(gs[1, 1]), rel, offsets, points)

    fig.text(0.07, 0.945, 'DBSCAN 클러스터 모양', fontsize=16,
             color=INK_PRIMARY, weight='bold')
    fig.text(0.07, 0.905,
             f'선택된 드론 클러스터 {len(times)}프레임 · 점 {len(points)}개 · '
             '센트로이드 기준 상대좌표',
             fontsize=9.5, color=INK_MUTED)
    # 0 축을 따라 보이는 십자 무늬는 물리 현상이 아니다: median은 홀수 표본에서
    # 실제 데이터값을 그대로 돌려주므로 축마다 점 하나가 정확히 0에 놓인다.
    fig.text(0.07, 0.876,
             '※ 0축의 십자 무늬는 median이 홀수 표본에서 실제 점 값을 '
             '반환해 생기는 통계 아티팩트',
             fontsize=8.5, color=INK_MUTED)

    out = os.path.join(os.path.dirname(os.path.expanduser(args.dump)),
                       'cluster_shape.png')
    fig.savefig(out, dpi=150, facecolor=SURFACE)
    print(f'그림 저장: {out}')
    if not args.save_only:
        try:
            plt.show()
        except Exception as e:
            print(f'창을 못 열었습니다({e}). PNG를 열어보세요: {out}')


if __name__ == '__main__':
    main()
