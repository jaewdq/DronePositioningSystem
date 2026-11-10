#!/usr/bin/env python3
"""
실내 케이지 논문 재현 — 평가와 그림 (paper_replicate.sh 의 eval / figs 단계).

  python3 paper_eval.py <root>          → <root>/cmp_state/aligned_flights.csv, metrics_mean_std.csv
  python3 paper_eval.py <root> --figs   → <root>/figs/fig4_xy_rmse_aligned.*, fig5_extrinsic_injection_aligned.*

평가 규칙은 논문 flight_logs/paper_1024/scripts/eval_aligned.py 와 같다.
  0.01 s 격자의 평가 시각마다 각 구조의 최신 사후 상태(0.25 s 이내)를 KF 등속 모델로
  평가 시각까지 예측(p + v·dt)해 참값과 비교. 호버 = ±0.3 s 평균 수평속도 ≤ 0.25 m/s.
  세 구조가 모두 유효한 시각만 쓴다.
바뀐 점은 하나 — 비행 구간 기준: 논문은 호버 고도 2 m에서 GT z ≥ 0.8 m, 실내는 이착륙도
보므로 GT z ≥ MIN_Z (ROI 하한 0.10 m 위로 기체가 올라온 뒤)로 둔다.
그림 양식(색·글꼴·크기)도 논문 그림 4·5와 같다.

실내 경로는 각 U턴(반원) 직전에 멈춰 호버한다 (indoor_survey_mission.TURN_HOLD_S).
"""

import csv
import glob
import os
import sys

import numpy as np

MIN_Z = float(os.environ.get('MIN_Z', '0.15'))   # 논문 데이터로 검증할 때는 MIN_Z=0.8
METHODS = ('PF', 'MF', 'SU')
CONDS = ('0', '5')
METRICS = (('rmse_xy_m', 'xy'), ('rmse_3d_m', 'd3'), ('hover_xy_rmse_m', 'hover'),
           ('moving_xy_rmse_m', 'moving'), ('z_bias_m', 'zbias'), ('z_std_m', 'zstd'))


def load(p):
    return np.atleast_2d(np.loadtxt(p, delimiter=',', skiprows=1))


def interp(d, t):
    return np.column_stack([np.interp(t, d[:, 0], d[:, j]) for j in (1, 2, 3)])


def rmse(e):
    return float(np.sqrt(np.mean(np.sum(e ** 2, 1)))) if len(e) else float('nan')


def evaluate(root):
    cmp = os.path.join(root, 'cmp_state')
    flights = sorted(os.path.basename(p) for p in glob.glob(os.path.join(cmp, 'F*')))
    rows = []
    for fl in flights:
        for c in CONDS:
            dirs = {m: os.path.join(cmp, fl, m + c) for m in METHODS}
            if not all(os.path.exists(os.path.join(d, 'state_details.csv')) for d in dirs.values()):
                print(f'  {fl} 조건 {c}: 재생 결과가 모자라 건너뜀')
                continue
            truth = load(os.path.join(dirs['PF'], 'actual_ground_truth_path.csv'))
            times = np.arange(np.ceil(truth[0, 0] / .01) * .01, truth[-1, 0], .01)
            gt = interp(truth, times)
            elig = gt[:, 2] >= MIN_Z
            sp = np.linalg.norm((interp(truth, times + .3) - interp(truth, times - .3))[:, :2], axis=1) / .6
            hover = sp <= .25
            est, val = {}, {}
            for m, d in dirs.items():
                s = load(os.path.join(d, 'state_details.csv'))
                t = s[:, 0]
                idx = np.searchsorted(t, times, side='right') - 1
                i = np.maximum(idx, 0)
                val[m] = (idx >= 0) & (times - t[i] <= .25)
                est[m] = s[i, 1:4] + s[i, 4:7] * (times - t[i])[:, None]
            com = elig & np.logical_and.reduce([val[m] for m in METHODS])
            for m in METHODS:
                err = est[m] - gt
                rows.append(dict(flight=fl, cond=c, method=m,
                                 xy=rmse(err[com, :2]), d3=rmse(err[com]),
                                 hover=rmse(err[com & hover, :2]), moving=rmse(err[com & ~hover, :2]),
                                 zbias=float(err[com, 2].mean()), zstd=float(err[com, 2].std()),
                                 valid_s=float(com.sum() / 100), hover_s=float((com & hover).sum() / 100)))
    if not rows:
        sys.exit(f'{cmp} 에 평가할 재생 결과가 없습니다 — paper_replicate.sh replay 를 먼저 돌리세요.')

    with open(os.path.join(cmp, 'aligned_flights.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    with open(os.path.join(cmp, 'metrics_mean_std.csv'), 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['method', 'metric', 'n_flights', 'mean', 'std'])
        for c in CONDS:
            for m in METHODS:
                sel = [r for r in rows if r['method'] == m and r['cond'] == c]
                for name, key in METRICS:
                    v = [r[key] for r in sel]
                    if v:
                        w.writerow([m + c, name, len(v), np.mean(v), np.std(v, ddof=1)])

    n = len({r['flight'] for r in rows})
    print(f'비행 {n}회 (평가 구간 평균 {np.mean([r["valid_s"] for r in rows]):.1f} s, '
          f'그중 호버 {np.mean([r["hover_s"] for r in rows]):.1f} s)')
    for c in CONDS:
        print(f'== 조건 {c} ({"오차 없음" if c == "0" else "LiDAR2 외재 오차 주입"}) — {n}회 평균 ± 표준편차 [m]')
        for _, key in METRICS:
            print(f'  {key:7s}', '  '.join(
                f"{m} {np.mean([r[key] for r in rows if r['cond'] == c and r['method'] == m]):.3f}"
                f"±{np.std([r[key] for r in rows if r['cond'] == c and r['method'] == m], ddof=1):.3f}"
                for m in METHODS))
    print('== 비행별 이동 구간 XY RMSE (조건 0)')
    for fl in sorted({r['flight'] for r in rows}):
        r = {x['method']: x['moving'] for x in rows if x['cond'] == '0' and x['flight'] == fl}
        if r:
            print(f'  {fl}', '  '.join(f'{m} {v:.3f}' for m, v in r.items()), ' 최소', min(r, key=r.get))
    print('저장:', os.path.join(cmp, 'metrics_mean_std.csv'))


def figures(root):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.colors import to_rgb
    from matplotlib.patches import Patch

    series = [('PF', '#2a78d6'), ('MF', '#eb6834'), ('SU', '#1baf7a')]
    navy, lavender = '#160666', '#e8e6f2'
    plt.rcParams.update({
        'font.family': 'serif', 'font.serif': ['Liberation Serif', 'Nimbus Roman', 'DejaVu Serif'],
        'font.size': 8, 'axes.labelsize': 8, 'xtick.labelsize': 8, 'ytick.labelsize': 7.5,
        'legend.fontsize': 7, 'axes.linewidth': 0.6, 'mathtext.fontset': 'stix',
        'pdf.fonttype': 42, 'ps.fonttype': 42})
    st = {(r['method'], r['metric']): (float(r['mean']), float(r['std']))
          for r in csv.DictReader(open(os.path.join(root, 'cmp_state', 'metrics_mean_std.csv')))}
    out_dir = os.path.join(root, 'figs')
    os.makedirs(out_dir, exist_ok=True)

    def style(ax):
        ax.yaxis.grid(True, color=lavender, lw=0.6, zorder=0)
        for s in ('top', 'right'):
            ax.spines[s].set_visible(False)
        ax.tick_params(axis='x', length=0)

    def ymax(keys, conds):
        top = max(st[(m + c, k)][0] + st[(m + c, k)][1] for m, _ in series for c in conds for k in keys)
        return float(np.ceil(top * 1.35 / 0.01) * 0.01)

    # 그림 4: 조건 0, 호버·이동 XY RMSE
    groups = [('Hovering', 'hover_xy_rmse_m'), ('Moving', 'moving_xy_rmse_m')]
    fig, ax = plt.subplots(figsize=(3.5, 1.85))
    w = 0.26
    for g, (_, key) in enumerate(groups):
        for i, (m, c) in enumerate(series):
            mean, std = st[(m + '0', key)]
            x = g + (i - 1) * (w + 0.02)
            ax.bar(x, mean, width=w, color=c, edgecolor=navy, lw=0.5, zorder=2, label=m if g == 0 else None)
            ax.errorbar(x, mean, yerr=std, color='black', lw=0.7, capsize=1.8, zorder=3)
            ax.text(x, mean + std + 0.001, f'{mean:.3f}', ha='center', va='bottom', fontsize=6.3)
    ax.set_xticks(range(2))
    ax.set_xticklabels([g for g, _ in groups])
    ax.set_ylabel('XY RMSE [m]')
    ax.set_ylim(0, ymax([k for _, k in groups], ('0',)))
    style(ax)
    ax.legend(ncol=3, frameon=False, loc='upper left', handlelength=1.2, columnspacing=0.8, borderaxespad=0.1)
    fig.tight_layout()
    out = os.path.join(out_dir, 'fig4_xy_rmse_aligned')
    fig.savefig(out + '.pdf', bbox_inches='tight', pad_inches=0.02)
    fig.savefig(out + '.png', dpi=600, bbox_inches='tight', pad_inches=0.02)
    print('저장:', out + '.png')

    # 그림 5: 조건 0 vs 5, (a) 호버 (b) 이동
    def tint(c, k=0.55):
        r, g, b = to_rgb(c)
        return (r + (1 - r) * k, g + (1 - g) * k, b + (1 - b) * k)

    panels = [('(a)', 'Hovering', 'hover_xy_rmse_m'), ('(b)', 'Moving', 'moving_xy_rmse_m')]
    top = ymax([k for *_, k in panels], CONDS)
    fig, axes = plt.subplots(2, 1, figsize=(3.5, 3.9))
    w = 0.36
    for ax, (tag, title, key) in zip(axes, panels):
        for i, (m, c) in enumerate(series):
            for j, (cond, face) in enumerate((('0', tint(c)), ('5', c))):
                mean, std = st[(m + cond, key)]
                x = i + (j - 0.5) * (w + 0.02)
                ax.bar(x, mean, width=w, color=face, edgecolor=navy, lw=0.5, zorder=2)
                ax.errorbar(x, mean, yerr=std, color='black', lw=0.7, capsize=1.8, zorder=3)
                ax.text(x, mean + std + 0.001, f'{mean:.3f}', ha='center', va='bottom', fontsize=6.3)
        ax.set_xticks(range(3))
        ax.set_xticklabels([m for m, _ in series])
        ax.set_ylabel('XY RMSE [m]')
        ax.set_ylim(0, top)
        ax.set_title(r'$\mathbf{%s}$ %s' % (tag, title), fontsize=8, loc='left')
        style(ax)
    handles = []
    for m, c in series:
        handles += [Patch(facecolor=tint(c), edgecolor=navy, lw=0.5, label=f'{m} (no error)'),
                    Patch(facecolor=c, edgecolor=navy, lw=0.5, label=f'{m} (with error)')]
    axes[0].legend(handles=handles, ncol=3, frameon=False, loc='upper left', handlelength=1.2,
                   columnspacing=0.8, borderaxespad=0.1)
    fig.tight_layout(h_pad=1.0)
    out = os.path.join(out_dir, 'fig5_extrinsic_injection_aligned')
    fig.savefig(out + '.pdf', bbox_inches='tight', pad_inches=0.02)
    fig.savefig(out + '.png', dpi=600, bbox_inches='tight', pad_inches=0.02)
    print('저장:', out + '.png')


if __name__ == '__main__':
    root = os.path.abspath(os.path.expanduser(sys.argv[1] if len(sys.argv) > 1 else '.'))
    if '--figs' in sys.argv:
        figures(root)
    else:
        evaluate(root)
