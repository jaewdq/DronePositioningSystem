#!/usr/bin/env python3
"""경로 5종 시뮬 평가: shapes_eval.py <root> → <root>/cmp_state/shapes_per_flight.csv, shapes_summary.csv

평가 규칙은 논문·paper_eval.py 와 같다. 0.01 s 격자마다 각 구조의 최신 상태(0.25 s 이내)를 등속으로
그 시각까지 예측해 참값과 비교하고, 세 구조가 모두 유효한 시각만 쓴다. 호버 = ±0.3 s 평균 수평속도
0.25 m/s 이하. 비행 구간 = 참값 z ≥ MIN_Z. 평균 ± 표준편차는 비행 단위다.
검출률은 비행 구간의 스캔 시각 중 측정 갱신이 있었던 시각의 비율이다.
"""
import csv, glob, math, os, sys
import numpy as np

MIN_Z = 0.15
M = ('PF', 'MF', 'SU')


def rmse(e):
    return math.sqrt((e ** 2).sum(1).mean()) if len(e) else float('nan')


def flight(F):
    G = np.loadtxt(f'{F}/PF0/actual_ground_truth_path.csv', delimiter=',', skiprows=1)
    G = G[np.argsort(G[:, 0])]
    I = lambda t: np.column_stack([np.interp(t, G[:, 0], G[:, k]) for k in (1, 2, 3)])
    t = np.arange(math.ceil(G[0, 0] / .01) * .01, G[-1, 0], .01)
    gt = I(t)
    sp = np.linalg.norm((I(t + .3) - I(t - .3))[:, :2], axis=1) / .6
    hover = sp <= .25
    est, val, det = {}, {}, {}
    for m in M:
        d = f'{F}/{m}0'
        S = np.loadtxt(f'{d}/state_details.csv', delimiter=',', skiprows=1, usecols=range(7))
        S = S[np.argsort(S[:, 0], kind='stable')]
        i = np.searchsorted(S[:, 0], t, side='right') - 1
        j = np.maximum(i, 0)
        val[m] = (i >= 0) & (t - S[j, 0] <= .25)
        est[m] = S[j, 1:4] + S[j, 4:7] * (t - S[j, 0])[:, None]
        # 스캔은 헤더 스탬프로 센다. 수신 시각으로 나누면 도착 지터 때문에 칸이 어긋난다.
        E = np.loadtxt(f'{d}/estimate_details.csv', delimiter=',', skiprows=1, usecols=(0, 1))
        ms = set(np.round(np.loadtxt(f'{d}/measurements.csv', delimiter=',', skiprows=1, usecols=(0,)), 4))
        lag = np.median(E[:, 0] - E[:, 1])                 # 처리 시계 - 스탬프
        done = np.unique(np.round(E[[round(a, 4) in ms for a in E[:, 0]], 1] / .1).astype(int))
        scans = np.arange(math.ceil((t[0] - lag) / .1), math.floor((t[-1] - lag) / .1) + 1)
        fly = I(scans * .1 + lag)[:, 2] >= MIN_Z
        det[m] = (int(np.isin(scans[fly], done).sum()), int(fly.sum()))
    com = (gt[:, 2] >= MIN_Z) & np.logical_and.reduce([val[m] for m in M])
    rows = []
    for m in M:
        e = est[m] - gt
        rows.append(dict(method=m, xy=rmse(e[com, :2]) * 100, d3=rmse(e[com]) * 100,
                         hover=rmse(e[com & hover, :2]) * 100, moving=rmse(e[com & ~hover, :2]) * 100,
                         zbias=e[com, 2].mean() * 100, det_ok=det[m][0], det_n=det[m][1], det=det[m][0] / max(det[m][1], 1) * 100,
                         flight_s=com.sum() / 100, v_mean=sp[com].mean(), v_max=np.percentile(sp[com], 99.5)))
    return rows


root = sys.argv[1]
cmp_ = os.path.join(root, 'cmp_state')
out = []
for F in sorted(glob.glob(cmp_ + '/F_*')):
    if not all(os.path.exists(f'{F}/{m}0/state_details.csv') for m in M):
        continue
    shape, run = os.path.basename(F)[2:].rsplit('_', 1)
    out += [dict(shape=shape, run=run, **r) for r in flight(F)]
with open(cmp_ + '/shapes_per_flight.csv', 'w', newline='') as f:
    w = csv.DictWriter(f, list(out[0])); w.writeheader(); w.writerows(out)
order = [s for s in ('helix', 'circle', 'figure8', 'square', 'backforth') if any(r['shape'] == s for r in out)]
summ = []
print(f'{"":10s}{"":4s}{"XY":>12s}{"hover":>12s}{"moving":>12s}{"3D":>7s}{"z bias":>8s}{"det %":>8s}{"v mean":>7s}{"v max":>6s}')
for g in order + ['all']:
    for m in M:
        sel = [r for r in out if r['method'] == m and (g == 'all' or r['shape'] == g)]
        s = dict(group=g, method=m, n=len(sel))
        for k in ('xy', 'hover', 'moving', 'd3', 'zbias', 'v_mean', 'v_max'):
            v = np.array([r[k] for r in sel]); s[k] = v.mean(); s[k + '_sd'] = v.std(ddof=1) if len(v) > 1 else 0.0
        s['det'] = sum(r['det_ok'] for r in sel) / max(sum(r['det_n'] for r in sel), 1) * 100
        summ.append(s)
        print(f'{g:10s}{m:4s}{s["xy"]:6.2f}±{s["xy_sd"]:<4.2f} {s["hover"]:6.2f}±{s["hover_sd"]:<4.2f} {s["moving"]:6.2f}±{s["moving_sd"]:<4.2f}'
              f'{s["d3"]:7.2f}{s["zbias"]:+8.2f}{s["det"]:8.1f}{s["v_mean"]:7.2f}{s["v_max"]:6.2f}')
with open(cmp_ + '/shapes_summary.csv', 'w', newline='') as f:
    w = csv.DictWriter(f, list(summ[0])); w.writeheader(); w.writerows(summ)
