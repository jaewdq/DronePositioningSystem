#!/usr/bin/env python3
"""
count_target_points.py 결과(points.csv)를 요약한다. 높이 비교 시험의 폴더를 여러 개
주면 높이별로 나란히 비교한다.

  python3 analyze_points.py ~/drone_project/flight_logs/indoor/height_sweep/h*

비행 구간만 본다(참값 z가 MIN_Z 이상). 착륙 상태는 바닥 점과 기체 점을 구분할 수
없어 뺀다.
"""

import csv
import math
import os
import sys
from collections import defaultdict

MIN_Z = 0.05        # 이보다 낮으면 바닥 위 (이착륙 접지 구간)
MIN_PTS = 3         # 트래커 cluster_min_points와 같은 기준


def load(path):
    rows = list(csv.DictReader(open(path)))
    for r in rows:
        for k in ('t', 'gt_x', 'gt_y', 'gt_z', 'dist_h', 'elev_deg',
                  'est_x', 'est_y', 'est_z', 'est_age'):
            r[k] = float(r[k])
        for k in ('sensor', 'n_pts', 'n_rings', 'landed'):
            r[k] = int(r[k])
    return rows


def pct(n, d):
    return 100.0 * n / d if d else float('nan')


def summarize(folder):
    rows = [r for r in load(os.path.join(folder, 'points.csv')) if r['gt_z'] >= MIN_Z]
    by_t = defaultdict(dict)
    for r in rows:
        by_t[r['t']][r['sensor']] = r
    ts = sorted(by_t)
    out = {'folder': os.path.basename(folder.rstrip('/')), 'frames': len(ts)}
    if len(ts) > 1:
        out['rate_hz'] = (len(ts) - 1) / (ts[-1] - ts[0])
    for s in (1, 2):
        rs = [r for r in rows if r['sensor'] == s]
        n = len(rs)
        out[f's{s}_hit'] = pct(sum(r['n_pts'] >= 1 for r in rs), n)
        out[f's{s}_ok'] = pct(sum(r['n_pts'] >= MIN_PTS for r in rs), n)
        pts = sorted(r['n_pts'] for r in rs)
        out[f's{s}_med'] = pts[n // 2] if n else float('nan')
    both = [by_t[t] for t in ts if len(by_t[t]) == 2]
    out['any_ok'] = pct(sum(max(f[1]['n_pts'], f[2]['n_pts']) >= MIN_PTS for f in both), len(both))
    out['both_ok'] = pct(sum(min(f[1]['n_pts'], f[2]['n_pts']) >= MIN_PTS for f in both), len(both))
    out['none'] = pct(sum(max(f[1]['n_pts'], f[2]['n_pts']) == 0 for f in both), len(both))
    # 트래커: 추정이 0.3 s 안에 갱신된 프레임 비율과 수평/수직 오차
    est = [f[1] for f in both if not math.isnan(f[1]['est_x']) and f[1]['est_age'] <= 0.3]
    out['trk_fresh'] = pct(len(est), len(both))
    if est:
        eh = [math.hypot(r['est_x'] - r['gt_x'], r['est_y'] - r['gt_y']) for r in est]
        ez = [r['est_z'] - r['gt_z'] for r in est]
        out['trk_xy_rmse'] = math.sqrt(sum(e * e for e in eh) / len(eh))
        out['trk_z_bias'] = sum(ez) / len(ez)
    return out, rows


def main():
    folders = sys.argv[1:] or ['.']
    cols = [('folder', '폴더', '{}'), ('frames', '프레임', '{}'), ('rate_hz', 'Hz', '{:.1f}'),
            ('s1_ok', 'L1≥3점%', '{:.0f}'), ('s2_ok', 'L2≥3점%', '{:.0f}'),
            ('s1_med', 'L1중앙', '{}'), ('s2_med', 'L2중앙', '{}'),
            ('any_ok', '하나라도%', '{:.0f}'), ('both_ok', '둘다%', '{:.0f}'),
            ('none', '0점%', '{:.0f}'), ('trk_fresh', '추정갱신%', '{:.0f}'),
            ('trk_xy_rmse', 'XY RMSE', '{:.3f}'), ('trk_z_bias', 'Z bias', '{:+.3f}')]
    print('  '.join(h for _, h, _ in cols))
    for f in folders:
        s, _ = summarize(f)
        print('  '.join(fmt.format(s[k]) if k in s else '-' for k, _, fmt in cols))


if __name__ == '__main__':
    main()
