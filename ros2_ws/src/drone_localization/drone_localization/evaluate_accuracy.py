#!/usr/bin/env python3
"""
측위 정확도 평가 — LiDAR 추정 vs PX4 실측(ground truth).

측정 대상은 "드론이 계획 경로를 잘 따라갔는가"(비행 성능)가 아니라
**"측위 시스템이 드론 위치를 얼마나 정확히 맞혔는가"**(장비 정확도)다.
따라서 기준(truth)은 계획 궤적이 아니라 PX4가 보고한 실제 위치다.

내는 수치:
  • 경로점(호버 구간)별 — 추정 median vs 실측 median, 수평오차/Z오차
  • 집계 — 평균 / RMSE, 수평(XY)과 수직(Z)을 분리
  • 구간 분리 — 전체 / 호버 / 이동

== 왜 XY와 Z를 나누는가 ==
수직 오차에는 성격이 다른 두 성분이 섞여 있다:
  bias : LiDAR가 보는 것은 기체 겉면의 중심(센트로이드)인데 PX4가 보고하는
         것은 비행제어기 기준점(base_link)이라, 그 사이 고정 offset이 항상
         얹힌다. 기체 형상에 따라 달라지는 "정의 차이"이지 측정 실패가 아니다.
  std  : 같은 조건에서 재도 프레임마다 흔들리는 양. 이쪽이 실제 측정 정밀도.
둘을 합쳐 하나의 RMSE로 보고하면 정의 차이 때문에 장비가 부정확해 보인다.
그래서 Z는 bias / std / RMSE를 모두 따로 출력한다.

== 시간 정렬에 대하여 ==
flight_recorder가 두 스트림을 같은 monotonic 클록으로 찍으므로 기본값은
"보정 없음"이다. 다만 LiDAR 파이프라인 처리지연 때문에 추정이 실측보다
조금 늦게 기록되는데, 이 지연을 그대로 두면 이동 구간에서 오차로 잡힌다
(1.5m/s × 0.1s = 0.15m). 그래서 최적 시프트를 **진단값으로 함께 출력**하되,
기본 수치에는 적용하지 않는다 — 오차가 최소가 되도록 시간을 밀어버리면
실제보다 좋은 수치가 나오기 때문. 처리지연을 뺀 순수 측위 정확도를 보고
싶으면 --auto_align 을 쓴다.

사용법:
  python3 evaluate_accuracy.py            # 어디서든 바로 실행 가능
  ros2 run drone_localization evaluate_accuracy   # (빌드했다면 이것도 가능)
  # 옵션:
  #   --output_dir DIR   로그 디렉토리 (기본 ~/drone_project/flight_logs)
  #   --auto_align       처리지연을 보정한 뒤 평가 (순수 측위 정확도)
  #   --speed_threshold  호버 판정 속도 (기본 0.25 m/s)
  #   --gate_radius      마커 근접 판정 반경 (기본 4.5 m)
"""

import argparse
import csv
import os
import sys

# ROS 노드가 아니라 오프라인 분석 스크립트이므로 `python3 evaluate_accuracy.py` 로도
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


def _load_csv(path):
    if not os.path.isfile(path):
        return None
    rows = []
    with open(path) as f:
        reader = csv.reader(f)
        next(reader, None)
        for r in reader:
            if len(r) >= 4:
                rows.append([float(v) for v in r[:4]])
    if not rows:
        return None
    a = np.array(rows)
    return a[:, 0], a[:, 1], a[:, 2], a[:, 3]


def _load_markers():
    try:
        from drone_mission.generate_groundtruth_trajectory import MARKERS
        return MARKERS
    except Exception:
        return None


def _smoothed_speed(t, x, y, half_window_s=0.3):
    """±half_window_s 중심차분으로 수평 속도 추정 (analyze_waypoints와 동일 방식)."""
    n = len(t)
    speed = np.zeros(n)
    for i in range(n):
        i0 = np.searchsorted(t, t[i] - half_window_s)
        i1 = np.searchsorted(t, t[i] + half_window_s) - 1
        i1 = max(i1, i0)
        if i1 > i0:
            dt = t[i1] - t[i0]
            if dt > 0:
                speed[i] = float(np.hypot(x[i1] - x[i0], y[i1] - y[i0]) / dt)
    return speed


def _align(actual, estimate, shift=0.0):
    """추정 타임스탬프에 실측을 보간해서 짝을 맞춘다.

    반환: (te, xe, ye, ze, xa, ya, za) — 실측 구간 안에 드는 표본만.
    """
    ta, xa, ya, za = actual
    te, xe, ye, ze = estimate
    ts = te + shift
    m = (ts >= ta[0]) & (ts <= ta[-1])
    if m.sum() < 5:
        return None
    return (te[m], xe[m], ye[m], ze[m],
            np.interp(ts[m], ta, xa),
            np.interp(ts[m], ta, ya),
            np.interp(ts[m], ta, za))


def _find_lag(actual, estimate):
    """수평오차 median을 최소로 만드는 시프트를 탐색 (진단용).

    _align이 estimate[te]를 actual[te+shift]와 맞추므로, shift가 음수면
    "추정값이 |shift|초 전의 실제 위치와 가장 잘 맞는다" = 추정이 그만큼
    뒤처져 있다는 뜻이다. 파이프라인 처리시간 + KF 평활 지연의 합.
    """
    best = None
    for shift in np.arange(-2.0, 2.001, 0.02):
        al = _align(actual, estimate, shift)
        if al is None:
            continue
        _, xe, ye, _, xa, ya, _ = al
        score = float(np.median(np.hypot(xe - xa, ye - ya)))
        if best is None or score < best[0]:
            best = (score, shift)
    return best[1] if best else 0.0


def _stats_xy(exy):
    return {
        'n': len(exy),
        'mean': float(np.mean(exy)),
        'rmse': float(np.sqrt(np.mean(exy ** 2))),
        'p95': float(np.percentile(exy, 95)),
        'max': float(np.max(exy)),
    }


def _stats_z(ez):
    """Z는 부호를 살려서 bias(계통) / std(랜덤) / RMSE(합계)를 모두 낸다."""
    return {
        'n': len(ez),
        'bias': float(np.mean(ez)),
        'std': float(np.std(ez)),
        'rmse': float(np.sqrt(np.mean(ez ** 2))),
        'absmax': float(np.max(np.abs(ez))),
    }


def _segment_visits(t, near_mask, hover_mask, nearest_marker,
                    min_duration_s, merge_gap_s):
    """호버+근접이 유지되는 구간을 마커별 방문으로 묶는다."""
    active = near_mask & hover_mask
    n = len(t)

    segments = []
    i = 0
    while i < n:
        if active[i]:
            j = i
            while j < n and active[j]:
                j += 1
            vals, counts = np.unique(nearest_marker[i:j], return_counts=True)
            segments.append([vals[np.argmax(counts)], i, j - 1])
            i = j
        else:
            i += 1

    merged = []
    for seg in segments:
        if merged and merged[-1][0] == seg[0] and \
                (t[seg[1]] - t[merged[-1][2]]) <= merge_gap_s:
            merged[-1][2] = seg[2]
        else:
            merged.append(list(seg))

    out = []
    visit_counts = {}
    for mid, i0, i1 in merged:
        idxs = np.array([k for k in range(i0, i1 + 1)
                         if active[k] and nearest_marker[k] == mid])
        if len(idxs) < 2:
            continue
        duration = t[idxs[-1]] - t[idxs[0]]
        if duration < min_duration_s:
            continue
        visit_counts[mid] = visit_counts.get(mid, 0) + 1
        out.append({'marker_id': int(mid), 'visit': visit_counts[mid],
                    'idxs': idxs, 'duration': duration})
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--output_dir',
                    default=os.path.expanduser('~/drone_project/flight_logs'))
    ap.add_argument('--auto_align', action='store_true')
    ap.add_argument('--speed_threshold', type=float, default=0.25)
    ap.add_argument('--gate_radius', type=float, default=4.5)
    ap.add_argument('--min_duration', type=float, default=0.4)
    ap.add_argument('--merge_gap', type=float, default=1.0)
    args, _ = ap.parse_known_args()

    out_dir = args.output_dir
    actual = _load_csv(os.path.join(out_dir, 'actual_path.csv'))
    estimate = _load_csv(os.path.join(out_dir, 'estimated_path.csv'))
    if actual is None or estimate is None:
        print(f'actual_path.csv / estimated_path.csv 를 찾을 수 없습니다: {out_dir}\n'
              'flight_recorder를 켜고 비행한 뒤 다시 실행하세요.')
        return

    markers = _load_markers()
    if markers is None:
        print('경고: MARKERS를 불러오지 못해 경로점별 표는 생략합니다.')

    lag = _find_lag(actual, estimate)
    shift = lag if args.auto_align else 0.0
    al = _align(actual, estimate, shift)
    if al is None:
        print('두 로그의 시간 구간이 겹치지 않습니다.')
        return
    te, xe, ye, ze, xa, ya, za = al

    exy = np.hypot(xe - xa, ye - ya)
    ez = ze - za

    # 호버/이동 분류는 **실측(truth) 속도** 기준. 추정 속도로 나누면
    # 측정 대상(추정)이 분류에도 개입해서 판정이 순환 논리가 된다.
    speed = _smoothed_speed(te, xa, ya)
    hover = speed <= args.speed_threshold

    print('=' * 62)
    print(' 측위 정확도 평가 (LiDAR 추정 vs PX4 실측)')
    print('=' * 62)
    print(f' 표본 {len(te)}개, 구간 {te[-1] - te[0]:.1f}초')
    if args.auto_align:
        print(f' 시간정렬: 추정 지연 {abs(lag):.2f}s 보정함 (--auto_align)')
    else:
        print(f' 시간정렬: 보정 없음 (공유 클록). 추정이 실측보다 '
              f'{abs(lag):.2f}s 뒤처짐')
        if abs(lag) > 0.05:
            print(f'   └ 이동 중에는 이 지연이 위치오차로 나타남 '
                  f'(1.5m/s × {abs(lag):.2f}s ≈ {1.5 * abs(lag):.2f}m).')
            print(f'     지연을 뺀 순수 측위 정확도는 --auto_align 으로 확인.')

    # 구간 소요시간은 표본수 × 프레임간격으로 낸다. 첫/끝 표본의 시각 차이를
    # 쓰면 호버처럼 띄엄띄엄 떨어진 구간에서 전체 비행시간에 가까운 값이 나온다.
    dt_med = float(np.median(np.diff(te))) if len(te) > 1 else 0.1

    def _block(title, mask):
        if mask.sum() < 2:
            print(f'\n── {title} ── 표본 부족')
            return None, None
        sxy = _stats_xy(exy[mask])
        sz = _stats_z(ez[mask])
        dur = int(mask.sum()) * dt_med
        print(f'\n── {title} ──  {sxy["n"]}표본 / {dur:.1f}초')
        print(f'  수평(XY)   평균 {sxy["mean"]:.3f}   RMSE {sxy["rmse"]:.3f}   '
              f'P95 {sxy["p95"]:.3f}   max {sxy["max"]:.3f}   [m]')
        print(f'  수직(Z)    bias {sz["bias"]:+.3f}  std {sz["std"]:.3f}  '
              f'RMSE {sz["rmse"]:.3f}  |max| {sz["absmax"]:.3f}   [m]')
        print(f'             └ bias=센트로이드 정의차(기체 형상 의존), '
              f'std=실제 측정 정밀도')
        return sxy, sz

    _block('전체 구간', np.ones(len(te), dtype=bool))
    _block('호버 구간', hover)
    _block('이동 구간', ~hover)

    # ---- 경로점(방문)별 ----
    rows = []
    if markers is not None:
        mids = list(markers.keys())
        mxy = np.array([markers[m] for m in mids])
        # 근접 판정은 실측 위치 기준 (truth로 구간을 정의)
        d = np.linalg.norm(np.stack([xa, ya], axis=1)[:, None, :]
                           - mxy[None, :, :], axis=2)
        ni = np.argmin(d, axis=1)
        near = d[np.arange(len(te)), ni] <= args.gate_radius
        nearest_marker = np.array([mids[i] for i in ni])

        visits = _segment_visits(te, near, hover, nearest_marker,
                                 args.min_duration, args.merge_gap)
        if visits:
            print('\n' + '=' * 62)
            print(' 경로점별 추정 정확도 (구간 median 기준)')
            print('=' * 62)
            print(f'{"id":>3} {"방문":>4} {"n":>4}  '
                  f'{"추정 (x, y, z)":>24}  {"실측 (x, y, z)":>24}  '
                  f'{"수평":>6} {"Z":>7}')
            for v in visits:
                k = v['idxs']
                pe = np.median(np.stack([xe[k], ye[k], ze[k]], axis=1), axis=0)
                pa = np.median(np.stack([xa[k], ya[k], za[k]], axis=1), axis=0)
                e_xy = float(np.hypot(pe[0] - pa[0], pe[1] - pa[1]))
                e_z = float(pe[2] - pa[2])
                print(f'{v["marker_id"]:>3} {v["visit"]:>4} {len(k):>4}  '
                      f'({pe[0]:7.3f},{pe[1]:7.3f},{pe[2]:6.3f})  '
                      f'({pa[0]:7.3f},{pa[1]:7.3f},{pa[2]:6.3f})  '
                      f'{e_xy:6.3f} {e_z:+7.3f}')
                rows.append({
                    'marker_id': v['marker_id'], 'visit': v['visit'],
                    'n_samples': len(k), 'duration_s': v['duration'],
                    'est_x': pe[0], 'est_y': pe[1], 'est_z': pe[2],
                    'act_x': pa[0], 'act_y': pa[1], 'act_z': pa[2],
                    'err_xy': e_xy, 'err_z': e_z,
                })

            e_xy_all = np.array([r['err_xy'] for r in rows])
            e_z_all = np.array([r['err_z'] for r in rows])
            print('-' * 62)
            print(f' 경로점 {len(rows)}개 집계')
            print(f'   수평  평균 {e_xy_all.mean():.3f}   '
                  f'RMSE {np.sqrt((e_xy_all ** 2).mean()):.3f}   '
                  f'max {e_xy_all.max():.3f}   [m]')
            print(f'   수직  bias {e_z_all.mean():+.3f}  '
                  f'std {e_z_all.std():.3f}  '
                  f'RMSE {np.sqrt((e_z_all ** 2).mean()):.3f}   [m]')

            out_csv = os.path.join(out_dir, 'accuracy_waypoints.csv')
            with open(out_csv, 'w', newline='') as f:
                w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
                w.writeheader()
                for r in rows:
                    w.writerow({k: (f'{v:.4f}' if isinstance(v, float) else v)
                                for k, v in r.items()})
            print(f'\n 경로점 상세 저장: {out_csv}')
        else:
            print('\n호버 구간을 찾지 못해 경로점별 표는 생략합니다.')

    print()


if __name__ == '__main__':
    main()
