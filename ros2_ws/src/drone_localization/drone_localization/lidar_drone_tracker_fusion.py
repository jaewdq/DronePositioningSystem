#!/usr/bin/env python3
"""
센서별 순차 갱신 트래커 (실험용).

기본 트래커(lidar_drone_tracker.py)는 두 LiDAR의 점을 합친 뒤 센트로이드
하나를 만들어 칼만필터에 1회 반영한다. 합치는 순간 어느 점이 어느 센서에서
왔는지 사라지므로, 각 센서가 "자기를 향한 면만 본다"는 데서 생기는 편향을
보정할 방법이 없다. 점이 많은(=가까운) 센서가 결과를 지배해 센트로이드가
그쪽으로 끌린다(실측 진폭 0.048m).

이 노드는 군집화까지는 동일하게 융합 클라우드로 하고(그래야 점이 적은
센서도 클러스터에 기여한다), 선택된 클러스터만 출처 태그로 나눠 센서별
관측 2개를 만든 뒤 칼만필터에 순차 반영한다:

    predict → 센서A로 update → (시각차만큼 predict) → 센서B로 update

각 센서의 관측 모델은 자기 방향의 편향을 포함한다:

    h_i(x) = p + δ · û_i          û_i = 표적 → 센서 i 단위벡터

δ는 기체마다 다르므로 상수로 박지 않고 **비행 중에 측정**한다. 양쪽 센서가
동시에 잡힌 프레임에서 두 센트로이드가 벌어진 정도가 곧 2δ이므로, 그 값을
이동 중앙값으로 누적한다. 큰 기체가 오면 큰 δ가, 작은 기체가 오면 작은 δ가
저절로 잡힌다.

측정 잡음 R은 센서별로 다르게 준다. 점이 적을수록/멀수록 센트로이드가
부정확하므로 R을 키운다 — 칼만이득이 알아서 낮아져 덜 반영된다.

디버깅:
  fusion_debug_csv 파라미터에 경로를 주면 프레임마다 아래를 기록한다.
    시각, 센서별 점 개수/거리/센트로이드, δ 추정, 각 갱신의 innovation과 이득
  로그에는 2초마다 요약(양쪽 확보율, δ, 평균 innovation)이 찍힌다.

사용:
  ros2 run drone_localization lidar_drone_tracker_fusion \\
      --ros-args --params-file <yaml> \\
      -p fusion_debug_csv:=~/drone_project/flight_logs/fusion_debug.csv

  # 기본(융합 1회 갱신) 방식과 비교하려면
  -p per_sensor_update:=false
"""

import csv
import os
import time
from collections import deque

import numpy as np
import rclpy

from drone_localization.lidar_drone_tracker import LidarDroneTracker


class DualLidarFusionTracker(LidarDroneTracker):
    def __init__(self):
        super().__init__()

        self.declare_parameter('per_sensor_update', True)
        # δ 추정에 쓸 최근 관측 개수. 프레임 하나의 값은 흔들림이 커서
        # (실측 std 0.041m) 그대로 쓸 수 없고, 100프레임(=10초)쯤 모으면
        # ±0.007m로 안정된다.
        self.declare_parameter('delta_window', 100)
        # 이만큼 쌓이기 전에는 보정을 걸지 않는다(초기 몇 초).
        self.declare_parameter('delta_min_samples', 20)
        # δ 상한. 이보다 크게 측정되면 군집이 잘못 잡힌 것으로 보고 버린다.
        self.declare_parameter('delta_max', 0.25)
        # 센서별 관측 잡음: base + per_point / 점개수 + per_metre * 거리
        self.declare_parameter('meas_noise_base', 0.05)
        self.declare_parameter('meas_noise_per_point', 0.20)
        self.declare_parameter('meas_noise_per_metre', 0.004)
        self.declare_parameter('min_pts_per_sensor', 2)
        self.declare_parameter('fusion_debug_csv', '')

        gp = lambda n: self.get_parameter(n).value  # noqa: E731
        self.per_sensor_update = gp('per_sensor_update')
        self.delta_window = gp('delta_window')
        self.delta_min_samples = gp('delta_min_samples')
        self.delta_max = gp('delta_max')
        self.mn_base = gp('meas_noise_base')
        self.mn_per_point = gp('meas_noise_per_point')
        self.mn_per_metre = gp('meas_noise_per_metre')
        self.min_pts_sensor = gp('min_pts_per_sensor')
        self.debug_csv_path = gp('fusion_debug_csv')

        self._delta_hist = deque(maxlen=self.delta_window)
        self._sensor_xyz = [np.array([c.x, c.y, c.z]) for c in self._sensors]

        # 요약 로그용 누적치
        self._f_frames = 0
        self._f_both = 0
        self._f_innov = []
        self._f_t0 = time.monotonic()

        self._dbg_f = self._dbg_w = None
        if self.debug_csv_path:
            p = os.path.expanduser(self.debug_csv_path)
            os.makedirs(os.path.dirname(p) or '.', exist_ok=True)
            self._dbg_f = open(p, 'w', newline='')
            self._dbg_w = csv.writer(self._dbg_f)
            self._dbg_w.writerow([
                't', 'mode', 'n1', 'n2', 'd1', 'd2',
                'c1x', 'c1y', 'c1z', 'c2x', 'c2y', 'c2z',
                'delta', 'delta_n',
                'innov1', 'gain1', 'innov2', 'gain2',
                'est_x', 'est_y', 'est_z'])
            self.get_logger().info(f'융합 디버그 CSV: {p}')

        self.get_logger().info(
            f'센서별 순차 갱신 {"ON" if self.per_sensor_update else "OFF"} · '
            f'δ 창 {self.delta_window}프레임 · '
            f'센서당 최소 {self.min_pts_sensor}점')

    # ------------------------------------------------------------------
    @property
    def delta(self) -> float:
        """현재 δ 추정값. 표본이 모자라면 0(보정 안 함)."""
        if len(self._delta_hist) < self.delta_min_samples:
            return 0.0
        return float(np.median(self._delta_hist))

    def _meas_noise(self, n_pts: int, dist: float) -> float:
        """센서 하나의 센트로이드 표준편차 추정.

        점이 적을수록 센트로이드가 흔들리고(≈1/√n), 멀수록 각분해능 때문에
        같은 점 수라도 실제 산포가 커진다. 두 항을 더해 쓴다."""
        return (self.mn_base
                + self.mn_per_point / max(n_pts, 1) ** 0.5
                + self.mn_per_metre * dist)

    def _update_delta(self, c1, c2):
        """두 센서 센트로이드의 벌어짐에서 δ를 측정해 누적.

        각 센트로이드는 자기 센서 쪽 면에 얹혀 있으므로, 둘을 잇는 선분의
        절반이 δ다. 센서 축 방향 성분만 취해 옆으로 흔들린 잡음을 뺀다."""
        mid = (c1 + c2) / 2.0
        u = self._sensor_xyz[0] - mid
        nu = np.linalg.norm(u)
        if nu < 1e-6:
            return
        u = u / nu
        d = float(np.dot(c1 - mid, u))
        if 0.0 <= d <= self.delta_max:
            self._delta_hist.append(d)

    def _update_with_offset(self, z, offset, sigma):
        """관측 모델 h(x) = p + offset 로 1회 갱신. (innovation, 이득) 반환.

        KF6D.update()는 h(x)=p 고정이라 편향 항을 못 넣는다. 여기서는 예상
        관측값에 offset을 더해 innovation을 계산한다 — 필터가 '이 센서는
        원래 이만큼 치우쳐 찍힌다'를 알고 그만큼 빼고 비교하게 된다."""
        kf = self._kf
        H = kf.H
        R = np.eye(3) * sigma ** 2
        y = z - (H @ kf.x + offset)
        S = H @ kf.P @ H.T + R
        K = kf.P @ H.T @ np.linalg.solve(S, np.eye(3))
        kf.x = kf.x + K @ y
        IKH = np.eye(6) - K @ H
        kf.P = IKH @ kf.P @ IKH.T + K @ R @ K.T
        return float(np.linalg.norm(y)), float(np.trace(K[:3, :3]) / 3.0)

    # ------------------------------------------------------------------
    def _apply_measurement(self, centroid, pts_f, src, header):
        self._f_frames += 1
        t = header.stamp.sec + header.stamp.nanosec * 1e-9

        parts = None
        if (self.per_sensor_update and src is not None
                and len(src) == len(pts_f) and self._last_sel_mask is not None):
            tags = src[self._last_sel_mask]
            sel = self._last_sel_pts
            a, b = sel[tags == 0], sel[tags == 1]
            if len(a) >= self.min_pts_sensor and len(b) >= self.min_pts_sensor:
                parts = (a, b)

        if parts is None:
            # 한쪽만 잡혔거나 기능이 꺼진 경우 — 기존 방식 그대로.
            self._kf.update(centroid)
            self._dbg_row(t, 'fused', None, None, centroid)
            return

        self._f_both += 1
        a, b = parts
        c1, c2 = a.mean(axis=0), b.mean(axis=0)
        self._update_delta(c1, c2)
        delta = self.delta

        # 가까운 센서를 먼저 반영한다. 순차 갱신은 순서에 따라 결과가 미세하게
        # 달라지는데, 정보량이 많은 쪽을 먼저 넣어야 두 번째 갱신이 이미 좁혀진
        # 공분산 위에서 이뤄져 이상치에 덜 흔들린다.
        obs = []
        for cfg_i, c, pts in ((0, c1, a), (1, c2, b)):
            sxyz = self._sensor_xyz[cfg_i]
            u = sxyz - c
            dist = float(np.linalg.norm(u))
            u = u / max(dist, 1e-6)
            obs.append((dist, cfg_i, c, len(pts), u))
        obs.sort(key=lambda o: o[0])

        res = []
        for dist, cfg_i, c, npt, u in obs:
            innov, gain = self._update_with_offset(
                c, delta * u, self._meas_noise(npt, dist))
            res.append((innov, gain))
            self._f_innov.append(innov)

        self._dbg_row(t, 'per_sensor', (c1, c2), (obs, res, delta), centroid)

    def _dbg_row(self, t, mode, cs, extra, centroid):
        if self._dbg_w is None:
            return
        p = self._kf.position
        if cs is None:
            self._dbg_w.writerow(
                [f'{t:.3f}', mode, '', '', '', '', '', '', '', '', '', '',
                 f'{self.delta:.4f}', len(self._delta_hist), '', '', '', '',
                 f'{p[0]:.4f}', f'{p[1]:.4f}', f'{p[2]:.4f}'])
        else:
            c1, c2 = cs
            obs, res, delta = extra
            by = {o[1]: (o[0], o[3]) for o in obs}
            d1, n1 = by.get(0, (0, 0))
            d2, n2 = by.get(1, (0, 0))
            (i1, g1), (i2, g2) = res[0], res[1]
            self._dbg_w.writerow([
                f'{t:.3f}', mode, n1, n2, f'{d1:.2f}', f'{d2:.2f}',
                f'{c1[0]:.4f}', f'{c1[1]:.4f}', f'{c1[2]:.4f}',
                f'{c2[0]:.4f}', f'{c2[1]:.4f}', f'{c2[2]:.4f}',
                f'{delta:.4f}', len(self._delta_hist),
                f'{i1:.4f}', f'{g1:.3f}', f'{i2:.4f}', f'{g2:.3f}',
                f'{p[0]:.4f}', f'{p[1]:.4f}', f'{p[2]:.4f}'])
        self._dbg_f.flush()

    # ------------------------------------------------------------------
    def _maybe_log_stats(self, now: float):
        super()._maybe_log_stats(now)
        if now - self._f_t0 < self._stat_period or self._f_frames == 0:
            return
        both = 100.0 * self._f_both / self._f_frames
        inn = np.mean(self._f_innov) if self._f_innov else 0.0
        d = self.delta
        state = (f'δ={d:.4f}m({len(self._delta_hist)}표본)'
                 if len(self._delta_hist) >= self.delta_min_samples
                 else f'δ 학습중({len(self._delta_hist)}/{self.delta_min_samples})')
        self.get_logger().info(
            f'  └ [융합] 양쪽확보 {self._f_both}/{self._f_frames}({both:.0f}%)  '
            f'{state}  평균 innovation {inn:.3f}m')
        self._f_frames = self._f_both = 0
        self._f_innov = []
        self._f_t0 = now

    def close(self):
        super().close()
        if self._dbg_f is not None:
            try:
                self._dbg_f.close()
                self.get_logger().info(
                    f'융합 디버그 저장: {os.path.expanduser(self.debug_csv_path)}')
            except Exception:
                pass


def main():
    rclpy.init()
    node = None
    try:
        node = DualLidarFusionTracker()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
