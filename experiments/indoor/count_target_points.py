#!/usr/bin/env python3
"""
지상 LiDAR가 기체에 점을 몇 개 찍는지 프레임마다 센다 (실내 케이지 시험용).

참값 기체 위치(/world/<world>/dynamic_pose/info, 0번 = 기체 모델 원점) 주변 박스 안에
들어온 각 센서의 점 수와 링 수를 CSV로 남긴다. 트래커 추정(/drone/estimated_pose)이
있으면 같이 기록해 검출·오차도 볼 수 있다.

센서 외부 파라미터는 트래커 params yaml에서 읽는다 (월드와 동기화된 값).

  python3 count_target_points.py --out run.csv \
      --params ~/drone_project/DronePositioningSystem/ros2_ws/src/drone_localization/params/lidar_localization_indoor.yaml
"""

import argparse
import bisect
import csv
import math
import os

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import PoseArray, PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2

# 기체 모델 원점(base_link) 기준 박스. Crazyflie 메시는 원점 위 0~2.9 cm, 수평 ±5 cm
# (x2면 0~5.7 cm, ±10 cm). 배율·자세 흔들림을 감안해 넉넉히 잡는다 — 필드 안에
# 기체 말고는 물체가 없어 박스를 키워도 다른 점이 섞이지 않는다.
BOX_XY = 0.15
BOX_Z = (-0.05, 0.12)
LANDED_Z = 0.02     # 참값 z가 이보다 낮으면 패드 위 — 바닥 점과 구분 불가라 따로 표시


def stamp_s(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def lidar_poses_from_sdf(path):
    """world SDF의 ground_lidar_N 모델 pose를 트래커 파라미터 이름으로 돌려준다.
    월드가 센서 위치의 원본이므로, 배치·높이 비교 시험에서 yaml과 어긋날 일이 없다."""
    import xml.etree.ElementTree as ET
    out = {}
    for m in ET.parse(os.path.expanduser(path)).getroot().iter('model'):
        name = m.get('name', '')
        if name.startswith('ground_lidar_'):
            i = int(name.rsplit('_', 1)[1])
            x, y, z, roll, pitch, yaw = map(float, m.find('pose').text.split())
            out.update({f'lidar{i}_x': x, f'lidar{i}_y': y, f'lidar{i}_z': z,
                        f'lidar{i}_roll': roll, f'lidar{i}_pitch': pitch, f'lidar{i}_yaw': yaw})
    return out


class Counter(Node):
    def __init__(self, args):
        super().__init__('count_target_points')
        p = yaml.safe_load(open(os.path.expanduser(args.params)))
        p = p['lidar_drone_tracker']['ros__parameters']
        if args.world_sdf:
            p.update(lidar_poses_from_sdf(args.world_sdf))
        self.sensors = {}
        for i in (1, 2):
            yaw = p[f'lidar{i}_yaw']
            c, s = math.cos(yaw), math.sin(yaw)
            R = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
            z = args.lidar_z if args.lidar_z is not None else p[f'lidar{i}_z']
            t = np.array([p[f'lidar{i}_x'], p[f'lidar{i}_y'], z])
            self.sensors[i] = (R, t)
            self.create_subscription(PointCloud2, p[f'lidar{i}_topic'],
                                     lambda m, i=i: self.on_cloud(i, m), qos_profile_sensor_data)
        self.create_subscription(PoseArray, f'/world/{args.world}/dynamic_pose/info',
                                 self.on_gt, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/drone/estimated_pose',
                                 self.on_est, 10)
        self.gt_t, self.gt_p = [], []
        self.est = None
        self.f = open(args.out, 'w', newline='')
        self.w = csv.writer(self.f)
        self.w.writerow(['t', 'sensor', 'gt_x', 'gt_y', 'gt_z', 'landed', 'dist_h', 'elev_deg',
                         'n_pts', 'n_rings', 'est_x', 'est_y', 'est_z', 'est_age'])
        self.n = 0

    def on_gt(self, m):
        if not m.poses:             # 시뮬 재시작 직후 등 기체가 아직 없을 때
            return
        t = stamp_s(m.header.stamp)
        if self.gt_t and t < self.gt_t[-1]:     # 시뮬 재시작으로 시각이 되돌아감
            self.gt_t.clear()
            self.gt_p.clear()
        self.gt_t.append(t)
        q = m.poses[0].position
        self.gt_p.append((q.x, q.y, q.z))
        if len(self.gt_t) > 2000:
            del self.gt_t[:1000], self.gt_p[:1000]

    def on_est(self, m):
        q = m.pose.pose.position
        self.est = (stamp_s(m.header.stamp), q.x, q.y, q.z)

    def gt_at(self, t):
        k = bisect.bisect_left(self.gt_t, t)
        if k == 0 or k >= len(self.gt_t):
            return None
        t0, t1 = self.gt_t[k - 1], self.gt_t[k]
        a = (t - t0) / (t1 - t0) if t1 > t0 else 0.0
        p0, p1 = np.array(self.gt_p[k - 1]), np.array(self.gt_p[k])
        return p0 + (p1 - p0) * a

    def on_cloud(self, i, m):
        t = stamp_s(m.header.stamp)
        g = self.gt_at(t)
        if g is None:
            return
        a = np.frombuffer(bytes(m.data), np.float32).reshape(-1, m.point_step // 4)[:, :3]
        rows = np.repeat(np.arange(m.height), m.width) if m.height > 1 else np.zeros(len(a), int)
        ok = np.isfinite(a).all(1)
        R, tr = self.sensors[i]
        w = a[ok] @ R.T + tr
        d = w - g
        inbox = (np.abs(d[:, 0]) <= BOX_XY) & (np.abs(d[:, 1]) <= BOX_XY) & \
                (d[:, 2] >= BOX_Z[0]) & (d[:, 2] <= BOX_Z[1])
        n_pts = int(inbox.sum())
        n_rings = int(len(np.unique(rows[ok][inbox])))
        dh = float(np.hypot(*(g[:2] - tr[:2])))
        elev = math.degrees(math.atan2(g[2] - tr[2], dh))
        if self.est is not None:
            ex, ey, ez, age = *self.est[1:], t - self.est[0]
        else:
            ex = ey = ez = age = float('nan')
        self.w.writerow([f'{t:.3f}', i, *(f'{v:.3f}' for v in g), int(g[2] < LANDED_Z),
                         f'{dh:.3f}', f'{elev:.1f}', n_pts, n_rings,
                         f'{ex:.3f}', f'{ey:.3f}', f'{ez:.3f}', f'{age:.3f}'])
        self.n += 1
        if self.n % 200 == 0:
            self.f.flush()
            self.get_logger().info(f'{self.n}행 기록 (최근: 센서{i} z={g[2]:.2f} 점 {n_pts})')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--params', required=True)
    ap.add_argument('--world', default='indoor_cage')
    ap.add_argument('--lidar-z', type=float, default=None,
                    help='두 LiDAR 높이를 yaml 대신 이 값으로 (높이 비교 시험용)')
    ap.add_argument('--world-sdf', default=None,
                    help='LiDAR 위치를 이 world SDF에서 읽는다 (yaml 값 대신)')
    ap.add_argument('--print-tracker-args', action='store_true',
                    help='--world-sdf의 LiDAR 위치를 트래커 -p 인자로 출력하고 끝낸다')
    args, ros_args = ap.parse_known_args()
    if args.print_tracker_args:
        poses = lidar_poses_from_sdf(args.world_sdf)
        print(' '.join(f'-p {k}:={v}' for k, v in sorted(poses.items())))
        return
    rclpy.init(args=ros_args)
    node = Counter(args)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.f.close()
        node.destroy_node()


if __name__ == '__main__':
    main()
