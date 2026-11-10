#!/usr/bin/env python3
"""
RViz용 실내 비행 궤적 표시 노드.

  /viz/planned_path    (nav_msgs/Path)   indoor_survey_mission 이 날릴 계획 경로 (회색)
  /viz/true_path       (nav_msgs/Path)   Gazebo 참값 궤적 — dynamic_pose 브리지가 있어야 함
  /viz/estimated_path  (nav_msgs/Path)   트래커 추정 궤적 (/drone/estimated_pose)
  /viz/cage            (MarkerArray)     필드·케이지 경계, LiDAR 위치

  python3 path_viz.py [--world indoor_cage] [--save 결과폴더]
궤적은 계속 쌓이고, 참값 기체가 이륙 지점 바닥으로 돌아오면(재이륙 대비) 비운다.
--save 를 주면 Ctrl+C로 끌 때 참값·추정 궤적 CSV와 결과 그림(path_result.png)을 남긴다.
"""

import argparse
import csv
import math
import os

import rclpy
from geometry_msgs.msg import Point, PoseArray, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray

from drone_mission.indoor_survey_mission import build_waypoints, sample_waypoints

FRAME = 'map'
MIN_STEP = 0.01          # 이보다 덜 움직이면 궤적에 점을 더하지 않는다 [m]
FIELD = (0.0, 0.0, 3.0, 2.0)
CAGE = (-1.0, -1.0, 4.0, 3.0)
LIDARS = ((1.5, -0.7, 1.2), (1.5, 2.7, 1.2))


class PathViz(Node):
    def __init__(self, world):
        super().__init__('indoor_path_viz')
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.pub_plan = self.create_publisher(Path, '/viz/planned_path', latched)
        self.pub_true = self.create_publisher(Path, '/viz/true_path', 10)
        self.pub_est = self.create_publisher(Path, '/viz/estimated_path', 10)
        self.pub_cage = self.create_publisher(MarkerArray, '/viz/cage', latched)
        self.true_path = Path()
        self.est_path = Path()
        self.true_log, self.est_log = [], []        # 저장용 (t, x, y, z) — 비우지 않고 전부 남긴다
        self.true_path.header.frame_id = self.est_path.header.frame_id = FRAME
        self.create_subscription(PoseArray, f'/world/{world}/dynamic_pose/info', self.on_true, 10)
        reliable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(PoseWithCovarianceStamped, '/drone/estimated_pose', self.on_est, reliable)
        self.publish_static()
        self.create_timer(0.2, self.publish_paths)

    def _pose(self, x, y, z, stamp=None):
        p = PoseStamped()
        p.header.frame_id = FRAME
        if stamp is not None:
            p.header.stamp = stamp
        p.pose.position.x, p.pose.position.y, p.pose.position.z = float(x), float(y), float(z)
        p.pose.orientation.w = 1.0
        return p

    @staticmethod
    def _append(path, pose):
        if path.poses:
            q = path.poses[-1].pose.position
            r = pose.pose.position
            if math.dist((q.x, q.y, q.z), (r.x, r.y, r.z)) < MIN_STEP:
                return
        path.poses.append(pose)

    def on_true(self, m):
        if not m.poses:
            return
        q = m.poses[0].position
        # 이륙 지점 바닥에 다시 앉으면 다음 비행을 위해 궤적을 비운다
        if self.true_path.poses and q.z < 0.03 and math.hypot(q.x, q.y) < 0.1 \
                and len(self.true_path.poses) > 50:
            self.true_path.poses.clear()
            self.est_path.poses.clear()
        self._append(self.true_path, self._pose(q.x, q.y, q.z, m.header.stamp))
        self.true_log.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, q.x, q.y, q.z))

    def on_est(self, m):
        q = m.pose.pose.position
        self._append(self.est_path, self._pose(q.x, q.y, q.z, m.header.stamp))
        self.est_log.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, q.x, q.y, q.z))

    def save(self, out_dir):
        """궤적 CSV 두 개와, 비행 구간(참값 z ≥ 0.15 m)의 위·시간별 비교 그림을 남긴다."""
        import numpy as np
        os.makedirs(out_dir, exist_ok=True)
        for name, log in (('true_path.csv', self.true_log), ('estimated_path.csv', self.est_log)):
            with open(os.path.join(out_dir, name), 'w', newline='') as f:
                w = csv.writer(f)
                w.writerow(['t', 'x', 'y', 'z'])
                w.writerows(log)
        if len(self.true_log) < 2 or len(self.est_log) < 2:
            print(f'궤적이 모자라 그림은 건너뜀 (참값 {len(self.true_log)}, 추정 {len(self.est_log)}) → {out_dir}')
            return
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        from matplotlib.patches import Rectangle
        for f in ('NanumGothic', 'Noto Sans CJK JP'):
            if any(f == x.name for x in matplotlib.font_manager.fontManager.ttflist):
                plt.rcParams['font.family'] = f
                break
        plt.rcParams['axes.unicode_minus'] = False
        tr, es = np.array(self.true_log), np.array(self.est_log)
        gt = np.column_stack([np.interp(es[:, 0], tr[:, 0], tr[:, j]) for j in (1, 2, 3)])
        fly = gt[:, 2] >= 0.15
        exy = np.linalg.norm(es[:, 1:3] - gt[:, :2], axis=1)
        rmse = float(np.sqrt(np.mean(exy[fly] ** 2))) if fly.any() else float('nan')
        plan = np.array(sample_waypoints(build_waypoints()))
        fig, (a, b) = plt.subplots(1, 2, figsize=(13, 4.8), gridspec_kw={'width_ratios': [1.1, 1]})
        a.add_patch(Rectangle((0, 0), 3, 2, fc='0.94', ec='k', ls='--'))
        a.plot(plan[:, 0], plan[:, 1], color='0.6', lw=1, label='계획 경로')
        tf = tr[tr[:, 3] >= 0.15]
        a.plot(tf[:, 1], tf[:, 2], color='tab:green', lw=2, label='실제 (참값)')
        a.plot(es[fly, 1], es[fly, 2], '.', color='tab:orange', ms=3, label='LiDAR 추정')
        for x, y, _ in LIDARS:
            a.plot(x, y, 'o', color='0.2', ms=9)
        a.set(aspect='equal', xlim=(-0.5, 3.5), ylim=(-1.0, 3.0), xlabel='x [m]', ylabel='y [m]',
              title='위에서 본 경로 (비행 구간)')
        a.legend(fontsize=8, loc='upper left')
        t0 = es[0, 0]
        b.plot(es[:, 0] - t0, np.where(fly, exy, np.nan), color='tab:orange', lw=1)
        b.set(xlabel='시간 [s]', ylabel='수평 오차 [m]', title=f'수평 오차 — RMSE {rmse:.3f} m (추정 {fly.sum()}개)')
        b.grid(alpha=0.3)
        fig.savefig(os.path.join(out_dir, 'path_result.png'), dpi=120, bbox_inches='tight')
        print(f'저장: {out_dir}  (수평 RMSE {rmse:.3f} m)')

    def publish_paths(self):
        now = self.get_clock().now().to_msg()
        for path, pub in ((self.true_path, self.pub_true), (self.est_path, self.pub_est)):
            path.header.stamp = now
            pub.publish(path)

    def publish_static(self):
        plan = Path()
        plan.header.frame_id = FRAME
        plan.poses = [self._pose(x, y, z) for x, y, z in sample_waypoints(build_waypoints())]
        self.pub_plan.publish(plan)

        arr = MarkerArray()
        # 필드는 바닥과 비행 고도 두 곳에 그린다 — 3D 시점에서 고도 1 m 경로와 비교하기 쉽게
        fly_z = max(z for _, _, z in sample_waypoints(build_waypoints()))
        bounds = ((0, FIELD, 0.0, (0.95, 0.95, 0.95), 0.03),
                  (1, CAGE, 0.0, (0.6, 0.45, 0.25), 0.02),
                  (2, FIELD, fly_z, (0.95, 0.95, 0.95), 0.01))
        for i, (x0, y0, x1, y1), z, color, width in bounds:
            mk = Marker()
            mk.header.frame_id = FRAME
            mk.ns, mk.id, mk.type, mk.action = 'bounds', i, Marker.LINE_STRIP, Marker.ADD
            mk.scale.x = width
            mk.color.r, mk.color.g, mk.color.b, mk.color.a = (*color, 1.0 if z == 0.0 else 0.4)
            mk.points = [Point(x=x, y=y, z=z) for x, y in ((x0, y0), (x1, y0), (x1, y1), (x0, y1), (x0, y0))]
            mk.pose.orientation.w = 1.0
            arr.markers.append(mk)
        for i, (x, y, z) in enumerate(LIDARS):
            mk = Marker()
            mk.header.frame_id = FRAME
            mk.ns, mk.id, mk.type, mk.action = 'lidars', i, Marker.CYLINDER, Marker.ADD
            mk.pose.position.x, mk.pose.position.y, mk.pose.position.z = x, y, z
            mk.pose.orientation.w = 1.0
            mk.scale.x = mk.scale.y = 0.09
            mk.scale.z = 0.08
            mk.color.r, mk.color.g, mk.color.b, mk.color.a = 1.0, 0.45, 0.05, 1.0
            arr.markers.append(mk)
        self.pub_cage.publish(arr)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--world', default='indoor_cage')
    ap.add_argument('--save', default=None, help='끌 때 궤적 CSV와 그림을 남길 폴더')
    args, ros_args = ap.parse_known_args()
    rclpy.init(args=ros_args)
    node = PathViz(args.world)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if args.save:
            node.save(os.path.expanduser(args.save))
        node.destroy_node()


if __name__ == '__main__':
    main()
