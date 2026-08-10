#!/usr/bin/env python3
"""
Flight recorder: logs the actual flown path and the LiDAR-estimated path
to CSV during a flight, so you can plot them afterwards with plot_flight.

Subscribes:
  /fmu/out/vehicle_local_position_v1  (PX4 ground-truth, NED)  → actual path
  /drone/estimated_pose               (LiDAR localization, world ENU) → estimate

Both are written in the Gazebo world frame (ENU) so they overlay directly
on the planned trajectory. PX4 local NED → world ENU:
  world_x = ned_y (east),  world_y = ned_x (north),  world_z = -ned_z (up)
(inverse of competition_mission's world_to_ned).

Rows are flushed as they arrive, so Ctrl+C (or a crash) still leaves a
complete CSV.

시작할 때 이전 기록이 남아 있으면 runNNN/ 으로 **자동 보관**한 뒤 새로
쓰기 시작한다 (auto_archive:=false 로 끌 수 있음). 이전에는 그냥
덮어썼기 때문에, 비행 후 archive_run을 깜빡하고 recorder를 다시 켜면
직전 비행 데이터가 사라졌다.

Usage:
  ros2 run drone_localization flight_recorder
  # ... fly the mission (competition_mission) ...
  # Ctrl+C when done, then:
  ros2 run drone_localization plot_flight
"""

import csv
import os
import re
import shutil
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
)
from geometry_msgs.msg import PoseWithCovarianceStamped

try:
    from px4_msgs.msg import VehicleLocalPosition
    HAS_PX4 = True
except ImportError:
    HAS_PX4 = False


class FlightRecorder(Node):
    def __init__(self):
        super().__init__('flight_recorder')

        default_dir = os.path.expanduser('~/drone_project/flight_logs')
        self.declare_parameter('output_dir', default_dir)
        self.declare_parameter(
            'actual_topic', '/fmu/out/vehicle_local_position_v1')
        self.declare_parameter('estimate_topic', '/drone/estimated_pose')
        # 시작할 때 기존 로그를 runNNN/ 으로 자동 보관할지 여부.
        self.declare_parameter('auto_archive', True)
        self.out_dir = self.get_parameter('output_dir').value
        os.makedirs(self.out_dir, exist_ok=True)

        self._actual_path = os.path.join(self.out_dir, 'actual_path.csv')
        self._est_path = os.path.join(self.out_dir, 'estimated_path.csv')

        # 아래에서 'w'로 여는 순간 이전 비행 기록이 사라진다. 매번 수동으로
        # archive_run을 돌리는 걸 잊으면 그대로 유실되므로, 지우기 직전에
        # 자동으로 보관한다.
        if self.get_parameter('auto_archive').value:
            self._auto_archive()

        # Open both files and write headers immediately
        self._actual_f = open(self._actual_path, 'w', newline='')
        self._actual_w = csv.writer(self._actual_f)
        self._actual_w.writerow(['t_sec', 'world_x', 'world_y', 'world_z'])

        self._est_f = open(self._est_path, 'w', newline='')
        self._est_w = csv.writer(self._est_f)
        self._est_w.writerow(['t_sec', 'world_x', 'world_y', 'world_z'])

        self._n_actual = 0
        self._n_est = 0
        # Single shared wall clock for BOTH streams. Using each message's own
        # embedded timestamp is wrong here: PX4 uses boot-microseconds and the
        # estimate uses ROS time — different clocks. Stamping both at reception
        # with one monotonic clock keeps them directly comparable (the <0.1s
        # LiDAR-processing latency is negligible for this analysis).
        self._t0 = time.monotonic()

        # PX4 publishes best-effort; estimate is reliable
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        est_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        if HAS_PX4:
            self.create_subscription(
                VehicleLocalPosition,
                self.get_parameter('actual_topic').value,
                self._actual_cb, px4_qos)
        else:
            self.get_logger().warn(
                'px4_msgs not found — actual (PX4) path will NOT be recorded. '
                'Only /drone/estimated_pose is logged.')

        self.create_subscription(
            PoseWithCovarianceStamped,
            self.get_parameter('estimate_topic').value,
            self._est_cb, est_qos)

        self.get_logger().info(
            f'Recording flight to:\n  {self._actual_path}\n  {self._est_path}\n'
            'Fly the mission, then Ctrl+C and run: ros2 run drone_localization plot_flight')

    # ------------------------------------------------------------------
    # 시작 시 이전 로그 자동 보관
    # ------------------------------------------------------------------
    def _n_rows(self, path):
        """헤더를 뺀 데이터 줄 수. 파일이 없으면 0."""
        if not os.path.isfile(path):
            return 0
        try:
            with open(path) as f:
                return max(sum(1 for _ in f) - 1, 0)
        except OSError:
            return 0

    def _auto_archive(self):
        n_act = self._n_rows(self._actual_path)
        n_est = self._n_rows(self._est_path)
        if n_act == 0 and n_est == 0:
            return  # 보관할 게 없음 (첫 실행이거나 이미 옮겨감)

        # 번호는 archive_run과 같은 규칙으로 매겨 두 경로가 섞이지 않게 한다.
        used = []
        for name in os.listdir(self.out_dir):
            m = re.fullmatch(r'run(\d+)', name)
            if m and os.path.isdir(os.path.join(self.out_dir, name)):
                used.append(int(m.group(1)))
        dest = os.path.join(self.out_dir,
                            f'run{max(used) + 1 if used else 1:03d}')
        os.makedirs(dest, exist_ok=True)

        for src in (self._actual_path, self._est_path):
            if os.path.isfile(src):
                shutil.move(src, os.path.join(dest, os.path.basename(src)))

        self.get_logger().info(
            f'이전 기록을 자동 보관했습니다 → {dest} '
            f'(실측 {n_act}, 추정 {n_est}표본)')

    def _actual_cb(self, msg: 'VehicleLocalPosition'):
        if not (msg.xy_valid and msg.z_valid):
            return
        t = time.monotonic() - self._t0        # shared clock
        # NED → world ENU
        world_x = msg.y
        world_y = msg.x
        world_z = -msg.z
        self._actual_w.writerow(
            [f'{t:.4f}', f'{world_x:.4f}', f'{world_y:.4f}', f'{world_z:.4f}'])
        self._actual_f.flush()
        self._n_actual += 1

    def _est_cb(self, msg: PoseWithCovarianceStamped):
        t = time.monotonic() - self._t0        # same shared clock
        p = msg.pose.pose.position
        self._est_w.writerow(
            [f'{t:.4f}', f'{p.x:.4f}', f'{p.y:.4f}', f'{p.z:.4f}'])
        self._est_f.flush()
        self._n_est += 1

    def close(self):
        try:
            self._actual_f.close()
            self._est_f.close()
        except Exception:
            pass
        self.get_logger().info(
            f'Saved {self._n_actual} actual + {self._n_est} estimated samples.')


def main():
    rclpy.init()
    node = FlightRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.try_shutdown()
